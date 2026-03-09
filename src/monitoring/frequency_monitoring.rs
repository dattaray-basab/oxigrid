//! Grid frequency monitoring and analysis.
//!
//! Implements phasor-based frequency measurement, ROCOF estimation,
//! IEEE 81/81R relays, underfrequency load shedding (UFLS), nadir
//! estimation via the swing equation, online inertia estimation, and
//! ENTSO-E / NERC compliance reporting.
//!
//! All physical quantities carry SI units unless noted:
//! - frequency  \[Hz\]
//! - rate-of-change \[Hz/s\]
//! - power       \[MW\]
//! - energy      \[MJ\]
//! - apparent    \[MVA\]
//! - time        \[s\]

// ──────────────────────────────────────────────────────────────────────────────
// FrequencyMeasurement
// ──────────────────────────────────────────────────────────────────────────────

/// A single frequency measurement snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct FrequencyMeasurement {
    /// Measured frequency \[Hz\].
    pub freq_hz: f64,
    /// Deviation from nominal \[Hz\].
    pub deviation_hz: f64,
    /// Rate of change of frequency \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// True when a frequency nadir event is currently detected.
    pub nadir_detected: bool,
    /// Timestamp of this measurement \[s\].
    pub timestamp_s: f64,
}

// ──────────────────────────────────────────────────────────────────────────────
// FrequencyMeter
// ──────────────────────────────────────────────────────────────────────────────

/// Phasor-based frequency meter with ROCOF estimation.
///
/// Maintains a circular buffer of recent frequency samples and derives
/// ROCOF by linear regression over the window.
///
/// # Example
/// ```
/// use oxigrid::monitoring::frequency_monitoring::FrequencyMeter;
/// let mut meter = FrequencyMeter::new(50.0, 10);
/// let m = meter.update(50.0);
/// assert!((m.freq_hz - 50.0).abs() < 1e-9);
/// ```
#[derive(Debug)]
pub struct FrequencyMeter {
    /// Nominal system frequency \[Hz\].
    pub nominal_hz: f64,
    /// Number of samples retained for ROCOF estimation.
    pub window_size: usize,
    /// Measurement deadband \[Hz\]; deviations smaller than this are zeroed.
    pub deadband_hz: f64,
    /// Internal circular buffer of (timestamp_s, freq_hz) pairs.
    sample_buffer: Vec<(f64, f64)>,
    /// Write pointer into the circular buffer.
    write_ptr: usize,
    /// Total samples received (saturates at usize::MAX).
    count: usize,
    /// Elapsed simulated time \[s\].
    time_s: f64,
    /// Approximate sampling interval \[s\].
    dt_s: f64,
    /// Previous frequency used for nadir detection \[Hz\].
    prev_freq: f64,
    /// Second-previous frequency for nadir detection \[Hz\].
    prev_prev_freq: f64,
}

impl FrequencyMeter {
    /// Create a new `FrequencyMeter`.
    ///
    /// * `nominal_hz`  — nominal system frequency \[Hz\], typically 50 or 60.
    /// * `window_size` — number of samples over which ROCOF is averaged.
    pub fn new(nominal_hz: f64, window_size: usize) -> Self {
        let window_size = window_size.max(2);
        Self {
            nominal_hz,
            window_size,
            deadband_hz: 1e-3,
            sample_buffer: Vec::with_capacity(window_size),
            write_ptr: 0,
            count: 0,
            time_s: 0.0,
            dt_s: 0.02, // default 50 ms / 20 Hz sample rate
            prev_freq: nominal_hz,
            prev_prev_freq: nominal_hz,
        }
    }

    /// Ingest one frequency sample and return a [`FrequencyMeasurement`].
    ///
    /// The first sample initialises the time base.  Subsequent samples
    /// advance `time_s` by the running average `dt_s`.
    pub fn update(&mut self, freq_sample: f64) -> FrequencyMeasurement {
        // Advance simulated time.
        if self.count > 0 {
            self.time_s += self.dt_s;
        }
        let ts = self.time_s;

        // Push into circular buffer.
        let entry = (ts, freq_sample);
        if self.sample_buffer.len() < self.window_size {
            self.sample_buffer.push(entry);
        } else {
            self.sample_buffer[self.write_ptr] = entry;
        }
        self.write_ptr = (self.write_ptr + 1) % self.window_size;
        self.count = self.count.saturating_add(1);

        // Compute ROCOF via least-squares over the window.
        let rocof = self.compute_rocof();

        // Deviation with deadband.
        let raw_dev = freq_sample - self.nominal_hz;
        let deviation_hz = if raw_dev.abs() < self.deadband_hz {
            0.0
        } else {
            raw_dev
        };

        // Nadir detection: previous sample was below current AND the one
        // before that was also declining — i.e., we just passed a trough.
        let nadir_detected =
            self.count >= 3 && self.prev_freq < self.prev_prev_freq && freq_sample > self.prev_freq;

        // Slide history.
        self.prev_prev_freq = self.prev_freq;
        self.prev_freq = freq_sample;

        FrequencyMeasurement {
            freq_hz: freq_sample,
            deviation_hz,
            rocof_hz_per_s: rocof,
            nadir_detected,
            timestamp_s: ts,
        }
    }

    /// Linear regression ROCOF over the current window \[Hz/s\].
    fn compute_rocof(&self) -> f64 {
        let n = self.sample_buffer.len();
        if n < 2 {
            return 0.0;
        }
        // Ordinary least-squares: y = a + b*t  →  b = ROCOF.
        let n_f = n as f64;
        let sum_t: f64 = self.sample_buffer.iter().map(|(t, _)| t).sum();
        let sum_f: f64 = self.sample_buffer.iter().map(|(_, f)| f).sum();
        let sum_t2: f64 = self.sample_buffer.iter().map(|(t, _)| t * t).sum();
        let sum_tf: f64 = self.sample_buffer.iter().map(|(t, f)| t * f).sum();

        let denom = n_f * sum_t2 - sum_t * sum_t;
        if denom.abs() < f64::EPSILON {
            return 0.0;
        }
        (n_f * sum_tf - sum_t * sum_f) / denom
    }

    /// Set the expected sampling interval \[s\] for timestamp advancement.
    pub fn set_dt(&mut self, dt_s: f64) {
        self.dt_s = dt_s.max(1e-6);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// RelayAction
// ──────────────────────────────────────────────────────────────────────────────

/// Action issued by a protective relay.
#[derive(Debug, Clone, PartialEq)]
pub enum RelayAction {
    /// No protective action required.
    NoAction,
    /// Advisory alarm; no disconnection.
    Alarm,
    /// Trip (disconnect) the monitored element.
    Trip,
    /// Shed the specified load \[MW\].
    LoadShed(f64),
}

// ──────────────────────────────────────────────────────────────────────────────
// RocofRelay  (IEEE 81R)
// ──────────────────────────────────────────────────────────────────────────────

/// IEEE 81R rate-of-change-of-frequency relay.
///
/// Trips when `|ROCOF| >= threshold_hz_per_s` persists for at least
/// `time_delay_s` seconds.
///
/// # Typical settings
/// - Island detection: 0.5 – 2.0 \[Hz/s\]
/// - Loss-of-mains: 1.0 \[Hz/s\] with 0.15 s delay
#[derive(Debug)]
pub struct RocofRelay {
    /// Trip threshold \[Hz/s\].
    pub threshold_hz_per_s: f64,
    /// Duration over which ROCOF is averaged \[s\] (informational).
    pub measurement_window_s: f64,
    /// Minimum consecutive time above threshold before trip \[s\].
    pub time_delay_s: f64,
    /// Whether the relay is in service.
    pub enabled: bool,
    /// Accumulated time above threshold \[s\].
    accumulated_time: f64,
    /// Whether an alarm has been issued.
    alarm_issued: bool,
}

impl RocofRelay {
    /// Construct an IEEE 81R relay.
    ///
    /// * `threshold_hz_per_s` — ROCOF trip threshold \[Hz/s\].
    /// * `time_delay_s`       — intentional time delay before trip \[s\].
    pub fn new(threshold_hz_per_s: f64, time_delay_s: f64) -> Self {
        Self {
            threshold_hz_per_s: threshold_hz_per_s.abs(),
            measurement_window_s: 0.1,
            time_delay_s,
            enabled: true,
            accumulated_time: 0.0,
            alarm_issued: false,
        }
    }

    /// Evaluate the relay given the latest measurement and time step `dt` \[s\].
    pub fn evaluate(&mut self, measurement: &FrequencyMeasurement, dt: f64) -> RelayAction {
        if !self.enabled {
            return RelayAction::NoAction;
        }
        if measurement.rocof_hz_per_s.abs() >= self.threshold_hz_per_s {
            self.accumulated_time += dt;
        } else {
            self.accumulated_time = 0.0;
            self.alarm_issued = false;
        }

        if self.accumulated_time >= self.time_delay_s {
            RelayAction::Trip
        } else if self.accumulated_time > self.time_delay_s * 0.5 && !self.alarm_issued {
            self.alarm_issued = true;
            RelayAction::Alarm
        } else {
            RelayAction::NoAction
        }
    }

    /// Reset accumulated timer (e.g., after a trip is cleared).
    pub fn reset(&mut self) {
        self.accumulated_time = 0.0;
        self.alarm_issued = false;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// FrequencyRelay  (IEEE 81)
// ──────────────────────────────────────────────────────────────────────────────

/// IEEE 81 under/over-frequency relay.
///
/// Issues `Trip` when frequency leaves the band
/// `[under_freq_threshold, over_freq_threshold]` for longer than
/// `time_delay_s` seconds.
#[derive(Debug)]
pub struct FrequencyRelay {
    /// Under-frequency trip threshold \[Hz\].
    pub under_freq_threshold: f64,
    /// Over-frequency trip threshold \[Hz\].
    pub over_freq_threshold: f64,
    /// Definite-time delay before trip \[s\].
    pub time_delay_s: f64,
    /// Additional intentional delay for coordination \[s\].
    pub intentional_delay_s: f64,
    /// Internal timer \[s\].
    timer: f64,
}

impl FrequencyRelay {
    /// Create a new `FrequencyRelay`.
    ///
    /// * `under_freq_threshold` — lower bound \[Hz\].
    /// * `over_freq_threshold`  — upper bound \[Hz\].
    /// * `time_delay_s`         — definite-time delay \[s\].
    pub fn new(under_freq_threshold: f64, over_freq_threshold: f64, time_delay_s: f64) -> Self {
        Self {
            under_freq_threshold,
            over_freq_threshold,
            time_delay_s,
            intentional_delay_s: 0.0,
            timer: 0.0,
        }
    }

    /// Evaluate given measured frequency `freq_hz` and time step `dt` \[s\].
    pub fn evaluate(&mut self, freq_hz: f64, dt: f64) -> RelayAction {
        let total_delay = self.time_delay_s + self.intentional_delay_s;
        if freq_hz < self.under_freq_threshold || freq_hz > self.over_freq_threshold {
            self.timer += dt;
            if self.timer >= total_delay {
                return RelayAction::Trip;
            }
            if self.timer > total_delay * 0.5 {
                return RelayAction::Alarm;
            }
        } else {
            self.timer = 0.0;
        }
        RelayAction::NoAction
    }

    /// Reset the internal timer.
    pub fn reset(&mut self) {
        self.timer = 0.0;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// UFLS — Underfrequency Load Shedding
// ──────────────────────────────────────────────────────────────────────────────

/// A single step in a UFLS scheme.
#[derive(Debug, Clone)]
pub struct UflsStep {
    /// Frequency below which this step activates \[Hz\].
    pub freq_threshold_hz: f64,
    /// Time delay after threshold is crossed \[s\].
    pub time_delay_s: f64,
    /// Fraction of `total_load_mw` to shed (0–1).
    pub shed_fraction: f64,
    /// Human-readable label.
    pub label: String,
    /// Whether this step has already fired.
    triggered: bool,
    /// Internal timer for the time delay \[s\].
    timer: f64,
}

impl UflsStep {
    /// Construct a UFLS step.
    pub fn new(
        freq_threshold_hz: f64,
        time_delay_s: f64,
        shed_fraction: f64,
        label: impl Into<String>,
    ) -> Self {
        Self {
            freq_threshold_hz,
            time_delay_s,
            shed_fraction,
            label: label.into(),
            triggered: false,
            timer: 0.0,
        }
    }
}

/// Underfrequency load shedding scheme.
///
/// Executes coordinated multi-step load shedding when frequency drops
/// below successive thresholds.
#[derive(Debug)]
pub struct UflsScheme {
    /// Ordered list of shedding steps (highest threshold first).
    pub steps: Vec<UflsStep>,
    /// Total system load \[MW\] used to scale `shed_fraction`.
    pub total_load_mw: f64,
    /// History of (timestamp \[s\], MW shed) events.
    pub shed_history: Vec<(f64, f64)>,
    /// Elapsed simulation time \[s\].
    time_s: f64,
}

impl UflsScheme {
    // ── Constructors ──────────────────────────────────────────────────────────

    /// Build the ENTSO-E 6-step UFLS scheme (Continental Europe).
    ///
    /// Steps at 49.0, 48.8, 48.6, 48.4, 48.2, 47.5 \[Hz\];
    /// each sheds 5 % except the last which sheds 10 % of total load.
    pub fn new_entso_e(total_load_mw: f64) -> Self {
        let steps = vec![
            UflsStep::new(49.0, 0.20, 0.05, "ENTSO-E Step 1 (49.0 Hz)"),
            UflsStep::new(48.8, 0.20, 0.05, "ENTSO-E Step 2 (48.8 Hz)"),
            UflsStep::new(48.6, 0.20, 0.05, "ENTSO-E Step 3 (48.6 Hz)"),
            UflsStep::new(48.4, 0.20, 0.05, "ENTSO-E Step 4 (48.4 Hz)"),
            UflsStep::new(48.2, 0.20, 0.05, "ENTSO-E Step 5 (48.2 Hz)"),
            UflsStep::new(47.5, 0.20, 0.10, "ENTSO-E Step 6 (47.5 Hz)"),
        ];
        Self {
            steps,
            total_load_mw,
            shed_history: Vec::new(),
            time_s: 0.0,
        }
    }

    /// Build a NERC-aligned 5-step UFLS scheme (60 Hz systems).
    ///
    /// Steps at 59.3, 58.9, 58.5, 58.1, 57.7 \[Hz\];
    /// each sheds 5 % of total load.
    pub fn new_nerc(total_load_mw: f64) -> Self {
        let steps = vec![
            UflsStep::new(59.3, 0.28, 0.05, "NERC Step 1 (59.3 Hz)"),
            UflsStep::new(58.9, 0.28, 0.05, "NERC Step 2 (58.9 Hz)"),
            UflsStep::new(58.5, 0.28, 0.05, "NERC Step 3 (58.5 Hz)"),
            UflsStep::new(58.1, 0.28, 0.05, "NERC Step 4 (58.1 Hz)"),
            UflsStep::new(57.7, 0.28, 0.05, "NERC Step 5 (57.7 Hz)"),
        ];
        Self {
            steps,
            total_load_mw,
            shed_history: Vec::new(),
            time_s: 0.0,
        }
    }

    // ── Simulation ────────────────────────────────────────────────────────────

    /// Advance the UFLS scheme by one time step.
    ///
    /// Returns the total load \[MW\] shed in this time step.
    pub fn step(&mut self, freq_hz: f64, dt: f64) -> f64 {
        self.time_s += dt;
        let mut shed_this_step = 0.0;

        for ufls_step in &mut self.steps {
            if ufls_step.triggered {
                continue;
            }
            if freq_hz < ufls_step.freq_threshold_hz {
                ufls_step.timer += dt;
                if ufls_step.timer >= ufls_step.time_delay_s {
                    ufls_step.triggered = true;
                    let mw = ufls_step.shed_fraction * self.total_load_mw;
                    shed_this_step += mw;
                }
            } else {
                // Reset timer if frequency recovers above threshold.
                ufls_step.timer = 0.0;
            }
        }

        if shed_this_step > 0.0 {
            self.shed_history.push((self.time_s, shed_this_step));
        }
        shed_this_step
    }

    /// Total cumulative load shed \[MW\].
    pub fn total_shed_mw(&self) -> f64 {
        self.shed_history.iter().map(|(_, mw)| mw).sum()
    }

    /// Reset all steps to un-triggered state (e.g., for a new event simulation).
    pub fn reset(&mut self) {
        for s in &mut self.steps {
            s.triggered = false;
            s.timer = 0.0;
        }
        self.shed_history.clear();
        self.time_s = 0.0;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// NadirEstimator
// ──────────────────────────────────────────────────────────────────────────────

/// Result of a nadir estimation.
#[derive(Debug, Clone, PartialEq)]
pub struct NadirEstimate {
    /// Estimated nadir frequency \[Hz\].
    pub nadir_freq_hz: f64,
    /// Estimated time from event to nadir \[s\].
    pub time_to_nadir_s: f64,
    /// Quasi-steady-state frequency after primary (governor) response \[Hz\].
    pub quasi_steady_state_hz: f64,
    /// Initial ROCOF immediately after disturbance \[Hz/s\].
    pub rocof_initial_hz_per_s: f64,
}

/// Frequency nadir estimator based on the swing equation with governor droop.
///
/// Uses a simplified analytical model:
///
/// ```text
/// ROCOF₀ = -ΔP · f₀ / (2·H·S_base)
/// f_qss  = f₀ - ΔP / (D + 1/R)   [after primary response]
/// t_nad  ≈ H · S_base · (f₀ - f_qss) / (f₀ · |ΔP|) · 2
/// ```
///
/// where `D` is the load damping coefficient and `R` is droop.
#[derive(Debug, Clone)]
pub struct NadirEstimator {
    /// System inertia constant H \[s\] (MJ/MVA).
    pub total_inertia_mj_per_mva: f64,
    /// System rated capacity \[MVA\].
    pub system_mva: f64,
    /// Aggregate droop response \[MW/Hz\].
    pub droop_response_mw_per_hz: f64,
    /// Governor time constant \[s\].
    pub governor_time_const_s: f64,
}

impl NadirEstimator {
    /// Construct a `NadirEstimator`.
    ///
    /// * `total_inertia_mj_per_mva` — system inertia H \[s\].
    /// * `system_mva`               — system rating \[MVA\].
    /// * `droop_response_mw_per_hz` — aggregate primary response gain \[MW/Hz\].
    /// * `governor_time_const_s`    — governor response time \[s\].
    pub fn new(
        total_inertia_mj_per_mva: f64,
        system_mva: f64,
        droop_response_mw_per_hz: f64,
        governor_time_const_s: f64,
    ) -> Self {
        Self {
            total_inertia_mj_per_mva,
            system_mva,
            droop_response_mw_per_hz,
            governor_time_const_s,
        }
    }

    /// Estimate the frequency nadir following a power imbalance event.
    ///
    /// * `initial_freq_hz`    — pre-disturbance frequency \[Hz\].
    /// * `power_imbalance_mw` — MW deficit (positive = generation loss).
    ///
    /// # Algorithm
    ///
    /// Numerical integration of the linearised swing equation with a
    /// first-order governor model (step-response approximation).
    pub fn estimate_nadir(&self, initial_freq_hz: f64, power_imbalance_mw: f64) -> NadirEstimate {
        let h = self.total_inertia_mj_per_mva.max(f64::EPSILON);
        let s_base = self.system_mva.max(f64::EPSILON);
        let f0 = initial_freq_hz;
        let dp = power_imbalance_mw; // MW; positive = generation loss

        // Initial ROCOF [Hz/s].
        let rocof0 = -dp * f0 / (2.0 * h * s_base);

        // Quasi-steady-state frequency deviation after primary response.
        // Δf_qss = -ΔP / D_total  where D_total = droop_response [MW/Hz].
        let d_total = self.droop_response_mw_per_hz.max(f64::EPSILON);
        let df_qss = -dp / d_total;
        let f_qss = f0 + df_qss;

        // Numerical integration of swing equation with governor (Euler, 10 ms).
        let dt = 0.01_f64; // [s]
        let t_gov = self.governor_time_const_s.max(f64::EPSILON);
        let mut freq = f0;
        let mut gov_output = 0.0_f64; // governor MW response
        let mut t = 0.0_f64;
        let mut nadir = f0;
        let mut t_nadir = 0.0_f64;
        let mut prev_rocof = rocof0;

        for _i in 0..2000 {
            // Governor first-order response toward steady-state output.
            let gov_ss = (f0 - freq) * d_total; // desired response
            let d_gov = (gov_ss - gov_output) / t_gov;
            gov_output += d_gov * dt;

            // Net imbalance [MW]: generation loss - governor relief.
            let net_imbalance = dp - gov_output;

            // Swing equation: df/dt = -net_imbalance * f0 / (2H * S_base).
            let rocof = -net_imbalance * f0 / (2.0 * h * s_base);
            freq += rocof * dt;
            t += dt;

            if freq < nadir {
                nadir = freq;
                t_nadir = t;
            }

            // Stop once ROCOF changes sign and frequency is recovering.
            if prev_rocof < 0.0 && rocof >= 0.0 {
                break;
            }
            prev_rocof = rocof;
        }

        NadirEstimate {
            nadir_freq_hz: nadir,
            time_to_nadir_s: t_nadir,
            quasi_steady_state_hz: f_qss,
            rocof_initial_hz_per_s: rocof0,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// InertiaEstimator
// ──────────────────────────────────────────────────────────────────────────────

/// Online inertia estimator from frequency event recordings.
///
/// Uses the swing equation:
/// ```text
/// H = -ΔP · f₀ / (2 · S_base · ROCOF)
/// ```
#[derive(Debug, Default)]
pub struct InertiaEstimator {
    /// Accumulated frequency measurements.
    measurements: Vec<FrequencyMeasurement>,
}

impl InertiaEstimator {
    /// Create an empty `InertiaEstimator`.
    pub fn new() -> Self {
        Self {
            measurements: Vec::new(),
        }
    }

    /// Append a frequency measurement.
    pub fn add_measurement(&mut self, m: FrequencyMeasurement) {
        self.measurements.push(m);
    }

    /// Estimate inertia H \[s\] using all stored measurements.
    ///
    /// * `power_imbalance_mw` — known or estimated MW disturbance.
    /// * `system_mva`         — system rating \[MVA\].
    ///
    /// Returns `Err` when insufficient data is available.
    pub fn estimate_inertia(
        &self,
        power_imbalance_mw: f64,
        system_mva: f64,
    ) -> Result<f64, String> {
        if self.measurements.is_empty() {
            return Err("No measurements available".into());
        }
        let s_base = system_mva.max(f64::EPSILON);

        // Average ROCOF and frequency over all measurements.
        let avg_rocof = self
            .measurements
            .iter()
            .map(|m| m.rocof_hz_per_s)
            .sum::<f64>()
            / self.measurements.len() as f64;

        let avg_freq = self.measurements.iter().map(|m| m.freq_hz).sum::<f64>()
            / self.measurements.len() as f64;

        if avg_rocof.abs() < 1e-6 {
            return Err("ROCOF too small for inertia estimation".into());
        }

        // H = -ΔP · f0 / (2 · S_base · ROCOF)
        let h = -power_imbalance_mw * avg_freq / (2.0 * s_base * avg_rocof);
        if h <= 0.0 {
            return Err(format!(
                "Inertia estimate non-physical: H = {h:.4} s (check sign of imbalance/ROCOF)"
            ));
        }
        Ok(h)
    }

    /// Estimate inertia \[s\] restricted to a time window `[start_s, end_s]`.
    ///
    /// * `start_s`            — window start \[s\].
    /// * `end_s`              — window end \[s\].
    /// * `power_imbalance_mw` — known MW disturbance.
    /// * `system_mva`         — system rating \[MVA\].
    pub fn estimate_from_event_window(
        &self,
        start_s: f64,
        end_s: f64,
        power_imbalance_mw: f64,
        system_mva: f64,
    ) -> Result<f64, String> {
        let window: Vec<&FrequencyMeasurement> = self
            .measurements
            .iter()
            .filter(|m| m.timestamp_s >= start_s && m.timestamp_s <= end_s)
            .collect();

        if window.is_empty() {
            return Err(format!(
                "No measurements in window [{start_s:.2}, {end_s:.2}] s"
            ));
        }

        let s_base = system_mva.max(f64::EPSILON);
        let n = window.len() as f64;
        let avg_rocof = window.iter().map(|m| m.rocof_hz_per_s).sum::<f64>() / n;
        let avg_freq = window.iter().map(|m| m.freq_hz).sum::<f64>() / n;

        if avg_rocof.abs() < 1e-6 {
            return Err("ROCOF too small for inertia estimation in window".into());
        }

        let h = -power_imbalance_mw * avg_freq / (2.0 * s_base * avg_rocof);
        if h <= 0.0 {
            return Err(format!(
                "Inertia estimate non-physical (window): H = {h:.4} s"
            ));
        }
        Ok(h)
    }

    /// Remove all stored measurements.
    pub fn clear(&mut self) {
        self.measurements.clear();
    }

    /// Number of stored measurements.
    pub fn len(&self) -> usize {
        self.measurements.len()
    }

    /// True when no measurements are stored.
    pub fn is_empty(&self) -> bool {
        self.measurements.is_empty()
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// FrequencyComplianceReport
// ──────────────────────────────────────────────────────────────────────────────

/// Frequency performance compliance report (ENTSO-E / NERC style).
///
/// Classifies time into three bands relative to the nominal frequency:
/// - **Normal**    : `|Δf| < 0.05` \[Hz\]
/// - **Alert**     : `0.05 ≤ |Δf| < 0.2` \[Hz\]
/// - **Emergency** : `|Δf| ≥ 0.2` \[Hz\]
///
/// A period is considered compliant when `time_in_emergency_pct < 0.1 %`
/// and `max_rocof_hz_per_s < 2.0` \[Hz/s\].
#[derive(Debug, Clone, PartialEq)]
pub struct FrequencyComplianceReport {
    /// Percentage of time within normal band \[%\].
    pub time_in_normal_band_pct: f64,
    /// Percentage of time within alert band \[%\].
    pub time_in_alert_band_pct: f64,
    /// Percentage of time in emergency state \[%\].
    pub time_in_emergency_pct: f64,
    /// Maximum ROCOF observed over the period \[Hz/s\].
    pub max_rocof_hz_per_s: f64,
    /// Number of nadir events detected.
    pub nadir_count: usize,
    /// Number of UFLS activations inferred from emergency excursions.
    pub ufls_activations: usize,
    /// True when the period passes ENTSO-E / NERC compliance criteria.
    pub compliant: bool,
}

impl FrequencyComplianceReport {
    /// Compute a compliance report from a slice of measurements.
    ///
    /// Assumes measurements are approximately uniformly spaced in time.
    pub fn compute(measurements: &[FrequencyMeasurement]) -> Self {
        if measurements.is_empty() {
            return Self {
                time_in_normal_band_pct: 100.0,
                time_in_alert_band_pct: 0.0,
                time_in_emergency_pct: 0.0,
                max_rocof_hz_per_s: 0.0,
                nadir_count: 0,
                ufls_activations: 0,
                compliant: true,
            };
        }

        let n = measurements.len() as f64;
        let mut normal = 0usize;
        let mut alert = 0usize;
        let mut emergency = 0usize;
        let mut max_rocof: f64 = 0.0;
        let mut nadir_count = 0usize;
        let mut ufls_activations = 0usize;
        let mut in_emergency = false;

        for m in measurements {
            let abs_dev = m.deviation_hz.abs();
            if abs_dev < 0.05 {
                normal += 1;
            } else if abs_dev < 0.2 {
                alert += 1;
            } else {
                emergency += 1;
                if !in_emergency {
                    ufls_activations += 1;
                    in_emergency = true;
                }
            }
            if abs_dev < 0.2 {
                in_emergency = false;
            }

            let abs_rocof = m.rocof_hz_per_s.abs();
            if abs_rocof > max_rocof {
                max_rocof = abs_rocof;
            }

            if m.nadir_detected {
                nadir_count += 1;
            }
        }

        let time_in_normal_band_pct = (normal as f64 / n) * 100.0;
        let time_in_alert_band_pct = (alert as f64 / n) * 100.0;
        let time_in_emergency_pct = (emergency as f64 / n) * 100.0;

        // Compliance criteria: emergency < 0.1 % of time, ROCOF < 2 Hz/s.
        let compliant = time_in_emergency_pct < 0.1 && max_rocof < 2.0;

        Self {
            time_in_normal_band_pct,
            time_in_alert_band_pct,
            time_in_emergency_pct,
            max_rocof_hz_per_s: max_rocof,
            nadir_count,
            ufls_activations,
            compliant,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── FrequencyMeter ────────────────────────────────────────────────────────

    #[test]
    fn test_frequency_meter_steady_rocof_near_zero() {
        let mut meter = FrequencyMeter::new(50.0, 20);
        meter.set_dt(0.02);
        let mut last = FrequencyMeasurement {
            freq_hz: 0.0,
            deviation_hz: 0.0,
            rocof_hz_per_s: 0.0,
            nadir_detected: false,
            timestamp_s: 0.0,
        };
        for _ in 0..20 {
            last = meter.update(50.0);
        }
        assert!(
            last.rocof_hz_per_s.abs() < 0.05,
            "steady ROCOF should be ≈ 0, got {}",
            last.rocof_hz_per_s
        );
    }

    #[test]
    fn test_frequency_meter_linear_decline_rocof() {
        // Frequency drops at exactly -1.0 Hz/s: f(k) = 50 - 0.02*k
        let mut meter = FrequencyMeter::new(50.0, 30);
        meter.set_dt(0.02);
        let rate = -1.0_f64; // Hz/s
        let dt = 0.02_f64;
        let mut last = FrequencyMeasurement {
            freq_hz: 50.0,
            deviation_hz: 0.0,
            rocof_hz_per_s: 0.0,
            nadir_detected: false,
            timestamp_s: 0.0,
        };
        for k in 0..30usize {
            let f = 50.0 + rate * (k as f64) * dt;
            last = meter.update(f);
        }
        let error = (last.rocof_hz_per_s - rate).abs();
        assert!(
            error < 0.15,
            "ROCOF should be ≈ -1.0 Hz/s, got {:.4}",
            last.rocof_hz_per_s
        );
    }

    #[test]
    fn test_frequency_meter_deviation_deadband() {
        let mut meter = FrequencyMeter::new(50.0, 5);
        // A tiny deviation inside the deadband should be zeroed.
        let m = meter.update(50.0 + 5e-4);
        assert_eq!(
            m.deviation_hz, 0.0,
            "inside deadband: deviation should be 0"
        );
    }

    // ── RocofRelay ────────────────────────────────────────────────────────────

    #[test]
    fn test_rocof_relay_no_trip_below_threshold() {
        let mut relay = RocofRelay::new(1.0, 0.1);
        let meas = FrequencyMeasurement {
            freq_hz: 49.8,
            deviation_hz: -0.2,
            rocof_hz_per_s: 0.4, // below 1.0 threshold
            nadir_detected: false,
            timestamp_s: 0.0,
        };
        // Simulate 1 second at dt=0.1 s
        let mut action = RelayAction::NoAction;
        for _ in 0..10 {
            action = relay.evaluate(&meas, 0.1);
        }
        assert_ne!(action, RelayAction::Trip, "ROCOF below threshold → no trip");
    }

    #[test]
    fn test_rocof_relay_trip_after_delay() {
        let mut relay = RocofRelay::new(1.0, 0.2); // trip after 0.2 s
        let meas = FrequencyMeasurement {
            freq_hz: 49.5,
            deviation_hz: -0.5,
            rocof_hz_per_s: 2.0, // exceeds 1.0 threshold
            nadir_detected: false,
            timestamp_s: 0.0,
        };
        // 25 steps × 0.02 s = 0.5 s > 0.2 s delay → must trip
        let mut action = RelayAction::NoAction;
        for _ in 0..25 {
            action = relay.evaluate(&meas, 0.02);
        }
        assert_eq!(action, RelayAction::Trip, "ROCOF above threshold → trip");
    }

    #[test]
    fn test_rocof_relay_reset_clears_accumulation() {
        let mut relay = RocofRelay::new(1.0, 0.1);
        let meas = FrequencyMeasurement {
            freq_hz: 49.5,
            deviation_hz: -0.5,
            rocof_hz_per_s: 2.5,
            nadir_detected: false,
            timestamp_s: 0.0,
        };
        // Trigger timer.
        for _ in 0..5 {
            relay.evaluate(&meas, 0.02);
        }
        relay.reset();
        assert!(
            relay.accumulated_time == 0.0,
            "reset should clear accumulated_time"
        );
    }

    // ── FrequencyRelay ────────────────────────────────────────────────────────

    #[test]
    fn test_frequency_relay_no_trip_at_nominal() {
        let mut relay = FrequencyRelay::new(49.0, 51.0, 0.3);
        let mut action = RelayAction::NoAction;
        for _ in 0..50 {
            action = relay.evaluate(50.0, 0.02);
        }
        assert_eq!(
            action,
            RelayAction::NoAction,
            "nominal frequency → no action"
        );
    }

    #[test]
    fn test_frequency_relay_trip_at_under_freq() {
        let mut relay = FrequencyRelay::new(49.0, 51.0, 0.2);
        // Frequency stays at 48.5 Hz (below 49.0 threshold)
        let mut action = RelayAction::NoAction;
        for _ in 0..20 {
            action = relay.evaluate(48.5, 0.02);
        }
        assert_eq!(action, RelayAction::Trip, "under-frequency → trip");
    }

    #[test]
    fn test_frequency_relay_trip_at_over_freq() {
        let mut relay = FrequencyRelay::new(49.0, 51.0, 0.2);
        let mut action = RelayAction::NoAction;
        for _ in 0..20 {
            action = relay.evaluate(51.5, 0.02);
        }
        assert_eq!(action, RelayAction::Trip, "over-frequency → trip");
    }

    // ── UflsScheme ────────────────────────────────────────────────────────────

    #[test]
    fn test_ufls_entso_e_has_six_steps() {
        let scheme = UflsScheme::new_entso_e(1000.0);
        assert_eq!(scheme.steps.len(), 6, "ENTSO-E scheme must have 6 steps");
    }

    #[test]
    fn test_ufls_nerc_has_five_steps() {
        let scheme = UflsScheme::new_nerc(1000.0);
        assert_eq!(scheme.steps.len(), 5, "NERC scheme must have 5 steps");
    }

    #[test]
    fn test_ufls_shed_fraction_at_49hz() {
        let mut scheme = UflsScheme::new_entso_e(1000.0);
        // Hold at 48.9 Hz (below 49.0 threshold, above 48.8 threshold).
        // After 0.2 s delay, step 1 should fire shedding 50 MW (5% of 1000).
        let mut total_shed = 0.0;
        for _ in 0..30 {
            total_shed += scheme.step(48.9, 0.02); // 30 × 0.02 = 0.6 s
        }
        assert!(
            (total_shed - 50.0).abs() < 1.0,
            "Expected ≈ 50 MW shed (step 1), got {total_shed:.1} MW"
        );
    }

    #[test]
    fn test_ufls_reset_clears_triggered() {
        let mut scheme = UflsScheme::new_entso_e(1000.0);
        // Trigger all steps by holding at very low frequency.
        for _ in 0..100 {
            scheme.step(47.0, 0.02);
        }
        let before = scheme.total_shed_mw();
        assert!(before > 0.0, "steps should have been triggered");

        scheme.reset();
        assert_eq!(scheme.total_shed_mw(), 0.0, "reset clears shed history");
        for s in &scheme.steps {
            assert!(!s.triggered, "reset clears triggered flags");
        }
    }

    // ── NadirEstimator ────────────────────────────────────────────────────────

    #[test]
    fn test_nadir_below_initial_for_generation_loss() {
        let estimator = NadirEstimator::new(5.0, 1000.0, 100.0, 8.0);
        let result = estimator.estimate_nadir(50.0, 100.0); // 100 MW loss
        assert!(
            result.nadir_freq_hz < 50.0,
            "nadir must be below initial frequency"
        );
    }

    #[test]
    fn test_nadir_larger_imbalance_lower_nadir() {
        let estimator = NadirEstimator::new(5.0, 1000.0, 100.0, 8.0);
        let small = estimator.estimate_nadir(50.0, 50.0);
        let large = estimator.estimate_nadir(50.0, 200.0);
        assert!(
            large.nadir_freq_hz < small.nadir_freq_hz,
            "larger disturbance → lower nadir"
        );
    }

    #[test]
    fn test_nadir_rocof_proportional_to_imbalance() {
        let estimator = NadirEstimator::new(5.0, 1000.0, 100.0, 8.0);
        let r1 = estimator.estimate_nadir(50.0, 100.0);
        let r2 = estimator.estimate_nadir(50.0, 200.0);
        assert!(
            r2.rocof_initial_hz_per_s.abs() > r1.rocof_initial_hz_per_s.abs(),
            "larger imbalance → larger initial ROCOF"
        );
    }

    // ── InertiaEstimator ──────────────────────────────────────────────────────

    #[test]
    fn test_inertia_estimator_physically_reasonable() {
        let mut est = InertiaEstimator::new();
        // Simulate a 100 MW loss: ROCOF ≈ -1 Hz/s, f ≈ 49.9 Hz
        for k in 0..20usize {
            est.add_measurement(FrequencyMeasurement {
                freq_hz: 49.9,
                deviation_hz: -0.1,
                rocof_hz_per_s: -1.0,
                nadir_detected: false,
                timestamp_s: k as f64 * 0.02,
            });
        }
        // H = -ΔP * f0 / (2 * S * ROCOF) = -100 * 49.9 / (2 * 1000 * -1) = 2.495 s
        let h = est
            .estimate_inertia(100.0, 1000.0)
            .expect("inertia estimate");
        assert!(
            h > 0.0 && h < 20.0,
            "inertia H should be in [0, 20] s, got {h:.3}"
        );
    }

    #[test]
    fn test_inertia_estimator_no_data_returns_err() {
        let est = InertiaEstimator::new();
        let result = est.estimate_inertia(100.0, 1000.0);
        assert!(result.is_err(), "no data should return Err");
    }

    #[test]
    fn test_inertia_estimator_window_subset() {
        let mut est = InertiaEstimator::new();
        for k in 0..50usize {
            let ts = k as f64 * 0.02;
            est.add_measurement(FrequencyMeasurement {
                freq_hz: 49.9,
                deviation_hz: -0.1,
                rocof_hz_per_s: if ts < 0.5 { -1.0 } else { -0.1 },
                nadir_detected: false,
                timestamp_s: ts,
            });
        }
        // Window [0, 0.4] contains only ROCOF = -1 Hz/s measurements.
        let h = est
            .estimate_from_event_window(0.0, 0.4, 100.0, 1000.0)
            .expect("window estimate");
        assert!(h > 0.0, "window H should be positive");
    }

    // ── FrequencyComplianceReport ─────────────────────────────────────────────

    #[test]
    fn test_compliance_all_normal_is_compliant() {
        let measurements: Vec<FrequencyMeasurement> = (0..100)
            .map(|k| FrequencyMeasurement {
                freq_hz: 50.0,
                deviation_hz: 0.0,
                rocof_hz_per_s: 0.0,
                nadir_detected: false,
                timestamp_s: k as f64 * 0.02,
            })
            .collect();
        let report = FrequencyComplianceReport::compute(&measurements);
        assert!(report.compliant, "all nominal → compliant");
        assert!((report.time_in_normal_band_pct - 100.0).abs() < 1e-9);
        assert_eq!(report.time_in_emergency_pct as u64, 0);
    }

    #[test]
    fn test_compliance_emergency_band_triggers_non_compliant() {
        // All samples in emergency band: |Δf| = 0.5 Hz >> 0.2 Hz.
        let measurements: Vec<FrequencyMeasurement> = (0..100)
            .map(|k| FrequencyMeasurement {
                freq_hz: 49.5,
                deviation_hz: -0.5,
                rocof_hz_per_s: 0.0,
                nadir_detected: false,
                timestamp_s: k as f64 * 0.02,
            })
            .collect();
        let report = FrequencyComplianceReport::compute(&measurements);
        assert!(!report.compliant, "emergency samples → non-compliant");
        assert!(report.time_in_emergency_pct > 0.0);
    }

    #[test]
    fn test_compliance_empty_measurements() {
        let report = FrequencyComplianceReport::compute(&[]);
        assert!(report.compliant, "empty → compliant by default");
        assert_eq!(report.nadir_count, 0);
        assert_eq!(report.ufls_activations, 0);
    }
}
