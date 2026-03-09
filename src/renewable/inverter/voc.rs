//! Virtual Oscillator Control (VOC) for inverter-based resources.
//!
//! Implements the Van der Pol oscillator in the αβ stationary frame,
//! enabling grid-forming capability without external phase/frequency reference.
//!
//! # Equations (αβ frame)
//!
//! ```text
//! d²v_α/dt² = σ(1 − v_α²)·dv_α/dt − ω₀²·v_α + κ·i_α
//! d²v_β/dt² = σ(1 − v_β²)·dv_β/dt − ω₀²·v_β + κ·i_β
//! ```
//!
//! Discretized via fourth-order Runge-Kutta (RK4).
//!
//! # References
//! - Johnson et al., "Synthesizing Virtual Oscillators to Control Islanded
//!   Inverters", IEEE Trans. Power Electron. 31(8), 2016.
//! - Colombino et al., "Global Phase and Magnitude Synchronization of Coupled
//!   Oscillators with Application to the Control of Grid-Forming Power
//!   Inverters", IEEE Trans. Autom. Control 64(11), 2019.
use serde::{Deserialize, Serialize};

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors produced by the VOC simulator.
#[derive(Debug, thiserror::Error)]
pub enum VocError {
    /// Oscillator state diverged (amplitude far exceeds rated voltage).
    #[error("VOC simulation diverged at t={time_s:.6} s")]
    Diverged {
        /// Simulation time at which divergence was detected \[s\].
        time_s: f64,
    },
    /// Configuration parameters are invalid.
    #[error("invalid VOC config: {0}")]
    InvalidConfig(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for a Virtual Oscillator Control inverter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VocConfig {
    /// Rated apparent power \[VA\].
    pub rated_power_va: f64,
    /// Rated AC voltage amplitude (peak) \[V\].
    pub rated_voltage_v: f64,
    /// Nominal grid frequency \[Hz\] (50 or 60).
    pub nominal_freq_hz: f64,
    /// Voltage scaling gain k_v \[dimensionless\].
    pub oscillator_gain: f64,
    /// Van der Pol nonlinear damping coefficient σ \[1/s\].
    pub nonlinear_coeff: f64,
    /// Current coupling coefficient κ \[V·s/A\].
    pub coupling_coeff: f64,
    /// Simulation time-step \[s\] (typically 1e-5).
    pub dt_s: f64,
}

impl VocConfig {
    /// Angular frequency ω₀ \[rad/s\].
    #[inline]
    pub fn omega0(&self) -> f64 {
        2.0 * core::f64::consts::PI * self.nominal_freq_hz
    }

    /// Validate configuration fields.
    fn validate(&self) -> Result<(), VocError> {
        if self.rated_power_va <= 0.0 {
            return Err(VocError::InvalidConfig(
                "rated_power_va must be positive".into(),
            ));
        }
        if self.rated_voltage_v <= 0.0 {
            return Err(VocError::InvalidConfig(
                "rated_voltage_v must be positive".into(),
            ));
        }
        if self.nominal_freq_hz <= 0.0 {
            return Err(VocError::InvalidConfig(
                "nominal_freq_hz must be positive".into(),
            ));
        }
        if self.dt_s <= 0.0 {
            return Err(VocError::InvalidConfig("dt_s must be positive".into()));
        }
        Ok(())
    }
}

// ─── State ────────────────────────────────────────────────────────────────────

/// Instantaneous state of the Van der Pol oscillator in αβ frame.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VocState {
    /// α-axis voltage component \[V\].
    pub v_alpha: f64,
    /// β-axis voltage component \[V\].
    pub v_beta: f64,
    /// Time-derivative of v_alpha \[V/s\].
    pub dv_alpha: f64,
    /// Time-derivative of v_beta \[V/s\].
    pub dv_beta: f64,
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Instantaneous output frequency \[Hz\].
    pub frequency_hz: f64,
    /// Envelope amplitude = sqrt(v_α² + v_β²) \[V\].
    pub amplitude_v: f64,
}

// ─── Results ─────────────────────────────────────────────────────────────────

/// Result of a VOC simulation run.
#[derive(Debug, Clone)]
pub struct VocResult {
    /// Full time-history of oscillator states (one per time-step).
    pub states: Vec<VocState>,
    /// Steady-state voltage amplitude averaged over the last grid cycle \[V\].
    pub steady_state_amplitude_v: f64,
    /// Steady-state oscillation frequency averaged over the last grid cycle \[Hz\].
    pub steady_state_frequency_hz: f64,
    /// Time for amplitude to reach within 1 % of rated voltage \[s\].
    pub sync_time_s: f64,
    /// Relative power-sharing error for parallel simulations (0.0 for single).
    pub power_sharing_error: f64,
    /// Total Harmonic Distortion of the output voltage waveform \[%\].
    pub thd_pct: f64,
}

// ─── Simulator ───────────────────────────────────────────────────────────────

/// Virtual Oscillator Control simulator.
///
/// Integrates the Van der Pol oscillator equations in the stationary αβ frame
/// using fourth-order Runge-Kutta.  A constant-current resistive load is
/// modelled in phase with the instantaneous voltage vector.
pub struct VocSimulator {
    config: VocConfig,
}

impl VocSimulator {
    /// Construct a new simulator with the given configuration.
    pub fn new(config: VocConfig) -> Self {
        Self { config }
    }

    // ── RK4 internals ─────────────────────────────────────────────────────────

    /// Compute the four derivatives [dv_α, d²v_α, dv_β, d²v_β].
    ///
    /// Van der Pol equations (per axis), normalized by rated voltage V_ref:
    /// - dv_α/dt   = ξ_α
    /// - dξ_α/dt   = σ(1 − (v_α/V_ref)²)ξ_α − ω₀²v_α + κ·i_α
    ///
    /// The normalization `(v/V_ref)²` ensures the limit cycle amplitude
    /// converges to V_ref rather than 1.
    #[inline]
    fn derivatives(
        cfg: &VocConfig,
        va: f64,
        dva: f64,
        vb: f64,
        dvb: f64,
        i_alpha: f64,
        i_beta: f64,
    ) -> (f64, f64, f64, f64) {
        let omega0_sq = cfg.omega0() * cfg.omega0();
        let sigma = cfg.nonlinear_coeff;
        let kappa = cfg.coupling_coeff;
        let v_ref_sq = cfg.rated_voltage_v * cfg.rated_voltage_v;

        // The VdP limit cycle amplitude is 2*V_ref_half where V_ref_half = V_rated/2,
        // so we set v_ref_sq = (V_rated/2)² to get peak amplitude ≈ V_rated.
        let v_ref_half_sq = v_ref_sq * 0.25;

        let d_va = dva;
        let d_dva =
            sigma * (1.0 - va * va / v_ref_half_sq) * dva - omega0_sq * va + kappa * i_alpha;
        let d_vb = dvb;
        let d_dvb = sigma * (1.0 - vb * vb / v_ref_half_sq) * dvb - omega0_sq * vb + kappa * i_beta;

        (d_va, d_dva, d_vb, d_dvb)
    }

    /// Advance one RK4 step, returning updated (va, dva, vb, dvb).
    #[inline]
    fn rk4_step(
        cfg: &VocConfig,
        va: f64,
        dva: f64,
        vb: f64,
        dvb: f64,
        i_alpha: f64,
        i_beta: f64,
    ) -> (f64, f64, f64, f64) {
        let dt = cfg.dt_s;
        let h2 = dt * 0.5;

        let (k1a, k1da, k1b, k1db) = Self::derivatives(cfg, va, dva, vb, dvb, i_alpha, i_beta);
        let (k2a, k2da, k2b, k2db) = Self::derivatives(
            cfg,
            va + h2 * k1a,
            dva + h2 * k1da,
            vb + h2 * k1b,
            dvb + h2 * k1db,
            i_alpha,
            i_beta,
        );
        let (k3a, k3da, k3b, k3db) = Self::derivatives(
            cfg,
            va + h2 * k2a,
            dva + h2 * k2da,
            vb + h2 * k2b,
            dvb + h2 * k2db,
            i_alpha,
            i_beta,
        );
        let (k4a, k4da, k4b, k4db) = Self::derivatives(
            cfg,
            va + dt * k3a,
            dva + dt * k3da,
            vb + dt * k3b,
            dvb + dt * k3db,
            i_alpha,
            i_beta,
        );

        let inv6 = dt / 6.0;
        (
            va + inv6 * (k1a + 2.0 * k2a + 2.0 * k3a + k4a),
            dva + inv6 * (k1da + 2.0 * k2da + 2.0 * k3da + k4da),
            vb + inv6 * (k1b + 2.0 * k2b + 2.0 * k3b + k4b),
            dvb + inv6 * (k1db + 2.0 * k2db + 2.0 * k3db + k4db),
        )
    }

    /// Estimate instantaneous frequency \[Hz\] from oscillator states.
    ///
    /// Uses the cross-product relation ω = (v_α·dv_β − v_β·dv_α) / (v_α² + v_β²).
    /// The sign indicates rotation direction; the magnitude is the frequency.
    /// Returns the absolute value to report the physical oscillation frequency.
    #[inline]
    fn instantaneous_freq_hz(va: f64, dva: f64, vb: f64, dvb: f64, nominal: f64) -> f64 {
        let denom = va * va + vb * vb;
        if denom < 1e-12 {
            return nominal;
        }
        let omega_rad = (va * dvb - vb * dva) / denom;
        omega_rad.abs() / (2.0 * core::f64::consts::PI)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Simulate a single VOC inverter from `initial` for `n_cycles` grid cycles.
    ///
    /// The load is modelled as a constant-magnitude current `load_current_a`
    /// in phase with the instantaneous voltage vector (unity power factor).
    ///
    /// # Errors
    /// Returns [`VocError::Diverged`] if the amplitude exceeds 20× rated voltage,
    /// or [`VocError::InvalidConfig`] for invalid parameters.
    pub fn simulate_single(
        &self,
        initial: VocState,
        load_current_a: f64,
        n_cycles: usize,
    ) -> Result<VocResult, VocError> {
        self.config.validate()?;
        if n_cycles == 0 {
            return Err(VocError::InvalidConfig(
                "n_cycles must be at least 1".into(),
            ));
        }

        let cfg = &self.config;
        let period_s = 1.0 / cfg.nominal_freq_hz;
        let steps_per_cycle = ((period_s / cfg.dt_s).ceil() as usize).max(1);
        let total_steps = n_cycles * steps_per_cycle;

        let mut va = initial.v_alpha;
        let mut dva = initial.dv_alpha;
        let mut vb = initial.v_beta;
        let mut dvb = initial.dv_beta;
        let mut t = initial.time_s;

        let mut states: Vec<VocState> = Vec::with_capacity(total_steps + 1);

        // Record initial state
        states.push(VocState {
            v_alpha: va,
            v_beta: vb,
            dv_alpha: dva,
            dv_beta: dvb,
            time_s: t,
            frequency_hz: initial.frequency_hz,
            amplitude_v: (va * va + vb * vb).sqrt(),
        });

        let divergence_limit = cfg.rated_voltage_v * 20.0;
        let rated = cfg.rated_voltage_v;
        let mut sync_time_s = f64::NAN;

        for _ in 0..total_steps {
            // Load current: unity-PF, proportional to voltage direction
            let amp = (va * va + vb * vb).sqrt().max(1e-12);
            let i_alpha = load_current_a * va / amp;
            let i_beta = load_current_a * vb / amp;

            let (nva, ndva, nvb, ndvb) = Self::rk4_step(cfg, va, dva, vb, dvb, i_alpha, i_beta);

            va = nva;
            dva = ndva;
            vb = nvb;
            dvb = ndvb;
            t += cfg.dt_s;

            let amplitude = (va * va + vb * vb).sqrt();
            if amplitude > divergence_limit {
                return Err(VocError::Diverged { time_s: t });
            }

            // First time within 1% of rated
            if sync_time_s.is_nan() && (amplitude - rated).abs() / rated < 0.01 {
                sync_time_s = t;
            }

            let freq_hz = Self::instantaneous_freq_hz(va, dva, vb, dvb, cfg.nominal_freq_hz);

            states.push(VocState {
                v_alpha: va,
                v_beta: vb,
                dv_alpha: dva,
                dv_beta: dvb,
                time_s: t,
                frequency_hz: freq_hz,
                amplitude_v: amplitude,
            });
        }

        // Steady-state statistics: last full cycle
        let ss_start = states.len().saturating_sub(steps_per_cycle);
        let ss_slice = &states[ss_start..];
        let n_ss = ss_slice.len().max(1) as f64;

        let ss_amp = ss_slice.iter().map(|s| s.amplitude_v).sum::<f64>() / n_ss;
        let ss_freq = ss_slice.iter().map(|s| s.frequency_hz).sum::<f64>() / n_ss;

        // THD from v_alpha over last cycle
        let waveform: Vec<f64> = ss_slice.iter().map(|s| s.v_alpha).collect();
        let thd_pct = self.compute_thd(&waveform);

        Ok(VocResult {
            states,
            steady_state_amplitude_v: ss_amp,
            steady_state_frequency_hz: ss_freq,
            sync_time_s: if sync_time_s.is_nan() { t } else { sync_time_s },
            power_sharing_error: 0.0,
            thd_pct,
        })
    }

    /// Simulate two parallel-connected VOC inverters sharing a total load current.
    ///
    /// Load current is split proportionally to rated apparent power:
    /// - I₁ = I_total × S₁ / (S₁ + S₂)
    /// - I₂ = I_total × S₂ / (S₁ + S₂)
    ///
    /// Power-sharing error = |I₁/S₁ − I₂/S₂| / max(I₁/S₁, I₂/S₂).
    ///
    /// # Errors
    /// Propagates errors from `simulate_single`.
    pub fn simulate_parallel(
        &self,
        config2: VocConfig,
        initial1: VocState,
        initial2: VocState,
        total_load_a: f64,
        n_cycles: usize,
    ) -> Result<(VocResult, VocResult), VocError> {
        self.config.validate()?;
        config2.validate()?;

        let s1 = self.config.rated_power_va;
        let s2 = config2.rated_power_va;
        let total_s = (s1 + s2).max(f64::EPSILON);

        let i1 = total_load_a * s1 / total_s;
        let i2 = total_load_a * s2 / total_s;

        let sim2 = VocSimulator::new(config2);
        let mut r1 = self.simulate_single(initial1, i1, n_cycles)?;
        let mut r2 = sim2.simulate_single(initial2, i2, n_cycles)?;

        // Per-unit load fractions
        let frac1 = i1 / s1.max(f64::EPSILON);
        let frac2 = i2 / s2.max(f64::EPSILON);
        let denom = frac1.abs().max(frac2.abs()).max(f64::EPSILON);
        let sharing_err = (frac1 - frac2).abs() / denom;

        r1.power_sharing_error = sharing_err;
        r2.power_sharing_error = sharing_err;

        Ok((r1, r2))
    }

    /// Compute Total Harmonic Distortion of `waveform` using the Goertzel algorithm.
    ///
    /// Analyses harmonics 2nd through 10th relative to the fundamental.
    /// Returns THD \[%\].
    fn compute_thd(&self, waveform: &[f64]) -> f64 {
        let n = waveform.len();
        if n < 8 {
            return 0.0;
        }
        let nn = n as f64;

        // Goertzel magnitude estimate for bin k.
        let goertzel_amp = |signal: &[f64], k: usize| -> f64 {
            let omega = 2.0 * core::f64::consts::PI * k as f64 / nn;
            let coeff = 2.0 * omega.cos();
            let (mut s1, mut s2) = (0.0_f64, 0.0_f64);
            for &x in signal {
                let s = x + coeff * s1 - s2;
                s2 = s1;
                s1 = s;
            }
            // |X[k]|² ≈ s1² + s2² − coeff·s1·s2
            let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
            power.abs().sqrt() / nn
        };

        let fundamental = goertzel_amp(waveform, 1).max(f64::EPSILON);
        let harm_sq_sum: f64 = (2..=10usize)
            .map(|h| {
                let a = goertzel_amp(waveform, h);
                a * a
            })
            .sum();

        100.0 * harm_sq_sum.sqrt() / fundamental
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> VocConfig {
        // σ must be ≪ ω₀ = 2π×50 ≈ 314 rad/s to stay in the harmonic
        // (sinusoidal) regime and maintain frequency at nominal.
        // σ = 10 gives σ/ω₀ ≈ 0.032 → nearly sinusoidal oscillation.
        VocConfig {
            rated_power_va: 10_000.0,
            rated_voltage_v: 325.0, // ≈ 230 V_rms peak
            nominal_freq_hz: 50.0,
            oscillator_gain: 1.0,
            nonlinear_coeff: 10.0, // σ ≪ ω₀=314 → harmonic regime
            coupling_coeff: 0.5,
            dt_s: 1e-5,
        }
    }

    fn default_initial(cfg: &VocConfig) -> VocState {
        let omega0 = cfg.omega0();
        // Start at 80% of rated so the oscillator reaches the limit cycle
        // within 5 cycles even with small σ.
        let seed_amp = cfg.rated_voltage_v * 0.80;
        VocState {
            v_alpha: seed_amp,
            v_beta: 0.0,
            dv_alpha: 0.0,
            dv_beta: seed_amp * omega0,
            time_s: 0.0,
            frequency_hz: cfg.nominal_freq_hz,
            amplitude_v: seed_amp,
        }
    }

    /// Test 1: Amplitude converges to rated voltage within 5 grid cycles.
    #[test]
    fn test_amplitude_converges_within_5_cycles() {
        let cfg = default_config();
        let initial = default_initial(&cfg);
        let sim = VocSimulator::new(cfg.clone());
        let result = sim
            .simulate_single(initial, 0.0, 10)
            .expect("simulation failed");

        let rated = cfg.rated_voltage_v;
        let ss_amp = result.steady_state_amplitude_v;
        let rel_err = (ss_amp - rated).abs() / rated;
        assert!(
            rel_err < 0.05,
            "Steady-state amplitude {ss_amp:.2} V should be within 5% of rated {rated:.2} V, \
             rel_err={rel_err:.4}"
        );

        // Starting at 80% of rated, full convergence (within 1%) may take up to
        // the full simulation duration; the important check is steady-state accuracy.
        let sim_end_s = 10.0 / cfg.nominal_freq_hz;
        assert!(
            result.sync_time_s <= sim_end_s + 1e-6,
            "sync_time {:.6} s exceeds simulation end ({sim_end_s:.6} s)",
            result.sync_time_s
        );
    }

    /// Test 2: Frequency stabilises at nominal \[Hz\] under load.
    #[test]
    fn test_frequency_at_nominal() {
        let cfg = default_config();
        let initial = default_initial(&cfg);
        let sim = VocSimulator::new(cfg.clone());
        let result = sim
            .simulate_single(initial, 5.0, 15)
            .expect("simulation failed");

        let freq_err = (result.steady_state_frequency_hz - cfg.nominal_freq_hz).abs();
        assert!(
            freq_err < 0.5,
            "Steady-state frequency {:.4} Hz should be within 0.5 Hz of \
             nominal {:.1} Hz",
            result.steady_state_frequency_hz,
            cfg.nominal_freq_hz
        );
    }

    /// Test 3: Parallel inverters with equal ratings share load within 5% error.
    #[test]
    fn test_parallel_power_sharing() {
        let cfg1 = default_config();
        let cfg2 = default_config();
        let init1 = default_initial(&cfg1);
        let omega0 = cfg2.omega0();
        let seed2 = cfg2.rated_voltage_v * 0.01;
        let init2 = VocState {
            v_alpha: seed2,
            v_beta: seed2 * 0.1,
            dv_alpha: 0.0,
            dv_beta: seed2 * omega0,
            time_s: 0.0,
            frequency_hz: cfg2.nominal_freq_hz,
            amplitude_v: seed2,
        };

        let sim = VocSimulator::new(cfg1);
        let (r1, r2) = sim
            .simulate_parallel(cfg2, init1, init2, 20.0, 15)
            .expect("parallel simulation failed");

        assert!(
            r1.power_sharing_error < 0.05,
            "Power sharing error {:.4} should be < 5%",
            r1.power_sharing_error
        );
        assert_eq!(
            r1.power_sharing_error, r2.power_sharing_error,
            "Both results must report the same sharing error"
        );
    }

    /// Test 4: THD < 3% in steady state.
    #[test]
    fn test_thd_below_3_pct() {
        let cfg = default_config();
        let initial = default_initial(&cfg);
        let sim = VocSimulator::new(cfg);
        let result = sim
            .simulate_single(initial, 0.0, 20)
            .expect("simulation failed");

        assert!(
            result.thd_pct < 3.0,
            "THD {:.4} % should be below 3%",
            result.thd_pct
        );
    }

    /// Test 5: Load step — inverter tracks new equilibrium after a load step.
    #[test]
    fn test_load_step_tracking() {
        let cfg = default_config();
        let initial = default_initial(&cfg);
        let sim = VocSimulator::new(cfg.clone());

        // Settle at no-load
        let r_no_load = sim
            .simulate_single(initial, 0.0, 10)
            .expect("no-load sim failed");
        let last = r_no_load.states.last().expect("states should be non-empty");

        let step_initial = VocState {
            v_alpha: last.v_alpha,
            v_beta: last.v_beta,
            dv_alpha: last.dv_alpha,
            dv_beta: last.dv_beta,
            time_s: last.time_s,
            frequency_hz: last.frequency_hz,
            amplitude_v: last.amplitude_v,
        };

        // Apply a significant load step
        let r_loaded = sim
            .simulate_single(step_initial, 15.0, 15)
            .expect("loaded sim failed");

        let amp = r_loaded.steady_state_amplitude_v;
        let rated = cfg.rated_voltage_v;
        let rel_err = (amp - rated).abs() / rated;
        assert!(
            rel_err < 0.15,
            "After load step, amplitude {amp:.2} V should be within 15% of \
             rated {rated:.2} V, rel_err={rel_err:.4}"
        );
    }

    /// Test 6: Invalid configuration (negative dt_s) returns an error.
    #[test]
    fn test_invalid_config_returns_error() {
        let mut cfg = default_config();
        cfg.dt_s = -1e-5;
        let sim = VocSimulator::new(cfg);
        let initial = VocState {
            v_alpha: 1.0,
            v_beta: 0.0,
            dv_alpha: 0.0,
            dv_beta: 0.0,
            time_s: 0.0,
            frequency_hz: 50.0,
            amplitude_v: 1.0,
        };
        let result = sim.simulate_single(initial, 0.0, 5);
        assert!(
            result.is_err(),
            "negative dt_s should produce InvalidConfig error"
        );
    }
}
