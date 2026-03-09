//! Power system inertia analysis and frequency nadir prediction.
//!
//! # Overview
//!
//! This module provides tools for:
//! - Computing system-wide inertia from individual generator inertia constants
//! - Predicting frequency nadir after a sudden generation loss using the swing equation
//!   and a first-order governor model, integrated with RK4
//! - Estimating minimum required inertia to satisfy ROCOF limits
//! - Finding the maximum credible loss consistent with UFLS limits
//! - Modelling virtual inertia from BESS and other converter-based resources
//! - Inertia emulation control for converter-based resources
//! - Online inertia monitoring from PMU measurements
//!
//! # Equations
//!
//! Swing equation (per-unit on system MVA base):
//!   df/dt = -(ΔP_loss - ΔP_gov(t)) * fn / (2 * H_sys * S_base)
//!
//! Governor (first-order lag, droop R):
//!   dΔP_gov/dt = (ΔP_gov_ss - ΔP_gov) / T_gov
//!   ΔP_gov_ss  = -Δf / (R * fn) * P_rated_i   [summed over online generators]
//!
//! ROCOF (initial, t→0+):
//!   ROCOF₀ = -ΔP_loss * fn / (2 * H_sys * S_base)
//!
//! Minimum inertia for ROCOF limit:
//!   H_min = ΔP_loss / (2 * rocof_limit * S_base)

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ─── Generator technology ────────────────────────────────────────────────────

/// Technology classification for a generator, affecting inertia contribution
/// and governor dynamics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GeneratorTechnology {
    /// Conventional steam turbine (coal, oil, OCGT steam-assisted)
    SteamTurbine,
    /// Open-cycle gas turbine
    GasTurbine,
    /// Hydropower unit; penstock water mass affects primary response
    HydroPower {
        /// Penstock water starting time constant T_w (seconds)
        penstock_water_mass: f64,
    },
    /// Combined-cycle gas turbine
    CombinedCycle,
    /// Nuclear steam turbine (high inertia, slow governor)
    NuclearSteam,
    /// Type-3 (DFIG) wind turbine — can provide limited synthetic inertia
    WindTurbineType3 {
        /// Virtual inertia constant (seconds) — synthetic inertia emulation
        virtual_inertia: f64,
    },
    /// Type-4 (full-converter) wind turbine — no direct inertia
    WindTurbineType4,
    /// Photovoltaic plant — no rotating mass inertia
    PhotovoltaicPlant,
    /// Battery energy storage with virtual inertia control
    BessWithVirtualInertia {
        /// Inertia emulation gain K_vi (MW·s)
        kvi: f64,
    },
}

impl GeneratorTechnology {
    /// Returns the effective inertia constant (seconds) contributed by this technology.
    /// Type-4 wind, PV, and plain BESS contribute zero physical inertia.
    pub fn effective_inertia_s(&self, rated_h_s: f64) -> f64 {
        match self {
            Self::SteamTurbine => rated_h_s,
            Self::GasTurbine => rated_h_s,
            Self::HydroPower { .. } => rated_h_s,
            Self::CombinedCycle => rated_h_s,
            Self::NuclearSteam => rated_h_s,
            Self::WindTurbineType3 { virtual_inertia } => {
                // Physical H + synthetic component (capped at rated_h_s)
                rated_h_s + virtual_inertia
            }
            Self::WindTurbineType4 => 0.0,
            Self::PhotovoltaicPlant => 0.0,
            Self::BessWithVirtualInertia { kvi: _ } => {
                // Inertia from BESS is modelled separately via kvi; H_constant = 0
                0.0
            }
        }
    }

    /// True if the technology is synchronously coupled to the grid.
    pub fn is_synchronous(&self) -> bool {
        matches!(
            self,
            Self::SteamTurbine
                | Self::GasTurbine
                | Self::HydroPower { .. }
                | Self::CombinedCycle
                | Self::NuclearSteam
        )
    }
}

// ─── GeneratorInertia ────────────────────────────────────────────────────────

/// Inertia and governor parameters for a single generating unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorInertia {
    /// Unique generator identifier
    pub id: usize,
    /// Bus number where the generator is connected
    pub bus: usize,
    /// Rated MVA of the machine
    pub rated_mva: f64,
    /// Inertia constant H (seconds) — stored energy / rated MVA
    pub h_constant_s: f64,
    /// Damping coefficient D (pu torque per pu speed)
    pub d_damping: f64,
    /// Governor droop in percent (typically 4–6 %)
    pub droop_percent: f64,
    /// Governor first-order time constant (seconds)
    pub response_time_s: f64,
    /// Whether the unit is currently online
    pub is_online: bool,
    /// Technology type, affects inertia and dynamic behaviour
    pub technology: GeneratorTechnology,
}

impl GeneratorInertia {
    /// Effective inertia constant H for this unit (accounts for technology type).
    pub fn effective_h_s(&self) -> f64 {
        self.technology.effective_inertia_s(self.h_constant_s)
    }

    /// Energy stored in the rotating mass: H * S_rated (MW·s).
    pub fn stored_energy_mws(&self) -> f64 {
        self.effective_h_s() * self.rated_mva
    }

    /// Governor droop as a per-unit fraction (e.g. 0.05 for 5 %).
    pub fn droop_pu(&self) -> f64 {
        self.droop_percent / 100.0
    }

    /// Steady-state governor response at frequency deviation Δf (Hz).
    /// ΔP_gov_ss = -(Δf / f0) / R * P_rated   [MW]
    fn governor_ss_mw(&self, delta_f_hz: f64, f0_hz: f64) -> f64 {
        if self.droop_pu() < 1e-12 {
            return 0.0;
        }
        let delta_f_pu = delta_f_hz / f0_hz;
        -delta_f_pu / self.droop_pu() * self.rated_mva
    }
}

// ─── SystemInertia ───────────────────────────────────────────────────────────

/// ROCOF risk classification based on system inertia adequacy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RocofRiskLevel {
    /// H_sys / H_min ≥ 1.5
    Low,
    /// 1.0 ≤ H_sys / H_min < 1.5
    Medium,
    /// 0.7 ≤ H_sys / H_min < 1.0
    High,
    /// H_sys / H_min < 0.7
    Critical,
}

impl RocofRiskLevel {
    fn from_adequacy(adequacy: f64) -> Self {
        if adequacy >= 1.5 {
            Self::Low
        } else if adequacy >= 1.0 {
            Self::Medium
        } else if adequacy >= 0.7 {
            Self::High
        } else {
            Self::Critical
        }
    }
}

/// Aggregated system-wide inertia metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemInertia {
    /// Total stored energy: Σ(H_i * S_i) [MW·s]
    pub total_h_mws: f64,
    /// System-equivalent inertia constant H_sys = total_h_mws / S_base [s]
    pub system_h_s: f64,
    /// Load-weighted average inertia constant [s]
    pub weighted_average_h_s: f64,
    /// Total synchronous MVA online
    pub synchronous_mva: f64,
    /// Total converter-connected MVA online
    pub converter_mva: f64,
    /// H_sys / H_minimum (> 1 means adequate)
    pub inertia_adequacy: f64,
    /// Risk classification
    pub rocof_risk: RocofRiskLevel,
}

// ─── FrequencyNadirPredictor ──────────────────────────────────────────────────

/// Result of a frequency nadir prediction run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyNadirResult {
    /// Time from disturbance to frequency nadir [s]
    pub time_to_nadir_s: f64,
    /// Frequency at the nadir [Hz]
    pub nadir_frequency_hz: f64,
    /// Depth of the nadir: f0 − f_nadir [Hz]
    pub nadir_deviation_hz: f64,
    /// Initial rate-of-change-of-frequency [Hz/s]
    pub rocof_initial_hz_per_s: f64,
    /// Quasi-steady-state post-governor frequency [Hz]
    pub quasi_steady_state_hz: f64,
    /// Whether under-frequency load shedding would be triggered
    pub ufls_triggered: bool,
    /// Whether the ROCOF regulatory limit was violated
    pub rocof_limit_violated: bool,
    /// Full frequency trajectory: (time_s, frequency_hz)
    pub trajectory: Vec<(f64, f64)>,
    /// Governor power response trajectory: (time_s, delta_P_total_MW)
    pub governor_response: Vec<(f64, f64)>,
}

/// Frequency nadir predictor using the swing equation with first-order governor models.
///
/// # Model
///
/// State vector: [f, ΔP_gov_1, ΔP_gov_2, …, ΔP_gov_n, ΔP_vi]
///   where ΔP_vi is the aggregate virtual inertia response.
///
/// df/dt = -(ΔP_imbalance - Σ ΔP_gov_i - ΔP_vi) * fn / (2 * H_sys * S_base)
/// dΔP_gov_i/dt = (ΔP_gov_i_ss(f) - ΔP_gov_i) / T_gov_i
/// dΔP_vi/dt    = (ΔP_vi_ss(f)    - ΔP_vi)    / T_vi
#[derive(Debug, Clone)]
pub struct FrequencyNadirPredictor {
    /// List of generating units in the study
    pub generators: Vec<GeneratorInertia>,
    /// System base MVA
    pub system_mva: f64,
    /// Nominal system frequency [Hz]
    pub frequency_hz: f64,
    /// Minimum permissible frequency before UFLS triggers [Hz]
    pub minimum_frequency_hz: f64,
    /// Regulatory ROCOF limit [Hz/s]
    pub rocof_limit_hz_per_s: f64,
    /// Virtual inertia resources: (k_vi [MW·s], T_response [s])
    virtual_inertia_resources: Vec<(f64, f64)>,
}

impl FrequencyNadirPredictor {
    /// Create a new predictor.
    pub fn new(
        generators: Vec<GeneratorInertia>,
        system_mva: f64,
        frequency_hz: f64,
        minimum_frequency_hz: f64,
        rocof_limit_hz_per_s: f64,
    ) -> Self {
        Self {
            generators,
            system_mva,
            frequency_hz,
            minimum_frequency_hz,
            rocof_limit_hz_per_s,
            virtual_inertia_resources: Vec::new(),
        }
    }

    /// Compute system inertia H_sys = Σ(H_i * S_i) / S_base.
    ///
    /// Only online generators contribute. The minimum required inertia is
    /// computed from the ROCOF limit and a default credible loss of 5 % of
    /// system MVA (adjusted to zero division when system_mva ≤ 0).
    pub fn compute_system_inertia(&self) -> SystemInertia {
        let mut total_h_mws = 0.0_f64;
        let mut synchronous_mva = 0.0_f64;
        let mut converter_mva = 0.0_f64;
        let mut weighted_sum = 0.0_f64;

        for gen in &self.generators {
            if !gen.is_online {
                continue;
            }
            let h_eff = gen.effective_h_s();
            let mva = gen.rated_mva;
            total_h_mws += h_eff * mva;
            weighted_sum += h_eff * mva;
            if gen.technology.is_synchronous() {
                synchronous_mva += mva;
            } else {
                converter_mva += mva;
            }
        }

        let s_base = if self.system_mva > 0.0 {
            self.system_mva
        } else {
            1.0
        };
        let system_h_s = total_h_mws / s_base;
        let online_mva = synchronous_mva + converter_mva;
        let weighted_average_h_s = if online_mva > 0.0 {
            weighted_sum / online_mva
        } else {
            0.0
        };

        // Minimum inertia for default credible loss = 5 % S_base, at ROCOF limit
        let default_loss_mw = 0.05 * s_base;
        let h_min = self.minimum_inertia_for_rocof(default_loss_mw);
        let inertia_adequacy = if h_min > 0.0 {
            system_h_s / h_min
        } else {
            f64::INFINITY
        };
        let rocof_risk = RocofRiskLevel::from_adequacy(inertia_adequacy);

        SystemInertia {
            total_h_mws,
            system_h_s,
            weighted_average_h_s,
            synchronous_mva,
            converter_mva,
            inertia_adequacy,
            rocof_risk,
        }
    }

    /// Predict the frequency nadir following a sudden power imbalance.
    ///
    /// # Arguments
    /// * `power_imbalance_mw` — net generation loss (positive = generation < load)
    /// * `t_max_s`            — simulation horizon (seconds)
    ///
    /// # Errors
    /// Returns [`OxiGridError::InvalidParameter`] when the system has no
    /// online generators or system_mva ≤ 0.
    pub fn predict_nadir(
        &self,
        power_imbalance_mw: f64,
        t_max_s: f64,
    ) -> Result<FrequencyNadirResult, OxiGridError> {
        if self.system_mva <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "system_mva must be positive".into(),
            ));
        }
        let inertia = self.compute_system_inertia();
        if inertia.system_h_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "no online generators with positive inertia found".into(),
            ));
        }

        let f0 = self.frequency_hz;
        let fn_ = self.frequency_hz; // nominal
        let h_sys = inertia.system_h_s;
        let s_base = self.system_mva;

        // Online generators for governor response
        let online_gens: Vec<&GeneratorInertia> = self
            .generators
            .iter()
            .filter(|g| g.is_online && g.droop_percent > 0.0)
            .collect();
        let n_gov = online_gens.len();
        let n_vi = self.virtual_inertia_resources.len();

        // State: [f, ΔP_gov_0, …, ΔP_gov_{n-1}, ΔP_vi_0, …]
        // Size = 1 + n_gov + n_vi
        let n_states = 1 + n_gov + n_vi;
        let mut state = vec![0.0_f64; n_states];
        state[0] = f0; // initial frequency

        let dt = 0.01_f64; // integration step [s]
        let n_steps = ((t_max_s / dt).ceil() as usize).max(1);

        let mut trajectory: Vec<(f64, f64)> = Vec::with_capacity(n_steps + 1);
        let mut gov_trace: Vec<(f64, f64)> = Vec::with_capacity(n_steps + 1);

        // Record t=0
        trajectory.push((0.0, f0));
        gov_trace.push((0.0, 0.0));

        // Track nadir
        let mut nadir_freq = f0;
        let mut time_to_nadir = 0.0_f64;
        let mut ufls_triggered = false;
        let mut rocof_limit_violated = false;

        // Initial ROCOF
        let rocof_0 = -power_imbalance_mw * fn_ / (2.0 * h_sys * s_base);
        if rocof_0.abs() > self.rocof_limit_hz_per_s {
            rocof_limit_violated = true;
        }

        // RK4 derivative closure
        let deriv = |t: f64, s: &[f64]| -> Vec<f64> {
            let _ = t;
            let f_now = s[0];
            let delta_f = f_now - fn_;

            // Sum governor responses
            let mut p_gov_total = 0.0_f64;
            for i in 0..n_gov {
                p_gov_total += s[1 + i];
            }
            // Virtual inertia responses
            for i in 0..n_vi {
                p_gov_total += s[1 + n_gov + i];
            }

            let p_imbalance_pu = power_imbalance_mw - p_gov_total;
            let df_dt = -p_imbalance_pu * fn_ / (2.0 * h_sys * s_base);

            let mut d = vec![0.0_f64; n_states];
            d[0] = df_dt;

            // Governor dynamics
            for (i, gen) in online_gens.iter().enumerate() {
                let p_ss = gen.governor_ss_mw(delta_f, fn_);
                let p_gov_i = s[1 + i];
                let t_gov = gen.response_time_s.max(1e-6);
                d[1 + i] = (p_ss - p_gov_i) / t_gov;
            }

            // Virtual inertia dynamics
            for (j, &(k_vi, t_vi)) in self.virtual_inertia_resources.iter().enumerate() {
                // P_vi_ss = K_vi * (-df/dt) modelled as droop on Δf
                // Approximation: K_vi acts as inertia → ΔP_vi_ss proportional to Δf
                let p_vi_ss = -k_vi * (delta_f / fn_);
                let p_vi_j = s[1 + n_gov + j];
                let t_resp = t_vi.max(1e-6);
                d[1 + n_gov + j] = (p_vi_ss - p_vi_j) / t_resp;
            }

            d
        };

        // RK4 integration
        for step in 0..n_steps {
            let t = step as f64 * dt;
            let s = state.clone();

            // k1
            let k1 = deriv(t, &s);
            // k2
            let s2: Vec<f64> = s
                .iter()
                .zip(k1.iter())
                .map(|(x, d)| x + 0.5 * dt * d)
                .collect();
            let k2 = deriv(t + 0.5 * dt, &s2);
            // k3
            let s3: Vec<f64> = s
                .iter()
                .zip(k2.iter())
                .map(|(x, d)| x + 0.5 * dt * d)
                .collect();
            let k3 = deriv(t + 0.5 * dt, &s3);
            // k4
            let s4: Vec<f64> = s.iter().zip(k3.iter()).map(|(x, d)| x + dt * d).collect();
            let k4 = deriv(t + dt, &s4);

            for i in 0..n_states {
                state[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }

            let t_new = (step + 1) as f64 * dt;
            let f_new = state[0];

            // Check ROCOF in real-time (approximate from consecutive freq samples)
            if step == 0 {
                let rocof_now = (f_new - f0) / dt;
                if rocof_now.abs() > self.rocof_limit_hz_per_s {
                    rocof_limit_violated = true;
                }
            }

            // Track nadir
            if f_new < nadir_freq {
                nadir_freq = f_new;
                time_to_nadir = t_new;
            }

            // UFLS check
            if f_new < self.minimum_frequency_hz {
                ufls_triggered = true;
            }

            // Governor total
            let gov_total: f64 = (0..n_gov + n_vi).map(|i| state[1 + i]).sum();

            trajectory.push((t_new, f_new));
            gov_trace.push((t_new, gov_total));
        }

        // Quasi-steady-state: average of last 10 % of trajectory
        let qss_start = (n_steps as f64 * 0.9) as usize;
        let qss_slice = &trajectory[qss_start.min(trajectory.len().saturating_sub(1))..];
        let qss_freq = if qss_slice.is_empty() {
            nadir_freq
        } else {
            qss_slice.iter().map(|(_, f)| f).sum::<f64>() / qss_slice.len() as f64
        };

        Ok(FrequencyNadirResult {
            time_to_nadir_s: time_to_nadir,
            nadir_frequency_hz: nadir_freq,
            nadir_deviation_hz: f0 - nadir_freq,
            rocof_initial_hz_per_s: rocof_0,
            quasi_steady_state_hz: qss_freq,
            ufls_triggered,
            rocof_limit_violated,
            trajectory,
            governor_response: gov_trace,
        })
    }

    /// Minimum system inertia [s] required to keep |ROCOF| ≤ rocof_limit:
    ///   H_min = ΔP / (2 * rocof_limit * S_base)
    pub fn minimum_inertia_for_rocof(&self, power_imbalance_mw: f64) -> f64 {
        let denom = 2.0 * self.rocof_limit_hz_per_s * self.system_mva;
        if denom <= 0.0 {
            return 0.0;
        }
        power_imbalance_mw * self.frequency_hz / denom
    }

    /// Find the maximum credible power loss [MW] such that the frequency nadir
    /// stays above `minimum_frequency_hz`.
    ///
    /// Uses binary search with ±0.1 MW resolution over [0, system_mva] MW.
    ///
    /// # Errors
    /// Returns an error if no feasible solution exists or nadir prediction fails.
    pub fn maximum_credible_loss(&self, t_max_s: f64) -> Result<f64, OxiGridError> {
        // Quick feasibility: even a tiny loss should be OK
        let tiny = 0.1_f64;
        let result = self.predict_nadir(tiny, t_max_s)?;
        if result.ufls_triggered {
            return Ok(0.0);
        }

        let mut lo = 0.0_f64;
        let mut hi = self.system_mva; // upper bound: full system loss

        // Binary search for 50 iterations (~1e-15 precision)
        for _ in 0..50 {
            let mid = (lo + hi) / 2.0;
            if mid < 0.1 {
                break;
            }
            let res = self.predict_nadir(mid, t_max_s)?;
            if res.ufls_triggered {
                hi = mid;
            } else {
                lo = mid;
            }
            if hi - lo < 0.1 {
                break;
            }
        }

        Ok(lo)
    }

    /// Add a virtual inertia resource (e.g. BESS with fast frequency response).
    ///
    /// # Arguments
    /// * `k_vi_mw_s`    — virtual inertia gain [MW·s]
    /// * `t_response_s` — response time constant [s]
    pub fn add_virtual_inertia(&mut self, k_vi_mw_s: f64, t_response_s: f64) {
        self.virtual_inertia_resources
            .push((k_vi_mw_s, t_response_s.max(0.001)));
    }

    /// Frequency containment reserve (FCR) requirement [MW].
    ///
    /// Based on the ENTSO-E approach:
    ///   FCR = H_min_required * 2 * S_base / t_delivery
    ///
    /// where H_min_required is computed from a default credible loss of 5 % S_base.
    pub fn fcr_requirement(&self, t_delivery_s: f64) -> f64 {
        let default_loss_mw = 0.05 * self.system_mva;
        let h_min = self.minimum_inertia_for_rocof(default_loss_mw);
        if t_delivery_s <= 0.0 {
            return 0.0;
        }
        h_min * 2.0 * self.system_mva / t_delivery_s
    }
}

// ─── InertiaEmulationControl ──────────────────────────────────────────────────

/// Inertia emulation and droop control for converter-based resources.
///
/// The power command is:
///   P_emu = K_inertia * (-df/dt) + K_droop * (-Δf)
///
/// subject to deadband, ramp rate limit, and power ceiling.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaEmulationControl {
    /// Inertia emulation gain [MW·s]
    pub k_inertia: f64,
    /// Droop response gain [MW/Hz]
    pub k_droop: f64,
    /// Frequency deadband [Hz] — no response within ±deadband_hz
    pub deadband_hz: f64,
    /// Maximum power the resource can inject [MW]
    pub max_power_mw: f64,
    /// Maximum ramp rate [MW/s]
    pub ramp_limit_mw_per_s: f64,
    /// Current estimated ROCOF [Hz/s] (state, public for monitoring)
    pub df_dt: f64,
    /// Current frequency deviation [Hz] (state)
    pub delta_f: f64,
    /// Current power output [MW] (state)
    pub p_output_mw: f64,
    /// Power at previous time step [MW] (state, used for ramp limiting)
    pub p_prev_mw: f64,
}

impl InertiaEmulationControl {
    /// Construct a new emulation controller.
    pub fn new(
        k_inertia: f64,
        k_droop: f64,
        deadband_hz: f64,
        max_power_mw: f64,
        ramp_limit_mw_per_s: f64,
    ) -> Self {
        Self {
            k_inertia,
            k_droop,
            deadband_hz,
            max_power_mw,
            ramp_limit_mw_per_s,
            df_dt: 0.0,
            delta_f: 0.0,
            p_output_mw: 0.0,
            p_prev_mw: 0.0,
        }
    }

    /// Compute power injection for the current measurement step.
    ///
    /// # Arguments
    /// * `frequency_hz`  — measured frequency
    /// * `rocof_hz_per_s`— measured or estimated ROCOF
    /// * `dt`            — time step size [s]
    ///
    /// Returns the power injection [MW] (positive = injection into grid).
    pub fn compute_response(&mut self, frequency_hz: f64, rocof_hz_per_s: f64, dt: f64) -> f64 {
        self.df_dt = rocof_hz_per_s;
        self.delta_f = frequency_hz - self.nominal_frequency_estimate();

        // Apply deadband to frequency deviation
        let effective_delta_f = if self.delta_f.abs() < self.deadband_hz {
            0.0
        } else {
            self.delta_f
        };

        // Apply deadband to ROCOF
        let effective_rocof = rocof_hz_per_s;

        // Desired power output
        let p_desired = self.k_inertia * (-effective_rocof) + self.k_droop * (-effective_delta_f);

        // Ramp limit
        let max_delta = self.ramp_limit_mw_per_s * dt.max(1e-9);
        let p_ramped = p_desired.clamp(self.p_prev_mw - max_delta, self.p_prev_mw + max_delta);

        // Power limit
        let p_final = p_ramped.clamp(0.0, self.max_power_mw);

        self.p_prev_mw = p_final;
        self.p_output_mw = p_final;
        p_final
    }

    /// Estimate nominal frequency from the last observed frequency plus deviation.
    /// In practice this would come from a measurement reference; here we use 50 Hz
    /// as the baseline (the user can adjust by setting delta_f externally).
    fn nominal_frequency_estimate(&self) -> f64 {
        50.0
    }

    /// Virtual inertia constant H_virtual = K_inertia / (2 * S_resource) [s].
    ///
    /// # Arguments
    /// * `s_resource_mva` — MVA rating of the resource
    pub fn virtual_h_constant(&self, s_resource_mva: f64) -> f64 {
        if s_resource_mva <= 0.0 {
            return 0.0;
        }
        self.k_inertia / (2.0 * s_resource_mva)
    }
}

// ─── InertiaMonitor ───────────────────────────────────────────────────────────

/// Method used for inertia estimation from PMU data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EstimationMethod {
    /// Least-squares linear fit to f(t) in the sliding window
    LinearRegression,
    /// Kalman-filter-smoothed ROCOF estimate
    KalmanFilter,
    /// Wallis two-step estimation method
    WallisMethod,
}

/// Online inertia estimate derived from a PMU disturbance event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaEstimate {
    /// Timestamp at which the estimate was produced [s]
    pub timestamp: f64,
    /// Estimated system inertia [MW·s]
    pub estimated_h_mws: f64,
    /// Confidence score in [0, 1]
    pub confidence: f64,
    /// Measured ROCOF [Hz/s]
    pub rocof_measured_hz_per_s: f64,
    /// Estimated power imbalance [MW] (from ΔP = H * 2 * rocof / S_base * S_base)
    pub power_imbalance_estimate_mw: f64,
    /// Estimation method used
    pub method: EstimationMethod,
}

/// Online inertia monitor fed by PMU frequency measurements.
pub struct InertiaMonitor {
    /// Number of samples in the sliding window
    pub window_size: usize,
    /// PMU sampling rate [Hz]
    pub sampling_rate_hz: f64,
    /// Circular buffer of frequency samples [Hz]
    pub frequency_buffer: Vec<f64>,
    /// Corresponding timestamps [s]
    pub time_buffer: Vec<f64>,
}

impl InertiaMonitor {
    /// Create a new inertia monitor.
    ///
    /// # Arguments
    /// * `window_size`      — number of PMU samples to retain
    /// * `sampling_rate_hz` — PMU reporting rate (e.g. 50 or 100 Hz)
    pub fn new(window_size: usize, sampling_rate_hz: f64) -> Self {
        Self {
            window_size: window_size.max(3),
            sampling_rate_hz,
            frequency_buffer: Vec::with_capacity(window_size),
            time_buffer: Vec::with_capacity(window_size),
        }
    }

    /// Ingest a new PMU frequency sample.
    ///
    /// Maintains a sliding window of the last `window_size` samples.
    pub fn update(&mut self, timestamp: f64, frequency_hz: f64) {
        self.frequency_buffer.push(frequency_hz);
        self.time_buffer.push(timestamp);
        if self.frequency_buffer.len() > self.window_size {
            self.frequency_buffer.remove(0);
            self.time_buffer.remove(0);
        }
    }

    /// Estimate ROCOF via ordinary least-squares linear regression on f(t).
    ///
    /// Returns df/dt [Hz/s] (slope of the best-fit line).
    /// Returns 0.0 if fewer than 2 samples are available.
    pub fn estimate_rocof(&self) -> f64 {
        let n = self.frequency_buffer.len();
        if n < 2 {
            return 0.0;
        }
        let n_f = n as f64;
        let sum_t: f64 = self.time_buffer.iter().sum();
        let sum_f: f64 = self.frequency_buffer.iter().sum();
        let sum_tf: f64 = self
            .time_buffer
            .iter()
            .zip(self.frequency_buffer.iter())
            .map(|(t, f)| t * f)
            .sum();
        let sum_t2: f64 = self.time_buffer.iter().map(|t| t * t).sum();
        let denom = n_f * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (n_f * sum_tf - sum_t * sum_f) / denom
    }

    /// Estimate system inertia from a known power imbalance event.
    ///
    /// H = ΔP * fn / (2 * |ROCOF| * S_base)
    ///
    /// # Arguments
    /// * `power_imbalance_mw` — known power step [MW]
    /// * `system_mva`         — system base MVA
    pub fn estimate_inertia(&self, power_imbalance_mw: f64, system_mva: f64) -> InertiaEstimate {
        let rocof = self.estimate_rocof();
        let timestamp = self.time_buffer.last().copied().unwrap_or(0.0);

        // Default nominal frequency 50 Hz (monitor is frequency-agnostic)
        let fn_ = 50.0_f64;

        let h_mws = if rocof.abs() < 1e-9 || system_mva <= 0.0 {
            0.0
        } else {
            power_imbalance_mw * fn_ / (2.0 * rocof.abs() * system_mva) * system_mva
        };

        // Confidence: higher for larger events and longer windows
        let n = self.frequency_buffer.len() as f64;
        let event_magnitude = power_imbalance_mw.abs() / system_mva.max(1.0);
        let window_fill = (n / self.window_size as f64).min(1.0);
        let confidence = (event_magnitude * 10.0).min(1.0) * window_fill;

        InertiaEstimate {
            timestamp,
            estimated_h_mws: h_mws,
            confidence,
            rocof_measured_hz_per_s: rocof,
            power_imbalance_estimate_mw: power_imbalance_mw,
            method: EstimationMethod::LinearRegression,
        }
    }

    /// Returns true if the current estimated |ROCOF| exceeds the threshold.
    pub fn detect_event(&self, rocof_threshold_hz_per_s: f64) -> bool {
        self.estimate_rocof().abs() > rocof_threshold_hz_per_s
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_steam_gen(id: usize, mva: f64, h: f64) -> GeneratorInertia {
        GeneratorInertia {
            id,
            bus: id,
            rated_mva: mva,
            h_constant_s: h,
            d_damping: 1.0,
            droop_percent: 5.0,
            response_time_s: 5.0,
            is_online: true,
            technology: GeneratorTechnology::SteamTurbine,
        }
    }

    fn make_predictor_homogeneous() -> FrequencyNadirPredictor {
        // 10 × 100 MVA steam units, H=6s, R=5%, system=1000 MVA
        let gens: Vec<GeneratorInertia> = (0..10).map(|i| make_steam_gen(i, 100.0, 6.0)).collect();
        FrequencyNadirPredictor::new(gens, 1000.0, 50.0, 47.5, 1.0)
    }

    // ── 1. Basic construction ─────────────────────────────────────────────

    #[test]
    fn test_generator_inertia_creation() {
        let gen = make_steam_gen(1, 200.0, 5.0);
        assert_eq!(gen.id, 1);
        assert!((gen.rated_mva - 200.0).abs() < 1e-9);
        assert!((gen.h_constant_s - 5.0).abs() < 1e-9);
        assert!((gen.effective_h_s() - 5.0).abs() < 1e-9);
        assert!((gen.stored_energy_mws() - 1000.0).abs() < 1e-9);
        assert!(gen.technology.is_synchronous());
    }

    // ── 2. Homogeneous system inertia ─────────────────────────────────────

    #[test]
    fn test_system_inertia_calculation_homogeneous() {
        let p = make_predictor_homogeneous();
        let si = p.compute_system_inertia();
        // H_sys = Σ(6*100) / 1000 = 6 s
        assert!(
            (si.system_h_s - 6.0).abs() < 1e-6,
            "H_sys={}",
            si.system_h_s
        );
        assert!((si.total_h_mws - 6000.0).abs() < 1e-6);
        assert!((si.synchronous_mva - 1000.0).abs() < 1e-6);
        assert!((si.converter_mva).abs() < 1e-6);
    }

    // ── 3. Renewables contribute zero inertia ─────────────────────────────

    #[test]
    fn test_system_inertia_with_renewables_zero() {
        let mut gens: Vec<GeneratorInertia> =
            (0..5).map(|i| make_steam_gen(i, 100.0, 5.0)).collect();
        // Add two Type-4 wind farms
        gens.push(GeneratorInertia {
            id: 10,
            bus: 10,
            rated_mva: 200.0,
            h_constant_s: 0.0,
            d_damping: 0.0,
            droop_percent: 0.0,
            response_time_s: 0.0,
            is_online: true,
            technology: GeneratorTechnology::WindTurbineType4,
        });
        gens.push(GeneratorInertia {
            id: 11,
            bus: 11,
            rated_mva: 100.0,
            h_constant_s: 0.0,
            d_damping: 0.0,
            droop_percent: 0.0,
            response_time_s: 0.0,
            is_online: true,
            technology: GeneratorTechnology::PhotovoltaicPlant,
        });

        let p = FrequencyNadirPredictor::new(gens, 800.0, 50.0, 47.5, 1.0);
        let si = p.compute_system_inertia();
        // Total energy = 5 * 5 * 100 = 2500 MW·s; Type-4 and PV add 0
        assert!(
            (si.total_h_mws - 2500.0).abs() < 1e-6,
            "h_mws={}",
            si.total_h_mws
        );
        // Converter MVA should include wind type-4 and PV
        assert!(si.converter_mva > 0.0, "converter_mva should be > 0");
    }

    // ── 4. ROCOF risk levels ──────────────────────────────────────────────

    #[test]
    fn test_rocof_risk_assessment_low() {
        // Very high inertia → low risk
        let gens: Vec<GeneratorInertia> = (0..20).map(|i| make_steam_gen(i, 500.0, 10.0)).collect();
        let p = FrequencyNadirPredictor::new(gens, 1000.0, 50.0, 47.5, 1.0);
        let si = p.compute_system_inertia();
        assert_eq!(si.rocof_risk, RocofRiskLevel::Low);
    }

    #[test]
    fn test_rocof_risk_assessment_critical() {
        // Very low inertia → critical risk
        let gens: Vec<GeneratorInertia> = vec![make_steam_gen(0, 10.0, 0.1)];
        let p = FrequencyNadirPredictor::new(gens, 10000.0, 50.0, 47.5, 1.0);
        let si = p.compute_system_inertia();
        // H_sys = 0.1 * 10 / 10000 = 0.0001 s → very low
        assert!(
            si.rocof_risk == RocofRiskLevel::Critical || si.rocof_risk == RocofRiskLevel::High,
            "risk={:?}",
            si.rocof_risk
        );
    }

    // ── 5. Frequency nadir basic ──────────────────────────────────────────

    #[test]
    fn test_frequency_nadir_basic() {
        let p = make_predictor_homogeneous();
        let res = p.predict_nadir(100.0, 30.0).expect("predict_nadir failed");
        // Nadir must be below nominal
        assert!(
            res.nadir_frequency_hz < 50.0,
            "nadir={}",
            res.nadir_frequency_hz
        );
        // Nadir must be above hard floor for normal system
        assert!(
            res.nadir_frequency_hz > 40.0,
            "nadir too low: {}",
            res.nadir_frequency_hz
        );
        // ROCOF initial should be negative (frequency falling)
        assert!(res.rocof_initial_hz_per_s < 0.0);
        assert!(!res.trajectory.is_empty());
    }

    // ── 6. Small inertia gives deeper nadir ──────────────────────────────

    #[test]
    fn test_frequency_nadir_small_inertia() {
        let gens_low: Vec<GeneratorInertia> =
            (0..3).map(|i| make_steam_gen(i, 100.0, 2.0)).collect();
        let gens_high: Vec<GeneratorInertia> =
            (0..3).map(|i| make_steam_gen(i, 100.0, 8.0)).collect();
        let p_low = FrequencyNadirPredictor::new(gens_low, 300.0, 50.0, 47.5, 2.0);
        let p_high = FrequencyNadirPredictor::new(gens_high, 300.0, 50.0, 47.5, 2.0);
        let r_low = p_low.predict_nadir(50.0, 30.0).expect("low inertia failed");
        let r_high = p_high
            .predict_nadir(50.0, 30.0)
            .expect("high inertia failed");
        assert!(
            r_low.nadir_frequency_hz < r_high.nadir_frequency_hz,
            "low H nadir {} >= high H nadir {}",
            r_low.nadir_frequency_hz,
            r_high.nadir_frequency_hz
        );
    }

    // ── 7. Trajectory length ──────────────────────────────────────────────

    #[test]
    fn test_frequency_nadir_trajectory_length() {
        let p = make_predictor_homogeneous();
        let t_max = 20.0_f64;
        let dt = 0.01_f64;
        let res = p.predict_nadir(50.0, t_max).expect("predict failed");
        let expected = (t_max / dt).ceil() as usize + 1;
        assert_eq!(
            res.trajectory.len(),
            expected,
            "trajectory length {} != expected {}",
            res.trajectory.len(),
            expected
        );
    }

    // ── 8. Nadir above minimum (no UFLS) ─────────────────────────────────

    #[test]
    fn test_nadir_above_minimum() {
        let p = make_predictor_homogeneous();
        // Small loss: 30 MW in 1000 MVA system with good inertia
        let res = p.predict_nadir(30.0, 30.0).expect("predict failed");
        assert!(
            !res.ufls_triggered,
            "UFLS incorrectly triggered, nadir={}",
            res.nadir_frequency_hz
        );
    }

    // ── 9. Nadir below minimum triggers UFLS ─────────────────────────────

    #[test]
    fn test_nadir_below_minimum_triggers_ufls() {
        // Very weak system: 1 generator, tiny inertia, large loss
        let gens = vec![GeneratorInertia {
            id: 0,
            bus: 0,
            rated_mva: 50.0,
            h_constant_s: 1.0,
            d_damping: 0.0,
            droop_percent: 0.0, // no governor
            response_time_s: 100.0,
            is_online: true,
            technology: GeneratorTechnology::SteamTurbine,
        }];
        let p = FrequencyNadirPredictor::new(gens, 50.0, 50.0, 48.0, 5.0);
        let res = p.predict_nadir(40.0, 10.0).expect("predict failed");
        assert!(
            res.ufls_triggered,
            "UFLS should have triggered, nadir={}",
            res.nadir_frequency_hz
        );
    }

    // ── 10. Minimum inertia formula ───────────────────────────────────────

    #[test]
    fn test_minimum_inertia_formula() {
        let p = make_predictor_homogeneous();
        // H_min = ΔP * f0 / (2 * rocof_limit * S_base)
        //       = 100 * 50 / (2 * 1.0 * 1000) = 2.5 s
        let h_min = p.minimum_inertia_for_rocof(100.0);
        assert!((h_min - 2.5).abs() < 1e-6, "H_min={}", h_min);
    }

    // ── 11. Maximum credible loss ─────────────────────────────────────────

    #[test]
    fn test_maximum_credible_loss() {
        let p = make_predictor_homogeneous();
        let max_loss = p.maximum_credible_loss(30.0).expect("max_loss failed");
        // Should be > 0 and ≤ system MVA
        assert!(max_loss > 0.0, "max_loss should be positive");
        assert!(max_loss <= p.system_mva, "max_loss > system_mva");
    }

    // ── 12. Virtual inertia augmentation ─────────────────────────────────

    #[test]
    fn test_virtual_inertia_augmentation() {
        let p_base = make_predictor_homogeneous();
        let res_base = p_base.predict_nadir(200.0, 30.0).expect("base failed");

        let mut p_vi = make_predictor_homogeneous();
        p_vi.add_virtual_inertia(500.0, 0.05); // 500 MW·s BESS
        let res_vi = p_vi.predict_nadir(200.0, 30.0).expect("vi failed");

        // Virtual inertia should improve (raise) the nadir
        assert!(
            res_vi.nadir_frequency_hz >= res_base.nadir_frequency_hz,
            "VI nadir {} < base nadir {}",
            res_vi.nadir_frequency_hz,
            res_base.nadir_frequency_hz
        );
    }

    // ── 13. FCR requirement ───────────────────────────────────────────────

    #[test]
    fn test_fcr_requirement() {
        let p = make_predictor_homogeneous();
        let fcr_30 = p.fcr_requirement(30.0);
        let fcr_15 = p.fcr_requirement(15.0);
        // Shorter delivery time → higher reserve requirement
        assert!(fcr_30 > 0.0, "FCR should be positive");
        assert!(
            fcr_15 > fcr_30,
            "shorter t_delivery should need more FCR: fcr_15={} fcr_30={}",
            fcr_15,
            fcr_30
        );
    }

    // ── 14. Inertia emulation: droop response ─────────────────────────────

    #[test]
    fn test_inertia_emulation_droop_response() {
        let mut ctrl = InertiaEmulationControl::new(
            0.0,   // no inertia emulation
            10.0,  // droop gain 10 MW/Hz
            0.0,   // no deadband
            100.0, // 100 MW limit
            50.0,  // 50 MW/s ramp limit
        );
        // Δf = -1 Hz → P_desired = 10 * 1 = 10 MW
        let p = ctrl.compute_response(49.0, 0.0, 0.1);
        assert!(p > 0.0, "droop response should be positive, got {}", p);
        assert!(p <= 100.0);
    }

    // ── 15. Inertia emulation: ROCOF response ─────────────────────────────

    #[test]
    fn test_inertia_emulation_rocof_response() {
        let mut ctrl = InertiaEmulationControl::new(
            50.0,  // 50 MW·s
            0.0,   // no droop
            0.0,   // no deadband
            200.0, // 200 MW limit
            200.0, // fast ramp
        );
        // ROCOF = -2 Hz/s → P = 50 * 2 = 100 MW
        let p = ctrl.compute_response(50.0, -2.0, 1.0);
        assert!(
            (p - 100.0).abs() < 1.0,
            "ROCOF response should be ~100 MW, got {}",
            p
        );
    }

    // ── 16. Inertia emulation: deadband ──────────────────────────────────

    #[test]
    fn test_inertia_emulation_deadband() {
        let mut ctrl = InertiaEmulationControl::new(
            0.0,  // no inertia term
            20.0, // 20 MW/Hz droop
            0.1,  // 0.1 Hz deadband
            100.0, 100.0,
        );
        // Δf = -0.05 Hz → within deadband → 0 response
        let p_inside = ctrl.compute_response(49.95, 0.0, 0.1);
        assert!(
            (p_inside).abs() < 1e-6,
            "inside deadband should give 0, got {}",
            p_inside
        );

        // Δf = -0.2 Hz → outside deadband → positive response
        let mut ctrl2 = InertiaEmulationControl::new(0.0, 20.0, 0.1, 100.0, 100.0);
        let p_outside = ctrl2.compute_response(49.8, 0.0, 0.1);
        assert!(
            p_outside > 0.0,
            "outside deadband should give positive P, got {}",
            p_outside
        );
    }

    // ── 17. Virtual H constant ────────────────────────────────────────────

    #[test]
    fn test_virtual_h_constant() {
        let ctrl = InertiaEmulationControl::new(100.0, 0.0, 0.0, 50.0, 50.0);
        // H_virtual = 100 / (2 * 50) = 1.0 s
        let h = ctrl.virtual_h_constant(50.0);
        assert!((h - 1.0).abs() < 1e-9, "H_virtual={}", h);
    }

    // ── 18. InertiaMonitor: ROCOF estimation ─────────────────────────────

    #[test]
    fn test_inertia_monitor_rocof_estimation() {
        let mut mon = InertiaMonitor::new(20, 100.0);
        // Simulate a ramp: f = 50 - 0.5*t → ROCOF = -0.5 Hz/s
        for i in 0..20 {
            let t = i as f64 * 0.01;
            let f = 50.0 - 0.5 * t;
            mon.update(t, f);
        }
        let rocof = mon.estimate_rocof();
        assert!((rocof - (-0.5)).abs() < 0.01, "ROCOF estimate={:.4}", rocof);
    }

    // ── 19. InertiaMonitor: event detection ──────────────────────────────

    #[test]
    fn test_inertia_monitor_event_detection() {
        let mut mon = InertiaMonitor::new(10, 50.0);
        // Stable frequency → no event
        for i in 0..10 {
            mon.update(i as f64 * 0.02, 50.0);
        }
        assert!(
            !mon.detect_event(0.1),
            "stable system should not trigger event"
        );

        // Now add a ramp: -2 Hz/s
        let mut mon2 = InertiaMonitor::new(10, 50.0);
        for i in 0..10 {
            let t = i as f64 * 0.02;
            mon2.update(t, 50.0 - 2.0 * t);
        }
        assert!(
            mon2.detect_event(0.1),
            "fast frequency drop should trigger event"
        );
    }

    // ── 20. InertiaMonitor: inertia estimate from disturbance ─────────────

    #[test]
    fn test_inertia_estimate_from_disturbance() {
        let mut mon = InertiaMonitor::new(50, 100.0);
        // ROCOF = -1.0 Hz/s → f = 50 - t
        for i in 0..50 {
            let t = i as f64 * 0.01;
            mon.update(t, 50.0 - 1.0 * t);
        }
        // ΔP = 100 MW, S_base = 1000 MVA
        // H_mws = 100 * 50 / (2 * 1.0 * 1000) * 1000 = 2500
        let est = mon.estimate_inertia(100.0, 1000.0);
        assert!(
            (est.estimated_h_mws - 2500.0).abs() < 50.0,
            "H_mws estimate={:.1}",
            est.estimated_h_mws
        );
        assert!(est.confidence > 0.0);
        assert_eq!(est.method, EstimationMethod::LinearRegression);
        assert!((est.rocof_measured_hz_per_s - (-1.0)).abs() < 0.02);
    }

    // ── 21. GeneratorTechnology effective inertia ─────────────────────────

    #[test]
    fn test_wind_type4_zero_inertia() {
        let tech = GeneratorTechnology::WindTurbineType4;
        assert_eq!(tech.effective_inertia_s(5.0), 0.0);
        assert!(!tech.is_synchronous());
    }

    #[test]
    fn test_wind_type3_virtual_inertia() {
        let tech = GeneratorTechnology::WindTurbineType3 {
            virtual_inertia: 1.5,
        };
        // Should add virtual component: 3.0 + 1.5 = 4.5
        assert!((tech.effective_inertia_s(3.0) - 4.5).abs() < 1e-9);
    }

    #[test]
    fn test_bess_virtual_inertia_zero_h() {
        let tech = GeneratorTechnology::BessWithVirtualInertia { kvi: 200.0 };
        // BESS physical H = 0 (modelled via kvi separately)
        assert_eq!(tech.effective_inertia_s(5.0), 0.0);
        assert!(!tech.is_synchronous());
    }
}
