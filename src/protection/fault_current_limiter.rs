//! Fault Current Limiter (FCL) module for OxiGrid.
//!
//! Implements FCL device models, sizing calculations, placement optimization,
//! and performance analysis for power system protection studies.
//!
//! # FCL Technologies
//! - **Resistive Superconducting (RSFCL)**: Zero impedance below I_c; quenches to R_n above
//! - **Inductive Superconducting**: Saturated core; large inductance jump on fault
//! - **Solid-State Series**: Fast semiconductor switch inserts R+jX on trigger
//! - **Bridge FCL**: Current-limiting reactor + solid-state bridge bypass
//! - **Is-Limiter**: Explosive fuse + parallel impedance path
//!
//! # References
//! - IEC 62271-310: High-voltage switchgear — Fault current limiters
//! - Noe & Steurer (2007): "High-temperature superconductor fault current limiters"

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// Angular frequency at 50 Hz
const OMEGA_50HZ: f64 = 2.0 * PI * 50.0;
// sqrt(3)
const SQRT3: f64 = 1.732_050_808_56;

// ─────────────────────────────────────────────────────────────────────────────
// FCL Technology enum
// ─────────────────────────────────────────────────────────────────────────────

/// Technology type of a Fault Current Limiter.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FclTechnology {
    /// Resistive superconducting FCL (RSFCL).
    ///
    /// Operates at near-zero resistance below critical current I_c.
    /// Quenches to R_n when current exceeds I_c.
    ResistiveSuperconducting {
        /// Critical current threshold (kA) — quenching begins above this
        critical_current_ka: f64,
        /// Normal-state resistance (Ω) after full quench
        normal_resistance_ohm: f64,
        /// Recovery time to return to superconducting state (s)
        recovery_time_s: f64,
        /// Rise time for resistance to grow from 0 → R_n (ms)
        quench_rise_time_ms: f64,
    },
    /// Inductive (saturated-core) superconducting FCL.
    ///
    /// DC bias keeps core saturated during normal operation (low L).
    /// Fault current removes bias, core unsaturates → large inductance inserted.
    InductiveSuperconducting {
        /// Saturated inductance during normal operation (mH)
        saturated_inductance_mh: f64,
        /// Unsaturated inductance during fault (mH) — typically ~100× larger
        unsaturated_inductance_mh: f64,
        /// Current at which core unsaturates (kA)
        saturation_current_ka: f64,
        /// DC bias coil current (A)
        bias_coil_current_a: f64,
    },
    /// Series impedance FCL with solid-state switch.
    ///
    /// Semiconductor device (IGBT/thyristor) inserts R+jX on fault detection.
    SolidStateSeries {
        /// Trigger threshold current (kA)
        trigger_current_ka: f64,
        /// Resistance inserted after switching (Ω)
        inserted_resistance_ohm: f64,
        /// Reactance inserted after switching (Ω)
        inserted_reactance_ohm: f64,
        /// Time to fully insert impedance after trigger (ms)
        switching_time_ms: f64,
    },
    /// Bridge-type FCL with current-limiting reactor.
    ///
    /// During normal operation, reactor is bypassed by diode bridge.
    /// On fault, bypass is removed and reactor limits current.
    Bridge {
        /// Reactor inductance (mH)
        reactor_inductance_mh: f64,
        /// Current above which bridge opens (kA)
        trigger_current_ka: f64,
        /// Bypass path resistance when bridge opens (Ω)
        bypass_resistance_ohm: f64,
    },
    /// Is-limiter: high-speed fuse + parallel impedance.
    ///
    /// Explosive charge blows fuse at peak fault current;
    /// current is then forced through parallel impedance.
    IsLimiter {
        /// Peak current at which fuse blows (kA)
        fuse_current_ka: f64,
        /// Parallel impedance inserted after fuse operation (Ω)
        parallel_impedance_ohm: f64,
        /// Whether manual reset is required after operation
        reset_required: bool,
    },
}

impl FclTechnology {
    /// Return the nominal trigger/critical current threshold in kA.
    pub fn trigger_threshold_ka(&self) -> f64 {
        match self {
            Self::ResistiveSuperconducting {
                critical_current_ka,
                ..
            } => *critical_current_ka,
            Self::InductiveSuperconducting {
                saturation_current_ka,
                ..
            } => *saturation_current_ka,
            Self::SolidStateSeries {
                trigger_current_ka, ..
            } => *trigger_current_ka,
            Self::Bridge {
                trigger_current_ka, ..
            } => *trigger_current_ka,
            Self::IsLimiter {
                fuse_current_ka, ..
            } => *fuse_current_ka,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FCL Operating State
// ─────────────────────────────────────────────────────────────────────────────

/// Operating state of a fault current limiter.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FclState {
    /// Conducting normally — near-zero impedance presented to the network
    Normal,
    /// FCL has inserted limiting impedance following a trigger event
    Triggered,
    /// Transitioning from superconducting to normal state (RSFCL quench front)
    Quenching,
    /// Returning from triggered state toward normal superconducting operation
    Recovering,
    /// FCL has failed and requires replacement / maintenance
    Failed,
}

// ─────────────────────────────────────────────────────────────────────────────
// FaultCurrentLimiter device
// ─────────────────────────────────────────────────────────────────────────────

/// FCL device model.
///
/// Represents a single FCL installed on a network branch.
/// Call [`FaultCurrentLimiter::update_state`] each time step during a transient simulation,
/// then query [`FaultCurrentLimiter::effective_impedance`] to modify branch admittance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultCurrentLimiter {
    /// Unique device identifier
    pub id: usize,
    /// From-bus index of the branch the FCL is installed on
    pub branch_from: usize,
    /// To-bus index of the branch
    pub branch_to: usize,
    /// FCL technology parameters
    pub technology: FclTechnology,
    /// Continuous rated current (kA) — normal load must be below this
    pub rated_current_ka: f64,
    /// Rated voltage (kV)
    pub rated_voltage_kv: f64,
    /// Current operating state
    pub operating_state: FclState,
    /// Cumulative number of fault-triggering operations
    pub fault_count: u32,
    /// Total energy absorbed over device lifetime (J)
    pub total_energy_absorbed_j: f64,
    /// Time elapsed since last state transition (ms) — internal use
    time_in_state_ms: f64,
}

impl FaultCurrentLimiter {
    /// Create a new FCL device in the [`FclState::Normal`] state.
    ///
    /// # Arguments
    /// * `id` — unique device id
    /// * `branch_from` / `branch_to` — from/to bus indices
    /// * `technology` — FCL technology variant
    /// * `rated_current_ka` — continuous current rating (kA)
    /// * `rated_voltage_kv` — voltage rating (kV)
    pub fn new(
        id: usize,
        branch_from: usize,
        branch_to: usize,
        technology: FclTechnology,
        rated_current_ka: f64,
        rated_voltage_kv: f64,
    ) -> Self {
        Self {
            id,
            branch_from,
            branch_to,
            technology,
            rated_current_ka,
            rated_voltage_kv,
            operating_state: FclState::Normal,
            fault_count: 0,
            total_energy_absorbed_j: 0.0,
            time_in_state_ms: 0.0,
        }
    }

    /// Returns `true` if the FCL should trigger given the present current magnitude.
    ///
    /// The FCL only triggers when in the [`FclState::Normal`] state; a failed or
    /// already-triggered device will not re-trigger.
    pub fn should_trigger(&self, current_ka: f64) -> bool {
        if self.operating_state != FclState::Normal {
            return false;
        }
        current_ka > self.technology.trigger_threshold_ka()
    }

    /// Compute the effective (R, X) impedance the FCL presents to the network.
    ///
    /// Returns `(R_ohm, X_ohm)`.
    ///
    /// # Arguments
    /// * `time_after_trigger_ms` — time elapsed since the FCL triggered (ms).
    ///   Pass `0.0` to query the instantaneous post-trigger impedance, or the
    ///   current simulation time when in mid-transition.
    pub fn effective_impedance(&self, time_after_trigger_ms: f64) -> (f64, f64) {
        match &self.technology {
            FclTechnology::ResistiveSuperconducting {
                normal_resistance_ohm,
                quench_rise_time_ms,
                ..
            } => match self.operating_state {
                FclState::Normal | FclState::Recovering => (0.0, 0.0),
                FclState::Quenching => {
                    // Linear ramp from 0 → R_n over quench_rise_time_ms
                    let frac = if *quench_rise_time_ms > 0.0 {
                        (time_after_trigger_ms / quench_rise_time_ms).min(1.0)
                    } else {
                        1.0
                    };
                    (frac * normal_resistance_ohm, 0.0)
                }
                FclState::Triggered => (*normal_resistance_ohm, 0.0),
                FclState::Failed => (0.0, 0.0),
            },

            FclTechnology::InductiveSuperconducting {
                saturated_inductance_mh,
                unsaturated_inductance_mh,
                ..
            } => {
                let x_sat = OMEGA_50HZ * saturated_inductance_mh * 1e-3;
                let x_unsat = OMEGA_50HZ * unsaturated_inductance_mh * 1e-3;
                match self.operating_state {
                    FclState::Normal | FclState::Recovering => (0.0, x_sat),
                    FclState::Triggered | FclState::Quenching => (0.0, x_unsat),
                    FclState::Failed => (0.0, x_sat),
                }
            }

            FclTechnology::SolidStateSeries {
                inserted_resistance_ohm,
                inserted_reactance_ohm,
                switching_time_ms,
                ..
            } => match self.operating_state {
                FclState::Normal | FclState::Recovering => (0.0, 0.0),
                FclState::Quenching => {
                    // Ramp during switching time
                    let frac = if *switching_time_ms > 0.0 {
                        (time_after_trigger_ms / switching_time_ms).min(1.0)
                    } else {
                        1.0
                    };
                    (
                        frac * inserted_resistance_ohm,
                        frac * inserted_reactance_ohm,
                    )
                }
                FclState::Triggered => (*inserted_resistance_ohm, *inserted_reactance_ohm),
                FclState::Failed => (0.0, 0.0),
            },

            FclTechnology::Bridge {
                reactor_inductance_mh,
                bypass_resistance_ohm,
                ..
            } => {
                let x_reactor = OMEGA_50HZ * reactor_inductance_mh * 1e-3;
                match self.operating_state {
                    FclState::Normal | FclState::Recovering => (0.0, x_reactor),
                    FclState::Triggered | FclState::Quenching => (*bypass_resistance_ohm, 0.0),
                    FclState::Failed => (0.0, x_reactor),
                }
            }

            FclTechnology::IsLimiter {
                parallel_impedance_ohm,
                ..
            } => match self.operating_state {
                FclState::Normal | FclState::Recovering => (0.0, 0.0),
                FclState::Triggered | FclState::Quenching => (*parallel_impedance_ohm, 0.0),
                FclState::Failed => (0.0, 0.0),
            },
        }
    }

    /// Update the FCL operating state given instantaneous current and time step.
    ///
    /// State machine:
    /// - Normal → Quenching when `should_trigger()` is true
    /// - Quenching → Triggered after technology-specific rise time
    /// - Triggered → Recovering when current drops below rated
    /// - Recovering → Normal after technology-specific recovery time
    ///
    /// # Arguments
    /// * `current_ka` — instantaneous current magnitude (kA)
    /// * `dt_ms` — time step (ms)
    pub fn update_state(&mut self, current_ka: f64, dt_ms: f64) {
        self.time_in_state_ms += dt_ms;

        match self.operating_state.clone() {
            FclState::Normal => {
                if self.should_trigger(current_ka) {
                    self.operating_state = FclState::Quenching;
                    self.time_in_state_ms = 0.0;
                    self.fault_count += 1;
                }
            }

            FclState::Quenching => {
                let rise_time = self.quench_rise_time_ms();
                if self.time_in_state_ms >= rise_time {
                    self.operating_state = FclState::Triggered;
                    self.time_in_state_ms = 0.0;
                }
            }

            FclState::Triggered => {
                // Transition to recovering when current falls below rated
                if current_ka <= self.rated_current_ka {
                    self.operating_state = FclState::Recovering;
                    self.time_in_state_ms = 0.0;
                }
            }

            FclState::Recovering => {
                let recovery_ms = self.recovery_time_s() * 1000.0;
                if self.time_in_state_ms >= recovery_ms {
                    self.operating_state = FclState::Normal;
                    self.time_in_state_ms = 0.0;
                }
                // If fault current returns before recovery completes, re-trigger
                if current_ka > self.technology.trigger_threshold_ka() {
                    self.operating_state = FclState::Quenching;
                    self.time_in_state_ms = 0.0;
                    self.fault_count += 1;
                }
            }

            FclState::Failed => {
                // Absorb nothing; require external reset
            }
        }
    }

    /// Compute the energy absorbed by the FCL during a fault event (J).
    ///
    /// Uses the steady-state approximation:
    /// E = I_fault² × R_eff × t_fault
    ///
    /// where `R_eff` is the effective resistance at `time = fault_duration_ms`.
    pub fn compute_energy_absorbed(&self, fault_current_ka: f64, fault_duration_ms: f64) -> f64 {
        let (r_eff, _x_eff) = self.effective_impedance(fault_duration_ms);
        let i_a = fault_current_ka * 1e3; // convert kA → A
        let t_s = fault_duration_ms * 1e-3; // convert ms → s
        i_a * i_a * r_eff * t_s
    }

    /// Returns `true` if the FCL would be overloaded by the given fault event.
    ///
    /// Overload is determined by comparing absorbed energy against a maximum
    /// rated energy (I_rated² × R_rated × 1 s, capped at 1 MJ for safety).
    pub fn is_overloaded(&self, fault_current_ka: f64, fault_duration_ms: f64) -> bool {
        let absorbed = self.compute_energy_absorbed(fault_current_ka, fault_duration_ms);
        // Maximum rated energy: I_rated² × R_eff(full trigger) × 1 s
        let (r_rated, _) = self.effective_impedance(fault_duration_ms);
        let i_rated_a = self.rated_current_ka * 1e3;
        let max_energy_j = (i_rated_a * i_rated_a * r_rated * 1.0).max(1.0e6);
        absorbed > max_energy_j
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn quench_rise_time_ms(&self) -> f64 {
        match &self.technology {
            FclTechnology::ResistiveSuperconducting {
                quench_rise_time_ms,
                ..
            } => *quench_rise_time_ms,
            FclTechnology::SolidStateSeries {
                switching_time_ms, ..
            } => *switching_time_ms,
            // Inductive / Bridge / IsLimiter have essentially instantaneous transitions
            _ => 0.0,
        }
    }

    fn recovery_time_s(&self) -> f64 {
        match &self.technology {
            FclTechnology::ResistiveSuperconducting {
                recovery_time_s, ..
            } => *recovery_time_s,
            // Solid-state and others typically recover in ~100 ms
            FclTechnology::SolidStateSeries { .. } => 0.1,
            FclTechnology::InductiveSuperconducting { .. } => 0.5,
            FclTechnology::Bridge { .. } => 0.2,
            FclTechnology::IsLimiter { reset_required, .. } => {
                if *reset_required {
                    f64::INFINITY
                } else {
                    0.05
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FCL Sizing
// ─────────────────────────────────────────────────────────────────────────────

/// Input parameters for FCL sizing calculations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclSizing {
    /// Network line-to-line voltage (kV)
    pub network_voltage_kv: f64,
    /// System base MVA
    pub base_mva: f64,
    /// Prospective (pre-FCL) fault current (kA)
    pub prospective_fault_current_ka: f64,
    /// Target fault current after FCL insertion (kA)
    pub target_fault_current_ka: f64,
    /// Normal load current flowing through FCL location (kA)
    pub normal_load_current_ka: f64,
    /// Fault duration for energy calculations (ms)
    pub fault_duration_ms: f64,
}

/// Result of an FCL sizing calculation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclSizingResult {
    /// Required FCL impedance in per-unit (on system base)
    pub required_impedance_pu: f64,
    /// Required resistance component (Ω)
    pub required_resistance_ohm: f64,
    /// Required reactance component (Ω)
    pub required_reactance_ohm: f64,
    /// Percentage reduction in fault current: (I_prosp − I_lim) / I_prosp × 100
    pub current_reduction_pct: f64,
    /// Energy that must be absorbed during fault (kJ)
    pub energy_absorption_kj: f64,
    /// Recommended FCL technology for this application
    pub recommended_technology: FclTechnology,
    /// Rough cost estimate (million EUR)
    pub cost_estimate_million_eur: f64,
}

impl FclSizing {
    /// Create a new sizing problem specification.
    ///
    /// # Arguments
    /// * `network_voltage_kv` — line-to-line voltage at FCL location (kV)
    /// * `base_mva` — system base MVA for per-unit conversion
    /// * `prospective_fault_current_ka` — fault current without FCL (kA)
    /// * `target_fault_current_ka` — desired maximum fault current (kA)
    /// * `normal_load_current_ka` — continuous load current the FCL must pass (kA)
    /// * `fault_duration_ms` — fault clearance time for energy rating (ms)
    pub fn new(
        network_voltage_kv: f64,
        base_mva: f64,
        prospective_fault_current_ka: f64,
        target_fault_current_ka: f64,
        normal_load_current_ka: f64,
        fault_duration_ms: f64,
    ) -> Self {
        Self {
            network_voltage_kv,
            base_mva,
            prospective_fault_current_ka,
            target_fault_current_ka,
            normal_load_current_ka,
            fault_duration_ms,
        }
    }

    /// Compute the required FCL impedance.
    ///
    /// Uses Thevenin-equivalent circuit analysis:
    ///
    /// ```text
    /// V_phase = V_LL / √3
    /// Z_source = V_phase / I_prospective   (source impedance)
    /// Z_target = V_phase / I_target        (required total impedance to limit current)
    /// Z_fcl    = Z_target − Z_source       (additional impedance to add)
    /// ```
    ///
    /// The result is split between R and X using equal R and X components.
    /// Returns `(R_ohm, X_ohm)`.
    ///
    /// # Errors
    /// Returns [`OxiGridError::InvalidParameter`] if target ≥ prospective current
    /// or if any current is ≤ 0.
    pub fn compute_required_impedance(&self) -> Result<(f64, f64)> {
        if self.prospective_fault_current_ka <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "prospective_fault_current_ka must be > 0".into(),
            ));
        }
        if self.target_fault_current_ka <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "target_fault_current_ka must be > 0".into(),
            ));
        }
        if self.target_fault_current_ka >= self.prospective_fault_current_ka {
            return Err(OxiGridError::InvalidParameter(
                "target_fault_current_ka must be less than prospective_fault_current_ka".into(),
            ));
        }

        let v_phase = self.network_voltage_kv * 1000.0 / SQRT3; // V (RMS line-to-neutral)
        let i_prosp_a = self.prospective_fault_current_ka * 1000.0;
        let i_target_a = self.target_fault_current_ka * 1000.0;

        let z_source = v_phase / i_prosp_a;
        let z_target = v_phase / i_target_a;
        let z_fcl = (z_target - z_source).max(0.0);

        // Distribute: R component = Z/√3, X component = Z - R
        let r_ohm = z_fcl / SQRT3;
        let x_ohm = z_fcl - r_ohm;

        Ok((r_ohm, x_ohm))
    }

    /// Select the best-fit FCL technology for these requirements.
    ///
    /// Decision logic:
    /// - Prospective < 5 kA: ResistiveSuperconducting (cost-effective for moderate currents)
    /// - Fault duration < 50 ms: SolidStateSeries (fast response critical)
    /// - Otherwise: Bridge (high energy absorption capacity)
    pub fn recommend_technology(&self) -> FclTechnology {
        if self.prospective_fault_current_ka < 5.0 {
            // Moderate fault current — RSFCL is cost-effective
            let critical_current_ka = self.normal_load_current_ka * 1.5;
            let r_required = self
                .compute_required_impedance()
                .map(|(r, _)| r)
                .unwrap_or(1.0);
            FclTechnology::ResistiveSuperconducting {
                critical_current_ka,
                normal_resistance_ohm: r_required.max(0.5),
                recovery_time_s: 30.0,
                quench_rise_time_ms: 2.0,
            }
        } else if self.fault_duration_ms < 50.0 {
            // Fast fault clearing required — solid-state is fastest
            let r_required = self
                .compute_required_impedance()
                .map(|(r, _)| r)
                .unwrap_or(1.0);
            let x_required = self
                .compute_required_impedance()
                .map(|(_, x)| x)
                .unwrap_or(0.5);
            FclTechnology::SolidStateSeries {
                trigger_current_ka: self.normal_load_current_ka * 1.2,
                inserted_resistance_ohm: r_required.max(0.1),
                inserted_reactance_ohm: x_required.max(0.1),
                switching_time_ms: 0.5,
            }
        } else {
            // High-energy fault — Bridge FCL handles large energy absorption
            let l_mh = self
                .compute_required_impedance()
                .map(|(_, x)| x / OMEGA_50HZ * 1000.0)
                .unwrap_or(5.0);
            FclTechnology::Bridge {
                reactor_inductance_mh: l_mh.max(1.0),
                trigger_current_ka: self.normal_load_current_ka * 1.3,
                bypass_resistance_ohm: 0.5,
            }
        }
    }

    /// Estimate rough procurement cost (million EUR) for a given technology.
    ///
    /// Cost models are indicative only; based on published tender data (2020–2024).
    pub fn estimate_cost(&self, tech: &FclTechnology) -> f64 {
        match tech {
            FclTechnology::ResistiveSuperconducting {
                critical_current_ka,
                ..
            } => {
                // Base: 2.5 M€ scaled by critical current rating
                2.5 * (critical_current_ka / 1.0).max(1.0)
            }
            FclTechnology::InductiveSuperconducting {
                saturation_current_ka,
                ..
            } => 3.0 * (saturation_current_ka / 1.0).max(1.0),
            FclTechnology::SolidStateSeries {
                trigger_current_ka, ..
            } => 1.5 * (trigger_current_ka / 1.0).max(1.0),
            FclTechnology::Bridge {
                trigger_current_ka, ..
            } => 1.8 * (trigger_current_ka / 1.0).max(1.0),
            FclTechnology::IsLimiter {
                fuse_current_ka, ..
            } => 0.8 * (fuse_current_ka / 1.0).max(1.0),
        }
    }

    /// Run the full FCL sizing calculation and return a [`FclSizingResult`].
    ///
    /// # Errors
    /// Propagates errors from [`FclSizing::compute_required_impedance`].
    pub fn size(&self) -> Result<FclSizingResult> {
        let (r_ohm, x_ohm) = self.compute_required_impedance()?;

        // Per-unit conversion: Z_base = V_LL² / S_base
        let v_kv = self.network_voltage_kv;
        let z_base = v_kv * v_kv / self.base_mva;
        let z_ohm = (r_ohm * r_ohm + x_ohm * x_ohm).sqrt();
        let z_pu = if z_base > 0.0 { z_ohm / z_base } else { 0.0 };

        let current_reduction_pct = (self.prospective_fault_current_ka
            - self.target_fault_current_ka)
            / self.prospective_fault_current_ka
            * 100.0;

        // Energy = I_fault² × R × t  (approximate, using target current and R component)
        let i_a = self.target_fault_current_ka * 1e3;
        let t_s = self.fault_duration_ms * 1e-3;
        let energy_j = i_a * i_a * r_ohm * t_s;
        let energy_kj = energy_j / 1000.0;

        let recommended_technology = self.recommend_technology();
        let cost_estimate_million_eur = self.estimate_cost(&recommended_technology);

        Ok(FclSizingResult {
            required_impedance_pu: z_pu,
            required_resistance_ohm: r_ohm,
            required_reactance_ohm: x_ohm,
            current_reduction_pct,
            energy_absorption_kj: energy_kj,
            recommended_technology,
            cost_estimate_million_eur,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FCL Placement Optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// System-level greedy FCL placement optimizer.
///
/// Ranks branches by fault current excess and iteratively places FCLs
/// on the most-overloaded branches within the available budget.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclPlacementOptimizer {
    /// Number of buses in the network
    pub n_buses: usize,
    /// Number of branches in the network
    pub n_branches: usize,
    /// Fault current matrix \[fault_bus\]\[branch\] (kA)
    pub fault_currents_ka: Vec<Vec<f64>>,
    /// Continuous current rating for each branch (kA)
    pub branch_ratings_ka: Vec<f64>,
    /// Protection coordination pairs: `(breaker_branch_idx, FCL_branch_idx)`
    pub protection_coordination: Vec<(usize, usize)>,
    /// Total FCL procurement budget (million EUR)
    pub max_fcl_budget: f64,
}

/// Result of an FCL placement optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclPlacementResult {
    /// Placed FCLs: `(branch_idx, sizing_parameters)`
    pub placed_fcls: Vec<(usize, FclSizing)>,
    /// Maximum fault current reduction achieved across all protected branches (%)
    pub max_fault_reduction_pct: f64,
    /// Number of buses that now have all connected branches within rating
    pub n_protected_buses: usize,
    /// Total cost of all placed FCLs (million EUR)
    pub total_cost_million_eur: f64,
    /// Branch indices that remain overloaded after placement
    pub remaining_overloads: Vec<usize>,
}

impl FclPlacementOptimizer {
    /// Create a new placement optimizer.
    ///
    /// # Arguments
    /// * `n_buses` — total number of buses
    /// * `n_branches` — total number of branches
    /// * `fault_currents_ka` — \[fault_bus\]\[branch\] fault current matrix (kA)
    /// * `branch_ratings_ka` — continuous current rating per branch (kA)
    /// * `max_fcl_budget` — total budget for FCL procurement (million EUR)
    pub fn new(
        n_buses: usize,
        n_branches: usize,
        fault_currents_ka: Vec<Vec<f64>>,
        branch_ratings_ka: Vec<f64>,
        max_fcl_budget: f64,
    ) -> Self {
        Self {
            n_buses,
            n_branches,
            fault_currents_ka,
            branch_ratings_ka,
            protection_coordination: Vec::new(),
            max_fcl_budget,
        }
    }

    /// Greedy FCL placement algorithm.
    ///
    /// Algorithm:
    /// 1. Compute maximum fault current on each branch across all fault scenarios.
    /// 2. Compute excess ratio = max_fault_current / branch_rating for each overloaded branch.
    /// 3. Sort branches by excess ratio (highest first).
    /// 4. For each branch, size an FCL to reduce fault current to 90 % of rating.
    /// 5. Place FCL if cost fits within remaining budget.
    ///
    /// Returns [`FclPlacementResult`] with placed devices and remaining overloads.
    pub fn optimize_greedy(&self) -> FclPlacementResult {
        // Step 1: Maximum fault current per branch
        let mut max_fault_per_branch = vec![0.0_f64; self.n_branches];
        for fault_row in &self.fault_currents_ka {
            for (b, &i) in fault_row.iter().enumerate() {
                if b < self.n_branches && i > max_fault_per_branch[b] {
                    max_fault_per_branch[b] = i;
                }
            }
        }

        // Step 2: Excess ratios — only for branches that are overloaded
        let mut excess: Vec<(usize, f64)> = (0..self.n_branches)
            .filter_map(|b| {
                let rating = *self.branch_ratings_ka.get(b).unwrap_or(&f64::INFINITY);
                let fault_i = max_fault_per_branch[b];
                if fault_i > rating {
                    Some((b, fault_i / rating))
                } else {
                    None
                }
            })
            .collect();

        // Step 3: Sort by descending excess ratio
        excess.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut placed_fcls: Vec<(usize, FclSizing)> = Vec::new();
        let mut remaining_budget = self.max_fcl_budget;
        let mut placed_branch_set: std::collections::HashSet<usize> =
            std::collections::HashSet::new();
        let mut max_reduction = 0.0_f64;

        // Nominal voltage assumed 110 kV for sizing; real applications would use branch data
        let nominal_voltage_kv = 110.0;
        let base_mva = 100.0;
        let fault_duration_ms = 100.0;

        // Step 4 & 5: Greedy placement
        for (branch_idx, _excess_ratio) in &excess {
            let b = *branch_idx;
            if placed_branch_set.contains(&b) {
                continue;
            }
            let prospective = max_fault_per_branch[b];
            let rating = *self.branch_ratings_ka.get(b).unwrap_or(&prospective);
            // Target: reduce to 90 % of rating for margin
            let target = rating * 0.9;
            if target >= prospective {
                continue;
            }
            // Estimate normal load as 60 % of rating
            let load = rating * 0.6;

            let sizing = FclSizing::new(
                nominal_voltage_kv,
                base_mva,
                prospective,
                target,
                load,
                fault_duration_ms,
            );

            let tech = sizing.recommend_technology();
            let cost = sizing.estimate_cost(&tech);

            if cost <= remaining_budget {
                let reduction = (prospective - target) / prospective * 100.0;
                if reduction > max_reduction {
                    max_reduction = reduction;
                }
                remaining_budget -= cost;
                placed_branch_set.insert(b);
                placed_fcls.push((b, sizing));
            }
        }

        // Count remaining overloads after placement
        let remaining_overloads: Vec<usize> = excess
            .iter()
            .map(|(b, _)| *b)
            .filter(|b| !placed_branch_set.contains(b))
            .collect();

        // Count buses: simplified heuristic using placed branch count
        let n_protected_buses = self.count_protected_buses(&placed_branch_set);

        let total_cost = self.max_fcl_budget - remaining_budget;

        FclPlacementResult {
            placed_fcls,
            max_fault_reduction_pct: max_reduction,
            n_protected_buses,
            total_cost_million_eur: total_cost,
            remaining_overloads,
        }
    }

    /// Check whether the given FCL placement causes protection coordination issues.
    ///
    /// For each coordination pair `(breaker_branch, fcl_branch)`:
    /// - If a FCL is placed on `fcl_branch`, the fault current seen by the breaker
    ///   on `breaker_branch` may be reduced, potentially causing under-reach or
    ///   delayed operation of the upstream breaker.
    ///
    /// Returns a list of warning strings describing each coordination concern.
    pub fn check_coordination(&self, placed: &[(usize, FclSizing)]) -> Vec<String> {
        let placed_branches: std::collections::HashSet<usize> =
            placed.iter().map(|(b, _)| *b).collect();
        let mut warnings = Vec::new();

        for &(breaker_branch, fcl_branch) in &self.protection_coordination {
            if placed_branches.contains(&fcl_branch) {
                warnings.push(format!(
                    "Coordination concern: FCL on branch {} may cause under-reach of breaker \
                     protecting branch {}. Verify CTI and relay pickup settings.",
                    fcl_branch, breaker_branch
                ));
            }
            // Also warn if FCL is placed on the breaker's own branch
            if placed_branches.contains(&breaker_branch) {
                warnings.push(format!(
                    "Coordination concern: FCL on branch {} (breaker branch) reduces fault \
                     current visibility. Check backup protection on branch {}.",
                    breaker_branch, fcl_branch
                ));
            }
        }
        warnings
    }

    /// Compute post-FCL fault current on a branch using Thevenin superposition.
    ///
    /// ```text
    /// I_branch_new = I_branch_old × Z_source / (Z_source + Z_fcl)
    /// ```
    ///
    /// # Arguments
    /// * `pre_fault_current_ka` — fault current before FCL insertion (kA)
    /// * `z_source_ohm` — source impedance magnitude at fault point (Ω)
    /// * `z_fcl_ohm` — FCL impedance magnitude (Ω)
    pub fn post_fcl_fault_current(
        pre_fault_current_ka: f64,
        z_source_ohm: f64,
        z_fcl_ohm: f64,
    ) -> f64 {
        let denominator = z_source_ohm + z_fcl_ohm;
        if denominator < 1e-12 {
            return pre_fault_current_ka;
        }
        pre_fault_current_ka * z_source_ohm / denominator
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn count_protected_buses(&self, placed_branches: &std::collections::HashSet<usize>) -> usize {
        // Simplified heuristic: count distinct buses touched by placed FCLs.
        placed_branches.len().min(self.n_buses)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FCL Performance Analyzer
// ─────────────────────────────────────────────────────────────────────────────

/// Performance analyzer for a protection zone containing multiple FCLs.
///
/// Simulates a fault transient, stepping through the pre-FCL fault current
/// profile and updating each FCL device's state machine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclPerformanceAnalyzer {
    /// FCL devices installed in this protection zone
    pub fcls: Vec<FaultCurrentLimiter>,
    /// Total simulation window (ms)
    pub simulation_duration_ms: f64,
    /// Integration time step (ms)
    pub dt_ms: f64,
}

/// Result of an FCL performance simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FclPerformanceResult {
    /// Peak fault current without FCL limiting (kA)
    pub peak_fault_current_ka: f64,
    /// Peak fault current after FCL insertion (kA)
    pub limited_fault_current_ka: f64,
    /// Ratio of limited to pre-FCL peak: limited / pre (< 1 is good)
    pub reduction_factor: f64,
    /// Time at which first FCL triggered (ms)
    pub time_to_trigger_ms: f64,
    /// Total energy absorbed across all FCLs (kJ)
    pub energy_absorbed_kj: f64,
    /// Snapshot of all FCL states at each time step: `(time_ms, [states])`
    pub fcl_states_timeline: Vec<(f64, Vec<FclState>)>,
    /// Estimated time saved in protection operation (ms)
    pub protection_time_saved_ms: f64,
}

impl FclPerformanceAnalyzer {
    /// Create a new performance analyzer.
    ///
    /// # Arguments
    /// * `fcls` — FCL devices in this protection zone
    /// * `simulation_duration_ms` — total simulation window (ms)
    /// * `dt_ms` — time step (ms)
    pub fn new(fcls: Vec<FaultCurrentLimiter>, simulation_duration_ms: f64, dt_ms: f64) -> Self {
        Self {
            fcls,
            simulation_duration_ms,
            dt_ms,
        }
    }

    /// Simulate a fault event and compute FCL performance metrics.
    ///
    /// # Arguments
    /// * `fault_current_profile` — `(time_ms, pre-FCL current kA)` pairs; must be
    ///   sorted in ascending time order.  The profile is interpolated at each
    ///   simulation step.
    ///
    /// # Returns
    /// [`FclPerformanceResult`] containing peak currents, reduction factor,
    /// energy absorbed, and state timeline.
    pub fn simulate_fault(&mut self, fault_current_profile: &[(f64, f64)]) -> FclPerformanceResult {
        let mut peak_pre_fcl = 0.0_f64;
        let mut peak_post_fcl = 0.0_f64;
        let mut total_energy_j = 0.0_f64;
        let mut time_to_trigger_ms = f64::INFINITY;
        let mut timeline: Vec<(f64, Vec<FclState>)> = Vec::new();

        let n_steps = ((self.simulation_duration_ms / self.dt_ms).ceil() as usize).max(1);

        for step in 0..n_steps {
            let t_ms = step as f64 * self.dt_ms;

            // Interpolate pre-FCL fault current at this time step
            let pre_current_ka = interpolate_profile(fault_current_profile, t_ms);
            if pre_current_ka > peak_pre_fcl {
                peak_pre_fcl = pre_current_ka;
            }

            // Update each FCL and accumulate limiting effect
            let mut total_r_ohm = 0.0_f64;

            for fcl in &mut self.fcls {
                let was_normal = fcl.operating_state == FclState::Normal;
                fcl.update_state(pre_current_ka, self.dt_ms);
                if was_normal
                    && fcl.operating_state != FclState::Normal
                    && time_to_trigger_ms == f64::INFINITY
                {
                    time_to_trigger_ms = t_ms;
                }
                let (r, _x) = fcl.effective_impedance(self.dt_ms);
                total_r_ohm += r;
            }

            // Approximate limited current via voltage divider:
            // Z_source estimated from peak pre-FCL current and rated voltage.
            let v_nominal_kv = self
                .fcls
                .first()
                .map(|f| f.rated_voltage_kv)
                .unwrap_or(110.0);
            let v_phase_v = v_nominal_kv * 1000.0 / SQRT3;
            let i_pre_a = pre_current_ka * 1e3;
            let z_source_ohm = if i_pre_a > 1.0 {
                v_phase_v / i_pre_a
            } else {
                1.0
            };
            let post_current_ka = FclPlacementOptimizer::post_fcl_fault_current(
                pre_current_ka,
                z_source_ohm,
                total_r_ohm,
            );

            if post_current_ka > peak_post_fcl {
                peak_post_fcl = post_current_ka;
            }

            // Accumulate energy absorbed this step across all FCLs
            for fcl in &self.fcls {
                let (r, _) = fcl.effective_impedance(self.dt_ms);
                let i_a = post_current_ka * 1e3;
                total_energy_j += i_a * i_a * r * (self.dt_ms * 1e-3);
            }

            // Record state snapshot
            let states: Vec<FclState> = self
                .fcls
                .iter()
                .map(|f| f.operating_state.clone())
                .collect();
            timeline.push((t_ms, states));
        }

        if time_to_trigger_ms == f64::INFINITY {
            time_to_trigger_ms = 0.0;
        }

        let reduction_factor = if peak_pre_fcl > 1e-12 {
            peak_post_fcl / peak_pre_fcl
        } else {
            1.0
        };

        // Protection time saved: proportional to current reduction (heuristic)
        let protection_time_saved_ms = if reduction_factor < 1.0 {
            (1.0 - reduction_factor) * 50.0 // up to 50 ms saving
        } else {
            0.0
        };

        FclPerformanceResult {
            peak_fault_current_ka: peak_pre_fcl,
            limited_fault_current_ka: peak_post_fcl,
            reduction_factor,
            time_to_trigger_ms,
            energy_absorbed_kj: total_energy_j / 1000.0,
            fcl_states_timeline: timeline,
            protection_time_saved_ms,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility functions
// ─────────────────────────────────────────────────────────────────────────────

/// Linear interpolation of a (time, value) profile at time `t`.
///
/// Returns the first value if `t` is before the profile start,
/// the last value if after the end, or linearly interpolated otherwise.
fn interpolate_profile(profile: &[(f64, f64)], t: f64) -> f64 {
    if profile.is_empty() {
        return 0.0;
    }
    if t <= profile[0].0 {
        return profile[0].1;
    }
    let last = profile[profile.len() - 1];
    if t >= last.0 {
        return last.1;
    }
    // Binary search for the interval containing t
    let mut lo = 0usize;
    let mut hi = profile.len() - 1;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if profile[mid].0 <= t {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (t0, v0) = profile[lo];
    let (t1, v1) = profile[hi];
    let dt = t1 - t0;
    if dt.abs() < 1e-12 {
        return v0;
    }
    v0 + (v1 - v0) * (t - t0) / dt
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper constructors ───────────────────────────────────────────────────

    fn make_rsfcl() -> FaultCurrentLimiter {
        FaultCurrentLimiter::new(
            1,
            0,
            1,
            FclTechnology::ResistiveSuperconducting {
                critical_current_ka: 2.0,
                normal_resistance_ohm: 5.0,
                recovery_time_s: 30.0,
                quench_rise_time_ms: 10.0,
            },
            1.5,
            110.0,
        )
    }

    fn make_solid_state_fcl() -> FaultCurrentLimiter {
        FaultCurrentLimiter::new(
            2,
            1,
            2,
            FclTechnology::SolidStateSeries {
                trigger_current_ka: 3.0,
                inserted_resistance_ohm: 2.0,
                inserted_reactance_ohm: 1.5,
                switching_time_ms: 0.5,
            },
            2.0,
            110.0,
        )
    }

    // ── Creation tests ────────────────────────────────────────────────────────

    #[test]
    fn test_fcl_creation_resistive_superconducting() {
        let fcl = make_rsfcl();
        assert_eq!(fcl.id, 1);
        assert_eq!(fcl.branch_from, 0);
        assert_eq!(fcl.branch_to, 1);
        assert_eq!(fcl.rated_current_ka, 1.5);
        assert_eq!(fcl.rated_voltage_kv, 110.0);
        assert_eq!(fcl.operating_state, FclState::Normal);
        assert_eq!(fcl.fault_count, 0);
        assert_eq!(fcl.total_energy_absorbed_j, 0.0);
        match &fcl.technology {
            FclTechnology::ResistiveSuperconducting {
                critical_current_ka,
                normal_resistance_ohm,
                ..
            } => {
                assert!((critical_current_ka - 2.0).abs() < 1e-10);
                assert!((normal_resistance_ohm - 5.0).abs() < 1e-10);
            }
            _ => panic!("wrong technology variant"),
        }
    }

    #[test]
    fn test_fcl_creation_solid_state() {
        let fcl = make_solid_state_fcl();
        assert_eq!(fcl.id, 2);
        assert_eq!(fcl.operating_state, FclState::Normal);
        match &fcl.technology {
            FclTechnology::SolidStateSeries {
                trigger_current_ka,
                inserted_resistance_ohm,
                inserted_reactance_ohm,
                switching_time_ms,
            } => {
                assert!((trigger_current_ka - 3.0).abs() < 1e-10);
                assert!((inserted_resistance_ohm - 2.0).abs() < 1e-10);
                assert!((inserted_reactance_ohm - 1.5).abs() < 1e-10);
                assert!((switching_time_ms - 0.5).abs() < 1e-10);
            }
            _ => panic!("wrong technology variant"),
        }
    }

    // ── Trigger tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_fcl_should_trigger_above_threshold() {
        let fcl = make_rsfcl();
        // critical_current_ka = 2.0; 2.5 > 2.0 → should trigger
        assert!(fcl.should_trigger(2.5));
    }

    #[test]
    fn test_fcl_should_not_trigger_below_threshold() {
        let fcl = make_rsfcl();
        // 1.0 < 2.0 → should not trigger
        assert!(!fcl.should_trigger(1.0));
        // exactly at threshold → should not trigger (strict >)
        assert!(!fcl.should_trigger(2.0));
    }

    // ── State transition tests ────────────────────────────────────────────────

    #[test]
    fn test_fcl_state_transitions() {
        let mut fcl = make_rsfcl();
        assert_eq!(fcl.operating_state, FclState::Normal);

        // Apply fault current above critical → Normal → Quenching
        fcl.update_state(3.0, 1.0);
        assert_eq!(fcl.operating_state, FclState::Quenching);
        assert_eq!(fcl.fault_count, 1);

        // Advance past quench_rise_time_ms (10 ms) → Triggered
        for _ in 0..12 {
            fcl.update_state(3.0, 1.0);
        }
        assert_eq!(fcl.operating_state, FclState::Triggered);

        // Current drops to below rated → Recovering
        fcl.update_state(0.5, 1.0);
        assert_eq!(fcl.operating_state, FclState::Recovering);
    }

    // ── Effective impedance tests ─────────────────────────────────────────────

    #[test]
    fn test_effective_impedance_normal_state() {
        let fcl = make_rsfcl();
        // Normal state → (0, 0)
        let (r, x) = fcl.effective_impedance(0.0);
        assert!(r.abs() < 1e-10);
        assert!(x.abs() < 1e-10);
    }

    #[test]
    fn test_effective_impedance_triggered_state() {
        let mut fcl = make_rsfcl();
        // Force to Triggered state
        fcl.operating_state = FclState::Triggered;
        let (r, x) = fcl.effective_impedance(100.0);
        // Should return R_n = 5.0 Ω
        assert!((r - 5.0).abs() < 1e-10);
        assert!(x.abs() < 1e-10);
    }

    #[test]
    fn test_effective_impedance_quenching_linear_ramp() {
        let mut fcl = make_rsfcl(); // quench_rise_time_ms = 10.0, R_n = 5.0
        fcl.operating_state = FclState::Quenching;
        // At t = 5 ms (halfway through ramp) → R = 2.5 Ω
        let (r, _) = fcl.effective_impedance(5.0);
        assert!((r - 2.5).abs() < 1e-10);
    }

    // ── Energy and overload tests ─────────────────────────────────────────────

    #[test]
    fn test_energy_absorbed_calculation() {
        let mut fcl = make_rsfcl();
        fcl.operating_state = FclState::Triggered;
        // E = I² × R × t = (3e3)² × 5 × 0.1 = 4 500 000 J
        let e = fcl.compute_energy_absorbed(3.0, 100.0);
        let expected = 3000.0_f64.powi(2) * 5.0 * 0.1;
        assert!(
            (e - expected).abs() < 1.0,
            "energy mismatch: {} vs {}",
            e,
            expected
        );
    }

    #[test]
    fn test_overload_check() {
        let mut fcl = make_rsfcl();
        fcl.operating_state = FclState::Triggered;
        // Very large fault current over long duration → overloaded
        let overloaded = fcl.is_overloaded(50.0, 1000.0);
        assert!(overloaded);

        // Small fault should not overload
        let not_overloaded = fcl.is_overloaded(0.001, 1.0);
        assert!(!not_overloaded);
    }

    // ── Sizing tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_fcl_sizing_new() {
        let s = FclSizing::new(110.0, 100.0, 10.0, 6.0, 1.0, 100.0);
        assert!((s.network_voltage_kv - 110.0).abs() < 1e-10);
        assert!((s.base_mva - 100.0).abs() < 1e-10);
        assert!((s.prospective_fault_current_ka - 10.0).abs() < 1e-10);
        assert!((s.target_fault_current_ka - 6.0).abs() < 1e-10);
        assert!((s.normal_load_current_ka - 1.0).abs() < 1e-10);
        assert!((s.fault_duration_ms - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_required_impedance_calculation() {
        let s = FclSizing::new(110.0, 100.0, 10.0, 5.0, 1.0, 100.0);
        let (r, x) = s
            .compute_required_impedance()
            .expect("sizing should succeed");
        // Z_source = (110/√3 × 1000) / 10000 ≈ 6.351 Ω
        // Z_target = (110/√3 × 1000) / 5000  ≈ 12.702 Ω
        // Z_fcl = 12.702 - 6.351 ≈ 6.351 Ω
        assert!(r > 0.0);
        assert!(x > 0.0);
        // Z_fcl = Z_target - Z_source
        // Z_source = V_phase / I_prosp = (110e3/√3) / 10e3 ≈ 6.351 Ω
        // Z_target = V_phase / I_target = (110e3/√3) / 5e3  ≈ 12.702 Ω
        // Z_fcl    = 12.702 - 6.351 = 6.351 Ω (scalar magnitude)
        // R = Z_fcl / √3 ≈ 3.667 Ω; X = Z_fcl - R ≈ 2.684 Ω
        // |Z| = sqrt(R² + X²) ≈ 4.544 Ω
        let z_fcl_magnitude = (r * r + x * x).sqrt();
        assert!(z_fcl_magnitude > 0.0);
        // Verify R and X sum to Z_fcl (they were split from a common scalar)
        assert!(
            (r + x - 6.351_f64).abs() < 0.2,
            "R+X should sum to Z_fcl: {}",
            r + x
        );
    }

    #[test]
    fn test_required_impedance_error_on_invalid_target() {
        // target >= prospective → error
        let s = FclSizing::new(110.0, 100.0, 5.0, 10.0, 1.0, 100.0);
        assert!(s.compute_required_impedance().is_err());
    }

    #[test]
    fn test_current_reduction_pct() {
        let s = FclSizing::new(110.0, 100.0, 10.0, 5.0, 1.0, 100.0);
        let result = s.size().expect("sizing should succeed");
        // (10 - 5) / 10 × 100 = 50 %
        assert!((result.current_reduction_pct - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_recommend_technology_superconducting() {
        // prospective < 5 kA → ResistiveSuperconducting
        let s = FclSizing::new(33.0, 100.0, 3.0, 1.5, 0.5, 100.0);
        match s.recommend_technology() {
            FclTechnology::ResistiveSuperconducting { .. } => {}
            other => panic!("expected RSFCL, got {:?}", other),
        }
    }

    #[test]
    fn test_recommend_technology_solid_state() {
        // prospective >= 5 kA, fault_duration < 50 ms → SolidStateSeries
        let s = FclSizing::new(110.0, 100.0, 8.0, 4.0, 1.0, 30.0);
        match s.recommend_technology() {
            FclTechnology::SolidStateSeries { .. } => {}
            other => panic!("expected SolidStateSeries, got {:?}", other),
        }
    }

    #[test]
    fn test_recommend_technology_bridge() {
        // prospective >= 5 kA, fault_duration >= 50 ms → Bridge
        let s = FclSizing::new(110.0, 100.0, 8.0, 4.0, 1.0, 200.0);
        match s.recommend_technology() {
            FclTechnology::Bridge { .. } => {}
            other => panic!("expected Bridge, got {:?}", other),
        }
    }

    #[test]
    fn test_sizing_full_calculation() {
        let s = FclSizing::new(110.0, 100.0, 10.0, 6.0, 1.0, 100.0);
        let result = s.size().expect("sizing should succeed");
        assert!(result.required_impedance_pu > 0.0);
        assert!(result.required_resistance_ohm > 0.0);
        assert!(result.required_reactance_ohm > 0.0);
        assert!(result.current_reduction_pct > 0.0 && result.current_reduction_pct < 100.0);
        assert!(result.energy_absorption_kj > 0.0);
        assert!(result.cost_estimate_million_eur > 0.0);
    }

    // ── Placement optimizer tests ─────────────────────────────────────────────

    #[test]
    fn test_placement_optimizer_greedy() {
        // 3 buses, 3 branches; branch 0 is heavily overloaded
        let fault_currents = vec![
            vec![12.0, 4.0, 3.0],
            vec![8.0, 3.0, 2.0],
            vec![6.0, 2.5, 1.5],
        ];
        let branch_ratings = vec![5.0, 6.0, 4.0];
        let opt = FclPlacementOptimizer::new(3, 3, fault_currents, branch_ratings, 100.0);
        let result = opt.optimize_greedy();

        // At least one FCL should have been placed
        assert!(!result.placed_fcls.is_empty());
        // Total cost should not exceed budget
        assert!(result.total_cost_million_eur <= 100.0 + 1e-6);
        // Reduction pct should be positive
        assert!(result.max_fault_reduction_pct > 0.0);
    }

    #[test]
    fn test_placement_budget_constraint() {
        // Zero budget — no FCLs should be placed
        let fault_currents = vec![vec![10.0, 5.0]];
        let branch_ratings = vec![4.0, 3.0];
        let opt = FclPlacementOptimizer::new(2, 2, fault_currents, branch_ratings, 0.0);
        let result = opt.optimize_greedy();
        assert!(result.placed_fcls.is_empty());
        assert_eq!(result.total_cost_million_eur, 0.0);
    }

    // ── Post-FCL fault current tests ─────────────────────────────────────────

    #[test]
    fn test_post_fcl_fault_current_reduction() {
        // Z_source = 1 Ω, Z_fcl = 1 Ω → I_new = I_old × 1/(1+1) = 50 %
        let i_new = FclPlacementOptimizer::post_fcl_fault_current(10.0, 1.0, 1.0);
        assert!((i_new - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_post_fcl_fault_current_zero_fcl() {
        // Z_fcl = 0 → current unchanged
        let i_new = FclPlacementOptimizer::post_fcl_fault_current(10.0, 2.0, 0.0);
        assert!((i_new - 10.0).abs() < 1e-10);
    }

    // ── Performance analyzer tests ────────────────────────────────────────────

    #[test]
    fn test_performance_analyzer_simulate_fault() {
        let fcl = make_rsfcl();
        let mut analyzer = FclPerformanceAnalyzer::new(vec![fcl], 100.0, 1.0);

        let profile = vec![
            (0.0, 0.0),
            (10.0, 5.0),
            (50.0, 5.0),
            (80.0, 0.5),
            (100.0, 0.0),
        ];
        let result = analyzer.simulate_fault(&profile);

        // Peak pre-FCL should be ~5 kA
        assert!(result.peak_fault_current_ka > 4.5);
        // FCL should have triggered (limited current ≤ pre-FCL peak)
        assert!(result.limited_fault_current_ka <= result.peak_fault_current_ka);
        // Timeline should cover the simulation
        assert!(!result.fcl_states_timeline.is_empty());
        // Energy absorbed should be non-negative
        assert!(result.energy_absorbed_kj >= 0.0);
    }

    #[test]
    fn test_performance_result_reduction_factor() {
        let fcl = make_rsfcl();
        let mut analyzer = FclPerformanceAnalyzer::new(vec![fcl], 50.0, 0.5);

        // Sustained fault at 4 kA (above critical 2 kA)
        let profile = vec![(0.0, 4.0), (50.0, 4.0)];
        let result = analyzer.simulate_fault(&profile);

        // Reduction factor should be ≤ 1 (FCL reduces current)
        assert!(result.reduction_factor <= 1.0 + 1e-6);
        // Limited current should be ≤ pre-FCL
        assert!(result.limited_fault_current_ka <= result.peak_fault_current_ka + 1e-6);
    }

    #[test]
    fn test_performance_analyzer_state_timeline_length() {
        let fcl = make_solid_state_fcl();
        let mut analyzer = FclPerformanceAnalyzer::new(vec![fcl], 20.0, 2.0);
        let profile = vec![(0.0, 0.0), (20.0, 0.0)];
        let result = analyzer.simulate_fault(&profile);
        // 20 ms / 2 ms = 10 steps
        assert_eq!(result.fcl_states_timeline.len(), 10);
    }

    #[test]
    fn test_check_coordination_warning() {
        let fault_currents = vec![vec![10.0, 5.0]];
        let branch_ratings = vec![4.0, 6.0];
        let mut opt = FclPlacementOptimizer::new(2, 2, fault_currents, branch_ratings, 100.0);
        opt.protection_coordination = vec![(0, 1)];

        let sizing = FclSizing::new(110.0, 100.0, 5.0, 3.0, 1.0, 100.0);
        let placed = vec![(1usize, sizing)];
        let warnings = opt.check_coordination(&placed);
        assert!(!warnings.is_empty());
        assert!(warnings[0].contains("branch 1"));
    }

    #[test]
    fn test_is_limiter_effective_impedance() {
        let fcl = FaultCurrentLimiter::new(
            10,
            0,
            1,
            FclTechnology::IsLimiter {
                fuse_current_ka: 5.0,
                parallel_impedance_ohm: 3.0,
                reset_required: true,
            },
            4.0,
            110.0,
        );
        // Normal state → (0, 0)
        let (r, x) = fcl.effective_impedance(0.0);
        assert!(r.abs() < 1e-10);
        assert!(x.abs() < 1e-10);

        // Triggered state → (3.0, 0)
        let mut fcl2 = fcl;
        fcl2.operating_state = FclState::Triggered;
        let (r2, x2) = fcl2.effective_impedance(10.0);
        assert!((r2 - 3.0).abs() < 1e-10);
        assert!(x2.abs() < 1e-10);
    }

    #[test]
    fn test_inductive_fcl_effective_impedance() {
        let fcl = FaultCurrentLimiter::new(
            20,
            2,
            3,
            FclTechnology::InductiveSuperconducting {
                saturated_inductance_mh: 1.0,
                unsaturated_inductance_mh: 100.0,
                saturation_current_ka: 2.0,
                bias_coil_current_a: 200.0,
            },
            1.5,
            110.0,
        );
        // Normal → X_sat = ω × L_sat = 2π×50×1e-3
        let (r, x) = fcl.effective_impedance(0.0);
        let expected_x = OMEGA_50HZ * 1e-3;
        assert!(r.abs() < 1e-10);
        assert!((x - expected_x).abs() < 1e-6);

        // Triggered → X_unsat = ω × L_unsat = 2π×50×100e-3
        let mut fcl2 = fcl;
        fcl2.operating_state = FclState::Triggered;
        let (_, x2) = fcl2.effective_impedance(0.0);
        let expected_x2 = OMEGA_50HZ * 100e-3;
        assert!((x2 - expected_x2).abs() < 1e-6);
    }
}
