//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

use super::types_4::BusTimeSeries;

/// Classification of a bus-level time-series injection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BusTimeSeriesType {
    /// Passive demand load (sign: positive = consuming).
    Load,
    /// Solar PV generator.
    SolarGeneration {
        /// Installed capacity `MW`.
        installed_mw: f64,
    },
    /// Wind generator.
    WindGeneration {
        /// Installed capacity `MW`.
        installed_mw: f64,
    },
    /// Battery / pumped-hydro storage.
    /// Convention: positive = discharge (inject to grid), negative = charge.
    Storage {
        /// If `true`, charging power is represented as negative values in the profile.
        charge_negative: bool,
    },
    /// Run-of-river / dispatchable hydro generator.
    HydroGeneration,
    /// Fixed schedule — neither load nor generator classification applies.
    FixedInjection,
}
impl BusTimeSeriesType {
    /// Returns `true` if this series represents a renewable source.
    pub fn is_renewable(&self) -> bool {
        matches!(
            self,
            Self::SolarGeneration { .. } | Self::WindGeneration { .. }
        )
    }
    /// Returns `true` if this series represents a storage unit.
    pub fn is_storage(&self) -> bool {
        matches!(self, Self::Storage { .. })
    }
}
/// Lightweight network description used by the time-series engine.
///
/// The engine uses an explicit conductance/susceptance matrix rather than
/// the full `PowerNetwork` struct so that it can be constructed independently
/// (e.g. from reduced equivalents or synthetic test networks).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesNetwork {
    /// Number of buses.
    pub n_buses: usize,
    /// Nodal conductance matrix G [n×n], real part of Y-bus `pu`.
    pub g_matrix: Vec<Vec<f64>>,
    /// Nodal susceptance matrix B [n×n], imaginary part of Y-bus `pu`.
    pub b_matrix: Vec<Vec<f64>>,
    /// Bus-level time-series injections (loads and renewables).
    pub bus_series: Vec<BusTimeSeries>,
    /// Conventional generator dispatch profiles.
    pub generators: Vec<GeneratorProfile>,
    /// Thermal rating per branch `MVA`.
    pub branch_ratings_mva: Vec<f64>,
    /// Branch connectivity: `(from_bus, to_bus)` indices (0-based).
    pub branches: Vec<(usize, usize)>,
    /// Slack bus index (0-based).
    pub slack_bus: usize,
    /// System base `MVA`.
    pub base_mva: f64,
}
impl TimeSeriesNetwork {
    /// Validate basic consistency of the network description.
    pub fn validate(&self) -> Result<()> {
        if self.n_buses == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "network must have at least one bus".into(),
            ));
        }
        if self.g_matrix.len() != self.n_buses || self.b_matrix.len() != self.n_buses {
            return Err(OxiGridError::InvalidNetwork(
                "Y-bus matrix dimensions must equal n_buses".into(),
            ));
        }
        if self.slack_bus >= self.n_buses {
            return Err(OxiGridError::InvalidNetwork(
                "slack_bus index out of range".into(),
            ));
        }
        if self.branch_ratings_mva.len() != self.branches.len() {
            return Err(OxiGridError::InvalidNetwork(
                "branch_ratings_mva length must equal branches length".into(),
            ));
        }
        Ok(())
    }
    /// Build the DC susceptance matrix B' from the nodal B matrix,
    /// returning only the off-diagonal susceptances as branch reactances.
    /// B'_ij = -B_ij for i≠j; B'_ii = sum_j B_ij for i=j.
    /// (B must already be the susceptance sub-matrix, i.e. imaginary Y-bus.)
    #[allow(clippy::needless_range_loop)]
    pub(super) fn build_b_prime(&self) -> Vec<Vec<f64>> {
        let n = self.n_buses;
        let mut bp = vec![vec![0.0_f64; n]; n];
        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                let bij = self.b_matrix[i][j];
                if bij.abs() > 1e-12 {
                    let inv_x = -bij;
                    bp[i][j] -= inv_x;
                    bp[i][i] += inv_x;
                }
            }
        }
        bp
    }
}
/// Aggregated statistics over the entire simulation horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesStatistics {
    /// Total number of timesteps simulated.
    pub n_timesteps: usize,
    /// Number of timesteps that converged.
    pub n_converged: usize,
    /// Fraction of converged timesteps (0–1).
    pub convergence_rate: f64,
    /// Maximum bus voltage across all buses and timesteps `pu`.
    pub max_voltage_pu: f64,
    /// Minimum bus voltage across all buses and timesteps `pu`.
    pub min_voltage_pu: f64,
    /// Average bus voltage across all buses and all timesteps `pu`.
    pub avg_voltage_pu: f64,
    /// Peak total load observed `MW`.
    pub peak_load_mw: f64,
    /// Average total load `MW`.
    pub avg_load_mw: f64,
    /// Load factor = avg_load / peak_load (0–1).
    pub load_factor: f64,
    /// Total energy consumed `TWh`.
    pub total_energy_twh: f64,
    /// Renewable generation as percentage of total generation [%].
    pub renewable_fraction_pct: f64,
    /// Total curtailed renewable energy `MWh`.
    pub total_curtailment_mwh: f64,
    /// Total system losses `MWh`.
    pub total_losses_mwh: f64,
    /// Maximum single-branch loading observed [%].
    pub max_branch_loading_pct: f64,
    /// Average branch loading [%].
    pub avg_branch_loading_pct: f64,
    /// Number of timesteps with at least one overloaded branch.
    pub n_overload_hours: usize,
    /// Number of timesteps with at least one bus voltage violation.
    pub n_voltage_violation_hours: usize,
    /// Estimated hosting capacity `MW` — set by `estimate_hosting_capacity`, else 0.0.
    pub hosting_capacity_estimate_mw: f64,
}
/// Full simulation output.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesResult {
    /// Per-timestep power flow results.
    pub timestep_results: Vec<TimeStepResult>,
    /// Aggregated statistics.
    pub statistics: TimeSeriesStatistics,
    /// Approximate wall-clock time for the simulation `s`.
    pub duration_s: f64,
}
/// Configuration for the time-series simulation engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesConfig {
    /// Total number of timesteps to simulate.
    pub n_timesteps: usize,
    /// Time resolution (controls `dt_hours`).
    pub resolution: TimeResolution,
    /// Lower voltage limit `pu` — below this is a violation. Default 0.95.
    pub voltage_lower_pu: f64,
    /// Upper voltage limit `pu` — above this is a violation. Default 1.05.
    pub voltage_upper_pu: f64,
    /// DC power flow iteration cap per timestep (informational; DC is 1-shot).
    pub max_pf_iterations: usize,
    /// Power flow convergence tolerance `pu`. Default 1e-4.
    pub pf_tolerance: f64,
    /// When `true`, renewable generation is curtailed to resolve over-voltage.
    pub enable_curtailment: bool,
    /// Storage dispatch heuristic.
    pub storage_dispatch_strategy: StorageStrategy,
}
/// Per-unit storage state tracked across timesteps.
#[derive(Debug, Clone)]
pub(super) struct StorageUnit {
    /// Index into `network.bus_series` for this storage entry.
    pub(super) series_idx: usize,
    /// Bus index.
    pub(super) bus_id: usize,
    /// Current state-of-charge (0–1).
    pub(super) soc: f64,
    /// Energy capacity `MWh`.
    pub(super) capacity_mwh: f64,
    /// Maximum charge/discharge rate `MW`.
    pub(super) power_mw: f64,
}
impl StorageUnit {
    /// Charge efficiency (round-trip ≈ 90 %).
    pub(super) const ETA_CHARGE: f64 = 0.95;
    /// Discharge efficiency.
    pub(super) const ETA_DISCHARGE: f64 = 0.95;
    /// Update SoC for `dt_hours` at power `p_mw` (+discharge, –charge).
    /// Returns actual power applied (clamped).
    pub(super) fn apply_power(&mut self, p_mw: f64, dt_hours: f64) -> f64 {
        let p_clamped = p_mw.clamp(-self.power_mw, self.power_mw);
        if p_clamped >= 0.0 {
            let energy_out = p_clamped * dt_hours / Self::ETA_DISCHARGE;
            let max_out = self.soc * self.capacity_mwh;
            let actual_energy = energy_out.min(max_out);
            self.soc -= actual_energy / self.capacity_mwh;
            self.soc = self.soc.clamp(0.0, 1.0);
            actual_energy * Self::ETA_DISCHARGE / dt_hours
        } else {
            let energy_in = (-p_clamped) * dt_hours * Self::ETA_CHARGE;
            let max_in = (1.0 - self.soc) * self.capacity_mwh;
            let actual_energy = energy_in.min(max_in);
            self.soc += actual_energy / self.capacity_mwh;
            self.soc = self.soc.clamp(0.0, 1.0);
            -(actual_energy / Self::ETA_CHARGE / dt_hours)
        }
    }
}
/// Power flow result for a single timestep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeStepResult {
    /// Timestep index (0-based).
    pub timestep: usize,
    /// Elapsed simulation time `h` from t = 0.
    pub time_hours: f64,
    /// Whether the DC power flow converged (always `true` for DC unless singular B').
    pub converged: bool,
    /// Bus voltage magnitudes `pu` — flat (1.0) for DC, Q-adjusted for estimate.
    pub voltage_magnitude: Vec<f64>,
    /// Bus voltage angles `rad`.
    pub voltage_angle: Vec<f64>,
    /// Branch loading as percentage of thermal rating [%].
    pub branch_loading_pct: Vec<f64>,
    /// Total conventional + renewable generation `MW`.
    pub total_generation_mw: f64,
    /// Total demand load `MW`.
    pub total_load_mw: f64,
    /// Approximate DC losses `MW` (= 0 for lossless DC).
    pub total_losses_mw: f64,
    /// Renewable (solar + wind) generation `MW` after curtailment.
    pub renewable_generation_mw: f64,
    /// Renewable curtailment applied this timestep `MW`.
    pub renewable_curtailment_mw: f64,
    /// State-of-charge per storage unit (0–1).
    pub storage_soc: Vec<f64>,
    /// Indices of branches with loading > 100 %.
    pub overloaded_branches: Vec<usize>,
    /// Buses with voltage violations: `(bus_idx, voltage_pu)`.
    pub voltage_violations: Vec<(usize, f64)>,
}
/// Scheduled dispatch profile for a conventional or storage generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorProfile {
    /// Generator index (informational).
    pub generator_id: usize,
    /// Bus index (0-based) where this generator is connected.
    pub bus: usize,
    /// Active power dispatch schedule `MW`, one value per timestep.
    pub p_dispatch_mw: Vec<f64>,
    /// Reactive power dispatch schedule `MVAr`, one value per timestep.
    pub q_dispatch_mvar: Vec<f64>,
    /// Maximum active power output `MW`.
    pub p_max_mw: f64,
    /// Minimum active power output `MW` (may be negative for storage charging).
    pub p_min_mw: f64,
    /// Energy cost [$/MWh] (used by price-arbitrage storage strategy).
    pub cost_per_mwh: f64,
}
impl GeneratorProfile {
    /// Active power dispatch at timestep `t`, clamped to \[p_min, p_max\].
    pub fn p_at(&self, t: usize) -> f64 {
        self.p_dispatch_mw
            .get(t)
            .copied()
            .unwrap_or(0.0)
            .clamp(self.p_min_mw, self.p_max_mw)
    }
    /// Reactive power dispatch at timestep `t`.
    pub fn q_at(&self, t: usize) -> f64 {
        self.q_dispatch_mvar.get(t).copied().unwrap_or(0.0)
    }
}
/// Temporal resolution of the quasi-static simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeResolution {
    /// 15-minute intervals — 96 steps per day.
    FifteenMinutes,
    /// 30-minute intervals — 48 steps per day.
    HalfHourly,
    /// 1-hour intervals — 24 steps per day.
    Hourly,
    /// 24-hour (daily) intervals — 1 step per day.
    Daily,
}
impl TimeResolution {
    /// Number of timesteps in one calendar day.
    pub fn steps_per_day(&self) -> usize {
        match self {
            Self::FifteenMinutes => 96,
            Self::HalfHourly => 48,
            Self::Hourly => 24,
            Self::Daily => 1,
        }
    }
    /// Duration of one timestep in hours.
    pub fn dt_hours(&self) -> f64 {
        match self {
            Self::FifteenMinutes => 0.25,
            Self::HalfHourly => 0.5,
            Self::Hourly => 1.0,
            Self::Daily => 24.0,
        }
    }
}
/// Dispatch heuristic for storage units.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StorageStrategy {
    /// Discharge when total load exceeds the threshold; charge otherwise.
    PeakShaving {
        /// Load threshold `MW` above which storage discharges.
        threshold_mw: f64,
    },
    /// Discharge at high prices; charge at low prices (vs. median price).
    PriceArbitrage {
        /// Market price profile [$/MWh], one per timestep.
        price_profile: Vec<f64>,
    },
    /// Inject reactive power to support voltage toward the target.
    VoltageSupport {
        /// Voltage setpoint `pu`.
        target_pu: f64,
    },
    /// Use the `p_dispatch_mw` from `GeneratorProfile` directly.
    ScheduledDispatch,
}
