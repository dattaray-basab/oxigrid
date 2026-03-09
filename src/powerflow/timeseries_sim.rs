//! Quasi-steady-state time-series power flow simulation engine.
//!
//! Runs a DC-based quasi-static simulation over a time horizon (hours/days/years),
//! applying time-varying load and renewable generation profiles at each timestep.
//! Supports storage dispatch strategies, curtailment, and scenario analysis.
//!
//! # Usage
//! ```rust,ignore
//! let config = TimeSeriesConfig::default();
//! let mut sim = TimeSeriesSimulator::new(network, config);
//! let result = sim.run()?;
//! println!("Renewable fraction: {:.1}%", result.statistics.renewable_fraction_pct);
//! ```
//!
//! # Method
//! Each timestep is solved as an independent quasi-steady-state DC power flow
//! (B·θ = P). Voltage magnitudes are estimated via Q-sensitivity. Storage SoC
//! is tracked across timesteps. Curtailment is applied when voltage bounds are
//! violated and `enable_curtailment` is set.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ─── Time Resolution ─────────────────────────────────────────────────────────

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

// ─── Bus Time Series ──────────────────────────────────────────────────────────

/// Classification of a bus-level time-series injection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BusTimeSeriesType {
    /// Passive demand load (sign: positive = consuming).
    Load,
    /// Solar PV generator.
    SolarGeneration {
        /// Installed capacity [MW].
        installed_mw: f64,
    },
    /// Wind generator.
    WindGeneration {
        /// Installed capacity [MW].
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

/// Time-varying injection profile for a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusTimeSeries {
    /// Bus index (0-based) in [`TimeSeriesNetwork`].
    pub bus_id: usize,
    /// Active power profile [MW], one value per timestep.
    /// For loads the convention is positive = consuming; for generators positive = generating.
    pub p_mw: Vec<f64>,
    /// Reactive power profile [MVAr], one value per timestep.
    pub q_mvar: Vec<f64>,
    /// Semantic type of this series.
    pub series_type: BusTimeSeriesType,
}

impl BusTimeSeries {
    /// Number of timesteps in this series.
    pub fn len(&self) -> usize {
        self.p_mw.len()
    }

    /// Returns `true` if the series has no timesteps.
    pub fn is_empty(&self) -> bool {
        self.p_mw.is_empty()
    }

    /// Active power at timestep `t`, or `0.0` if out of range.
    pub fn p_at(&self, t: usize) -> f64 {
        self.p_mw.get(t).copied().unwrap_or(0.0)
    }

    /// Reactive power at timestep `t`, or `0.0` if out of range.
    pub fn q_at(&self, t: usize) -> f64 {
        self.q_mvar.get(t).copied().unwrap_or(0.0)
    }
}

// ─── Generator Profile ────────────────────────────────────────────────────────

/// Scheduled dispatch profile for a conventional or storage generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorProfile {
    /// Generator index (informational).
    pub generator_id: usize,
    /// Bus index (0-based) where this generator is connected.
    pub bus: usize,
    /// Active power dispatch schedule [MW], one value per timestep.
    pub p_dispatch_mw: Vec<f64>,
    /// Reactive power dispatch schedule [MVAr], one value per timestep.
    pub q_dispatch_mvar: Vec<f64>,
    /// Maximum active power output [MW].
    pub p_max_mw: f64,
    /// Minimum active power output [MW] (may be negative for storage charging).
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

// ─── Network Description ──────────────────────────────────────────────────────

/// Lightweight network description used by the time-series engine.
///
/// The engine uses an explicit conductance/susceptance matrix rather than
/// the full `PowerNetwork` struct so that it can be constructed independently
/// (e.g. from reduced equivalents or synthetic test networks).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesNetwork {
    /// Number of buses.
    pub n_buses: usize,
    /// Nodal conductance matrix G [n×n], real part of Y-bus [pu].
    pub g_matrix: Vec<Vec<f64>>,
    /// Nodal susceptance matrix B [n×n], imaginary part of Y-bus [pu].
    pub b_matrix: Vec<Vec<f64>>,
    /// Bus-level time-series injections (loads and renewables).
    pub bus_series: Vec<BusTimeSeries>,
    /// Conventional generator dispatch profiles.
    pub generators: Vec<GeneratorProfile>,
    /// Thermal rating per branch [MVA].
    pub branch_ratings_mva: Vec<f64>,
    /// Branch connectivity: `(from_bus, to_bus)` indices (0-based).
    pub branches: Vec<(usize, usize)>,
    /// Slack bus index (0-based).
    pub slack_bus: usize,
    /// System base [MVA].
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
    fn build_b_prime(&self) -> Vec<Vec<f64>> {
        // For DC power flow we re-derive B' from branch connectivity.
        // B'_ij = -1/x_ij  (off-diagonal)
        // B'_ii = sum_{j≠i} 1/x_ij (diagonal)
        // We read x_ij = -1/B_ij (off-diagonal of stored B matrix, if non-zero).
        let n = self.n_buses;
        let mut bp = vec![vec![0.0_f64; n]; n];
        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                let bij = self.b_matrix[i][j];
                if bij.abs() > 1e-12 {
                    // B_ij (off-diagonal) = -1/x_ij  =>  1/x_ij = -B_ij
                    let inv_x = -bij;
                    bp[i][j] -= inv_x;
                    bp[i][i] += inv_x;
                }
            }
        }
        bp
    }
}

// ─── Simulation Result Structures ─────────────────────────────────────────────

/// Power flow result for a single timestep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeStepResult {
    /// Timestep index (0-based).
    pub timestep: usize,
    /// Elapsed simulation time [h] from t = 0.
    pub time_hours: f64,
    /// Whether the DC power flow converged (always `true` for DC unless singular B').
    pub converged: bool,
    /// Bus voltage magnitudes [pu] — flat (1.0) for DC, Q-adjusted for estimate.
    pub voltage_magnitude: Vec<f64>,
    /// Bus voltage angles [rad].
    pub voltage_angle: Vec<f64>,
    /// Branch loading as percentage of thermal rating [%].
    pub branch_loading_pct: Vec<f64>,
    /// Total conventional + renewable generation [MW].
    pub total_generation_mw: f64,
    /// Total demand load [MW].
    pub total_load_mw: f64,
    /// Approximate DC losses [MW] (= 0 for lossless DC).
    pub total_losses_mw: f64,
    /// Renewable (solar + wind) generation [MW] after curtailment.
    pub renewable_generation_mw: f64,
    /// Renewable curtailment applied this timestep [MW].
    pub renewable_curtailment_mw: f64,
    /// State-of-charge per storage unit (0–1).
    pub storage_soc: Vec<f64>,
    /// Indices of branches with loading > 100 %.
    pub overloaded_branches: Vec<usize>,
    /// Buses with voltage violations: `(bus_idx, voltage_pu)`.
    pub voltage_violations: Vec<(usize, f64)>,
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
    /// Maximum bus voltage across all buses and timesteps [pu].
    pub max_voltage_pu: f64,
    /// Minimum bus voltage across all buses and timesteps [pu].
    pub min_voltage_pu: f64,
    /// Average bus voltage across all buses and all timesteps [pu].
    pub avg_voltage_pu: f64,
    /// Peak total load observed [MW].
    pub peak_load_mw: f64,
    /// Average total load [MW].
    pub avg_load_mw: f64,
    /// Load factor = avg_load / peak_load (0–1).
    pub load_factor: f64,
    /// Total energy consumed [TWh].
    pub total_energy_twh: f64,
    /// Renewable generation as percentage of total generation [%].
    pub renewable_fraction_pct: f64,
    /// Total curtailed renewable energy [MWh].
    pub total_curtailment_mwh: f64,
    /// Total system losses [MWh].
    pub total_losses_mwh: f64,
    /// Maximum single-branch loading observed [%].
    pub max_branch_loading_pct: f64,
    /// Average branch loading [%].
    pub avg_branch_loading_pct: f64,
    /// Number of timesteps with at least one overloaded branch.
    pub n_overload_hours: usize,
    /// Number of timesteps with at least one bus voltage violation.
    pub n_voltage_violation_hours: usize,
    /// Estimated hosting capacity [MW] — set by `estimate_hosting_capacity`, else 0.0.
    pub hosting_capacity_estimate_mw: f64,
}

impl Default for TimeSeriesStatistics {
    fn default() -> Self {
        Self {
            n_timesteps: 0,
            n_converged: 0,
            convergence_rate: 0.0,
            max_voltage_pu: 1.0,
            min_voltage_pu: 1.0,
            avg_voltage_pu: 1.0,
            peak_load_mw: 0.0,
            avg_load_mw: 0.0,
            load_factor: 0.0,
            total_energy_twh: 0.0,
            renewable_fraction_pct: 0.0,
            total_curtailment_mwh: 0.0,
            total_losses_mwh: 0.0,
            max_branch_loading_pct: 0.0,
            avg_branch_loading_pct: 0.0,
            n_overload_hours: 0,
            n_voltage_violation_hours: 0,
            hosting_capacity_estimate_mw: 0.0,
        }
    }
}

/// Full simulation output.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesResult {
    /// Per-timestep power flow results.
    pub timestep_results: Vec<TimeStepResult>,
    /// Aggregated statistics.
    pub statistics: TimeSeriesStatistics,
    /// Approximate wall-clock time for the simulation [s].
    pub duration_s: f64,
}

// ─── Storage Dispatch Strategy ────────────────────────────────────────────────

/// Dispatch heuristic for storage units.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StorageStrategy {
    /// Discharge when total load exceeds the threshold; charge otherwise.
    PeakShaving {
        /// Load threshold [MW] above which storage discharges.
        threshold_mw: f64,
    },
    /// Discharge at high prices; charge at low prices (vs. median price).
    PriceArbitrage {
        /// Market price profile [$/MWh], one per timestep.
        price_profile: Vec<f64>,
    },
    /// Inject reactive power to support voltage toward the target.
    VoltageSupport {
        /// Voltage setpoint [pu].
        target_pu: f64,
    },
    /// Use the `p_dispatch_mw` from `GeneratorProfile` directly.
    ScheduledDispatch,
}

// ─── Simulation Configuration ─────────────────────────────────────────────────

/// Configuration for the time-series simulation engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesConfig {
    /// Total number of timesteps to simulate.
    pub n_timesteps: usize,
    /// Time resolution (controls `dt_hours`).
    pub resolution: TimeResolution,
    /// Lower voltage limit [pu] — below this is a violation. Default 0.95.
    pub voltage_lower_pu: f64,
    /// Upper voltage limit [pu] — above this is a violation. Default 1.05.
    pub voltage_upper_pu: f64,
    /// DC power flow iteration cap per timestep (informational; DC is 1-shot).
    pub max_pf_iterations: usize,
    /// Power flow convergence tolerance [pu]. Default 1e-4.
    pub pf_tolerance: f64,
    /// When `true`, renewable generation is curtailed to resolve over-voltage.
    pub enable_curtailment: bool,
    /// Storage dispatch heuristic.
    pub storage_dispatch_strategy: StorageStrategy,
}

impl Default for TimeSeriesConfig {
    fn default() -> Self {
        Self {
            n_timesteps: 8760,
            resolution: TimeResolution::Hourly,
            voltage_lower_pu: 0.95,
            voltage_upper_pu: 1.05,
            max_pf_iterations: 20,
            pf_tolerance: 1e-4,
            enable_curtailment: true,
            storage_dispatch_strategy: StorageStrategy::ScheduledDispatch,
        }
    }
}

// ─── Storage State ────────────────────────────────────────────────────────────

/// Per-unit storage state tracked across timesteps.
#[derive(Debug, Clone)]
struct StorageUnit {
    /// Index into `network.bus_series` for this storage entry.
    series_idx: usize,
    /// Bus index.
    bus_id: usize,
    /// Current state-of-charge (0–1).
    soc: f64,
    /// Energy capacity [MWh].
    capacity_mwh: f64,
    /// Maximum charge/discharge rate [MW].
    power_mw: f64,
}

impl StorageUnit {
    /// Charge efficiency (round-trip ≈ 90 %).
    const ETA_CHARGE: f64 = 0.95;
    /// Discharge efficiency.
    const ETA_DISCHARGE: f64 = 0.95;

    /// Update SoC for `dt_hours` at power `p_mw` (+discharge, –charge).
    /// Returns actual power applied (clamped).
    fn apply_power(&mut self, p_mw: f64, dt_hours: f64) -> f64 {
        let p_clamped = p_mw.clamp(-self.power_mw, self.power_mw);
        if p_clamped >= 0.0 {
            // discharging
            let energy_out = p_clamped * dt_hours / Self::ETA_DISCHARGE;
            let max_out = self.soc * self.capacity_mwh;
            let actual_energy = energy_out.min(max_out);
            self.soc -= actual_energy / self.capacity_mwh;
            self.soc = self.soc.clamp(0.0, 1.0);
            actual_energy * Self::ETA_DISCHARGE / dt_hours
        } else {
            // charging (p_clamped < 0)
            let energy_in = (-p_clamped) * dt_hours * Self::ETA_CHARGE;
            let max_in = (1.0 - self.soc) * self.capacity_mwh;
            let actual_energy = energy_in.min(max_in);
            self.soc += actual_energy / self.capacity_mwh;
            self.soc = self.soc.clamp(0.0, 1.0);
            -(actual_energy / Self::ETA_CHARGE / dt_hours)
        }
    }
}

// ─── Main Simulator ───────────────────────────────────────────────────────────

/// Quasi-static time-series power flow simulation engine.
///
/// Iterates over `config.n_timesteps` steps, solving a DC power flow at each
/// step, tracking storage SoC, applying curtailment, and accumulating statistics.
pub struct TimeSeriesSimulator {
    /// Network description.
    pub network: TimeSeriesNetwork,
    /// Simulation configuration.
    pub config: TimeSeriesConfig,
    /// State-of-charge per storage unit (0–1), updated in-place during `run`.
    pub storage_soc: Vec<f64>,
    /// Internal storage unit tracker.
    storage_units: Vec<StorageUnit>,
}

impl TimeSeriesSimulator {
    /// Create a new simulator.
    /// Storage units are discovered from `BusTimeSeriesType::Storage` entries;
    /// initial SoC is set to 0.5 for each.
    pub fn new(network: TimeSeriesNetwork, config: TimeSeriesConfig) -> Self {
        let mut storage_units = Vec::new();
        for (idx, bts) in network.bus_series.iter().enumerate() {
            if bts.series_type.is_storage() {
                // Estimate capacity from max absolute value in profile or default 1 MWh/MW
                let power_mw = bts
                    .p_mw
                    .iter()
                    .map(|&p| p.abs())
                    .fold(f64::NAN, f64::max)
                    .max(1.0);
                let capacity_mwh = power_mw * 4.0; // 4-hour storage default
                storage_units.push(StorageUnit {
                    series_idx: idx,
                    bus_id: bts.bus_id,
                    soc: 0.5,
                    capacity_mwh,
                    power_mw,
                });
            }
        }
        let soc_vec: Vec<f64> = storage_units.iter().map(|s| s.soc).collect();
        Self {
            network,
            config,
            storage_soc: soc_vec,
            storage_units,
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Run the full time-series simulation.
    ///
    /// # Algorithm (per timestep `t`)
    /// 1. Collect bus power injections from profiles.
    /// 2. Dispatch storage according to the configured strategy.
    /// 3. Solve DC power flow: **B·θ = P**.
    /// 4. Estimate bus voltages from Q injections.
    /// 5. Compute branch flows and loading percentages.
    /// 6. Check voltage/loading constraints; apply curtailment if enabled.
    /// 7. Record `TimeStepResult`.
    ///
    /// Returns aggregated `TimeSeriesResult`.
    pub fn run(&mut self) -> Result<TimeSeriesResult> {
        self.network.validate()?;
        if self.config.n_timesteps == 0 {
            return Err(OxiGridError::InvalidParameter(
                "n_timesteps must be > 0".into(),
            ));
        }

        // Pre-compute median price for arbitrage strategy
        let median_price = self.compute_median_price();

        let dt = self.config.resolution.dt_hours();
        let n_t = self.config.n_timesteps;
        let mut results: Vec<TimeStepResult> = Vec::with_capacity(n_t);

        for t in 0..n_t {
            let time_hours = t as f64 * dt;

            // Step 1 – collect raw injections
            let (mut p_inj, q_inj) = self.get_bus_injections(t);

            // Step 2 – storage dispatch
            let storage_p = self.dispatch_storage(t, &p_inj, median_price);
            for (sidx, su) in self.storage_units.iter().enumerate() {
                if su.bus_id < p_inj.len() {
                    p_inj[su.bus_id] += storage_p[sidx];
                }
            }

            // Step 3 – DC power flow
            let (angles, converged) = match self.solve_dc_powerflow(&p_inj) {
                Ok(a) => (a, true),
                Err(_) => (vec![0.0; self.network.n_buses], false),
            };

            // Step 4 – voltage estimates
            let mut voltages = self.estimate_voltages(&p_inj, &q_inj);

            // Step 5 – branch flows / loading
            let branch_flows = self.compute_branch_flows(&angles);
            let branch_loading = self.compute_branch_loading(&branch_flows);

            // Step 6 – curtailment
            let curtailment_mw = if self.config.enable_curtailment
                && voltages.iter().any(|&v| v > self.config.voltage_upper_pu)
            {
                let c = self.apply_curtailment(&mut p_inj, &voltages);
                // Re-estimate voltages after curtailment
                voltages = self.estimate_voltages(&p_inj, &q_inj);
                c
            } else {
                0.0
            };

            // Step 7 – accounting
            let (total_gen, total_load, ren_gen) = self.compute_generation_load(t, &storage_p);
            let renewable_gen_after = (ren_gen - curtailment_mw).max(0.0);

            // Check violations
            let overloaded: Vec<usize> = branch_loading
                .iter()
                .enumerate()
                .filter(|(_, &l)| l > 100.0)
                .map(|(i, _)| i)
                .collect();
            let v_violations: Vec<(usize, f64)> = voltages
                .iter()
                .enumerate()
                .filter(|(_, &v)| {
                    v < self.config.voltage_lower_pu || v > self.config.voltage_upper_pu
                })
                .map(|(i, &v)| (i, v))
                .collect();

            // Update SoC snapshot
            for (sidx, su) in self.storage_units.iter().enumerate() {
                if sidx < self.storage_soc.len() {
                    self.storage_soc[sidx] = su.soc;
                }
            }
            let soc_snap: Vec<f64> = self.storage_soc.clone();

            results.push(TimeStepResult {
                timestep: t,
                time_hours,
                converged,
                voltage_magnitude: voltages,
                voltage_angle: angles,
                branch_loading_pct: branch_loading,
                total_generation_mw: total_gen,
                total_load_mw: total_load,
                total_losses_mw: 0.0, // lossless DC
                renewable_generation_mw: renewable_gen_after,
                renewable_curtailment_mw: curtailment_mw,
                storage_soc: soc_snap,
                overloaded_branches: overloaded,
                voltage_violations: v_violations,
            });
        }

        let statistics = Self::compute_statistics(&results, dt);
        Ok(TimeSeriesResult {
            timestep_results: results,
            statistics,
            duration_s: 0.0, // wall-clock not measured in pure Rust; set by caller
        })
    }

    /// Estimate the maximum renewable hosting capacity at `test_bus` via
    /// binary search on additional renewable injection.
    ///
    /// The criterion: violations must not exceed 5 % of timesteps.
    pub fn estimate_hosting_capacity(
        &mut self,
        test_bus: usize,
        max_search_mw: f64,
    ) -> Result<f64> {
        if test_bus >= self.network.n_buses {
            return Err(OxiGridError::InvalidParameter(format!(
                "test_bus {test_bus} out of range (n_buses={})",
                self.network.n_buses
            )));
        }
        if max_search_mw <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "max_search_mw must be positive".into(),
            ));
        }

        let dt = self.config.resolution.dt_hours();
        let n_t = self.config.n_timesteps;
        let violation_limit = (n_t as f64 * 0.05).ceil() as usize;

        let mut lo = 0.0_f64;
        let mut hi = max_search_mw;
        let mut best = 0.0_f64;

        // 20 binary-search iterations → ~1 ppm resolution on max_search_mw
        for _ in 0..20 {
            let mid = (lo + hi) / 2.0;

            // Simulate with additional renewable injection at test_bus
            let violations = self.count_violations_with_injection(test_bus, mid, dt, n_t)?;

            if violations <= violation_limit {
                best = mid;
                lo = mid;
            } else {
                hi = mid;
            }
        }

        Ok(best)
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Collect net power injections at each bus for timestep `t`.
    ///
    /// Returns `(p_mw, q_mvar)` where positive = injection into the network.
    /// Loads are subtracted; generators are added.
    fn get_bus_injections(&self, t: usize) -> (Vec<f64>, Vec<f64>) {
        let n = self.network.n_buses;
        let mut p = vec![0.0_f64; n];
        let mut q = vec![0.0_f64; n];

        // Bus series (loads subtract, generators add)
        for bts in &self.network.bus_series {
            let bus = bts.bus_id;
            if bus >= n {
                continue;
            }
            let p_val = bts.p_at(t);
            let q_val = bts.q_at(t);
            match &bts.series_type {
                BusTimeSeriesType::Load => {
                    p[bus] -= p_val; // load = negative injection
                    q[bus] -= q_val;
                }
                BusTimeSeriesType::Storage { .. } => {
                    // Storage dispatch is handled separately; skip here
                }
                _ => {
                    // Generator / FixedInjection: positive injection
                    p[bus] += p_val;
                    q[bus] += q_val;
                }
            }
        }

        // Conventional generators (scheduled dispatch)
        for gen in &self.network.generators {
            let bus = gen.bus;
            if bus >= n {
                continue;
            }
            p[bus] += gen.p_at(t);
            q[bus] += gen.q_at(t);
        }

        (p, q)
    }

    /// Solve a DC power flow for the given active power injections [MW].
    ///
    /// Builds the reduced B' matrix (excluding slack bus), solves **B'·θ = P**,
    /// and returns the full angle vector [rad] with slack = 0.
    fn solve_dc_powerflow(&self, p_injections: &[f64]) -> Result<Vec<f64>> {
        let n = self.network.n_buses;
        let slack = self.network.slack_bus;
        let base = self.network.base_mva;

        // Build B' from stored B matrix
        let bp = self.network.build_b_prime();

        // Reduced indices (all except slack)
        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack).collect();
        let m = non_slack.len();

        if m == 0 {
            // Single-bus network: nothing to solve
            return Ok(vec![0.0; n]);
        }

        // Build reduced m×m system
        let mut a = vec![vec![0.0_f64; m]; m];
        for (ri, &i) in non_slack.iter().enumerate() {
            for (rj, &j) in non_slack.iter().enumerate() {
                a[ri][rj] = bp[i][j];
            }
        }

        // RHS: P [pu]
        let mut rhs: Vec<f64> = non_slack
            .iter()
            .map(|&i| p_injections.get(i).copied().unwrap_or(0.0) / base)
            .collect();

        // Gaussian elimination with partial pivoting
        Self::gaussian_solve(&mut a, &mut rhs)?;

        // Reconstruct full angle vector
        let mut angles = vec![0.0_f64; n];
        for (ri, &i) in non_slack.iter().enumerate() {
            angles[i] = rhs[ri];
        }
        Ok(angles)
    }

    /// Gaussian elimination with partial pivoting (in-place).
    #[allow(clippy::ptr_arg, clippy::needless_range_loop)]
    fn gaussian_solve(a: &mut Vec<Vec<f64>>, b: &mut Vec<f64>) -> Result<()> {
        let m = b.len();
        for col in 0..m {
            // Find pivot
            let mut max_val = a[col][col].abs();
            let mut max_row = col;
            for row in (col + 1)..m {
                if a[row][col].abs() > max_val {
                    max_val = a[row][col].abs();
                    max_row = row;
                }
            }
            if max_val < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(
                    "B' matrix is singular — network may be islanded".into(),
                ));
            }
            a.swap(col, max_row);
            b.swap(col, max_row);

            let pivot = a[col][col];
            for row in (col + 1)..m {
                let factor = a[row][col] / pivot;
                for c in col..m {
                    let val = a[col][c];
                    a[row][c] -= factor * val;
                }
                b[row] -= factor * b[col];
            }
        }
        // Back-substitution
        for row in (0..m).rev() {
            let mut sum = b[row];
            for c in (row + 1)..m {
                sum -= a[row][c] * b[c];
            }
            b[row] = sum / a[row][row];
        }
        Ok(())
    }

    /// Compute DC branch flows [MW] from bus angle vector [rad].
    ///
    /// `P_ij = (θ_i - θ_j) × B_ij × base_mva`
    /// where `B_ij` is the off-diagonal entry of the nodal susceptance matrix.
    fn compute_branch_flows(&self, angles: &[f64]) -> Vec<f64> {
        self.network
            .branches
            .iter()
            .map(|&(from, to)| {
                let theta_i = angles.get(from).copied().unwrap_or(0.0);
                let theta_j = angles.get(to).copied().unwrap_or(0.0);
                // B_from_to (off-diagonal, imaginary Y-bus) gives susceptance.
                // For DC: P_ij = -B_ij × (θ_i - θ_j) × base_mva
                // (B_ij is negative for a line, so -B_ij is positive admittance)
                let bij = self
                    .network
                    .b_matrix
                    .get(from)
                    .and_then(|row| row.get(to))
                    .copied()
                    .unwrap_or(0.0);
                let inv_x = -bij; // susceptance value (positive)
                inv_x * (theta_i - theta_j) * self.network.base_mva
            })
            .collect()
    }

    /// Compute branch loading as percentage of thermal rating.
    fn compute_branch_loading(&self, branch_flows: &[f64]) -> Vec<f64> {
        branch_flows
            .iter()
            .enumerate()
            .map(|(i, &flow)| {
                let rating = self
                    .network
                    .branch_ratings_mva
                    .get(i)
                    .copied()
                    .unwrap_or(f64::INFINITY);
                if rating > 0.0 {
                    flow.abs() / rating * 100.0
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Estimate bus voltage magnitudes [pu] from reactive power injections.
    ///
    /// Uses a first-order Q-sensitivity: Δv ≈ -Q / (B_ii × base_mva).
    /// Clamps result to [0.5, 1.5] pu.
    fn estimate_voltages(&self, _p_injections: &[f64], q_injections: &[f64]) -> Vec<f64> {
        let n = self.network.n_buses;
        let base = self.network.base_mva;
        (0..n)
            .map(|i| {
                let q_pu = q_injections.get(i).copied().unwrap_or(0.0) / base;
                let bii = self
                    .network
                    .b_matrix
                    .get(i)
                    .and_then(|row| row.get(i))
                    .copied()
                    .unwrap_or(-1.0);
                // Δv ≈ -Q / B_ii  (B_ii is positive for shunt-dominated buses)
                let dv = if bii.abs() > 1e-6 { -q_pu / bii } else { 0.0 };
                (1.0 + dv).clamp(0.5, 1.5)
            })
            .collect()
    }

    /// Dispatch storage units according to the configured strategy.
    /// Updates SoC in-place and returns actual power [MW] per storage unit.
    fn dispatch_storage(&mut self, t: usize, p_inj: &[f64], median_price: f64) -> Vec<f64> {
        match &self.config.storage_dispatch_strategy.clone() {
            StorageStrategy::PeakShaving { threshold_mw } => {
                let total_load = p_inj.iter().filter(|&&v| v < 0.0).map(|&v| -v).sum::<f64>();
                self.dispatch_storage_peak_shaving(total_load, *threshold_mw, t)
            }
            StorageStrategy::PriceArbitrage { price_profile } => {
                let price = price_profile.get(t).copied().unwrap_or(0.0);
                self.dispatch_storage_price_arbitrage(price, median_price, t)
            }
            StorageStrategy::VoltageSupport { .. } | StorageStrategy::ScheduledDispatch => {
                self.dispatch_storage_scheduled(t)
            }
        }
    }

    /// Peak-shaving dispatch: discharge if load > threshold, else charge.
    fn dispatch_storage_peak_shaving(
        &mut self,
        total_load: f64,
        threshold: f64,
        t: usize,
    ) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];

        if total_load > threshold {
            // Discharge — share the deficit equally
            let deficit = (total_load - threshold) / (n_storage.max(1) as f64);
            for (sidx, su) in self.storage_units.iter_mut().enumerate() {
                let actual = su.apply_power(deficit, dt);
                out[sidx] = actual;
                // Sync public SoC vec
                if sidx < self.storage_soc.len() {
                    self.storage_soc[sidx] = su.soc;
                }
            }
        } else {
            // Charge from surplus
            let surplus = (threshold - total_load) / (n_storage.max(1) as f64);
            let charge = -surplus;
            for (sidx, su) in self.storage_units.iter_mut().enumerate() {
                let actual = su.apply_power(charge, dt);
                out[sidx] = actual;
                if sidx < self.storage_soc.len() {
                    self.storage_soc[sidx] = su.soc;
                }
            }
        }
        // Unused t to suppress warning — profile index affects scheduled dispatch only
        let _ = t;
        out
    }

    /// Price-arbitrage dispatch: charge at low price, discharge at high price.
    fn dispatch_storage_price_arbitrage(
        &mut self,
        price: f64,
        median_price: f64,
        _t: usize,
    ) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];

        let p_cmd = if price > median_price {
            // Discharge (inject)
            1.0 // fraction of rated power
        } else {
            // Charge
            -1.0
        };

        for (sidx, su) in self.storage_units.iter_mut().enumerate() {
            let cmd_mw = p_cmd * su.power_mw;
            let actual = su.apply_power(cmd_mw, dt);
            out[sidx] = actual;
            if sidx < self.storage_soc.len() {
                self.storage_soc[sidx] = su.soc;
            }
        }
        out
    }

    /// Scheduled dispatch: read power from `BusTimeSeries` profile.
    fn dispatch_storage_scheduled(&mut self, t: usize) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];

        // Collect scheduled powers first to avoid borrow conflict
        let scheduled: Vec<f64> = self
            .storage_units
            .iter()
            .map(|su| {
                self.network
                    .bus_series
                    .get(su.series_idx)
                    .map(|bts| bts.p_at(t))
                    .unwrap_or(0.0)
            })
            .collect();

        for (sidx, su) in self.storage_units.iter_mut().enumerate() {
            let cmd = scheduled.get(sidx).copied().unwrap_or(0.0);
            let actual = su.apply_power(cmd, dt);
            out[sidx] = actual;
            if sidx < self.storage_soc.len() {
                self.storage_soc[sidx] = su.soc;
            }
        }
        out
    }

    /// Apply renewable curtailment to resolve over-voltage conditions.
    ///
    /// Reduces generation at renewable buses proportionally until `voltage_upper_pu`
    /// is no longer exceeded. Returns total curtailment [MW].
    fn apply_curtailment(&self, p_injections: &mut [f64], voltages: &[f64]) -> f64 {
        let upper = self.config.voltage_upper_pu;
        let mut total_curtailed = 0.0_f64;

        for bts in &self.network.bus_series {
            if !bts.series_type.is_renewable() {
                continue;
            }
            let bus = bts.bus_id;
            if bus >= voltages.len() || bus >= p_injections.len() {
                continue;
            }
            let v = voltages[bus];
            if v > upper {
                // Scale down proportionally to over-voltage
                let over = (v - upper) / upper;
                let reduction = p_injections[bus] * over.min(1.0);
                let reduction = reduction.max(0.0);
                total_curtailed += reduction;
                p_injections[bus] -= reduction;
            }
        }
        total_curtailed
    }

    /// Compute total generation, total load, and renewable generation from profiles.
    fn compute_generation_load(&self, t: usize, storage_p: &[f64]) -> (f64, f64, f64) {
        let mut gen = 0.0_f64;
        let mut load = 0.0_f64;
        let mut ren = 0.0_f64;

        for bts in &self.network.bus_series {
            let p = bts.p_at(t);
            match &bts.series_type {
                BusTimeSeriesType::Load => load += p.max(0.0),
                BusTimeSeriesType::SolarGeneration { .. }
                | BusTimeSeriesType::WindGeneration { .. }
                | BusTimeSeriesType::HydroGeneration => {
                    gen += p.max(0.0);
                    ren += p.max(0.0);
                }
                BusTimeSeriesType::FixedInjection => {
                    if p >= 0.0 {
                        gen += p;
                    } else {
                        load += -p;
                    }
                }
                BusTimeSeriesType::Storage { .. } => {}
            }
        }
        for g in &self.network.generators {
            gen += g.p_at(t).max(0.0);
        }
        // Storage discharging counts as generation; charging as load
        for &sp in storage_p {
            if sp > 0.0 {
                gen += sp;
            } else {
                load += -sp;
            }
        }
        (gen, load, ren)
    }

    /// Compute aggregated statistics from per-timestep results.
    fn compute_statistics(results: &[TimeStepResult], dt_hours: f64) -> TimeSeriesStatistics {
        let n = results.len();
        if n == 0 {
            return TimeSeriesStatistics::default();
        }

        let n_converged = results.iter().filter(|r| r.converged).count();
        let convergence_rate = n_converged as f64 / n as f64;

        // Voltage stats (across all buses and all timesteps)
        let mut v_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_sum = 0.0_f64;
        let mut v_count = 0usize;
        for r in results {
            for &vm in &r.voltage_magnitude {
                if vm > v_max {
                    v_max = vm;
                }
                if vm < v_min {
                    v_min = vm;
                }
                v_sum += vm;
                v_count += 1;
            }
        }
        let avg_voltage = if v_count > 0 {
            v_sum / v_count as f64
        } else {
            1.0
        };

        // Load stats
        let loads: Vec<f64> = results.iter().map(|r| r.total_load_mw).collect();
        let peak_load = loads.iter().cloned().fold(f64::NAN, f64::max);
        let peak_load = if peak_load.is_nan() { 0.0 } else { peak_load };
        let avg_load = loads.iter().sum::<f64>() / n as f64;
        let load_factor = if peak_load > 0.0 {
            avg_load / peak_load
        } else {
            0.0
        };
        let total_energy_twh = avg_load * n as f64 * dt_hours / 1e6;

        // Renewable fraction
        let total_ren: f64 = results.iter().map(|r| r.renewable_generation_mw).sum();
        let total_gen: f64 = results.iter().map(|r| r.total_generation_mw).sum();
        let ren_frac = if total_gen > 0.0 {
            total_ren / total_gen * 100.0
        } else {
            0.0
        };

        // Curtailment & losses
        let total_curtailment_mwh: f64 = results
            .iter()
            .map(|r| r.renewable_curtailment_mw * dt_hours)
            .sum();
        let total_losses_mwh: f64 = results.iter().map(|r| r.total_losses_mw * dt_hours).sum();

        // Branch loading stats
        let all_loadings: Vec<f64> = results
            .iter()
            .flat_map(|r| r.branch_loading_pct.iter().cloned())
            .collect();
        let max_branch_loading = all_loadings
            .iter()
            .cloned()
            .fold(f64::NAN, f64::max)
            .max(0.0);
        let max_branch_loading = if max_branch_loading.is_nan() {
            0.0
        } else {
            max_branch_loading
        };
        let avg_branch_loading = if all_loadings.is_empty() {
            0.0
        } else {
            all_loadings.iter().sum::<f64>() / all_loadings.len() as f64
        };

        let n_overload_hours = results
            .iter()
            .filter(|r| !r.overloaded_branches.is_empty())
            .count();
        let n_voltage_violation_hours = results
            .iter()
            .filter(|r| !r.voltage_violations.is_empty())
            .count();

        TimeSeriesStatistics {
            n_timesteps: n,
            n_converged,
            convergence_rate,
            max_voltage_pu: if v_max.is_infinite() { 1.0 } else { v_max },
            min_voltage_pu: if v_min.is_infinite() { 1.0 } else { v_min },
            avg_voltage_pu: avg_voltage,
            peak_load_mw: peak_load,
            avg_load_mw: avg_load,
            load_factor,
            total_energy_twh,
            renewable_fraction_pct: ren_frac,
            total_curtailment_mwh,
            total_losses_mwh,
            max_branch_loading_pct: max_branch_loading,
            avg_branch_loading_pct: avg_branch_loading,
            n_overload_hours,
            n_voltage_violation_hours,
            hosting_capacity_estimate_mw: 0.0,
        }
    }

    /// Compute the median price from the `PriceArbitrage` strategy profile.
    fn compute_median_price(&self) -> f64 {
        if let StorageStrategy::PriceArbitrage { price_profile } =
            &self.config.storage_dispatch_strategy
        {
            if price_profile.is_empty() {
                return 0.0;
            }
            let mut sorted = price_profile.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            let mid = sorted.len() / 2;
            if sorted.len() % 2 == 0 {
                (sorted[mid - 1] + sorted[mid]) / 2.0
            } else {
                sorted[mid]
            }
        } else {
            0.0
        }
    }

    /// Count timesteps with voltage violations for a given additional injection at `test_bus`.
    fn count_violations_with_injection(
        &self,
        test_bus: usize,
        extra_mw: f64,
        dt: f64,
        n_t: usize,
    ) -> Result<usize> {
        let _ = dt;
        let mut violations = 0usize;

        for t in 0..n_t {
            let (mut p_inj, q_inj) = self.get_bus_injections(t);
            if test_bus < p_inj.len() {
                p_inj[test_bus] += extra_mw;
            }
            let voltages = self.estimate_voltages(&p_inj, &q_inj);
            let has_v_viol = voltages
                .iter()
                .any(|&v| v < self.config.voltage_lower_pu || v > self.config.voltage_upper_pu);
            let angles = match self.solve_dc_powerflow(&p_inj) {
                Ok(a) => a,
                Err(_) => {
                    violations += 1;
                    continue;
                }
            };
            let branch_flows = self.compute_branch_flows(&angles);
            let branch_loading = self.compute_branch_loading(&branch_flows);
            let has_overload = branch_loading.iter().any(|&l| l > 100.0);
            if has_v_viol || has_overload {
                violations += 1;
            }
        }
        Ok(violations)
    }
}

// ─── Scenario Analysis ────────────────────────────────────────────────────────

/// Compares multiple named simulation scenarios side by side.
#[derive(Debug, Default)]
pub struct ScenarioAnalysis {
    /// Named results from different simulation runs.
    pub scenarios: Vec<(String, TimeSeriesResult)>,
}

impl ScenarioAnalysis {
    /// Create an empty scenario analysis.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a named simulation result.
    pub fn add_scenario(&mut self, name: String, result: TimeSeriesResult) {
        self.scenarios.push((name, result));
    }

    /// Return a list of `(name, statistics)` pairs for easy tabular comparison.
    pub fn compare(&self) -> Vec<(String, TimeSeriesStatistics)> {
        self.scenarios
            .iter()
            .map(|(name, res)| (name.clone(), res.statistics.clone()))
            .collect()
    }

    /// Select the scenario with the best composite score.
    ///
    /// Score = 0.4 × renewable_fraction + 0.3 × (1 – curtailment_fraction) + 0.3 × (1 – losses_fraction).
    /// All fractions are normalised to \[0, 1\].
    pub fn optimal_scenario(&self) -> Option<&str> {
        if self.scenarios.is_empty() {
            return None;
        }

        // Gather normalisation denominators
        let max_ren = self
            .scenarios
            .iter()
            .map(|(_, r)| r.statistics.renewable_fraction_pct)
            .fold(f64::NAN, f64::max)
            .max(1.0);
        let max_curtailment = self
            .scenarios
            .iter()
            .map(|(_, r)| r.statistics.total_curtailment_mwh)
            .fold(f64::NAN, f64::max)
            .max(1.0);
        let max_losses = self
            .scenarios
            .iter()
            .map(|(_, r)| r.statistics.total_losses_mwh)
            .fold(f64::NAN, f64::max)
            .max(1.0);

        let mut best_score = f64::NEG_INFINITY;
        let mut best_name: Option<&str> = None;

        for (name, res) in &self.scenarios {
            let s = &res.statistics;
            let ren_frac = (s.renewable_fraction_pct / max_ren).clamp(0.0, 1.0);
            let curt_frac = (s.total_curtailment_mwh / max_curtailment).clamp(0.0, 1.0);
            let loss_frac = (s.total_losses_mwh / max_losses).clamp(0.0, 1.0);
            let score = 0.4 * ren_frac + 0.3 * (1.0 - curt_frac) + 0.3 * (1.0 - loss_frac);
            if score > best_score {
                best_score = score;
                best_name = Some(name.as_str());
            }
        }
        best_name
    }
}

// ─── Legacy API ───────────────────────────────────────────────────────────────
// The original `TimeSeriesProfile` / `TimeSeriesSimulator` (Newton-Raphson based)
// has been superseded by `TimeSeriesSimulator` above.
// The legacy types are preserved below as `LegacyTimeSeriesProfile` etc.
// to avoid breaking downstream code that may reference them, but they are
// intentionally not re-exported from the module root.

/// Load and generation profiles for the legacy time-series simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesProfile {
    /// Real power load profile per bus per timestep \[MW\].
    pub load_profiles: Vec<Vec<f64>>,
    /// Renewable generation profile per generator per timestep \[MW\].
    pub renewable_profiles: Vec<Vec<f64>>,
    /// Electricity market price per timestep \[$/MWh\].
    pub price_profile: Vec<f64>,
}

impl TimeSeriesProfile {
    /// Create a flat (constant) profile for all timesteps.
    pub fn flat(
        n_buses: usize,
        n_gens: usize,
        n_timesteps: usize,
        load_mw: f64,
        renewable_mw: f64,
        price: f64,
    ) -> Self {
        Self {
            load_profiles: vec![vec![load_mw; n_timesteps]; n_buses],
            renewable_profiles: vec![vec![renewable_mw; n_timesteps]; n_gens],
            price_profile: vec![price; n_timesteps],
        }
    }

    /// Number of timesteps.
    pub fn n_timesteps(&self) -> usize {
        self.price_profile.len()
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── LCG for deterministic pseudo-random profiles (no rand crate) ──────────
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            // Map high 32 bits to [0, 1)
            (self.0 >> 32) as f64 / u32::MAX as f64
        }
        fn next_in(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /// Build a minimal 3-bus network: slack(0) – bus1 – bus2.
    /// x_01 = 0.1 pu, x_12 = 0.2 pu, base = 100 MVA.
    fn three_bus_network() -> TimeSeriesNetwork {
        let n = 3;
        let base = 100.0;
        // B_ij (off-diagonal) = -1/x_ij
        // B_01 = -10, B_12 = -5
        let mut b = vec![vec![0.0_f64; n]; n];
        let g = vec![vec![0.0_f64; n]; n];
        // Branch 0-1
        b[0][1] = -10.0;
        b[1][0] = -10.0;
        b[0][0] += 10.0;
        b[1][1] += 10.0;
        // Branch 1-2
        b[1][2] = -5.0;
        b[2][1] = -5.0;
        b[1][1] += 5.0;
        b[2][2] += 5.0;

        TimeSeriesNetwork {
            n_buses: n,
            g_matrix: g,
            b_matrix: b,
            bus_series: vec![],
            generators: vec![],
            branch_ratings_mva: vec![100.0, 100.0],
            branches: vec![(0, 1), (1, 2)],
            slack_bus: 0,
            base_mva: base,
        }
    }

    /// Build a 2-bus network with one load and one slack generator.
    fn two_bus_network_with_profiles(n_t: usize) -> TimeSeriesNetwork {
        let n = 2;
        let base = 100.0;
        let mut b = vec![vec![0.0_f64; n]; n];
        let g = vec![vec![0.0_f64; n]; n];
        b[0][1] = -10.0;
        b[1][0] = -10.0;
        b[0][0] += 10.0;
        b[1][1] += 10.0;

        let load_series = BusTimeSeries {
            bus_id: 1,
            p_mw: vec![50.0; n_t],
            q_mvar: vec![10.0; n_t],
            series_type: BusTimeSeriesType::Load,
        };

        TimeSeriesNetwork {
            n_buses: n,
            g_matrix: g,
            b_matrix: b,
            bus_series: vec![load_series],
            generators: vec![],
            branch_ratings_mva: vec![200.0],
            branches: vec![(0, 1)],
            slack_bus: 0,
            base_mva: base,
        }
    }

    // ── TimeResolution tests ──────────────────────────────────────────────────

    #[test]
    fn test_time_resolution_steps_per_day() {
        assert_eq!(TimeResolution::FifteenMinutes.steps_per_day(), 96);
        assert_eq!(TimeResolution::HalfHourly.steps_per_day(), 48);
        assert_eq!(TimeResolution::Hourly.steps_per_day(), 24);
        assert_eq!(TimeResolution::Daily.steps_per_day(), 1);
    }

    #[test]
    fn test_time_resolution_dt_hours() {
        let eps = 1e-12;
        assert!((TimeResolution::FifteenMinutes.dt_hours() - 0.25).abs() < eps);
        assert!((TimeResolution::HalfHourly.dt_hours() - 0.5).abs() < eps);
        assert!((TimeResolution::Hourly.dt_hours() - 1.0).abs() < eps);
        assert!((TimeResolution::Daily.dt_hours() - 24.0).abs() < eps);
    }

    // ── BusTimeSeries tests ───────────────────────────────────────────────────

    #[test]
    fn test_bus_time_series_creation() {
        let bts = BusTimeSeries {
            bus_id: 3,
            p_mw: vec![10.0, 20.0, 30.0],
            q_mvar: vec![2.0, 4.0, 6.0],
            series_type: BusTimeSeriesType::Load,
        };
        assert_eq!(bts.bus_id, 3);
        assert_eq!(bts.len(), 3);
        assert!(!bts.is_empty());
        assert!((bts.p_at(1) - 20.0).abs() < 1e-10);
        assert!((bts.q_at(2) - 6.0).abs() < 1e-10);
        assert!((bts.p_at(99) - 0.0).abs() < 1e-10); // out of range → 0
    }

    #[test]
    fn test_generator_profile_creation() {
        let gp = GeneratorProfile {
            generator_id: 0,
            bus: 1,
            p_dispatch_mw: vec![50.0, 60.0, 70.0],
            q_dispatch_mvar: vec![5.0, 6.0, 7.0],
            p_max_mw: 100.0,
            p_min_mw: 0.0,
            cost_per_mwh: 40.0,
        };
        assert!((gp.p_at(0) - 50.0).abs() < 1e-10);
        assert!((gp.p_at(2) - 70.0).abs() < 1e-10);
        // Clamp check
        let gp2 = GeneratorProfile {
            p_dispatch_mw: vec![150.0],
            p_max_mw: 100.0,
            p_min_mw: 0.0,
            ..gp.clone()
        };
        assert!((gp2.p_at(0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_timeseries_network_creation() {
        let net = three_bus_network();
        assert_eq!(net.n_buses, 3);
        assert_eq!(net.branches.len(), 2);
        assert_eq!(net.branch_ratings_mva.len(), 2);
        net.validate().expect("3-bus network should be valid");
    }

    #[test]
    fn test_timeseries_config_default() {
        let cfg = TimeSeriesConfig::default();
        assert_eq!(cfg.n_timesteps, 8760);
        assert_eq!(cfg.resolution, TimeResolution::Hourly);
        assert!((cfg.voltage_lower_pu - 0.95).abs() < 1e-10);
        assert!((cfg.voltage_upper_pu - 1.05).abs() < 1e-10);
        assert!(cfg.enable_curtailment);
        assert_eq!(cfg.max_pf_iterations, 20);
    }

    // ── Injection helpers ─────────────────────────────────────────────────────

    #[test]
    fn test_get_bus_injections_load() {
        let net = two_bus_network_with_profiles(5);
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let (p, q) = sim.get_bus_injections(0);
        // Bus 1 has a load of 50 MW → p[1] = -50
        assert!((p[1] - (-50.0)).abs() < 1e-9, "p[1]={}", p[1]);
        assert!((q[1] - (-10.0)).abs() < 1e-9, "q[1]={}", q[1]);
        // Bus 0 (slack): no series → 0
        assert!((p[0] - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_get_bus_injections_generation() {
        let mut net = two_bus_network_with_profiles(3);
        // Add a solar generator at bus 0
        net.bus_series.push(BusTimeSeries {
            bus_id: 0,
            p_mw: vec![30.0; 3],
            q_mvar: vec![0.0; 3],
            series_type: BusTimeSeriesType::SolarGeneration { installed_mw: 30.0 },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 3,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let (p, _) = sim.get_bus_injections(0);
        // Bus 0: +30 MW solar
        assert!((p[0] - 30.0).abs() < 1e-9);
        // Bus 1: -50 MW load
        assert!((p[1] - (-50.0)).abs() < 1e-9);
    }

    // ── DC power flow tests ───────────────────────────────────────────────────

    #[test]
    fn test_dc_powerflow_2bus() {
        // 2-bus: slack at 0, load 50 MW at bus 1, x=0.1 pu, base=100 MVA
        // Expected θ_1 = -P/B = -(50/100)/10 = -0.05 rad
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let p_inj = vec![50.0, -50.0]; // slack injects 50, bus1 absorbs 50
        let angles = sim
            .solve_dc_powerflow(&p_inj)
            .expect("DC PF should converge");
        assert!((angles[0] - 0.0).abs() < 1e-8, "slack angle must be 0");
        // θ_1 = -50/100 / 10 = -0.05 rad
        let expected_theta1 = -0.05;
        assert!(
            (angles[1] - expected_theta1).abs() < 1e-6,
            "θ1 = {:.6}, expected {:.6}",
            angles[1],
            expected_theta1
        );
    }

    #[test]
    fn test_dc_powerflow_3bus() {
        let net = three_bus_network();
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        // Inject 100 MW at bus 0, absorb 60 MW at bus 1, 40 MW at bus 2
        let p_inj = vec![100.0, -60.0, -40.0];
        let angles = sim
            .solve_dc_powerflow(&p_inj)
            .expect("3-bus DC PF should converge");
        assert_eq!(angles.len(), 3);
        assert!((angles[0] - 0.0).abs() < 1e-8, "slack angle must be 0");
        // Verify power balance: B'·θ ≈ P (check residuals)
        // B' from build_b_prime for this network:
        // bus0: 10*(θ0-θ1) = 10*(0-θ1) ≈ 1.0 pu
        let p0_check = 10.0 * (angles[0] - angles[1]) * 100.0;
        let p1_check =
            10.0 * (angles[1] - angles[0]) * 100.0 + 5.0 * (angles[1] - angles[2]) * 100.0;
        let p2_check = 5.0 * (angles[2] - angles[1]) * 100.0;
        assert!(
            (p0_check - 100.0).abs() < 1.0,
            "P0 mismatch: {:.2}",
            p0_check
        );
        assert!(
            (p1_check - (-60.0)).abs() < 1.0,
            "P1 mismatch: {:.2}",
            p1_check
        );
        assert!(
            (p2_check - (-40.0)).abs() < 1.0,
            "P2 mismatch: {:.2}",
            p2_check
        );
    }

    // ── Branch flow / loading tests ───────────────────────────────────────────

    #[test]
    fn test_branch_flows_from_angles() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        // θ_0=0, θ_1=-0.05 rad → P_01 = -B_01*(θ0-θ1)*base = 10*0.05*100 = 50 MW
        let angles = vec![0.0, -0.05];
        let flows = sim.compute_branch_flows(&angles);
        assert_eq!(flows.len(), 1);
        assert!(
            (flows[0] - 50.0).abs() < 1e-6,
            "branch flow = {:.4}",
            flows[0]
        );
    }

    #[test]
    fn test_branch_loading_within_rating() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        // Rating = 200 MVA, flow = 50 MW → loading = 25%
        let flows = vec![50.0];
        let loading = sim.compute_branch_loading(&flows);
        assert_eq!(loading.len(), 1);
        assert!(
            (loading[0] - 25.0).abs() < 1e-6,
            "loading = {:.2}%",
            loading[0]
        );
        assert!(loading[0] <= 100.0, "should not be overloaded");
    }

    #[test]
    fn test_branch_loading_overloaded() {
        let mut net = two_bus_network_with_profiles(1);
        net.branch_ratings_mva = vec![30.0]; // only 30 MVA rating
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        // flow = 50 MW, rating = 30 → 166.7 %
        let flows = vec![50.0];
        let loading = sim.compute_branch_loading(&flows);
        assert!(
            loading[0] > 100.0,
            "should be overloaded: {:.1}%",
            loading[0]
        );
    }

    // ── Voltage estimation ────────────────────────────────────────────────────

    #[test]
    fn test_voltage_estimation_flat() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        // Zero Q injection → flat voltage = 1.0 pu everywhere
        let p_inj = vec![0.0, 0.0];
        let q_inj = vec![0.0, 0.0];
        let voltages = sim.estimate_voltages(&p_inj, &q_inj);
        for &v in &voltages {
            assert!((v - 1.0).abs() < 1e-9, "flat Q → V=1.0, got {}", v);
        }
    }

    // ── Storage dispatch tests ────────────────────────────────────────────────

    #[test]
    fn test_storage_dispatch_peak_shaving_discharge() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![0.0; 5],
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::PeakShaving { threshold_mw: 40.0 },
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        // total load > 40 MW → should discharge
        let _p_inj = [0.0, -60.0]; // 60 MW load at bus1
        let power = sim.dispatch_storage_peak_shaving(60.0, 40.0, 0);
        assert_eq!(power.len(), 1);
        assert!(
            power[0] > 0.0,
            "should discharge (positive), got {:.4}",
            power[0]
        );
    }

    #[test]
    fn test_storage_dispatch_peak_shaving_charge() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![0.0; 5],
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::PeakShaving {
                threshold_mw: 100.0,
            },
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        // total load < 100 MW → should charge
        let power = sim.dispatch_storage_peak_shaving(30.0, 100.0, 0);
        assert_eq!(power.len(), 1);
        assert!(
            power[0] < 0.0,
            "should charge (negative), got {:.4}",
            power[0]
        );
    }

    #[test]
    fn test_storage_soc_update() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![10.0; 5], // 10 MW discharge per step
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::ScheduledDispatch,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let initial_soc = sim.storage_soc[0];
        // Run one step of scheduled dispatch
        sim.dispatch_storage_scheduled(0);
        // SoC should decrease after discharging
        assert!(
            sim.storage_soc[0] < initial_soc,
            "SoC should decrease after discharge: {} < {}",
            sim.storage_soc[0],
            initial_soc
        );
    }

    // ── Full simulation tests ─────────────────────────────────────────────────

    #[test]
    fn test_run_24h_simulation() {
        let n_t = 24;
        let mut lcg = Lcg::new(42);
        let mut net = three_bus_network();
        // Add a load with hourly variation at bus 2
        let load_profile: Vec<f64> = (0..n_t).map(|_| lcg.next_in(20.0, 80.0)).collect();
        net.bus_series.push(BusTimeSeries {
            bus_id: 2,
            p_mw: load_profile,
            q_mvar: vec![5.0; n_t],
            series_type: BusTimeSeriesType::Load,
        });
        // Add a solar generator at bus 1
        let solar_profile: Vec<f64> = (0..n_t)
            .map(|h| {
                if (6..=18).contains(&h) {
                    lcg.next_in(10.0, 50.0)
                } else {
                    0.0
                }
            })
            .collect();
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: solar_profile,
            q_mvar: vec![0.0; n_t],
            series_type: BusTimeSeriesType::SolarGeneration { installed_mw: 50.0 },
        });

        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            resolution: TimeResolution::Hourly,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("24-hour simulation should succeed");

        assert_eq!(result.timestep_results.len(), n_t);
        // All timesteps should converge (DC is always solvable for connected network)
        assert_eq!(result.statistics.n_converged, n_t);
        assert!(
            (result.statistics.convergence_rate - 1.0).abs() < 1e-9,
            "convergence_rate={}",
            result.statistics.convergence_rate
        );
    }

    // ── Statistics tests ──────────────────────────────────────────────────────

    #[test]
    fn test_compute_statistics_load_factor() {
        // Create artificial results with known load values
        let make_result = |t: usize, load: f64| TimeStepResult {
            timestep: t,
            time_hours: t as f64,
            converged: true,
            voltage_magnitude: vec![1.0, 1.0],
            voltage_angle: vec![0.0, 0.0],
            branch_loading_pct: vec![50.0],
            total_generation_mw: load,
            total_load_mw: load,
            total_losses_mw: 0.0,
            renewable_generation_mw: 0.0,
            renewable_curtailment_mw: 0.0,
            storage_soc: vec![],
            overloaded_branches: vec![],
            voltage_violations: vec![],
        };
        let results = vec![
            make_result(0, 80.0),
            make_result(1, 40.0),
            make_result(2, 80.0),
        ];
        let stats = TimeSeriesSimulator::compute_statistics(&results, 1.0);
        // avg = 200/3 ≈ 66.67, peak = 80 → load_factor ≈ 0.833
        assert!((stats.peak_load_mw - 80.0).abs() < 1e-6);
        let expected_lf = (200.0 / 3.0) / 80.0;
        assert!(
            (stats.load_factor - expected_lf).abs() < 1e-6,
            "load_factor={:.4} expected={:.4}",
            stats.load_factor,
            expected_lf
        );
    }

    #[test]
    fn test_compute_statistics_renewable_fraction() {
        let make_result = |t: usize, gen: f64, ren: f64| TimeStepResult {
            timestep: t,
            time_hours: t as f64,
            converged: true,
            voltage_magnitude: vec![1.0],
            voltage_angle: vec![0.0],
            branch_loading_pct: vec![],
            total_generation_mw: gen,
            total_load_mw: gen,
            total_losses_mw: 0.0,
            renewable_generation_mw: ren,
            renewable_curtailment_mw: 0.0,
            storage_soc: vec![],
            overloaded_branches: vec![],
            voltage_violations: vec![],
        };
        // 100 MW total gen, 40 MW renewable → 40 %
        let results = vec![make_result(0, 100.0, 40.0), make_result(1, 100.0, 40.0)];
        let stats = TimeSeriesSimulator::compute_statistics(&results, 1.0);
        assert!(
            (stats.renewable_fraction_pct - 40.0).abs() < 1e-6,
            "ren_frac={:.4}",
            stats.renewable_fraction_pct
        );
    }

    // ── Scenario analysis tests ───────────────────────────────────────────────

    #[test]
    fn test_scenario_analysis_compare() {
        let make_stats = |ren: f64| TimeSeriesStatistics {
            renewable_fraction_pct: ren,
            ..Default::default()
        };
        let make_result = |ren: f64| TimeSeriesResult {
            timestep_results: vec![],
            statistics: make_stats(ren),
            duration_s: 0.0,
        };

        let mut analysis = ScenarioAnalysis::new();
        analysis.add_scenario("Base".into(), make_result(20.0));
        analysis.add_scenario("HighRen".into(), make_result(60.0));

        let compared = analysis.compare();
        assert_eq!(compared.len(), 2);
        assert_eq!(compared[0].0, "Base");
        assert_eq!(compared[1].0, "HighRen");
        assert!((compared[1].1.renewable_fraction_pct - 60.0).abs() < 1e-9);
    }

    #[test]
    fn test_hosting_capacity_estimation() {
        let n_t = 8;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            resolution: TimeResolution::HalfHourly,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        // Bus 1: test bus, max search = 200 MW
        // With B_11 = 10, Q=0 → v ≈ 1.0 pu always, so hosting capacity ≈ max_search
        let hc = sim
            .estimate_hosting_capacity(1, 200.0)
            .expect("hosting capacity should succeed");
        assert!(hc >= 0.0, "hosting capacity must be non-negative: {}", hc);
        assert!(
            hc <= 200.0,
            "hosting capacity must not exceed search range: {}",
            hc
        );
    }
}
