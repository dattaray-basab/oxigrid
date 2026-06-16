//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

use super::types::{
    BusTimeSeriesType, StorageStrategy, StorageUnit, TimeSeriesConfig, TimeSeriesNetwork,
    TimeSeriesResult, TimeSeriesStatistics, TimeStepResult,
};

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
    pub(super) storage_units: Vec<StorageUnit>,
}
impl TimeSeriesSimulator {
    /// Create a new simulator.
    /// Storage units are discovered from `BusTimeSeriesType::Storage` entries;
    /// initial SoC is set to 0.5 for each.
    pub fn new(network: TimeSeriesNetwork, config: TimeSeriesConfig) -> Self {
        let mut storage_units = Vec::new();
        for (idx, bts) in network.bus_series.iter().enumerate() {
            if bts.series_type.is_storage() {
                let power_mw = bts
                    .p_mw
                    .iter()
                    .map(|&p| p.abs())
                    .fold(f64::NAN, f64::max)
                    .max(1.0);
                let capacity_mwh = power_mw * 4.0;
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
        let median_price = self.compute_median_price();
        let dt = self.config.resolution.dt_hours();
        let n_t = self.config.n_timesteps;
        let mut results: Vec<TimeStepResult> = Vec::with_capacity(n_t);
        for t in 0..n_t {
            let time_hours = t as f64 * dt;
            let (mut p_inj, q_inj) = self.get_bus_injections(t);
            let storage_p = self.dispatch_storage(t, &p_inj, median_price);
            for (sidx, su) in self.storage_units.iter().enumerate() {
                if su.bus_id < p_inj.len() {
                    p_inj[su.bus_id] += storage_p[sidx];
                }
            }
            let (angles, converged) = match self.solve_dc_powerflow(&p_inj) {
                Ok(a) => (a, true),
                Err(_) => (vec![0.0; self.network.n_buses], false),
            };
            let mut voltages = self.estimate_voltages(&p_inj, &q_inj);
            let branch_flows = self.compute_branch_flows(&angles);
            let branch_loading = self.compute_branch_loading(&branch_flows);
            let curtailment_mw = if self.config.enable_curtailment
                && voltages.iter().any(|&v| v > self.config.voltage_upper_pu)
            {
                let c = self.apply_curtailment(&mut p_inj, &voltages);
                voltages = self.estimate_voltages(&p_inj, &q_inj);
                c
            } else {
                0.0
            };
            let (total_gen, total_load, ren_gen) = self.compute_generation_load(t, &storage_p);
            let renewable_gen_after = (ren_gen - curtailment_mw).max(0.0);
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
                total_losses_mw: 0.0,
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
            duration_s: 0.0,
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
        for _ in 0..20 {
            let mid = (lo + hi) / 2.0;
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
    /// Collect net power injections at each bus for timestep `t`.
    ///
    /// Returns `(p_mw, q_mvar)` where positive = injection into the network.
    /// Loads are subtracted; generators are added.
    pub(super) fn get_bus_injections(&self, t: usize) -> (Vec<f64>, Vec<f64>) {
        let n = self.network.n_buses;
        let mut p = vec![0.0_f64; n];
        let mut q = vec![0.0_f64; n];
        for bts in &self.network.bus_series {
            let bus = bts.bus_id;
            if bus >= n {
                continue;
            }
            let p_val = bts.p_at(t);
            let q_val = bts.q_at(t);
            match &bts.series_type {
                BusTimeSeriesType::Load => {
                    p[bus] -= p_val;
                    q[bus] -= q_val;
                }
                BusTimeSeriesType::Storage { .. } => {}
                _ => {
                    p[bus] += p_val;
                    q[bus] += q_val;
                }
            }
        }
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
    /// Solve a DC power flow for the given active power injections `MW`.
    ///
    /// Builds the reduced B' matrix (excluding slack bus), solves **B'·θ = P**,
    /// and returns the full angle vector `rad` with slack = 0.
    pub(super) fn solve_dc_powerflow(&self, p_injections: &[f64]) -> Result<Vec<f64>> {
        let n = self.network.n_buses;
        let slack = self.network.slack_bus;
        let base = self.network.base_mva;
        let bp = self.network.build_b_prime();
        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack).collect();
        let m = non_slack.len();
        if m == 0 {
            return Ok(vec![0.0; n]);
        }
        let mut a = vec![vec![0.0_f64; m]; m];
        for (ri, &i) in non_slack.iter().enumerate() {
            for (rj, &j) in non_slack.iter().enumerate() {
                a[ri][rj] = bp[i][j];
            }
        }
        let mut rhs: Vec<f64> = non_slack
            .iter()
            .map(|&i| p_injections.get(i).copied().unwrap_or(0.0) / base)
            .collect();
        Self::gaussian_solve(&mut a, &mut rhs)?;
        let mut angles = vec![0.0_f64; n];
        for (ri, &i) in non_slack.iter().enumerate() {
            angles[i] = rhs[ri];
        }
        Ok(angles)
    }
    /// Gaussian elimination with partial pivoting (in-place).
    #[allow(clippy::ptr_arg, clippy::needless_range_loop)]
    pub(super) fn gaussian_solve(a: &mut Vec<Vec<f64>>, b: &mut Vec<f64>) -> Result<()> {
        let m = b.len();
        for col in 0..m {
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
        for row in (0..m).rev() {
            let mut sum = b[row];
            for c in (row + 1)..m {
                sum -= a[row][c] * b[c];
            }
            b[row] = sum / a[row][row];
        }
        Ok(())
    }
    /// Compute DC branch flows `MW` from bus angle vector `rad`.
    ///
    /// `P_ij = (θ_i - θ_j) × B_ij × base_mva`
    /// where `B_ij` is the off-diagonal entry of the nodal susceptance matrix.
    pub(super) fn compute_branch_flows(&self, angles: &[f64]) -> Vec<f64> {
        self.network
            .branches
            .iter()
            .map(|&(from, to)| {
                let theta_i = angles.get(from).copied().unwrap_or(0.0);
                let theta_j = angles.get(to).copied().unwrap_or(0.0);
                let bij = self
                    .network
                    .b_matrix
                    .get(from)
                    .and_then(|row| row.get(to))
                    .copied()
                    .unwrap_or(0.0);
                let inv_x = -bij;
                inv_x * (theta_i - theta_j) * self.network.base_mva
            })
            .collect()
    }
    /// Compute branch loading as percentage of thermal rating.
    pub(super) fn compute_branch_loading(&self, branch_flows: &[f64]) -> Vec<f64> {
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
    /// Estimate bus voltage magnitudes `pu` from reactive power injections.
    ///
    /// Uses a first-order Q-sensitivity: Δv ≈ -Q / (B_ii × base_mva).
    /// Clamps result to [0.5, 1.5] pu.
    pub(super) fn estimate_voltages(
        &self,
        _p_injections: &[f64],
        q_injections: &[f64],
    ) -> Vec<f64> {
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
                let dv = if bii.abs() > 1e-6 { -q_pu / bii } else { 0.0 };
                (1.0 + dv).clamp(0.5, 1.5)
            })
            .collect()
    }
    /// Dispatch storage units according to the configured strategy.
    /// Updates SoC in-place and returns actual power `MW` per storage unit.
    pub(super) fn dispatch_storage(
        &mut self,
        t: usize,
        p_inj: &[f64],
        median_price: f64,
    ) -> Vec<f64> {
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
    pub(super) fn dispatch_storage_peak_shaving(
        &mut self,
        total_load: f64,
        threshold: f64,
        t: usize,
    ) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];
        if total_load > threshold {
            let deficit = (total_load - threshold) / (n_storage.max(1) as f64);
            for (sidx, su) in self.storage_units.iter_mut().enumerate() {
                let actual = su.apply_power(deficit, dt);
                out[sidx] = actual;
                if sidx < self.storage_soc.len() {
                    self.storage_soc[sidx] = su.soc;
                }
            }
        } else {
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
        let _ = t;
        out
    }
    /// Price-arbitrage dispatch: charge at low price, discharge at high price.
    pub(super) fn dispatch_storage_price_arbitrage(
        &mut self,
        price: f64,
        median_price: f64,
        _t: usize,
    ) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];
        let p_cmd = if price > median_price { 1.0 } else { -1.0 };
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
    pub(super) fn dispatch_storage_scheduled(&mut self, t: usize) -> Vec<f64> {
        let dt = self.config.resolution.dt_hours();
        let n_storage = self.storage_units.len();
        let mut out = vec![0.0_f64; n_storage];
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
    /// is no longer exceeded. Returns total curtailment `MW`.
    pub(super) fn apply_curtailment(&self, p_injections: &mut [f64], voltages: &[f64]) -> f64 {
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
    pub(super) fn compute_generation_load(&self, t: usize, storage_p: &[f64]) -> (f64, f64, f64) {
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
    pub(super) fn compute_statistics(
        results: &[TimeStepResult],
        dt_hours: f64,
    ) -> TimeSeriesStatistics {
        let n = results.len();
        if n == 0 {
            return TimeSeriesStatistics::default();
        }
        let n_converged = results.iter().filter(|r| r.converged).count();
        let convergence_rate = n_converged as f64 / n as f64;
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
        let total_ren: f64 = results.iter().map(|r| r.renewable_generation_mw).sum();
        let total_gen: f64 = results.iter().map(|r| r.total_generation_mw).sum();
        let ren_frac = if total_gen > 0.0 {
            total_ren / total_gen * 100.0
        } else {
            0.0
        };
        let total_curtailment_mwh: f64 = results
            .iter()
            .map(|r| r.renewable_curtailment_mw * dt_hours)
            .sum();
        let total_losses_mwh: f64 = results.iter().map(|r| r.total_losses_mw * dt_hours).sum();
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
    pub(super) fn compute_median_price(&self) -> f64 {
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
    pub(super) fn count_violations_with_injection(
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
/// Time-varying injection profile for a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusTimeSeries {
    /// Bus index (0-based) in [`TimeSeriesNetwork`].
    pub bus_id: usize,
    /// Active power profile `MW`, one value per timestep.
    /// For loads the convention is positive = consuming; for generators positive = generating.
    pub p_mw: Vec<f64>,
    /// Reactive power profile `MVAr`, one value per timestep.
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
