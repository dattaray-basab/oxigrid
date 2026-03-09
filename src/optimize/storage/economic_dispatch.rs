//! Storage-inclusive economic dispatch: co-optimisation of thermal generators
//! and battery storage for minimum cost subject to operational constraints.
//!
//! Implements two complementary dispatch algorithms:
//! - **Lambda-iteration** (equal-incremental-cost): binary search on the system
//!   lambda until the power balance is satisfied.
//! - **Merit-order dispatch**: sort generators by short-run marginal cost and
//!   commit them in order until residual load is served.
//!
//! Both methods are extended with storage (charge/discharge) co-optimisation
//! and support multi-period rolling-horizon operation with SoC continuity.

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Classification of thermal generating unit technology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GeneratorType {
    /// Coal-fired steam unit.
    Coal,
    /// Open-cycle natural-gas turbine.
    NaturalGas,
    /// Nuclear steam unit.
    Nuclear,
    /// Conventional run-of-river or reservoir hydro.
    Hydro,
    /// Pumped-storage hydro (generator mode).
    PumpedHydro,
    /// Gas peaker (quick-start OCGT).
    Peaker,
    /// Combined-cycle gas turbine.
    CombinedCycle,
}

/// Operating mode for a storage battery in the dispatch problem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StorageDispatchMode {
    /// Buy-low / sell-high energy arbitrage.
    EnergyArbitrage,
    /// Reserve / frequency response provision.
    AncillaryServices,
    /// Shave demand peaks to reduce peak-capacity costs.
    PeakShaving,
    /// Frequency inertia and voltage stabilisation support.
    GridStabilization,
    /// Combined objective across multiple services.
    Hybrid,
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// Thermal generating unit participating in economic dispatch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalGenerator {
    /// Unique generator identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Technology type.
    pub gen_type: GeneratorType,
    /// Minimum stable generation [MW].
    pub p_min_mw: f64,
    /// Maximum continuous rating [MW].
    pub p_max_mw: f64,
    /// No-load (fixed) operating cost [USD/h].
    pub cost_a_usd_per_h: f64,
    /// Linear fuel cost coefficient [USD/MWh].
    pub cost_b_usd_per_mwh: f64,
    /// Quadratic fuel cost coefficient [USD/(MW²·h)].
    pub cost_c_usd_per_mw2h: f64,
    /// Ramp-up rate [MW/min].
    pub ramp_up_mw_per_min: f64,
    /// Ramp-down rate [MW/min].
    pub ramp_down_mw_per_min: f64,
    /// One-time cost to start the unit from cold [USD].
    pub startup_cost_usd: f64,
    /// One-time cost to shut the unit down [USD].
    pub shutdown_cost_usd: f64,
    /// Minimum continuous online duration [h].
    pub min_up_time_h: f64,
    /// Minimum continuous offline duration before restart [h].
    pub min_down_time_h: f64,
    /// Whether the unit is currently committed (online).
    pub online: bool,
    /// Current real power output [MW].
    pub current_output_mw: f64,
}

/// Battery energy storage system participating in the dispatch problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StorageBattery {
    /// Unique storage identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Usable energy capacity [MWh].
    pub energy_capacity_mwh: f64,
    /// Maximum charge / discharge power [MW].
    pub power_capacity_mw: f64,
    /// Round-trip efficiency (0–1).
    pub roundtrip_efficiency: f64,
    /// Minimum allowed state of charge (0–1).
    pub soc_min: f64,
    /// Maximum allowed state of charge (0–1).
    pub soc_max: f64,
    /// Current state of charge (0–1).
    pub soc_current: f64,
    /// Degradation (cycling) cost [USD/MWh of throughput].
    pub charging_cost_usd_per_mwh: f64,
    /// Primary operating mode.
    pub mode: StorageDispatchMode,
}

impl StorageBattery {
    /// One-way charge efficiency: √η_rt.
    pub fn charge_efficiency(&self) -> f64 {
        self.roundtrip_efficiency.sqrt()
    }

    /// One-way discharge efficiency: √η_rt.
    pub fn discharge_efficiency(&self) -> f64 {
        self.roundtrip_efficiency.sqrt()
    }

    /// Maximum energy that can be charged in one step of `dt_h` hours [MWh].
    pub fn max_charge_energy_mwh(&self, dt_h: f64) -> f64 {
        let e_to_full = (self.soc_max - self.soc_current) * self.energy_capacity_mwh;
        (self.power_capacity_mw * dt_h).min(e_to_full.max(0.0))
    }

    /// Maximum energy that can be discharged in one step of `dt_h` hours [MWh].
    pub fn max_discharge_energy_mwh(&self, dt_h: f64) -> f64 {
        let e_available = (self.soc_current - self.soc_min) * self.energy_capacity_mwh;
        (self.power_capacity_mw * dt_h).min(e_available.max(0.0))
    }
}

/// Single-period dispatch result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispatchResult {
    /// Period index (hours from simulation start).
    pub timestamp_h: usize,
    /// Real-power output per generator `(gen_id, MW)`.
    pub generator_outputs: Vec<(usize, f64)>,
    /// Charging power per storage unit `(storage_id, MW)` — positive = charging.
    pub storage_charge: Vec<(usize, f64)>,
    /// Discharging power per storage unit `(storage_id, MW)` — positive = discharging.
    pub storage_discharge: Vec<(usize, f64)>,
    /// State of charge after this period per storage unit `(storage_id, SoC)`.
    pub storage_soc: Vec<(usize, f64)>,
    /// Total generation + storage cost for this period [USD].
    pub total_cost_usd: f64,
    /// System lambda (marginal price / LMP at balance node) [USD/MWh].
    pub lambda_usd_per_mwh: f64,
    /// Total load served [MW].
    pub load_served_mw: f64,
    /// Must-take renewable generation injected [MW].
    pub renewable_mw: f64,
    /// Renewable curtailment (when renewable > load) [MW].
    pub curtailment_mw: f64,
}

/// Economic dispatch problem definition for one or more periods.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EconomicDispatchProblem {
    /// Thermal generating units in the portfolio.
    pub generators: Vec<ThermalGenerator>,
    /// Battery storage units in the portfolio.
    pub storage: Vec<StorageBattery>,
    /// Load to be served in the current period [MW].
    pub load_mw: f64,
    /// Must-take renewable injection for the current period [MW].
    pub renewable_mw: f64,
    /// Minimum spinning reserve required [MW].
    pub spinning_reserve_mw: f64,
    /// Dispatch time-step [h] (default 1.0).
    pub dt_h: f64,
}

/// Aggregated result from a multi-period economic dispatch run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispatchSolution {
    /// Per-period dispatch results.
    pub results: Vec<DispatchResult>,
    /// Total cost across all periods [USD].
    pub total_cost_usd: f64,
    /// Fraction of total load served by renewables [0–1].
    pub total_renewable_pct: f64,
    /// Average system lambda across periods [USD/MWh].
    pub avg_lambda_usd_per_mwh: f64,
    /// Equivalent full discharge cycles per storage unit.
    pub storage_cycles: Vec<f64>,
    /// Human-readable descriptions of any violated constraints.
    pub constraint_violations: Vec<String>,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl EconomicDispatchProblem {
    /// Compute the full operating cost of `gen` at `output_mw` for one period.
    ///
    /// Uses the standard quadratic cost curve: `C(P) = a + b·P + c·P²`.
    pub fn compute_generator_cost(gen: &ThermalGenerator, output_mw: f64) -> f64 {
        gen.cost_a_usd_per_h
            + gen.cost_b_usd_per_mwh * output_mw
            + gen.cost_c_usd_per_mw2h * output_mw * output_mw
    }

    /// Compute the incremental (marginal) cost of `gen` at `output_mw`.
    ///
    /// Derivative of the quadratic cost: `dC/dP = b + 2·c·P`.
    pub fn compute_marginal_cost(gen: &ThermalGenerator, output_mw: f64) -> f64 {
        gen.cost_b_usd_per_mwh + 2.0 * gen.cost_c_usd_per_mw2h * output_mw
    }

    /// Return the feasible `[min_output, max_output]` range for `gen` given
    /// its previous-period output and ramp-rate limits.
    ///
    /// Both endpoints are additionally clamped to the generator's `[p_min, p_max]`.
    pub fn check_ramp_constraints(
        &self,
        prev_output: f64,
        gen: &ThermalGenerator,
        dt_h: f64,
    ) -> (f64, f64) {
        let ramp_up_limit = prev_output + gen.ramp_up_mw_per_min * dt_h * 60.0;
        let ramp_down_limit = prev_output - gen.ramp_down_mw_per_min * dt_h * 60.0;
        let min_feasible = ramp_down_limit.max(gen.p_min_mw);
        let max_feasible = ramp_up_limit.min(gen.p_max_mw);
        (min_feasible, max_feasible.max(min_feasible))
    }

    /// Available reserve above dispatched output for all online generators.
    ///
    /// Reserve = Σ(p_max_i − output_i) for online generators referenced in `result`.
    pub fn compute_reserve_margin(&self, result: &DispatchResult) -> f64 {
        let mut reserve = 0.0_f64;
        for &(gen_id, output) in &result.generator_outputs {
            if let Some(gen) = self.generators.iter().find(|g| g.id == gen_id && g.online) {
                reserve += (gen.p_max_mw - output).max(0.0);
            }
        }
        reserve
    }

    /// Decompose the system lambda into its LMP components.
    ///
    /// Returns `(energy_component, congestion_component, loss_component)` where
    /// `energy = lambda − congestion − loss`.
    pub fn compute_lmp_decomposition(
        lambda: f64,
        congestion_usd_per_mwh: f64,
        loss_usd_per_mwh: f64,
    ) -> (f64, f64, f64) {
        let energy = lambda - congestion_usd_per_mwh - loss_usd_per_mwh;
        (energy, congestion_usd_per_mwh, loss_usd_per_mwh)
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Optimal output for generator `gen` at system lambda using the equal-
    /// incremental-cost rule: `P = (λ − b) / (2c)`.
    ///
    /// For linear cost curves (`c ≈ 0`) the generator is either fully on
    /// (b ≤ λ) or off.  Result is clamped to `[p_min, p_max]`.
    fn lambda_dispatch_output(gen: &ThermalGenerator, lambda: f64) -> f64 {
        if !gen.online {
            return 0.0;
        }
        let p = if gen.cost_c_usd_per_mw2h.abs() < 1e-9 {
            if gen.cost_b_usd_per_mwh <= lambda {
                gen.p_max_mw
            } else {
                gen.p_min_mw
            }
        } else {
            (lambda - gen.cost_b_usd_per_mwh) / (2.0 * gen.cost_c_usd_per_mw2h)
        };
        p.clamp(gen.p_min_mw, gen.p_max_mw)
    }

    /// Compute net storage contribution [MW] at a given lambda.
    ///
    /// Returns `(net_mw, charge_per_unit, discharge_per_unit)`.
    /// Positive net means storage is net generating (discharging more than charging).
    fn storage_net_mw_at_lambda(&self, lambda: f64, dt_h: f64) -> (f64, Vec<f64>, Vec<f64>) {
        let mut total_net = 0.0_f64;
        let mut charge_mw = vec![0.0_f64; self.storage.len()];
        let mut discharge_mw = vec![0.0_f64; self.storage.len()];

        for (idx, bat) in self.storage.iter().enumerate() {
            let threshold = bat.charging_cost_usd_per_mwh;
            if lambda > threshold + 1e-6 {
                // Discharge: profitable when lambda > degradation cost.
                let max_e = bat.max_discharge_energy_mwh(dt_h);
                let p = (max_e / dt_h).min(bat.power_capacity_mw);
                discharge_mw[idx] = p;
                total_net += p;
            } else if lambda < threshold - 1e-6 {
                // Charge: cheap energy available to store.
                let max_e = bat.max_charge_energy_mwh(dt_h);
                let p = (max_e / dt_h).min(bat.power_capacity_mw);
                charge_mw[idx] = p;
                total_net -= p;
            }
        }
        (total_net, charge_mw, discharge_mw)
    }

    // -----------------------------------------------------------------------
    // Public dispatch solvers
    // -----------------------------------------------------------------------

    /// Solve the single-period economic dispatch via **lambda-iteration**.
    ///
    /// Performs a binary search on the system lambda (incremental cost) in
    /// `[0, 500]` USD/MWh until the power balance
    /// `Σgen + Σdischarge − Σcharge ≈ load − renewable`
    /// is satisfied to within 0.1 MW or 50 iterations, whichever comes first.
    ///
    /// Storage units participate based on whether the system lambda exceeds
    /// their individual degradation cost threshold.
    pub fn solve_lambda_iteration(&self) -> DispatchResult {
        let dt_h = self.dt_h;
        let curtailment = (self.renewable_mw - self.load_mw).max(0.0);
        let effective_renewable = self.renewable_mw - curtailment;
        let residual_load = (self.load_mw - effective_renewable).max(0.0);

        let mut lo = 0.0_f64;
        let mut hi = 500.0_f64;
        let mut lambda = (lo + hi) / 2.0;
        let tol_mw = 0.1_f64;

        let mut gen_outputs = vec![0.0_f64; self.generators.len()];
        let mut charge_mw = vec![0.0_f64; self.storage.len()];
        let mut discharge_mw = vec![0.0_f64; self.storage.len()];

        for _ in 0..50 {
            lambda = (lo + hi) / 2.0;

            let mut gen_total = 0.0_f64;
            for (i, gen) in self.generators.iter().enumerate() {
                gen_outputs[i] = Self::lambda_dispatch_output(gen, lambda);
                gen_total += gen_outputs[i];
            }

            let (storage_net, c_mw, d_mw) = self.storage_net_mw_at_lambda(lambda, dt_h);
            charge_mw = c_mw;
            discharge_mw = d_mw;

            let total_supply = gen_total + storage_net;
            let imbalance = total_supply - residual_load;

            if imbalance.abs() <= tol_mw {
                break;
            }
            if imbalance > 0.0 {
                hi = lambda;
            } else {
                lo = lambda;
            }
        }

        // Build result vectors.
        let mut generator_outputs = Vec::with_capacity(self.generators.len());
        let mut total_cost = 0.0_f64;
        for (i, gen) in self.generators.iter().enumerate() {
            if gen.online {
                let p = gen_outputs[i];
                generator_outputs.push((gen.id, p));
                total_cost += Self::compute_generator_cost(gen, p);
            }
        }

        let mut storage_charge_vec = Vec::with_capacity(self.storage.len());
        let mut storage_discharge_vec = Vec::with_capacity(self.storage.len());
        let mut storage_soc_vec = Vec::with_capacity(self.storage.len());

        for (idx, bat) in self.storage.iter().enumerate() {
            let c = charge_mw[idx];
            let d = discharge_mw[idx];
            storage_charge_vec.push((bat.id, c));
            storage_discharge_vec.push((bat.id, d));

            let eta_c = bat.charge_efficiency();
            let eta_d = bat.discharge_efficiency();
            let delta_soc = (c * eta_c - d / eta_d) * dt_h / bat.energy_capacity_mwh;
            let new_soc = (bat.soc_current + delta_soc).clamp(bat.soc_min, bat.soc_max);
            storage_soc_vec.push((bat.id, new_soc));

            total_cost += (c + d) * bat.charging_cost_usd_per_mwh * dt_h;
        }

        let load_served = self.load_mw - curtailment;

        DispatchResult {
            timestamp_h: 0,
            generator_outputs,
            storage_charge: storage_charge_vec,
            storage_discharge: storage_discharge_vec,
            storage_soc: storage_soc_vec,
            total_cost_usd: total_cost,
            lambda_usd_per_mwh: lambda,
            load_served_mw: load_served,
            renewable_mw: effective_renewable,
            curtailment_mw: curtailment,
        }
    }

    /// Solve the single-period economic dispatch via **merit-order**.
    ///
    /// Generators are sorted by their short-run marginal cost evaluated at the
    /// mid-point output `(p_min + p_max) / 2`.  They are committed in ascending
    /// cost order until the residual load (after renewable and storage) is met.
    /// The lambda is set to the marginal cost of the last dispatched generator.
    ///
    /// Storage: discharge when load > 80 % of total online capacity (peak
    /// condition); charge otherwise.
    pub fn solve_merit_order(&self) -> DispatchResult {
        let dt_h = self.dt_h;
        let curtailment = (self.renewable_mw - self.load_mw).max(0.0);
        let effective_renewable = self.renewable_mw - curtailment;

        // Sort online generators by marginal cost at mid-point output.
        let mut order: Vec<usize> = self
            .generators
            .iter()
            .enumerate()
            .filter(|(_, g)| g.online)
            .map(|(i, _)| i)
            .collect();
        order.sort_by(|&a, &b| {
            let p_avg_a = (self.generators[a].p_min_mw + self.generators[a].p_max_mw) / 2.0;
            let p_avg_b = (self.generators[b].p_min_mw + self.generators[b].p_max_mw) / 2.0;
            let mc_a = Self::compute_marginal_cost(&self.generators[a], p_avg_a);
            let mc_b = Self::compute_marginal_cost(&self.generators[b], p_avg_b);
            mc_a.partial_cmp(&mc_b).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Total online capacity for peak detection.
        let total_online_cap: f64 = order.iter().map(|&i| self.generators[i].p_max_mw).sum();
        let is_peak = total_online_cap > 0.0 && self.load_mw > 0.8 * total_online_cap;

        // Storage dispatch (peak shaving / valley filling).
        let mut charge_mw = vec![0.0_f64; self.storage.len()];
        let mut discharge_mw = vec![0.0_f64; self.storage.len()];
        let mut storage_net = 0.0_f64;

        for (idx, bat) in self.storage.iter().enumerate() {
            if is_peak {
                let max_e = bat.max_discharge_energy_mwh(dt_h);
                let p = (max_e / dt_h).min(bat.power_capacity_mw);
                discharge_mw[idx] = p;
                storage_net += p;
            } else {
                let max_e = bat.max_charge_energy_mwh(dt_h);
                let p = (max_e / dt_h).min(bat.power_capacity_mw);
                charge_mw[idx] = p;
                storage_net -= p;
            }
        }

        let mut residual = (self.load_mw - effective_renewable - storage_net).max(0.0);
        let mut gen_outputs = vec![0.0_f64; self.generators.len()];
        let mut lambda = 0.0_f64;
        let mut total_cost = 0.0_f64;

        for &idx in &order {
            if residual <= 1e-6 {
                break;
            }
            let gen = &self.generators[idx];
            // Clamp dispatch to [p_min, p_max]; take only what residual needs.
            let p = gen.p_min_mw.max(residual.min(gen.p_max_mw));
            gen_outputs[idx] = p;
            residual = (residual - p).max(0.0);
            lambda = Self::compute_marginal_cost(gen, p);
            total_cost += Self::compute_generator_cost(gen, p);
        }

        // Build result vectors.
        let mut generator_outputs = Vec::with_capacity(self.generators.len());
        for (i, gen) in self.generators.iter().enumerate() {
            if gen.online && gen_outputs[i] > 0.0 {
                generator_outputs.push((gen.id, gen_outputs[i]));
            }
        }

        let mut storage_charge_vec = Vec::with_capacity(self.storage.len());
        let mut storage_discharge_vec = Vec::with_capacity(self.storage.len());
        let mut storage_soc_vec = Vec::with_capacity(self.storage.len());

        for (idx, bat) in self.storage.iter().enumerate() {
            let c = charge_mw[idx];
            let d = discharge_mw[idx];
            storage_charge_vec.push((bat.id, c));
            storage_discharge_vec.push((bat.id, d));
            let eta_c = bat.charge_efficiency();
            let eta_d = bat.discharge_efficiency();
            let delta_soc = (c * eta_c - d / eta_d) * dt_h / bat.energy_capacity_mwh;
            let new_soc = (bat.soc_current + delta_soc).clamp(bat.soc_min, bat.soc_max);
            storage_soc_vec.push((bat.id, new_soc));
            total_cost += (c + d) * bat.charging_cost_usd_per_mwh * dt_h;
        }

        let load_served = self.load_mw - curtailment;

        DispatchResult {
            timestamp_h: 0,
            generator_outputs,
            storage_charge: storage_charge_vec,
            storage_discharge: storage_discharge_vec,
            storage_soc: storage_soc_vec,
            total_cost_usd: total_cost,
            lambda_usd_per_mwh: lambda,
            load_served_mw: load_served,
            renewable_mw: effective_renewable,
            curtailment_mw: curtailment,
        }
    }

    /// Solve a **multi-period** economic dispatch over `horizon_h` time-steps.
    ///
    /// Updates storage SoC state between periods so that consecutive periods
    /// share a continuous SoC trajectory.  For each period the lambda-iteration
    /// solver is invoked.
    ///
    /// # Arguments
    /// - `loads`      — load demand for each period [MW].
    /// - `renewable`  — must-take renewable for each period [MW].
    /// - `horizon_h`  — number of periods to dispatch (capped at `loads.len()`).
    pub fn solve_multi_period(
        &mut self,
        loads: &[f64],
        renewable: &[f64],
        horizon_h: usize,
    ) -> DispatchSolution {
        let n = horizon_h.min(loads.len()).min(renewable.len());
        let mut results = Vec::with_capacity(n);
        let mut total_cost = 0.0_f64;
        let mut total_renewable_energy = 0.0_f64;
        let mut total_load_energy = 0.0_f64;
        let mut lambda_sum = 0.0_f64;
        let mut discharge_energy = vec![0.0_f64; self.storage.len()];
        let mut violations: Vec<String> = Vec::new();

        for t in 0..n {
            self.load_mw = loads[t];
            self.renewable_mw = renewable[t];

            let mut result = self.solve_lambda_iteration();
            result.timestamp_h = t;

            // Persist SoC into the problem state for the next period.
            for (bat_idx, bat) in self.storage.iter_mut().enumerate() {
                if let Some(&(_, new_soc)) = result.storage_soc.get(bat_idx) {
                    bat.soc_current = new_soc;
                }
                if let Some(&(_, d_mw)) = result.storage_discharge.get(bat_idx) {
                    discharge_energy[bat_idx] += d_mw * self.dt_h;
                }
            }

            // Check spinning reserve.
            let reserve = self.compute_reserve_margin(&result);
            if reserve < self.spinning_reserve_mw {
                violations.push(format!(
                    "Period {t}: reserve {reserve:.1} MW < required {:.1} MW",
                    self.spinning_reserve_mw
                ));
            }

            total_cost += result.total_cost_usd;
            total_renewable_energy += result.renewable_mw * self.dt_h;
            total_load_energy += result.load_served_mw * self.dt_h;
            lambda_sum += result.lambda_usd_per_mwh;
            results.push(result);
        }

        let storage_cycles: Vec<f64> = self
            .storage
            .iter()
            .enumerate()
            .map(|(i, bat)| {
                if bat.energy_capacity_mwh > 0.0 {
                    discharge_energy[i] / bat.energy_capacity_mwh
                } else {
                    0.0
                }
            })
            .collect();

        let total_renewable_pct = if total_load_energy > 0.0 {
            total_renewable_energy / total_load_energy
        } else {
            0.0
        };

        let avg_lambda = if n > 0 { lambda_sum / n as f64 } else { 0.0 };

        DispatchSolution {
            results,
            total_cost_usd: total_cost,
            total_renewable_pct,
            avg_lambda_usd_per_mwh: avg_lambda,
            storage_cycles,
            constraint_violations: violations,
        }
    }
}

// ---------------------------------------------------------------------------
// Post-processing analytics
// ---------------------------------------------------------------------------

/// Post-processing analytics for dispatch results.
pub struct DispatchAnalytics;

impl DispatchAnalytics {
    /// Heat rate of `gen` at `output_mw` [BTU/kWh].
    ///
    /// Uses the quadratic model: `HR = 3412 · (a/P + b + c·P)`.
    /// Returns 0 if `output_mw ≤ 0`.
    pub fn compute_heat_rate_btu_per_kwh(gen: &ThermalGenerator, output_mw: f64) -> f64 {
        if output_mw <= 0.0 {
            return 0.0;
        }
        3412.0
            * (gen.cost_a_usd_per_h / output_mw
                + gen.cost_b_usd_per_mwh
                + gen.cost_c_usd_per_mw2h * output_mw)
    }

    /// CO₂ emissions rate from `gen` at `output_mw` [kg CO₂/h].
    ///
    /// Emission factors per technology (kg CO₂ per kWh of output):
    /// Coal 0.90, NaturalGas 0.45, CombinedCycle 0.35, Peaker 0.50,
    /// Nuclear 0.005, Hydro/PumpedHydro 0.01.
    pub fn estimate_emissions_kg_co2_per_h(gen: &ThermalGenerator, output_mw: f64) -> f64 {
        let factor = match gen.gen_type {
            GeneratorType::Coal => 0.90,
            GeneratorType::NaturalGas => 0.45,
            GeneratorType::CombinedCycle => 0.35,
            GeneratorType::Peaker => 0.50,
            GeneratorType::Nuclear => 0.005,
            GeneratorType::Hydro | GeneratorType::PumpedHydro => 0.01,
        };
        factor * output_mw * 1000.0
    }

    /// Capacity factor: ratio of average output to rated capacity.
    ///
    /// Returns a value in `[0, 1]`.  Returns 0 if `p_max ≤ 0` or `outputs` is empty.
    pub fn compute_capacity_factor(outputs: &[f64], p_max: f64) -> f64 {
        if p_max <= 0.0 || outputs.is_empty() {
            return 0.0;
        }
        let avg = outputs.iter().copied().sum::<f64>() / outputs.len() as f64;
        (avg / p_max).clamp(0.0, 1.0)
    }

    /// Identify the marginal generating unit in `result`.
    ///
    /// The marginal unit is the online generator whose incremental cost at its
    /// dispatched output is closest to `result.lambda_usd_per_mwh`.
    /// Returns the generator `id` (not its index in the slice).
    pub fn identify_marginal_unit(
        result: &DispatchResult,
        generators: &[ThermalGenerator],
    ) -> Option<usize> {
        let lambda = result.lambda_usd_per_mwh;
        let mut best_id: Option<usize> = None;
        let mut best_diff = f64::MAX;

        for &(gen_id, output) in &result.generator_outputs {
            if let Some(gen) = generators.iter().find(|g| g.id == gen_id) {
                let mc = EconomicDispatchProblem::compute_marginal_cost(gen, output);
                let diff = (mc - lambda).abs();
                if diff < best_diff {
                    best_diff = diff;
                    best_id = Some(gen_id);
                }
            }
        }
        best_id
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn coal_gen() -> ThermalGenerator {
        ThermalGenerator {
            id: 1,
            name: "Coal-1".into(),
            gen_type: GeneratorType::Coal,
            p_min_mw: 50.0,
            p_max_mw: 300.0,
            cost_a_usd_per_h: 500.0,
            cost_b_usd_per_mwh: 20.0,
            cost_c_usd_per_mw2h: 0.05,
            ramp_up_mw_per_min: 2.0,
            ramp_down_mw_per_min: 2.0,
            startup_cost_usd: 5000.0,
            shutdown_cost_usd: 1000.0,
            min_up_time_h: 4.0,
            min_down_time_h: 4.0,
            online: true,
            current_output_mw: 150.0,
        }
    }

    fn gas_gen() -> ThermalGenerator {
        ThermalGenerator {
            id: 2,
            name: "Gas-1".into(),
            gen_type: GeneratorType::NaturalGas,
            p_min_mw: 10.0,
            p_max_mw: 150.0,
            cost_a_usd_per_h: 100.0,
            cost_b_usd_per_mwh: 40.0,
            cost_c_usd_per_mw2h: 0.10,
            ramp_up_mw_per_min: 5.0,
            ramp_down_mw_per_min: 5.0,
            startup_cost_usd: 2000.0,
            shutdown_cost_usd: 500.0,
            min_up_time_h: 1.0,
            min_down_time_h: 1.0,
            online: true,
            current_output_mw: 80.0,
        }
    }

    fn nuclear_gen() -> ThermalGenerator {
        ThermalGenerator {
            id: 3,
            name: "Nuclear-1".into(),
            gen_type: GeneratorType::Nuclear,
            p_min_mw: 200.0,
            p_max_mw: 1000.0,
            cost_a_usd_per_h: 2000.0,
            cost_b_usd_per_mwh: 8.0,
            cost_c_usd_per_mw2h: 0.002,
            ramp_up_mw_per_min: 1.0,
            ramp_down_mw_per_min: 1.0,
            startup_cost_usd: 100_000.0,
            shutdown_cost_usd: 50_000.0,
            min_up_time_h: 24.0,
            min_down_time_h: 48.0,
            online: true,
            current_output_mw: 800.0,
        }
    }

    fn sample_battery() -> StorageBattery {
        StorageBattery {
            id: 10,
            name: "BESS-1".into(),
            energy_capacity_mwh: 100.0,
            power_capacity_mw: 25.0,
            roundtrip_efficiency: 0.90,
            soc_min: 0.10,
            soc_max: 0.90,
            soc_current: 0.50,
            charging_cost_usd_per_mwh: 30.0,
            mode: StorageDispatchMode::EnergyArbitrage,
        }
    }

    fn two_gen_problem(load_mw: f64) -> EconomicDispatchProblem {
        EconomicDispatchProblem {
            generators: vec![coal_gen(), gas_gen()],
            storage: vec![],
            load_mw,
            renewable_mw: 0.0,
            spinning_reserve_mw: 20.0,
            dt_h: 1.0,
        }
    }

    // -----------------------------------------------------------------------
    // Cost model tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_generator_cost_quadratic() {
        let gen = coal_gen();
        let p = 100.0_f64;
        let expected = 500.0 + 20.0 * p + 0.05 * p * p;
        let actual = EconomicDispatchProblem::compute_generator_cost(&gen, p);
        assert!((actual - expected).abs() < 1e-6, "Cost mismatch: {actual}");
    }

    #[test]
    fn test_marginal_cost_linear() {
        let gen = coal_gen();
        let p = 100.0_f64;
        let expected = 20.0 + 2.0 * 0.05 * p;
        let actual = EconomicDispatchProblem::compute_marginal_cost(&gen, p);
        assert!((actual - expected).abs() < 1e-9, "MC mismatch: {actual}");
    }

    // -----------------------------------------------------------------------
    // Lambda-iteration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lambda_iteration_balances_load() {
        let prob = two_gen_problem(200.0);
        let result = prob.solve_lambda_iteration();
        let gen_total: f64 = result.generator_outputs.iter().map(|&(_, p)| p).sum();
        let charge: f64 = result.storage_charge.iter().map(|&(_, c)| c).sum();
        let discharge: f64 = result.storage_discharge.iter().map(|&(_, d)| d).sum();
        let net_supply = gen_total + discharge - charge;
        assert!(
            (net_supply - 200.0).abs() <= 1.0,
            "Balance error: supply={net_supply:.2} MW"
        );
    }

    #[test]
    fn test_lambda_iteration_convergence() {
        let prob = two_gen_problem(300.0);
        let result = prob.solve_lambda_iteration();
        assert!(result.lambda_usd_per_mwh.is_finite());
        assert!(result.lambda_usd_per_mwh >= 0.0);
    }

    // -----------------------------------------------------------------------
    // Merit-order tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_merit_order_sort() {
        // Coal has lower b (20) than Gas (40), so coal dispatched first.
        let prob = two_gen_problem(100.0);
        let result = prob.solve_merit_order();
        let coal_out = result
            .generator_outputs
            .iter()
            .find(|&&(id, _)| id == 1)
            .map(|&(_, p)| p)
            .unwrap_or(0.0);
        assert!(
            coal_out >= 50.0,
            "Coal should be dispatched first, got {coal_out} MW"
        );
    }

    #[test]
    fn test_merit_order_dispatch_simple() {
        let prob = two_gen_problem(60.0);
        let result = prob.solve_merit_order();
        assert!(
            !result.generator_outputs.is_empty(),
            "Should have dispatched generators"
        );
    }

    // -----------------------------------------------------------------------
    // Ramp constraint tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ramp_constraints() {
        let prob = two_gen_problem(200.0);
        let gen = coal_gen(); // ramp 2 MW/min => 120 MW/h
        let (min_f, max_f) = prob.check_ramp_constraints(150.0, &gen, 1.0);
        // ramp up: 150 + 120 = 270, capped at p_max=300
        // ramp dn: 150 - 120 = 30, capped at p_min=50
        assert!((max_f - 270.0_f64.min(300.0)).abs() < 1e-6, "max_f={max_f}");
        assert!((min_f - 50.0_f64).abs() < 1e-6, "min_f={min_f}");
    }

    #[test]
    fn test_ramp_min_max_bounds() {
        let prob = two_gen_problem(100.0);
        let gen = coal_gen();
        let (min_f, _) = prob.check_ramp_constraints(50.0, &gen, 1.0);
        assert!(min_f >= gen.p_min_mw - 1e-6, "min_f={min_f} < p_min");
    }

    // -----------------------------------------------------------------------
    // Storage SoC tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_storage_charges_off_peak() {
        // Light load => low lambda => battery should charge.
        let prob = EconomicDispatchProblem {
            generators: vec![coal_gen()],
            storage: vec![sample_battery()], // threshold 30 $/MWh
            load_mw: 60.0,
            renewable_mw: 0.0,
            spinning_reserve_mw: 0.0,
            dt_h: 1.0,
        };
        let result = prob.solve_lambda_iteration();
        let soc_after = result.storage_soc.first().map(|&(_, s)| s).unwrap_or(0.5);
        // SoC should be >= initial (charging) or unchanged.
        assert!(
            soc_after >= 0.50 - 1e-6,
            "Expected SoC >= 0.50, got {soc_after:.4}"
        );
    }

    #[test]
    fn test_storage_discharges_peak() {
        // Heavy load => high lambda => battery discharges.
        let prob = EconomicDispatchProblem {
            generators: vec![coal_gen(), gas_gen()],
            storage: vec![sample_battery()],
            load_mw: 440.0,
            renewable_mw: 0.0,
            spinning_reserve_mw: 0.0,
            dt_h: 1.0,
        };
        let result = prob.solve_lambda_iteration();
        let discharge = result
            .storage_discharge
            .first()
            .map(|&(_, d)| d)
            .unwrap_or(0.0);
        assert!(discharge >= 0.0, "discharge={discharge}");
    }

    // -----------------------------------------------------------------------
    // Reserve and renewable tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_reserve_margin_positive() {
        let prob = two_gen_problem(200.0);
        let result = prob.solve_lambda_iteration();
        let reserve = prob.compute_reserve_margin(&result);
        assert!(reserve >= 0.0, "reserve={reserve}");
    }

    #[test]
    fn test_renewable_curtailment() {
        let prob = EconomicDispatchProblem {
            generators: vec![coal_gen()],
            storage: vec![],
            load_mw: 100.0,
            renewable_mw: 150.0,
            spinning_reserve_mw: 0.0,
            dt_h: 1.0,
        };
        let result = prob.solve_lambda_iteration();
        assert!(
            result.curtailment_mw > 0.0,
            "Expected curtailment, got {:.2}",
            result.curtailment_mw
        );
    }

    // -----------------------------------------------------------------------
    // Multi-period tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_period_soc_continuity() {
        let mut prob = EconomicDispatchProblem {
            generators: vec![coal_gen(), gas_gen()],
            storage: vec![sample_battery()],
            load_mw: 200.0,
            renewable_mw: 0.0,
            spinning_reserve_mw: 10.0,
            dt_h: 1.0,
        };
        let loads = vec![180.0, 220.0, 200.0, 240.0];
        let renew = vec![20.0, 10.0, 30.0, 5.0];
        let sol = prob.solve_multi_period(&loads, &renew, 4);
        assert_eq!(sol.results.len(), 4);
        for r in &sol.results {
            for &(_, soc) in &r.storage_soc {
                assert!((0.10 - 1e-6..=0.90 + 1e-6).contains(&soc), "SoC={soc}");
            }
        }
    }

    // -----------------------------------------------------------------------
    // LMP decomposition test
    // -----------------------------------------------------------------------

    #[test]
    fn test_lmp_decomposition_sum() {
        let lambda = 45.0;
        let cong = 5.0;
        let loss = 2.0;
        let (energy, c, l) = EconomicDispatchProblem::compute_lmp_decomposition(lambda, cong, loss);
        assert!(
            (energy + c + l - lambda).abs() < 1e-9,
            "sum={:.6}",
            energy + c + l
        );
    }

    // -----------------------------------------------------------------------
    // Analytics tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_heat_rate_reasonable() {
        // Use a generator with fuel-curve coefficients in physical BTU units:
        // a = 200 BTU/h (no-load heat), b = 8500 BTU/MWh, c = 0.5 BTU/MW²h
        // HR(150 MW) = 3412 * (200/150 + 8500 + 0.5*150) ≈ 3412 * 8576.3 ≈ but
        // the 3412 factor converts cost-curve units to BTU/kWh only when
        // a,b,c are in $/h, $/MWh, $/MW²h with a heat-rate multiplier embedded.
        // Here we scale coefficients so HR lands in [7000, 15000] BTU/kWh.
        let gen = ThermalGenerator {
            id: 99,
            name: "Coal-HR-Test".into(),
            gen_type: GeneratorType::Coal,
            p_min_mw: 50.0,
            p_max_mw: 300.0,
            // a=0 eliminates divergence at low P; b=2.5, c=0.001 → HR ≈ 3412*(2.5+0.15) ≈ 9042
            cost_a_usd_per_h: 0.0,
            cost_b_usd_per_mwh: 2.5,
            cost_c_usd_per_mw2h: 0.001,
            ramp_up_mw_per_min: 2.0,
            ramp_down_mw_per_min: 2.0,
            startup_cost_usd: 0.0,
            shutdown_cost_usd: 0.0,
            min_up_time_h: 0.0,
            min_down_time_h: 0.0,
            online: true,
            current_output_mw: 150.0,
        };
        let hr = DispatchAnalytics::compute_heat_rate_btu_per_kwh(&gen, 150.0);
        assert!(
            (7000.0..=15_000.0).contains(&hr),
            "Heat rate {hr:.0} BTU/kWh out of expected range [7000, 15000]"
        );
    }

    #[test]
    fn test_emissions_coal_highest() {
        let p = 100.0;
        let coal = coal_gen();
        let gas = gas_gen();
        let nuc = nuclear_gen();
        let e_coal = DispatchAnalytics::estimate_emissions_kg_co2_per_h(&coal, p);
        let e_gas = DispatchAnalytics::estimate_emissions_kg_co2_per_h(&gas, p);
        let e_nuc = DispatchAnalytics::estimate_emissions_kg_co2_per_h(&nuc, p);
        assert!(e_coal > e_gas, "coal={e_coal} gas={e_gas}");
        assert!(e_gas > e_nuc, "gas={e_gas} nuc={e_nuc}");
    }

    #[test]
    fn test_capacity_factor_range() {
        let outputs = vec![50.0, 100.0, 150.0, 200.0];
        let cf = DispatchAnalytics::compute_capacity_factor(&outputs, 300.0);
        assert!((0.0..=1.0).contains(&cf), "CF={cf}");
    }

    #[test]
    fn test_marginal_unit_identified() {
        let prob = two_gen_problem(200.0);
        let result = prob.solve_lambda_iteration();
        let mu = DispatchAnalytics::identify_marginal_unit(&result, &prob.generators);
        assert!(mu.is_some(), "Should identify marginal unit");
    }

    // -----------------------------------------------------------------------
    // Edge-case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_no_generators_online() {
        let mut gen = coal_gen();
        gen.online = false;
        let prob = EconomicDispatchProblem {
            generators: vec![gen],
            storage: vec![],
            load_mw: 100.0,
            renewable_mw: 0.0,
            spinning_reserve_mw: 0.0,
            dt_h: 1.0,
        };
        let result = prob.solve_lambda_iteration();
        let total: f64 = result.generator_outputs.iter().map(|&(_, p)| p).sum();
        assert_eq!(total, 0.0, "offline gen should contribute 0 MW");
    }

    #[test]
    fn test_single_generator_dispatch() {
        let prob = EconomicDispatchProblem {
            generators: vec![coal_gen()],
            storage: vec![],
            load_mw: 200.0,
            renewable_mw: 0.0,
            spinning_reserve_mw: 0.0,
            dt_h: 1.0,
        };
        let result = prob.solve_lambda_iteration();
        let gen_total: f64 = result.generator_outputs.iter().map(|&(_, p)| p).sum();
        assert!(
            (gen_total - 200.0).abs() <= 1.0,
            "gen_total={gen_total:.2} MW"
        );
    }

    #[test]
    fn test_capacity_factor_zero_p_max() {
        let cf = DispatchAnalytics::compute_capacity_factor(&[100.0], 0.0);
        assert_eq!(cf, 0.0);
    }

    #[test]
    fn test_heat_rate_zero_output() {
        let gen = coal_gen();
        let hr = DispatchAnalytics::compute_heat_rate_btu_per_kwh(&gen, 0.0);
        assert_eq!(hr, 0.0);
    }
}
