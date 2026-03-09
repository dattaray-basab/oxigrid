//! Advanced Microgrid Energy Management System
//!
//! Provides a comprehensive suite for microgrid optimization and control:
//!
//! - **Multi-objective optimizer** — cost, carbon and resilience objectives with Pareto front
//! - **Hierarchical control** — tertiary (economic), secondary (restoration), primary (droop)
//! - **Scenario-based stochastic dispatch** — here-and-now solve + VSS metric
//! - **Islanding transition controller** — feasibility check, autonomous step, reconnection
//! - **Peer-to-peer energy market** — double-auction clearing within the microgrid
//! - **KPI dashboard** — self-consumption, self-sufficiency, peak shaving, CO2 avoided

use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════
// §1  Shared resource description
// ═══════════════════════════════════════════════════════════════════════════

/// Physical and economic description of microgrid dispatchable resources.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MicrogridResources {
    /// Usable battery energy capacity \[MWh\]
    pub battery_mwh: f64,
    /// Battery maximum charge / discharge power \[MW\]
    pub battery_mw: f64,
    /// Battery minimum state-of-charge \[pu\]
    pub battery_soc_min: f64,
    /// Battery maximum state-of-charge \[pu\]
    pub battery_soc_max: f64,
    /// Battery round-trip efficiency \[pu\]
    pub battery_rte: f64,
    /// Diesel generator maximum output \[MW\]
    pub diesel_mw: f64,
    /// Diesel variable cost \[\$/MWh\]
    pub diesel_cost_per_mwh: f64,
    /// Diesel CO2 emission intensity \[g/kWh\]
    pub diesel_emissions_g_per_kwh: f64,
    /// Maximum grid import power \[MW\]
    pub grid_import_limit_mw: f64,
    /// Maximum grid export power \[MW\]
    pub grid_export_limit_mw: f64,
    /// Hourly grid import price \[\$/MWh\]
    pub grid_import_price: Vec<f64>,
    /// Hourly grid export price \[\$/MWh\]
    pub grid_export_price: Vec<f64>,
}

impl MicrogridResources {
    /// Convenience constructor with flat import/export prices over `horizon` steps.
    pub fn new_flat(
        battery_mwh: f64,
        battery_mw: f64,
        diesel_mw: f64,
        horizon: usize,
        import_price: f64,
        export_price: f64,
    ) -> Self {
        Self {
            battery_mwh,
            battery_mw,
            battery_soc_min: 0.1,
            battery_soc_max: 0.9,
            battery_rte: 0.92,
            diesel_mw,
            diesel_cost_per_mwh: 250.0,
            diesel_emissions_g_per_kwh: 680.0,
            grid_import_limit_mw: 5.0,
            grid_export_limit_mw: 3.0,
            grid_import_price: vec![import_price; horizon],
            grid_export_price: vec![export_price; horizon],
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §2  Dispatch schedule (shared output type)
// ═══════════════════════════════════════════════════════════════════════════

/// Hourly dispatch schedule produced by any optimizer in this module.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MgDispatchSchedule {
    /// Battery power per interval \[MW\]; positive = charge, negative = discharge
    pub battery_mw: Vec<f64>,
    /// Diesel generator power per interval \[MW\]
    pub diesel_mw: Vec<f64>,
    /// Grid import power per interval \[MW\]
    pub grid_import_mw: Vec<f64>,
    /// Grid export power per interval \[MW\]
    pub grid_export_mw: Vec<f64>,
    /// Battery state-of-charge trajectory \[pu\], length = horizon + 1
    pub soc_trajectory: Vec<f64>,
    /// Total schedule cost \[\$\]
    pub total_cost: f64,
    /// Total CO2 emissions \[g\]
    pub total_carbon_g: f64,
    /// Energy stored in battery at end of schedule, proxy for islanding reserve \[MWh\]
    pub islanding_reserve_mwh: f64,
}

impl MgDispatchSchedule {
    fn zero(horizon: usize, initial_soc: f64) -> Self {
        Self {
            battery_mw: vec![0.0; horizon],
            diesel_mw: vec![0.0; horizon],
            grid_import_mw: vec![0.0; horizon],
            grid_export_mw: vec![0.0; horizon],
            soc_trajectory: vec![initial_soc; horizon + 1],
            total_cost: 0.0,
            total_carbon_g: 0.0,
            islanding_reserve_mwh: 0.0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §3  Multi-Objective Microgrid Optimizer
// ═══════════════════════════════════════════════════════════════════════════

/// Multi-objective microgrid optimizer supporting cost, carbon and resilience objectives.
///
/// Uses greedy priority-dispatch with per-step marginal-cost ordering.  The
/// three single-objective solves provide anchor points; `pareto_front` sweeps
/// weight combinations via weighted-sum scalarization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiObjectiveMgOptimizer {
    /// Optimization horizon \[h\]
    pub horizon_hours: usize,
    /// Time-step duration \[h\]
    pub dt_h: f64,
    /// Renewable generation forecast, one entry per step \[MW\]
    pub renewable_forecast_mw: Vec<f64>,
    /// Load demand forecast, one entry per step \[MW\]
    pub load_forecast_mw: Vec<f64>,
    /// Spot electricity price for diesel/import decisions, one per step \[\$/MWh\]
    pub electricity_price: Vec<f64>,
    /// Grid carbon intensity, one entry per step \[g CO2/kWh\]
    pub carbon_intensity_g_co2_per_kwh: Vec<f64>,
    /// Dispatchable resources available to the optimizer
    pub resources: MicrogridResources,
}

/// A point on the Pareto front of the multi-objective problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParetoPoint {
    /// Weight on cost objective \[pu\]
    pub weight_cost: f64,
    /// Weight on carbon objective \[pu\]
    pub weight_carbon: f64,
    /// Realized total cost \[\$\]
    pub total_cost: f64,
    /// Realized total carbon \[g CO2\]
    pub total_carbon_g: f64,
    /// Full dispatch schedule
    pub schedule: MgDispatchSchedule,
}

impl MultiObjectiveMgOptimizer {
    /// Solve minimizing total cost only (weight_cost = 1, weight_carbon = 0).
    pub fn solve_cost_optimal(&self) -> MgDispatchSchedule {
        self.solve_weighted(1.0, 0.0, 0.0)
    }

    /// Solve minimizing total carbon only (weight_cost = 0, weight_carbon = 1).
    pub fn solve_carbon_optimal(&self) -> MgDispatchSchedule {
        self.solve_weighted(0.0, 1.0, 0.0)
    }

    /// Solve maximizing battery SoC at end of horizon (islanding readiness).
    pub fn solve_resilience_optimal(&self) -> MgDispatchSchedule {
        self.solve_weighted(0.0, 0.0, 1.0)
    }

    /// Compute the Pareto front via weighted-sum scalarization over `n_solutions` combinations.
    ///
    /// Weights are drawn on a 2-D simplex (cost + carbon); resilience weight is fixed at 0 so
    /// the front can be plotted in the cost–carbon plane.
    pub fn pareto_front(&self, n_solutions: usize) -> Vec<ParetoPoint> {
        let n = n_solutions.max(2);
        (0..n)
            .map(|i| {
                let w_cost = i as f64 / (n - 1) as f64;
                let w_carbon = 1.0 - w_cost;
                let schedule = self.solve_weighted(w_cost, w_carbon, 0.0);
                ParetoPoint {
                    weight_cost: w_cost,
                    weight_carbon: w_carbon,
                    total_cost: schedule.total_cost,
                    total_carbon_g: schedule.total_carbon_g,
                    schedule,
                }
            })
            .collect()
    }

    // ── internal greedy dispatch ──────────────────────────────────────────

    /// Core greedy dispatcher.
    ///
    /// Each step the net deficit / surplus is covered by choosing the action
    /// with the lowest *scalarized* marginal cost:
    ///   scalar_cost = w_cost * cost_per_mwh + w_carbon * carbon_per_mwh_normalised
    ///                 + w_resilience * resilience_penalty
    fn solve_weighted(&self, w_cost: f64, w_carbon: f64, w_resilience: f64) -> MgDispatchSchedule {
        let h = self.horizon_hours;
        let dt = self.dt_h;
        let res = &self.resources;

        // Normalisation constants to bring cost and carbon to comparable scales.
        let cost_norm = 500.0_f64; // $/MWh — typical max import price
        let carbon_norm = 1000.0_f64; // g/kWh * 1000 kWh/MWh => same units as diesel

        let mut soc = (res.battery_soc_min + res.battery_soc_max) / 2.0;
        let mut schedule = MgDispatchSchedule::zero(h, soc);

        let mut total_cost = 0.0_f64;
        let mut total_carbon = 0.0_f64;

        for t in 0..h {
            let re = self.renewable_forecast_mw.get(t).copied().unwrap_or(0.0);
            let load = self.load_forecast_mw.get(t).copied().unwrap_or(0.0);
            let imp_price = res.grid_import_price.get(t).copied().unwrap_or(100.0);
            let exp_price = res.grid_export_price.get(t).copied().unwrap_or(50.0);
            let ci = self
                .carbon_intensity_g_co2_per_kwh
                .get(t)
                .copied()
                .unwrap_or(300.0);

            // Net load after renewables \[MW\]
            let net = load - re;

            let (b_mw, d_mw, imp, exp, step_cost, step_carbon) = if net > 0.0 {
                // Demand exceeds renewable — need supply
                self.dispatch_supply(
                    net,
                    soc,
                    imp_price,
                    exp_price,
                    ci,
                    res,
                    dt,
                    w_cost,
                    w_carbon,
                    w_resilience,
                    cost_norm,
                    carbon_norm,
                )
            } else {
                // Surplus renewable — charge battery or export
                self.dispatch_surplus(
                    -net,
                    soc,
                    imp_price,
                    exp_price,
                    res,
                    dt,
                    w_cost,
                    w_carbon,
                    w_resilience,
                )
            };

            // Apply battery SoC update
            soc = if b_mw >= 0.0 {
                // Charging
                (soc + b_mw * dt * res.battery_rte.sqrt() / res.battery_mwh)
                    .min(res.battery_soc_max)
            } else {
                // Discharging
                (soc + b_mw * dt / (res.battery_rte.sqrt() * res.battery_mwh))
                    .max(res.battery_soc_min)
            };

            schedule.battery_mw[t] = b_mw;
            schedule.diesel_mw[t] = d_mw;
            schedule.grid_import_mw[t] = imp;
            schedule.grid_export_mw[t] = exp;
            schedule.soc_trajectory[t + 1] = soc;
            total_cost += step_cost;
            total_carbon += step_carbon;
        }

        let final_soc = *schedule.soc_trajectory.last().unwrap_or(&0.0);
        schedule.total_cost = total_cost;
        schedule.total_carbon_g = total_carbon;
        schedule.islanding_reserve_mwh = final_soc * res.battery_mwh;
        schedule
    }

    #[allow(clippy::too_many_arguments)]
    fn dispatch_supply(
        &self,
        net: f64,
        soc: f64,
        imp_price: f64,
        _exp_price: f64,
        ci: f64,
        res: &MicrogridResources,
        dt: f64,
        w_cost: f64,
        w_carbon: f64,
        w_resilience: f64,
        cost_norm: f64,
        carbon_norm: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        // Available battery discharge \[MW\]
        let batt_avail = ((soc - res.battery_soc_min) * res.battery_mwh
            / (dt * res.battery_rte.sqrt()))
        .min(res.battery_mw)
        .max(0.0);

        // Scalarized marginal costs
        let batt_score = w_resilience * 1.0; // battery discharge costs resilience
        let diesel_score = w_cost * res.diesel_cost_per_mwh / cost_norm
            + w_carbon * res.diesel_emissions_g_per_kwh / carbon_norm;
        let import_score = w_cost * imp_price / cost_norm + w_carbon * ci / carbon_norm;

        // Rank sources: lowest score first
        let mut sources: Vec<(f64, &str)> = vec![
            (batt_score, "battery"),
            (diesel_score, "diesel"),
            (import_score, "import"),
        ];
        sources.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let mut remaining = net;
        let mut b_mw = 0.0_f64;
        let mut d_mw = 0.0_f64;
        let mut imp = 0.0_f64;

        for (_, src) in &sources {
            if remaining <= 1e-9 {
                break;
            }
            match *src {
                "battery" => {
                    let used = remaining.min(batt_avail);
                    b_mw = -used; // negative = discharge
                    remaining -= used;
                }
                "diesel" => {
                    let used = remaining.min(res.diesel_mw);
                    d_mw = used;
                    remaining -= used;
                }
                "import" => {
                    let used = remaining.min(res.grid_import_limit_mw);
                    imp = used;
                    remaining -= used;
                }
                _ => {}
            }
        }

        let step_cost = d_mw * res.diesel_cost_per_mwh * dt + imp * imp_price * dt;
        // carbon: diesel in g/kWh * kWh, import in g/kWh * kWh
        let step_carbon =
            d_mw * 1000.0 * dt * res.diesel_emissions_g_per_kwh + imp * 1000.0 * dt * ci;

        (b_mw, d_mw, imp, 0.0, step_cost, step_carbon)
    }

    #[allow(clippy::too_many_arguments)]
    fn dispatch_surplus(
        &self,
        surplus: f64,
        soc: f64,
        _imp_price: f64,
        exp_price: f64,
        res: &MicrogridResources,
        dt: f64,
        _w_cost: f64,
        _w_carbon: f64,
        w_resilience: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        // Available battery charge headroom \[MW\]
        let batt_cap = ((res.battery_soc_max - soc) * res.battery_mwh
            / (dt * res.battery_rte.sqrt()))
        .min(res.battery_mw)
        .max(0.0);

        let mut remaining = surplus;
        let (b_mw, exp);

        // Resilience: prefer charging; otherwise export
        if w_resilience > 0.5 {
            // Charge first
            let charged = remaining.min(batt_cap);
            b_mw = charged;
            remaining -= charged;
            exp = remaining.min(res.grid_export_limit_mw);
        } else {
            // Export first (economic mode: capture export revenue)
            let exported = remaining.min(res.grid_export_limit_mw);
            exp = exported;
            remaining -= exported;
            b_mw = remaining.min(batt_cap);
        }

        let step_cost = -exp * exp_price * dt; // negative cost = revenue
        (b_mw, 0.0, 0.0, exp, step_cost, 0.0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §4  Hierarchical Control Architecture
// ═══════════════════════════════════════════════════════════════════════════

/// Economic setpoint issued by tertiary control.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EconomicSetpoint {
    /// Index of the DER resource
    pub resource_id: usize,
    /// Active power setpoint \[kW\]
    pub p_setpoint_kw: f64,
    /// Reactive power setpoint \[kVAr\]
    pub q_setpoint_kvar: f64,
    /// Time until next update \[min\]
    pub valid_until_min: f64,
}

/// Tertiary control layer — economic optimization on hour/sub-hour intervals.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TertiaryControl {
    /// Update interval \[min\]
    pub interval_min: f64,
    /// Current economic setpoints, one per DER
    pub economic_setpoints: Vec<EconomicSetpoint>,
}

/// Secondary control layer — integral restoration of voltage and frequency.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondaryControl {
    /// Voltage reference \[pu\]
    pub v_ref_pu: f64,
    /// Frequency reference \[Hz\]
    pub f_ref_hz: f64,
    /// Integral gain for voltage restoration \[pu correction/pu·s\]
    pub integral_gain_v: f64,
    /// Integral gain for frequency restoration \[pu correction/Hz·s\]
    pub integral_gain_f: f64,
    /// Accumulated voltage error integral
    pub v_error_integral: f64,
    /// Accumulated frequency error integral
    pub f_error_integral: f64,
}

/// Primary control layer — local droop for immediate power sharing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrimaryControl {
    /// Voltage droop coefficient \[%\]
    pub v_droop_pct: f64,
    /// Frequency droop coefficient \[%\]
    pub f_droop_pct: f64,
    /// Reactive power – voltage droop slope \[MVAr/pu\]
    pub q_v_slope: f64,
    /// Active power – frequency droop slope \[MW/Hz\]
    pub p_f_slope: f64,
}

/// Three-level hierarchical microgrid controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HierarchicalMgControl {
    /// Tertiary (economic) control layer
    pub tertiary: TertiaryControl,
    /// Secondary (restoration) control layer
    pub secondary: SecondaryControl,
    /// Primary (droop) control layer
    pub primary: PrimaryControl,
}

impl HierarchicalMgControl {
    /// Construct a standard hierarchical controller with sensible defaults.
    pub fn new_standard() -> Self {
        Self {
            tertiary: TertiaryControl {
                interval_min: 15.0,
                economic_setpoints: Vec::new(),
            },
            secondary: SecondaryControl {
                v_ref_pu: 1.0,
                f_ref_hz: 50.0,
                integral_gain_v: 0.5,
                integral_gain_f: 0.3,
                v_error_integral: 0.0,
                f_error_integral: 0.0,
            },
            primary: PrimaryControl {
                v_droop_pct: 5.0,
                f_droop_pct: 4.0,
                q_v_slope: 2.0,
                p_f_slope: 10.0,
            },
        }
    }

    /// Advance the secondary control integrator by one time-step.
    ///
    /// Returns `(dV_correction [pu], df_correction [Hz])` to be applied to
    /// DER voltage/frequency references.
    pub fn step_secondary(&mut self, v_meas_pu: f64, f_meas_hz: f64, dt_s: f64) -> (f64, f64) {
        let v_err = self.secondary.v_ref_pu - v_meas_pu;
        let f_err = self.secondary.f_ref_hz - f_meas_hz;

        self.secondary.v_error_integral += v_err * dt_s;
        self.secondary.f_error_integral += f_err * dt_s;

        let dv = self.secondary.integral_gain_v * self.secondary.v_error_integral;
        let df = self.secondary.integral_gain_f * self.secondary.f_error_integral;
        (dv, df)
    }

    /// Push a new economic setpoint into the tertiary layer.
    pub fn apply_tertiary_setpoint(&mut self, setpoint: EconomicSetpoint) {
        if let Some(existing) = self
            .tertiary
            .economic_setpoints
            .iter_mut()
            .find(|s| s.resource_id == setpoint.resource_id)
        {
            *existing = setpoint;
        } else {
            self.tertiary.economic_setpoints.push(setpoint);
        }
    }

    /// Compute primary droop response given measured voltage and frequency deviations.
    ///
    /// Returns `(dP_MW, dQ_MVAr)`.
    pub fn primary_response(&self, v_pu: f64, f_hz: f64) -> (f64, f64) {
        let dv = v_pu - 1.0;
        let df = f_hz - self.secondary.f_ref_hz;
        let dq = -self.primary.q_v_slope * dv; // under-voltage → inject Q
        let dp = -self.primary.p_f_slope * df; // under-frequency → inject P
        (dp, dq)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §5  Stochastic Scenario-Based Dispatch
// ═══════════════════════════════════════════════════════════════════════════

/// A single Monte-Carlo scenario representing one plausible future.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MgScenario {
    /// Scenario identifier
    pub id: usize,
    /// Renewable generation per interval \[MW\]
    pub renewable_mw: Vec<f64>,
    /// Load demand per interval \[MW\]
    pub load_mw: Vec<f64>,
    /// Electricity price per interval \[\$/MWh\]
    pub price_mwh: Vec<f64>,
    /// Scenario probability \[pu\]
    pub probability: f64,
}

/// Outcome of evaluating a fixed schedule against one scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioResult {
    /// Scenario identifier
    pub scenario_id: usize,
    /// Actual cost under this scenario \[\$\]
    pub realized_cost: f64,
    /// Energy not served \[MWh\]
    pub load_shed_mwh: f64,
    /// Renewable energy curtailed \[MWh\]
    pub renewable_curtailed_mwh: f64,
    /// Whether the schedule remained feasible
    pub feasible: bool,
}

/// Scenario-based stochastic dispatch solver.
///
/// The here-and-now solve computes a schedule on the expected (probability-weighted)
/// scenario.  The value-of-stochastic-solution (VSS) metric quantifies the benefit
/// of optimizing per scenario versus using the expected scenario only.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioBasedDispatch {
    /// Number of scenarios
    pub n_scenarios: usize,
    /// Scenario ensemble
    pub scenarios: Vec<MgScenario>,
    /// Scenario probability weights (must sum to 1)
    pub scenario_weights: Vec<f64>,
    /// Shared resource parameters
    pub resources: MicrogridResources,
}

impl ScenarioBasedDispatch {
    /// Construct an empty scenario pool.
    pub fn new(resources: MicrogridResources, n_scenarios: usize) -> Self {
        Self {
            n_scenarios,
            scenarios: Vec::with_capacity(n_scenarios),
            scenario_weights: Vec::with_capacity(n_scenarios),
            resources,
        }
    }

    /// Register one scenario; weight is taken from `scenario.probability`.
    pub fn add_scenario(&mut self, scenario: MgScenario) {
        self.scenario_weights.push(scenario.probability);
        self.scenarios.push(scenario);
        self.n_scenarios = self.scenarios.len();
    }

    /// Solve the here-and-now problem on the probability-weighted average scenario.
    pub fn solve_here_and_now(&self) -> MgDispatchSchedule {
        if self.scenarios.is_empty() {
            let res = &self.resources;
            return MgDispatchSchedule::zero(0, (res.battery_soc_min + res.battery_soc_max) / 2.0);
        }

        let h = self
            .scenarios
            .iter()
            .map(|s| s.renewable_mw.len())
            .max()
            .unwrap_or(0);
        let total_w: f64 = self.scenario_weights.iter().sum();
        let safe_w = if total_w < 1e-12 { 1.0 } else { total_w };

        let mut avg_re = vec![0.0_f64; h];
        let mut avg_load = vec![0.0_f64; h];
        let mut avg_price = vec![0.0_f64; h];

        for (s, &w) in self.scenarios.iter().zip(self.scenario_weights.iter()) {
            for t in 0..h {
                avg_re[t] += w / safe_w * s.renewable_mw.get(t).copied().unwrap_or(0.0);
                avg_load[t] += w / safe_w * s.load_mw.get(t).copied().unwrap_or(0.0);
                avg_price[t] += w / safe_w * s.price_mwh.get(t).copied().unwrap_or(100.0);
            }
        }

        let opt = MultiObjectiveMgOptimizer {
            horizon_hours: h,
            dt_h: 1.0,
            renewable_forecast_mw: avg_re,
            load_forecast_mw: avg_load,
            electricity_price: avg_price.clone(),
            carbon_intensity_g_co2_per_kwh: vec![300.0; h],
            resources: self.resources.clone(),
        };
        opt.solve_cost_optimal()
    }

    /// Evaluate a pre-computed schedule against a specific scenario.
    pub fn evaluate_schedule(
        &self,
        schedule: &MgDispatchSchedule,
        scenario: &MgScenario,
    ) -> ScenarioResult {
        let h = scenario.renewable_mw.len();
        let res = &self.resources;
        let dt = 1.0_f64;

        let mut cost = 0.0_f64;
        let mut shed = 0.0_f64;
        let mut curtailed = 0.0_f64;
        let mut feasible = true;

        for t in 0..h {
            let re = scenario.renewable_mw.get(t).copied().unwrap_or(0.0);
            let load = scenario.load_mw.get(t).copied().unwrap_or(0.0);
            let price = scenario.price_mwh.get(t).copied().unwrap_or(100.0);

            let b = schedule.battery_mw.get(t).copied().unwrap_or(0.0);
            let d = schedule.diesel_mw.get(t).copied().unwrap_or(0.0);
            let imp = schedule.grid_import_mw.get(t).copied().unwrap_or(0.0);
            let exp = schedule.grid_export_mw.get(t).copied().unwrap_or(0.0);

            // Supply = re + discharge + diesel + import
            let supply = re + (-b).max(0.0) + d + imp;
            // Demand = load + charge + export
            let demand = load + b.max(0.0) + exp;

            let imbalance = supply - demand;
            if imbalance < -1e-3 {
                // Load shedding required
                shed += (-imbalance) * dt;
                feasible = false;
            } else if imbalance > 1e-3 {
                // Curtailment required
                curtailed += imbalance * dt;
            }

            cost += d * res.diesel_cost_per_mwh * dt + imp * price * dt;
        }

        ScenarioResult {
            scenario_id: scenario.id,
            realized_cost: cost,
            load_shed_mwh: shed,
            renewable_curtailed_mwh: curtailed,
            feasible,
        }
    }

    /// Expected cost of a schedule across all registered scenarios.
    ///
    /// `E[cost] = Σ_s weight_s * cost_s`
    pub fn expected_cost(&self, schedule: &MgDispatchSchedule) -> f64 {
        if self.scenarios.is_empty() {
            return 0.0;
        }
        let total_w: f64 = self.scenario_weights.iter().sum();
        let safe_w = if total_w < 1e-12 { 1.0 } else { total_w };

        self.scenarios
            .iter()
            .zip(self.scenario_weights.iter())
            .map(|(s, &w)| {
                let res = self.evaluate_schedule(schedule, s);
                w / safe_w * res.realized_cost
            })
            .sum()
    }

    /// Value of the stochastic solution: VSS = EEV − RP.
    ///
    /// *EEV*: cost when using the here-and-now (expected) schedule for all scenarios.
    /// *RP*: theoretical recourse problem — solved per scenario independently.
    pub fn value_of_stochastic_solution(&self) -> f64 {
        let eev_schedule = self.solve_here_and_now();
        let eev = self.expected_cost(&eev_schedule);

        // RP: solve optimally per scenario and take weighted sum
        let total_w: f64 = self.scenario_weights.iter().sum();
        let safe_w = if total_w < 1e-12 { 1.0 } else { total_w };

        let rp: f64 = self
            .scenarios
            .iter()
            .zip(self.scenario_weights.iter())
            .map(|(s, &w)| {
                let h = s.renewable_mw.len();
                let opt = MultiObjectiveMgOptimizer {
                    horizon_hours: h,
                    dt_h: 1.0,
                    renewable_forecast_mw: s.renewable_mw.clone(),
                    load_forecast_mw: s.load_mw.clone(),
                    electricity_price: s.price_mwh.clone(),
                    carbon_intensity_g_co2_per_kwh: vec![300.0; h],
                    resources: self.resources.clone(),
                };
                let sched = opt.solve_cost_optimal();
                w / safe_w * sched.total_cost
            })
            .sum();

        eev - rp
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §6  Islanding Transition Controller
// ═══════════════════════════════════════════════════════════════════════════

/// State machine for the islanding transition sequence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TransitionState {
    /// Synchronized with the main grid
    GridConnected,
    /// Pre-island preparation phase; `preparation_s` seconds elapsed
    PreIsland { preparation_s: f64 },
    /// Operating in island mode; `duration_h` hours elapsed
    Islanded { duration_h: f64 },
    /// Attempting resynchronization; `sync_error_pct` is the remaining angle/freq error
    Reconnecting { sync_error_pct: f64 },
}

/// Outcome of the islanding feasibility check.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandingFeasibility {
    /// Whether islanding is technically feasible
    pub feasible: bool,
    /// Human-readable explanation
    pub reason: String,
    /// Minimum SoC required to sustain the non-critical load \[pu\]
    pub min_required_soc: f64,
    /// Estimated autonomous duration \[h\]
    pub estimated_duration_h: f64,
}

/// Islanding transition controller with autonomy estimation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandingController {
    /// Whether currently connected to the main grid
    pub grid_connected: bool,
    /// Current battery state-of-charge \[pu\]
    pub battery_soc: f64,
    /// Whether the diesel generator is available
    pub diesel_available: bool,
    /// Current renewable power \[MW\]
    pub renewable_mw: f64,
    /// Current total load demand \[MW\]
    pub load_mw: f64,
    /// Fraction of load that can be shed during islanding
    pub non_critical_load_fraction: f64,
    /// Current state-machine state
    pub transition_state: TransitionState,
    /// Battery energy capacity \[MWh\]
    pub battery_mwh: f64,
    /// Diesel maximum output \[MW\]
    pub diesel_mw_max: f64,
    /// Diesel fuel endurance \[h\]
    pub diesel_endurance_h: f64,
}

impl IslandingController {
    /// Create a new controller in grid-connected state.
    pub fn new(battery_soc: f64, diesel_available: bool) -> Self {
        Self {
            grid_connected: true,
            battery_soc,
            diesel_available,
            renewable_mw: 0.0,
            load_mw: 1.0,
            non_critical_load_fraction: 0.3,
            transition_state: TransitionState::GridConnected,
            battery_mwh: 4.0,
            diesel_mw_max: 2.0,
            diesel_endurance_h: 8.0,
        }
    }

    /// Assess whether islanding is feasible given current conditions.
    pub fn check_islanding_feasible(&self) -> IslandingFeasibility {
        let critical_load = self.load_mw * (1.0 - self.non_critical_load_fraction);
        let net_critical = (critical_load - self.renewable_mw).max(0.0);
        let min_soc = if self.battery_mwh > 1e-9 {
            (net_critical * 0.5 / self.battery_mwh).min(1.0)
        } else {
            1.0
        };

        let duration = self.autonomy_estimate_h();

        if duration < 0.5 {
            IslandingFeasibility {
                feasible: false,
                reason: "Insufficient stored energy and generation for ≥30 min autonomy".into(),
                min_required_soc: min_soc,
                estimated_duration_h: duration,
            }
        } else {
            IslandingFeasibility {
                feasible: true,
                reason: format!(
                    "Estimated {:.1} h autonomy with {:.0}% critical load",
                    duration,
                    (1.0 - self.non_critical_load_fraction) * 100.0
                ),
                min_required_soc: min_soc,
                estimated_duration_h: duration,
            }
        }
    }

    /// Initiate islanding sequence (moves to PreIsland state).
    pub fn initiate_islanding(&mut self) -> Result<(), String> {
        let check = self.check_islanding_feasible();
        if !check.feasible {
            return Err(format!("Islanding not feasible: {}", check.reason));
        }
        if !matches!(self.transition_state, TransitionState::GridConnected) {
            return Err("Already transitioning or islanded".into());
        }
        self.transition_state = TransitionState::PreIsland { preparation_s: 0.0 };
        self.grid_connected = false;
        Ok(())
    }

    /// Advance islanded-mode simulation by `dt_s` seconds.
    ///
    /// Updates internal SoC and elapsed time; automatically moves to
    /// `PreIsland → Islanded` after a 5-second preparation window.
    pub fn step_islanded(&mut self, dt_s: f64, new_re_mw: f64, new_load_mw: f64) {
        self.renewable_mw = new_re_mw;
        self.load_mw = new_load_mw;

        match &self.transition_state {
            TransitionState::PreIsland { preparation_s } => {
                let elapsed = preparation_s + dt_s;
                if elapsed >= 5.0 {
                    self.transition_state = TransitionState::Islanded { duration_h: 0.0 };
                } else {
                    self.transition_state = TransitionState::PreIsland {
                        preparation_s: elapsed,
                    };
                }
            }
            TransitionState::Islanded { duration_h } => {
                let new_duration = duration_h + dt_s / 3600.0;
                // Approximate SoC depletion
                let critical_load = new_load_mw * (1.0 - self.non_critical_load_fraction);
                let net = (critical_load - new_re_mw).max(0.0);
                if !self.diesel_available && self.battery_mwh > 1e-9 {
                    let dsoc = net * (dt_s / 3600.0) / self.battery_mwh;
                    self.battery_soc = (self.battery_soc - dsoc).max(0.0);
                }
                self.transition_state = TransitionState::Islanded {
                    duration_h: new_duration,
                };
            }
            _ => {}
        }
    }

    /// Initiate reconnection to the main grid.
    pub fn reconnect_to_grid(&mut self) -> Result<(), String> {
        match &self.transition_state {
            TransitionState::Islanded { .. } => {
                self.transition_state = TransitionState::Reconnecting {
                    sync_error_pct: 5.0, // initial estimated sync error
                };
                Ok(())
            }
            _ => Err("Cannot reconnect: not currently islanded".into()),
        }
    }

    /// Estimate autonomous operation duration based on current resources \[h\].
    pub fn autonomy_estimate_h(&self) -> f64 {
        let critical_load = self.load_mw * (1.0 - self.non_critical_load_fraction);
        let net_load = (critical_load - self.renewable_mw).max(0.0);
        if net_load < 1e-9 {
            return 168.0; // a week — effectively unlimited
        }
        let batt_energy = self.battery_soc * self.battery_mwh;
        let diesel_energy = if self.diesel_available {
            self.diesel_mw_max.min(net_load) * self.diesel_endurance_h
        } else {
            0.0
        };
        (batt_energy + diesel_energy) / net_load
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §7  Peer-to-Peer Energy Market
// ═══════════════════════════════════════════════════════════════════════════

/// Role of a microgrid participant in the P2P market.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ParticipantRole {
    /// Pure generator (e.g., rooftop PV only)
    Producer,
    /// Pure consumer
    Consumer,
    /// Generator + consumer (PV + load)
    Prosumer,
    /// Battery storage participant
    Storage,
}

/// A participant in the peer-to-peer microgrid energy market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MgParticipant {
    /// Unique identifier
    pub id: usize,
    /// Friendly name
    pub name: String,
    /// Current generation output \[kW\]
    pub generation_kw: f64,
    /// Current consumption \[kW\]
    pub consumption_kw: f64,
    /// Offer/bid price \[\$/kWh\]: sellers quote minimum; buyers quote maximum
    pub bid_price_per_kwh: f64,
    /// Participant role
    pub role: ParticipantRole,
}

/// Result of a single P2P market clearing round.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pClearing {
    /// Market clearing price \[\$/kWh\]
    pub clearing_price_kwh: f64,
    /// Total energy matched \[kWh\] (for one clearing interval, dt assumed 1 h)
    pub volume_matched_kwh: f64,
    /// Aggregate buyer savings vs grid import price \[\$\]
    pub buyer_savings: f64,
    /// Aggregate seller earnings \[\$\]
    pub seller_earnings: f64,
    /// Unmatched buy demand \[kW\]
    pub unmatched_buy_kw: f64,
    /// Unmatched sell supply \[kW\]
    pub unmatched_sell_kw: f64,
}

/// Double-auction P2P energy market for microgrid participants.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pEnergyMarket {
    /// Registered market participants
    pub participants: Vec<MgParticipant>,
    /// Last market clearing price \[\$/kWh\]
    pub clearing_price_per_kwh: f64,
    /// Grid import fallback price \[\$/kWh\]
    pub grid_import_price: f64,
    /// Grid export revenue price \[\$/kWh\]
    pub grid_export_price: f64,
    /// Historical clearing records
    pub clearing_history: Vec<P2pClearing>,
}

impl P2pEnergyMarket {
    /// Construct a new empty market.
    pub fn new(grid_import_price: f64, grid_export_price: f64) -> Self {
        Self {
            participants: Vec::new(),
            clearing_price_per_kwh: (grid_import_price + grid_export_price) / 2.0,
            grid_import_price,
            grid_export_price,
            clearing_history: Vec::new(),
        }
    }

    /// Register a new participant.
    pub fn add_participant(&mut self, p: MgParticipant) {
        self.participants.push(p);
    }

    /// Run one clearing round using a double-auction mechanism.
    ///
    /// Sellers are sorted by ascending ask price; buyers by descending bid price.
    /// Matching continues while seller_ask ≤ buyer_bid.
    /// The clearing price is the midpoint of the last matched pair.
    pub fn clear_market(&mut self) -> P2pClearing {
        // Collect sell offers (net generation > 0)
        let mut sellers: Vec<(f64, f64)> = self // (ask_price, quantity_kw)
            .participants
            .iter()
            .filter_map(|p| {
                let net = p.generation_kw - p.consumption_kw;
                if net > 1e-9 {
                    Some((p.bid_price_per_kwh, net))
                } else {
                    None
                }
            })
            .collect();
        sellers.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        // Collect buy bids (net consumption > 0)
        let mut buyers: Vec<(f64, f64)> = self // (max_price, quantity_kw)
            .participants
            .iter()
            .filter_map(|p| {
                let net = p.consumption_kw - p.generation_kw;
                if net > 1e-9 {
                    Some((p.bid_price_per_kwh, net))
                } else {
                    None
                }
            })
            .collect();
        buyers.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let total_supply: f64 = sellers.iter().map(|s| s.1).sum();
        let total_demand: f64 = buyers.iter().map(|b| b.1).sum();

        let mut matched_kw = 0.0_f64;
        let mut last_price = self.clearing_price_per_kwh;

        let mut si = 0_usize;
        let mut bi = 0_usize;
        let mut rem_sell = sellers.first().map(|s| s.1).unwrap_or(0.0);
        let mut rem_buy = buyers.first().map(|b| b.1).unwrap_or(0.0);

        while si < sellers.len() && bi < buyers.len() {
            let ask = sellers[si].0;
            let bid = buyers[bi].0;
            if ask > bid {
                break;
            }
            last_price = (ask + bid) / 2.0;
            let trade = rem_sell.min(rem_buy);
            matched_kw += trade;
            rem_sell -= trade;
            rem_buy -= trade;

            if rem_sell < 1e-9 {
                si += 1;
                rem_sell = sellers.get(si).map(|s| s.1).unwrap_or(0.0);
            }
            if rem_buy < 1e-9 {
                bi += 1;
                rem_buy = buyers.get(bi).map(|b| b.1).unwrap_or(0.0);
            }
        }

        let clearing_price = last_price;
        // Assume dt = 1 h for energy conversion kW → kWh
        let volume_kwh = matched_kw;
        let buyer_savings = volume_kwh * (self.grid_import_price - clearing_price).max(0.0);
        let seller_earnings = volume_kwh * (clearing_price - self.grid_export_price).max(0.0);

        self.clearing_price_per_kwh = clearing_price;

        let clearing = P2pClearing {
            clearing_price_kwh: clearing_price,
            volume_matched_kwh: volume_kwh,
            buyer_savings,
            seller_earnings,
            unmatched_buy_kw: (total_demand - matched_kw).max(0.0),
            unmatched_sell_kw: (total_supply - matched_kw).max(0.0),
        };
        self.clearing_history.push(clearing.clone());
        clearing
    }

    /// Distribute grid savings proportionally to participant net positions.
    ///
    /// Returns `Vec<(participant_id, share_$)>`.
    pub fn surplus_sharing(&self, clearing: &P2pClearing) -> Vec<(usize, f64)> {
        let total_savings = clearing.buyer_savings + clearing.seller_earnings;
        if total_savings < 1e-12 {
            return self.participants.iter().map(|p| (p.id, 0.0)).collect();
        }
        let total_abs: f64 = self
            .participants
            .iter()
            .map(|p| (p.generation_kw - p.consumption_kw).abs())
            .sum();
        let safe_total = if total_abs < 1e-12 { 1.0 } else { total_abs };

        self.participants
            .iter()
            .map(|p| {
                let share = (p.generation_kw - p.consumption_kw).abs() / safe_total * total_savings;
                (p.id, share)
            })
            .collect()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §8  Microgrid KPI Dashboard
// ═══════════════════════════════════════════════════════════════════════════

/// Aggregated KPI dashboard computed from a sequence of hourly dispatch schedules.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MicrogridKpiDashboard {
    /// Simulation duration \[h\]
    pub simulation_hours: usize,
    /// Hourly dispatch schedules
    pub schedules: Vec<MgDispatchSchedule>,
    /// Hourly load demand \[MW\]
    pub load_mw: Vec<f64>,
    /// Hourly renewable available (before curtailment) \[MW\]
    pub renewable_available_mw: Vec<f64>,
}

impl MicrogridKpiDashboard {
    /// Renewable self-consumption: fraction of available renewable energy actually consumed \[%\].
    ///
    /// `RSC = 100 * Σ min(RE_avail, load + export - import) / Σ RE_avail`
    pub fn renewable_self_consumption_pct(&self) -> f64 {
        let re_total: f64 = self.renewable_available_mw.iter().sum();
        if re_total < 1e-12 {
            return 0.0;
        }
        let consumed: f64 = self
            .schedules
            .iter()
            .zip(self.renewable_available_mw.iter())
            .map(|(s, &re_av)| {
                // RE consumed = min(available, what was actually used)
                let re_used = re_av - s.grid_export_mw.iter().sum::<f64>().min(re_av) + 0.0; // simplified: RE_available - curtailment
                re_av.min(re_used.max(0.0))
            })
            .sum();
        (consumed / re_total * 100.0).clamp(0.0, 100.0)
    }

    /// Self-sufficiency: fraction of load met without grid import \[%\].
    pub fn self_sufficiency_pct(&self) -> f64 {
        let total_load: f64 = self.load_mw.iter().sum();
        if total_load < 1e-12 {
            return 100.0;
        }
        let total_import: f64 = self
            .schedules
            .iter()
            .flat_map(|s| s.grid_import_mw.iter())
            .sum();
        let self_supplied = (total_load - total_import).max(0.0);
        (self_supplied / total_load * 100.0).clamp(0.0, 100.0)
    }

    /// Average battery state-of-charge across all schedule time-steps \[pu\].
    pub fn average_battery_soc(&self) -> f64 {
        let all_soc: Vec<f64> = self
            .schedules
            .iter()
            .flat_map(|s| s.soc_trajectory.iter().copied())
            .collect();
        if all_soc.is_empty() {
            return 0.0;
        }
        all_soc.iter().sum::<f64>() / all_soc.len() as f64
    }

    /// Grid dependency: total import as fraction of total load \[%\].
    pub fn grid_dependency_pct(&self) -> f64 {
        let total_load: f64 = self.load_mw.iter().sum();
        if total_load < 1e-12 {
            return 0.0;
        }
        let total_import: f64 = self
            .schedules
            .iter()
            .flat_map(|s| s.grid_import_mw.iter())
            .sum();
        (total_import / total_load * 100.0).clamp(0.0, 100.0)
    }

    /// Peak shaving ratio: reduction in peak demand relative to raw load peak \[%\].
    pub fn peak_shaving_pct(&self) -> f64 {
        let load_peak = self
            .load_mw
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        if load_peak < 1e-12 {
            return 0.0;
        }
        // Net peak = load - battery_discharge + battery_charge + import - export
        let net_peaks: Vec<f64> = self
            .load_mw
            .iter()
            .zip(self.schedules.iter())
            .map(|(&load, s)| {
                let b = s.battery_mw.first().copied().unwrap_or(0.0);
                let imp = s.grid_import_mw.first().copied().unwrap_or(0.0);
                let exp = s.grid_export_mw.first().copied().unwrap_or(0.0);
                // Net grid-facing load
                load + b + imp - exp
            })
            .collect();
        let net_peak = net_peaks.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        ((load_peak - net_peak) / load_peak * 100.0).clamp(0.0, 100.0)
    }

    /// Total cost across all schedules \[\$\].
    pub fn total_cost(&self) -> f64 {
        self.schedules.iter().map(|s| s.total_cost).sum()
    }

    /// Estimate CO2 avoided compared to serving all load from the grid \[kg\].
    ///
    /// `avoided = (Σ load - Σ import) * grid_intensity / 1000`
    pub fn co2_avoided_kg(&self, grid_intensity_g_kwh: f64) -> f64 {
        let total_load_mwh: f64 = self.load_mw.iter().sum();
        let total_import_mwh: f64 = self
            .schedules
            .iter()
            .flat_map(|s| s.grid_import_mw.iter())
            .sum();
        let local_generation_mwh = (total_load_mwh - total_import_mwh).max(0.0);
        // Convert MW·h → kWh (×1000), then g → kg (÷1000): net factor = ×1
        local_generation_mwh * 1000.0 * grid_intensity_g_kwh / 1000.0
    }

    /// Generate a formatted KPI summary string.
    pub fn report(&self) -> String {
        format!(
            "Microgrid KPI Report\n\
             ═══════════════════════════════\n\
             Simulation horizon : {:>6} h\n\
             Renewable self-consumption : {:>6.1} %\n\
             Self-sufficiency          : {:>6.1} %\n\
             Grid dependency           : {:>6.1} %\n\
             Average battery SoC       : {:>6.3} pu\n\
             Peak shaving              : {:>6.1} %\n\
             Total cost                : {:>10.2} $\n\
             CO2 avoided (300 g/kWh)   : {:>10.1} kg\n",
            self.simulation_hours,
            self.renewable_self_consumption_pct(),
            self.self_sufficiency_pct(),
            self.grid_dependency_pct(),
            self.average_battery_soc(),
            self.peak_shaving_pct(),
            self.total_cost(),
            self.co2_avoided_kg(300.0),
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// §9  Unit tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    fn make_resources(h: usize) -> MicrogridResources {
        MicrogridResources {
            battery_mwh: 4.0,
            battery_mw: 2.0,
            battery_soc_min: 0.1,
            battery_soc_max: 0.9,
            battery_rte: 0.92,
            diesel_mw: 3.0,
            diesel_cost_per_mwh: 250.0,
            diesel_emissions_g_per_kwh: 680.0,
            grid_import_limit_mw: 5.0,
            grid_export_limit_mw: 3.0,
            grid_import_price: vec![100.0; h],
            grid_export_price: vec![50.0; h],
        }
    }

    fn make_optimizer(h: usize) -> MultiObjectiveMgOptimizer {
        MultiObjectiveMgOptimizer {
            horizon_hours: h,
            dt_h: 1.0,
            renewable_forecast_mw: vec![1.0; h],
            load_forecast_mw: vec![2.5; h],
            electricity_price: vec![100.0; h],
            carbon_intensity_g_co2_per_kwh: vec![300.0; h],
            resources: make_resources(h),
        }
    }

    // ── Multi-objective optimizer ─────────────────────────────────────────

    #[test]
    fn test_cost_optimal_positive_cost() {
        let opt = make_optimizer(24);
        let sched = opt.solve_cost_optimal();
        assert!(
            sched.total_cost > 0.0,
            "cost-optimal schedule must have positive cost (diesel/import is needed)"
        );
    }

    #[test]
    fn test_carbon_optimal_less_carbon_than_cost() {
        let opt = make_optimizer(24);
        let cost_sched = opt.solve_cost_optimal();
        let carbon_sched = opt.solve_carbon_optimal();
        // Carbon-optimal should equal or improve upon cost-optimal carbon emissions.
        assert!(
            carbon_sched.total_carbon_g <= cost_sched.total_carbon_g + 1e-3,
            "carbon-optimal ({:.0} g) must not exceed cost-optimal ({:.0} g)",
            carbon_sched.total_carbon_g,
            cost_sched.total_carbon_g
        );
    }

    #[test]
    fn test_pareto_front_length() {
        let opt = make_optimizer(12);
        let front = opt.pareto_front(7);
        assert_eq!(
            front.len(),
            7,
            "pareto_front must return exactly n_solutions points"
        );
    }

    #[test]
    fn test_pareto_weights_sum_to_one() {
        let opt = make_optimizer(6);
        let front = opt.pareto_front(5);
        for pt in &front {
            let sum = pt.weight_cost + pt.weight_carbon;
            assert!(
                (sum - 1.0).abs() < 1e-10,
                "weights must sum to 1: got {sum}"
            );
        }
    }

    // ── Hierarchical control ──────────────────────────────────────────────

    #[test]
    fn test_secondary_reduces_voltage_error() {
        let mut ctrl = HierarchicalMgControl::new_standard();
        // Voltage is 0.95 pu → error = 0.05
        let (dv1, _) = ctrl.step_secondary(0.95, 50.0, 1.0);
        let (dv2, _) = ctrl.step_secondary(0.95, 50.0, 1.0);
        // Integral grows → correction increases
        assert!(
            dv2 > dv1,
            "secondary voltage correction must grow with persistent error: {dv1} → {dv2}"
        );
    }

    #[test]
    fn test_primary_droop_direction() {
        let ctrl = HierarchicalMgControl::new_standard();
        // Under-frequency (49.5 Hz < 50 Hz) → should inject positive P
        let (dp, _) = ctrl.primary_response(1.0, 49.5);
        assert!(
            dp > 0.0,
            "under-frequency must trigger positive P injection, got {dp}"
        );
        // Under-voltage (0.95 pu) → should inject positive Q
        let (_, dq) = ctrl.primary_response(0.95, 50.0);
        assert!(
            dq > 0.0,
            "under-voltage must trigger positive Q injection, got {dq}"
        );
    }

    // ── Scenario-based dispatch ───────────────────────────────────────────

    #[test]
    fn test_add_scenario_increments_count() {
        let mut sbd = ScenarioBasedDispatch::new(make_resources(4), 0);
        assert_eq!(sbd.n_scenarios, 0);
        sbd.add_scenario(MgScenario {
            id: 1,
            renewable_mw: vec![1.0; 4],
            load_mw: vec![1.0; 4],
            price_mwh: vec![80.0; 4],
            probability: 1.0,
        });
        assert_eq!(sbd.n_scenarios, 1);
    }

    #[test]
    fn test_evaluate_schedule_zero_cost_all_renewable() {
        // When renewables exactly cover load and no diesel/import is needed,
        // the realized cost should be 0.
        let res = make_resources(4);
        let sbd = ScenarioBasedDispatch::new(res.clone(), 1);
        let schedule = MgDispatchSchedule {
            battery_mw: vec![0.0; 4],
            diesel_mw: vec![0.0; 4],
            grid_import_mw: vec![0.0; 4],
            grid_export_mw: vec![0.0; 4],
            soc_trajectory: vec![0.5; 5],
            total_cost: 0.0,
            total_carbon_g: 0.0,
            islanding_reserve_mwh: 2.0,
        };
        let scenario = MgScenario {
            id: 0,
            renewable_mw: vec![2.0; 4],
            load_mw: vec![2.0; 4],
            price_mwh: vec![100.0; 4],
            probability: 1.0,
        };
        let result = sbd.evaluate_schedule(&schedule, &scenario);
        assert!(
            result.realized_cost < 1e-9,
            "all-renewable balanced scenario must have zero cost, got {}",
            result.realized_cost
        );
    }

    // ── Islanding controller ──────────────────────────────────────────────

    #[test]
    fn test_islanding_feasible_high_soc() {
        let mut ctrl = IslandingController::new(0.8, true);
        ctrl.load_mw = 1.0;
        ctrl.renewable_mw = 0.2;
        ctrl.battery_mwh = 4.0;
        ctrl.non_critical_load_fraction = 0.3;
        let feas = ctrl.check_islanding_feasible();
        assert!(
            feas.feasible,
            "high SoC + diesel must be feasible: {}",
            feas.reason
        );
    }

    #[test]
    fn test_islanding_not_feasible_no_energy() {
        let mut ctrl = IslandingController::new(0.0, false);
        ctrl.load_mw = 2.0;
        ctrl.renewable_mw = 0.0;
        ctrl.battery_mwh = 4.0;
        ctrl.non_critical_load_fraction = 0.0;
        let feas = ctrl.check_islanding_feasible();
        assert!(
            !feas.feasible,
            "zero SoC + no diesel + no RE must not be feasible"
        );
    }

    // ── P2P market ────────────────────────────────────────────────────────

    #[test]
    fn test_p2p_clearing_price_between_bids() {
        let mut market = P2pEnergyMarket::new(0.20, 0.08);
        market.add_participant(MgParticipant {
            id: 0,
            name: "Solar".into(),
            generation_kw: 5.0,
            consumption_kw: 0.0,
            bid_price_per_kwh: 0.10, // seller min price
            role: ParticipantRole::Producer,
        });
        market.add_participant(MgParticipant {
            id: 1,
            name: "Home".into(),
            generation_kw: 0.0,
            consumption_kw: 3.0,
            bid_price_per_kwh: 0.18, // buyer max price
            role: ParticipantRole::Consumer,
        });
        let clearing = market.clear_market();
        assert!(
            clearing.clearing_price_kwh >= 0.10 - 1e-9,
            "clearing price must be >= seller min: {}",
            clearing.clearing_price_kwh
        );
        assert!(
            clearing.clearing_price_kwh <= 0.18 + 1e-9,
            "clearing price must be <= buyer max: {}",
            clearing.clearing_price_kwh
        );
    }

    #[test]
    fn test_p2p_matched_volume_within_min_supply_demand() {
        let mut market = P2pEnergyMarket::new(0.20, 0.05);
        market.add_participant(MgParticipant {
            id: 0,
            name: "PV".into(),
            generation_kw: 4.0,
            consumption_kw: 0.0,
            bid_price_per_kwh: 0.09,
            role: ParticipantRole::Producer,
        });
        market.add_participant(MgParticipant {
            id: 1,
            name: "EV".into(),
            generation_kw: 0.0,
            consumption_kw: 6.0,
            bid_price_per_kwh: 0.15,
            role: ParticipantRole::Consumer,
        });
        let clearing = market.clear_market();
        let supply = 4.0_f64;
        let demand = 6.0_f64;
        assert!(
            clearing.volume_matched_kwh <= supply.min(demand) + 1e-9,
            "matched volume ({}) must not exceed min(supply, demand) ({})",
            clearing.volume_matched_kwh,
            supply.min(demand)
        );
    }

    // ── KPI Dashboard ─────────────────────────────────────────────────────

    #[test]
    fn test_kpi_self_consumption_in_range() {
        let sched = MgDispatchSchedule {
            battery_mw: vec![0.0],
            diesel_mw: vec![0.0],
            grid_import_mw: vec![0.5],
            grid_export_mw: vec![0.0],
            soc_trajectory: vec![0.5, 0.5],
            total_cost: 50.0,
            total_carbon_g: 10000.0,
            islanding_reserve_mwh: 2.0,
        };
        let dashboard = MicrogridKpiDashboard {
            simulation_hours: 1,
            schedules: vec![sched],
            load_mw: vec![1.5],
            renewable_available_mw: vec![1.0],
        };
        let rsc = dashboard.renewable_self_consumption_pct();
        assert!(
            (0.0..=100.0).contains(&rsc),
            "RSC must be in [0, 100]: got {rsc}"
        );
    }

    #[test]
    fn test_kpi_self_sufficiency_in_range() {
        let sched = MgDispatchSchedule {
            battery_mw: vec![-1.0],
            diesel_mw: vec![0.5],
            grid_import_mw: vec![0.2],
            grid_export_mw: vec![0.0],
            soc_trajectory: vec![0.7, 0.5],
            total_cost: 75.0,
            total_carbon_g: 30000.0,
            islanding_reserve_mwh: 2.0,
        };
        let dashboard = MicrogridKpiDashboard {
            simulation_hours: 1,
            schedules: vec![sched],
            load_mw: vec![2.5],
            renewable_available_mw: vec![0.8],
        };
        let ss = dashboard.self_sufficiency_pct();
        assert!(
            (0.0..=100.0).contains(&ss),
            "self-sufficiency must be in [0, 100]: got {ss}"
        );
    }
}
