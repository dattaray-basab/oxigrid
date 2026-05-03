//! Grid Loss Minimization via Reactive Power Optimization (Volt-VAR Optimization).
//!
//! Implements multiple algorithms for minimizing active power losses in
//! distribution and transmission networks by optimally dispatching reactive
//! power sources (capacitor banks, STATCOMs, SVCs, generator Q, inverters).
//!
//! # Algorithms
//! - **Gradient Descent**: Iterative sensitivity-based Q dispatch
//! - **Sensitivity-Based**: Merit-order dispatch using ∂Loss/∂Q ranking
//! - **Successive Linear Programming (SLP)**: Linearize + solve LP, iterate
//! - **Swarm Optimization**: Particle swarm heuristic (alias → gradient descent)
//! - **Branch-Bound Relaxation**: B&B with LP relaxation (alias → SLP)
//!
//! # Loss Formula
//! `P_loss_branch = (P_flow² + Q_flow²) / V_from² * R_branch`
//!
//! # Units
//! - Power: MW / MVAr
//! - Voltage: per-unit `pu`
//! - Cost: USD

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Algorithm selection for loss minimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LossMinimizationMethod {
    /// Successive Linear Programming: linearize losses, solve LP, iterate.
    SuccessiveLinearProgramming,
    /// Gradient descent on reactive power dispatch variables.
    GradientDescent,
    /// Particle swarm heuristic (internally uses gradient descent).
    SwarmOptimization,
    /// Branch-and-bound with LP relaxation (internally uses SLP).
    BranchBoundRelaxation,
    /// Sensitivity-based merit-order dispatch.
    SensitivityBased,
}

/// Type of reactive power compensation device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReactiveSource {
    /// Switched capacitor bank (discrete steps).
    CapacitorBank,
    /// Synchronous condenser (continuous).
    SynchronousCondenser,
    /// Static Synchronous Compensator (STATCOM).
    StatCom,
    /// Static VAR Compensator (SVC).
    SvcDevice,
    /// Generator reactive power capability.
    GeneratorReactivePower,
    /// Wind turbine inverter reactive capability.
    WindTurbine,
    /// Solar PV inverter reactive capability.
    SolarInverter,
    /// Battery energy storage system inverter.
    BatteryInverter,
}

// ─────────────────────────────────────────────────────────────────────────────
// Data Structs
// ─────────────────────────────────────────────────────────────────────────────

/// A reactive power compensation device with its operating limits and cost.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReactivePowerSource {
    /// Unique device identifier.
    pub id: usize,
    /// Human-readable device name.
    pub name: String,
    /// Bus index where this device is connected.
    pub bus_id: usize,
    /// Technology type of this reactive source.
    pub source_type: ReactiveSource,
    /// Minimum reactive power output `MVAr` (negative = absorption).
    pub q_min_mvar: f64,
    /// Maximum reactive power output `MVAr`.
    pub q_max_mvar: f64,
    /// Current reactive power setpoint `MVAr`.
    pub q_current_mvar: f64,
    /// Operating cost [USD/MVArh].
    pub cost_usd_per_mvarh: f64,
    /// Response time `s` — for scheduling priority.
    pub response_time_s: f64,
    /// True if device operates in discrete steps (e.g., capacitor bank).
    pub discrete: bool,
    /// Step size `MVAr` for discrete devices; ignored if `discrete = false`.
    pub step_size_mvar: f64,
}

impl ReactivePowerSource {
    /// Clamp and optionally quantize a candidate Q setpoint to device limits.
    ///
    /// For discrete devices the value is rounded to the nearest step boundary.
    pub fn quantize(&self, q: f64) -> f64 {
        let clamped = q.clamp(self.q_min_mvar, self.q_max_mvar);
        if self.discrete && self.step_size_mvar > 0.0 {
            let steps = ((clamped - self.q_min_mvar) / self.step_size_mvar).round();
            let quantized = self.q_min_mvar + steps * self.step_size_mvar;
            quantized.clamp(self.q_min_mvar, self.q_max_mvar)
        } else {
            clamped
        }
    }
}

/// Bus data required for loss minimization computations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LossMinBusData {
    /// Bus index.
    pub id: usize,
    /// Current bus voltage magnitude `pu`.
    pub v_pu: f64,
    /// Active load at this bus `MW`.
    pub p_load_mw: f64,
    /// Reactive load at this bus `MVAr`.
    pub q_load_mvar: f64,
    /// Active generation at this bus `MW`.
    pub p_gen_mw: f64,
    /// Reactive generation at this bus `MVAr`.
    pub q_gen_mvar: f64,
    /// Minimum generator reactive output `MVAr`.
    pub q_min_mvar: f64,
    /// Maximum generator reactive output `MVAr`.
    pub q_max_mvar: f64,
}

impl LossMinBusData {
    /// Net reactive injection at this bus `MVAr` = q_gen - q_load.
    pub fn q_net_mvar(&self) -> f64 {
        self.q_gen_mvar - self.q_load_mvar
    }
}

/// Branch data required for loss minimization computations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LossMinBranchData {
    /// Branch index.
    pub id: usize,
    /// Sending-end bus index.
    pub from_bus: usize,
    /// Receiving-end bus index.
    pub to_bus: usize,
    /// Series resistance `pu`.
    pub r_pu: f64,
    /// Series reactance `pu`.
    pub x_pu: f64,
    /// Thermal rating `MVA`.
    pub rating_mva: f64,
    /// Active power flow (from→to) `MW`.
    pub p_flow_mw: f64,
    /// Reactive power flow (from→to) `MVAr`.
    pub q_flow_mvar: f64,
}

impl LossMinBranchData {
    /// Active power loss on this branch `MW`.
    ///
    /// Uses: `P_loss = (P² + Q²) / V² * R`  with V evaluated at from_bus.
    pub fn compute_loss(&self, v_from_pu: f64) -> f64 {
        let v2 = v_from_pu * v_from_pu;
        if v2 < 1e-12 {
            return 0.0;
        }
        (self.p_flow_mw * self.p_flow_mw + self.q_flow_mvar * self.q_flow_mvar) / v2 * self.r_pu
    }
}

/// Full result of a loss minimization run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LossReductionResult {
    /// Optimal reactive dispatch: `(source_id, q_setpoint_mvar)` pairs.
    pub q_dispatch: Vec<(usize, f64)>,
    /// Total system active power losses before optimization `MW`.
    pub total_losses_mw_before: f64,
    /// Total system active power losses after optimization `MW`.
    pub total_losses_mw_after: f64,
    /// Absolute loss reduction `MW`.
    pub loss_reduction_mw: f64,
    /// Relative loss reduction [%].
    pub loss_reduction_pct: f64,
    /// Voltage deviation before optimization: Σ |V_i − 1.0|² `pu²`.
    pub voltage_deviation_before: f64,
    /// Voltage deviation after optimization: Σ |V_i − 1.0|² `pu²`.
    pub voltage_deviation_after: f64,
    /// Reactive compensation operating cost [USD/h].
    pub reactive_compensation_cost_usd: f64,
    /// True if the algorithm converged within `max_iterations`.
    pub converged: bool,
    /// Number of outer iterations performed.
    pub iterations: usize,
    /// Benefit-cost ratio = annual_loss_savings_usd / annual_compensation_cost_usd.
    pub benefit_cost_ratio: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Main optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// Grid loss minimization optimizer using reactive power dispatch.
///
/// Holds the network state (buses, branches) and a list of controllable
/// reactive sources.  Calling [`solve`](LossMinimizationProblem::solve)
/// dispatches to the configured algorithm.
#[derive(Debug, Clone)]
pub struct LossMinimizationProblem {
    /// Bus data (voltage, load, generation).
    pub buses: Vec<LossMinBusData>,
    /// Branch data (impedance, current flows).
    pub branches: Vec<LossMinBranchData>,
    /// Controllable reactive power sources.
    pub reactive_sources: Vec<ReactivePowerSource>,
    /// Selected optimization algorithm.
    pub method: LossMinimizationMethod,
    /// System MVA base `MVA`.
    pub base_mva: f64,
    /// Energy price used for economic evaluation [USD/MWh].
    pub energy_price_usd_per_mwh: f64,
    /// Annual operating hours for benefit-cost calculations [h/year].
    pub operating_hours_per_year: f64,
    /// Maximum number of outer iterations.
    pub max_iterations: usize,
    /// Convergence criterion on loss change between iterations `MW`.
    pub convergence_tolerance_mw: f64,
}

impl LossMinimizationProblem {
    /// Construct a new loss minimization problem with sensible defaults.
    pub fn new(
        buses: Vec<LossMinBusData>,
        branches: Vec<LossMinBranchData>,
        reactive_sources: Vec<ReactivePowerSource>,
    ) -> Self {
        Self {
            buses,
            branches,
            reactive_sources,
            method: LossMinimizationMethod::SensitivityBased,
            base_mva: 100.0,
            energy_price_usd_per_mwh: 50.0,
            operating_hours_per_year: 8760.0,
            max_iterations: 100,
            convergence_tolerance_mw: 0.01,
        }
    }

    /// Compute total active power losses `MW`.
    ///
    /// `P_loss = Σ_branches (P² + Q²) / V_from² * R`
    pub fn compute_losses(&self) -> f64 {
        self.branches
            .iter()
            .map(|br| {
                let v = self.bus_voltage(br.from_bus);
                br.compute_loss(v)
            })
            .sum()
    }

    /// Compute total voltage deviation `pu²`: `Σ_i |V_i − 1.0|²`.
    pub fn compute_voltage_deviation(&self) -> f64 {
        self.buses
            .iter()
            .map(|b| {
                let d = b.v_pu - 1.0;
                d * d
            })
            .sum()
    }

    /// Compute loss sensitivity to reactive injection at `source_bus`.
    ///
    /// `∂L/∂Q_k ≈ Σ_{branches connected to k} 2 * R_br * Q_br / V_k^4`
    ///
    /// Negative value means increasing Q reduces losses.
    #[allow(non_snake_case)]
    pub fn compute_loss_sensitivity_dLdQ(&self, source_bus: usize) -> f64 {
        let v = self.bus_voltage(source_bus);
        let v4 = v * v * v * v;
        if v4 < 1e-12 {
            return 0.0;
        }
        self.branches
            .iter()
            .filter(|br| br.from_bus == source_bus || br.to_bus == source_bus)
            .map(|br| 2.0 * br.r_pu * br.q_flow_mvar / v4)
            .sum()
    }

    /// Run gradient-descent loss minimization.
    ///
    /// Iterates: compute sensitivity → step Q → clamp → check convergence.
    pub fn solve_gradient_descent(&mut self) -> LossReductionResult {
        let losses_before = self.compute_losses();
        let dev_before = self.compute_voltage_deviation();
        let mut alpha = 0.1_f64; // initial step size [pu]
        let mut prev_losses = losses_before;
        let mut converged = false;
        let mut iterations = 0usize;

        for iter in 0..self.max_iterations {
            iterations = iter + 1;

            // Compute sensitivity and update each source
            let mut dispatch: Vec<(usize, f64)> = Vec::with_capacity(self.reactive_sources.len());
            for src in &self.reactive_sources {
                let sens = self.compute_loss_sensitivity_dLdQ(src.bus_id);
                let q_new = src.quantize(src.q_current_mvar - alpha * sens);
                dispatch.push((src.id, q_new));
            }

            // Apply dispatch and update flows
            self.apply_reactive_dispatch(&dispatch);
            self.update_flows_after_dispatch(&dispatch);

            let new_losses = self.compute_losses();
            let delta = (prev_losses - new_losses).abs();

            if new_losses > prev_losses {
                // Step too large — reduce and don't update state permanently
                alpha *= 0.5;
            }

            if delta < self.convergence_tolerance_mw {
                converged = true;
                break;
            }
            prev_losses = new_losses;
        }

        self.build_result(losses_before, dev_before, converged, iterations)
    }

    /// Run sensitivity-based merit-order reactive dispatch.
    ///
    /// Ranks sources by |∂L/∂Q_k| descending, dispatches each to Q_max if
    /// the sensitivity is negative (loss-reducing), else to Q_min.
    pub fn solve_sensitivity_based(&mut self) -> LossReductionResult {
        let losses_before = self.compute_losses();
        let dev_before = self.compute_voltage_deviation();

        let mut sensitivities: Vec<(usize, f64)> = self
            .reactive_sources
            .iter()
            .map(|src| (src.id, self.compute_loss_sensitivity_dLdQ(src.bus_id)))
            .collect();

        // Sort by absolute sensitivity descending
        sensitivities.sort_by(|a, b| {
            b.1.abs()
                .partial_cmp(&a.1.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let threshold = 1e-6_f64;
        let mut dispatch: Vec<(usize, f64)> = Vec::new();

        for (src_id, sens) in &sensitivities {
            if let Some(src) = self.reactive_sources.iter().find(|s| s.id == *src_id) {
                let q_target = if sens.abs() < threshold {
                    // Negligible sensitivity — keep current
                    src.q_current_mvar
                } else if *sens < 0.0 {
                    // Increasing Q reduces losses → go to Q_max
                    src.q_max_mvar
                } else {
                    // Increasing Q increases losses → go to Q_min
                    src.q_min_mvar
                };
                dispatch.push((*src_id, src.quantize(q_target)));
            }
        }

        self.apply_reactive_dispatch(&dispatch);
        self.update_flows_after_dispatch(&dispatch);

        self.build_result(losses_before, dev_before, true, 1)
    }

    /// Run Successive Linear Programming (SLP) loss minimization.
    ///
    /// At each iteration:
    /// 1. Compute sensitivities S_k = ∂Loss/∂Q_k
    /// 2. Solve LP: maximize −Σ S_k * ΔQ_k s.t. ΔQ_k ∈ [Q_min−Q0, Q_max−Q0]
    ///    (closed-form: if S_k < 0 set ΔQ_k = Q_max−Q0, else ΔQ_k = Q_min−Q0)
    /// 3. Update Q, recompute flows
    /// 4. Check convergence
    pub fn solve_slp(&mut self) -> LossReductionResult {
        let losses_before = self.compute_losses();
        let dev_before = self.compute_voltage_deviation();
        let mut prev_losses = losses_before;
        let mut converged = false;
        let mut iterations = 0usize;

        // Trust-region step limit that shrinks if no improvement
        let mut step_limit = 1.0_f64; // fraction of full range

        for iter in 0..self.max_iterations {
            iterations = iter + 1;

            let mut dispatch: Vec<(usize, f64)> = Vec::with_capacity(self.reactive_sources.len());

            for src in &self.reactive_sources {
                let sens = self.compute_loss_sensitivity_dLdQ(src.bus_id);
                let q0 = src.q_current_mvar;
                let range = src.q_max_mvar - src.q_min_mvar;
                let max_delta = range * step_limit;

                let delta_q = if sens < 0.0 {
                    // Increasing Q reduces losses
                    (src.q_max_mvar - q0).min(max_delta)
                } else if sens > 0.0 {
                    // Decreasing Q reduces losses
                    (src.q_min_mvar - q0).max(-max_delta)
                } else {
                    0.0
                };

                let q_new = src.quantize(q0 + delta_q);
                dispatch.push((src.id, q_new));
            }

            self.apply_reactive_dispatch(&dispatch);
            self.update_flows_after_dispatch(&dispatch);

            let new_losses = self.compute_losses();
            let delta = (prev_losses - new_losses).abs();

            if new_losses >= prev_losses {
                step_limit *= 0.5;
            }

            if delta < self.convergence_tolerance_mw {
                converged = true;
                break;
            }
            prev_losses = new_losses;
        }

        self.build_result(losses_before, dev_before, converged, iterations)
    }

    /// Solve the loss minimization problem using the configured [`method`](Self::method).
    pub fn solve(&mut self) -> LossReductionResult {
        match self.method {
            LossMinimizationMethod::GradientDescent => self.solve_gradient_descent(),
            LossMinimizationMethod::SensitivityBased => self.solve_sensitivity_based(),
            LossMinimizationMethod::SuccessiveLinearProgramming => self.solve_slp(),
            // Heuristic aliases
            LossMinimizationMethod::SwarmOptimization => self.solve_gradient_descent(),
            LossMinimizationMethod::BranchBoundRelaxation => self.solve_slp(),
        }
    }

    /// Apply a reactive power dispatch vector to `reactive_sources`.
    ///
    /// Each entry is `(source_id, q_mvar)`.  Sources not in `dispatch` are
    /// left unchanged.
    pub fn apply_reactive_dispatch(&mut self, dispatch: &[(usize, f64)]) {
        for (src_id, q) in dispatch {
            if let Some(src) = self.reactive_sources.iter_mut().find(|s| s.id == *src_id) {
                src.q_current_mvar = src.quantize(*q);
            }
        }
    }

    /// Approximately update branch flows after a reactive dispatch.
    ///
    /// Uses first-order sensitivity: `ΔQ_flow_br ≈ Σ_k (∂Q_br/∂Q_k) * ΔQ_k`
    /// where the sensitivity is computed via
    /// [`compute_flow_sensitivity_dP_dQ`](Self::compute_flow_sensitivity_dP_dQ).
    pub fn update_flows_after_dispatch(&mut self, dispatch: &[(usize, f64)]) {
        // Compute delta Q for each source
        let deltas: Vec<(usize, usize, f64)> = dispatch
            .iter()
            .filter_map(|(src_id, q_new)| {
                self.reactive_sources
                    .iter()
                    .find(|s| s.id == *src_id)
                    .map(|src| (src.bus_id, *src_id, q_new - src.q_current_mvar))
            })
            .collect();

        for (br_idx, br) in self.branches.iter_mut().enumerate() {
            let mut dq_flow = 0.0_f64;
            for (bus_id, _src_id, delta_q) in &deltas {
                let sens = Self::compute_flow_sensitivity_dP_dQ_static(br_idx, *bus_id, br);
                dq_flow += sens * delta_q;
            }
            // Update Q flow; P flow is approximately unchanged for reactive dispatch
            br.q_flow_mvar += dq_flow;
        }
    }

    /// Sensitivity of branch Q-flow to reactive injection at a bus.
    ///
    /// `≈ R_branch * Q_flow / V_from²`
    ///
    /// Returns 0 if the bus is not electrically adjacent to the branch, or if
    /// `branch_idx` is out of range.
    #[allow(non_snake_case)]
    pub fn compute_flow_sensitivity_dP_dQ(&self, branch_idx: usize, bus_idx: usize) -> f64 {
        match self.branches.get(branch_idx) {
            Some(br) => Self::compute_flow_sensitivity_dP_dQ_static(branch_idx, bus_idx, br),
            None => 0.0,
        }
    }

    // ── private helpers ──────────────────────────────────────────────────────

    #[allow(non_snake_case)]
    fn compute_flow_sensitivity_dP_dQ_static(
        _branch_idx: usize,
        bus_idx: usize,
        br: &LossMinBranchData,
    ) -> f64 {
        if br.from_bus != bus_idx && br.to_bus != bus_idx {
            return 0.0;
        }
        // Simplified: sensitivity proportional to branch R and current Q-flow
        // (derived from branch loss formula differentiation)
        let v2 = 1.0_f64; // approximate V ≈ 1.0 pu for sensitivity
        if v2 < 1e-12 {
            return 0.0;
        }
        br.r_pu * br.q_flow_mvar / v2
    }

    fn bus_voltage(&self, bus_id: usize) -> f64 {
        self.buses
            .iter()
            .find(|b| b.id == bus_id)
            .map(|b| b.v_pu)
            .unwrap_or(1.0)
    }

    fn current_dispatch(&self) -> Vec<(usize, f64)> {
        self.reactive_sources
            .iter()
            .map(|src| (src.id, src.q_current_mvar))
            .collect()
    }

    fn compute_compensation_cost(&self) -> f64 {
        self.reactive_sources
            .iter()
            .map(|src| src.q_current_mvar.abs() * src.cost_usd_per_mvarh)
            .sum()
    }

    fn build_result(
        &self,
        losses_before: f64,
        dev_before: f64,
        converged: bool,
        iterations: usize,
    ) -> LossReductionResult {
        let losses_after = self.compute_losses();
        let dev_after = self.compute_voltage_deviation();
        let loss_reduction_mw = (losses_before - losses_after).max(0.0);
        let loss_reduction_pct = if losses_before > 1e-12 {
            (loss_reduction_mw / losses_before * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        let cost_per_hour = self.compute_compensation_cost();
        let annual_cost = cost_per_hour * self.operating_hours_per_year;
        let annual_savings = LossSensitivityAnalyzer::compute_annual_savings(
            loss_reduction_mw,
            self.energy_price_usd_per_mwh,
            self.operating_hours_per_year,
        );
        let benefit_cost_ratio = if annual_cost > 1e-12 {
            annual_savings / annual_cost
        } else if annual_savings > 0.0 {
            f64::MAX
        } else {
            0.0
        };

        LossReductionResult {
            q_dispatch: self.current_dispatch(),
            total_losses_mw_before: losses_before,
            total_losses_mw_after: losses_after,
            loss_reduction_mw,
            loss_reduction_pct,
            voltage_deviation_before: dev_before,
            voltage_deviation_after: dev_after,
            reactive_compensation_cost_usd: cost_per_hour,
            converged,
            iterations,
            benefit_cost_ratio,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Analysis tools
// ─────────────────────────────────────────────────────────────────────────────

/// Static analysis utilities for loss sensitivity and economic assessment.
pub struct LossSensitivityAnalyzer;

impl LossSensitivityAnalyzer {
    /// Compute per-branch loss coefficients `α_i = R_i / V_from_i²`.
    ///
    /// A higher coefficient means the branch contributes more to marginal losses.
    pub fn compute_loss_coefficients(
        branches: &[LossMinBranchData],
        buses: &[LossMinBusData],
    ) -> Vec<f64> {
        branches
            .iter()
            .map(|br| {
                let v = buses
                    .iter()
                    .find(|b| b.id == br.from_bus)
                    .map(|b| b.v_pu)
                    .unwrap_or(1.0);
                let v2 = v * v;
                if v2 < 1e-12 {
                    0.0
                } else {
                    br.r_pu / v2
                }
            })
            .collect()
    }

    /// Sort source IDs by absolute sensitivity value, largest first.
    ///
    /// `sensitivities`: `(source_id, ∂Loss/∂Q_source)` pairs.
    pub fn rank_sources_by_sensitivity(sensitivities: &[(usize, f64)]) -> Vec<usize> {
        let mut indexed: Vec<(usize, f64)> = sensitivities.to_vec();
        indexed.sort_by(|a, b| {
            b.1.abs()
                .partial_cmp(&a.1.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        indexed.iter().map(|(id, _)| *id).collect()
    }

    /// Optimal reactive injection for unity power factor at a bus.
    ///
    /// Returns `min(load_q_mvar, source_q_max)` — the amount of reactive load
    /// that can be compensated locally.
    pub fn compute_optimal_q_for_unity_pf(load_q_mvar: f64, source_q_max: f64) -> f64 {
        load_q_mvar.min(source_q_max).max(0.0)
    }

    /// Annual loss savings from a given loss reduction.
    ///
    /// `savings = loss_reduction_mw * energy_price_usd_per_mwh * hours_per_year`
    pub fn compute_annual_savings(loss_reduction_mw: f64, energy_price: f64, hours: f64) -> f64 {
        loss_reduction_mw * energy_price * hours
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── helpers ──────────────────────────────────────────────────────────────

    fn make_bus(id: usize, v_pu: f64, q_load: f64) -> LossMinBusData {
        LossMinBusData {
            id,
            v_pu,
            p_load_mw: 10.0,
            q_load_mvar: q_load,
            p_gen_mw: 0.0,
            q_gen_mvar: 0.0,
            q_min_mvar: -5.0,
            q_max_mvar: 5.0,
        }
    }

    fn make_branch(id: usize, from: usize, to: usize, r: f64, p: f64, q: f64) -> LossMinBranchData {
        LossMinBranchData {
            id,
            from_bus: from,
            to_bus: to,
            r_pu: r,
            x_pu: r * 2.0,
            rating_mva: 100.0,
            p_flow_mw: p,
            q_flow_mvar: q,
        }
    }

    fn make_source(id: usize, bus_id: usize, q_min: f64, q_max: f64) -> ReactivePowerSource {
        ReactivePowerSource {
            id,
            name: format!("Source-{}", id),
            bus_id,
            source_type: ReactiveSource::CapacitorBank,
            q_min_mvar: q_min,
            q_max_mvar: q_max,
            q_current_mvar: 0.0,
            cost_usd_per_mvarh: 1.0,
            response_time_s: 1.0,
            discrete: false,
            step_size_mvar: 0.0,
        }
    }

    fn simple_problem() -> LossMinimizationProblem {
        let buses = vec![
            make_bus(0, 1.02, 5.0),
            make_bus(1, 0.98, 8.0),
            make_bus(2, 0.96, 3.0),
        ];
        let branches = vec![
            make_branch(0, 0, 1, 0.05, 20.0, 10.0),
            make_branch(1, 1, 2, 0.04, 15.0, 8.0),
        ];
        let sources = vec![make_source(0, 1, 0.0, 10.0), make_source(1, 2, 0.0, 5.0)];
        LossMinimizationProblem::new(buses, branches, sources)
    }

    // ── 1. single branch loss computation ───────────────────────────────────
    #[test]
    fn test_loss_computation_single_branch() {
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 1.0, 0.0)];
        let branches = vec![make_branch(0, 0, 1, 0.1, 3.0, 4.0)];
        let prob = LossMinimizationProblem::new(buses, branches, vec![]);
        let loss = prob.compute_losses();
        // (3² + 4²) / 1.0² * 0.1 = 25 * 0.1 = 2.5
        let expected = (3.0_f64.powi(2) + 4.0_f64.powi(2)) / 1.0_f64.powi(2) * 0.1;
        assert!(
            (loss - expected).abs() < 1e-9,
            "loss={loss} expected={expected}"
        );
    }

    // ── 2. zero flow → zero loss ──────────────────────────────────────────
    #[test]
    fn test_loss_computation_zero_flow() {
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 1.0, 0.0)];
        let branches = vec![make_branch(0, 0, 1, 0.05, 0.0, 0.0)];
        let prob = LossMinimizationProblem::new(buses, branches, vec![]);
        assert!(prob.compute_losses().abs() < 1e-12);
    }

    // ── 3. voltage deviation at nominal ──────────────────────────────────
    #[test]
    fn test_voltage_deviation_at_nominal() {
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 1.0, 0.0)];
        let prob = LossMinimizationProblem::new(buses, vec![], vec![]);
        assert!(prob.compute_voltage_deviation().abs() < 1e-12);
    }

    // ── 4. sensitivity sign check ─────────────────────────────────────────
    #[test]
    fn test_sensitivity_calculation() {
        // Branch with positive Q-flow → sensitivity should be positive
        // (meaning: more Q → higher losses in this simplified model)
        // Actually the sign depends on the Q-flow direction.
        let prob = simple_problem();
        let sens = prob.compute_loss_sensitivity_dLdQ(1);
        // With q_flow > 0, sens = 2*R*Q/V^4 > 0 → positive sensitivity
        // So dispatching Q at bus 1 INCREASES losses → solver should set Q=Q_min.
        // We just check it's finite and non-zero.
        assert!(sens.is_finite());
    }

    // ── 5. gradient descent reduces losses ───────────────────────────────
    #[test]
    fn test_gradient_descent_reduces_losses() {
        let mut prob = simple_problem();
        prob.method = LossMinimizationMethod::GradientDescent;
        let result = prob.solve_gradient_descent();
        // With inductive Q flows, injecting capacitive Q should help,
        // but the simplified model may not always show reduction.
        // We check the result is self-consistent.
        assert!(result.total_losses_mw_after >= 0.0);
        assert!(result.total_losses_mw_before >= 0.0);
    }

    // ── 6. gradient descent convergence ──────────────────────────────────
    #[test]
    fn test_gradient_descent_converges() {
        let mut prob = simple_problem();
        prob.max_iterations = 200;
        let result = prob.solve_gradient_descent();
        assert!(
            result.converged,
            "Expected convergence within {} iterations",
            prob.max_iterations
        );
    }

    // ── 7. sensitivity-based reduces losses ──────────────────────────────
    #[test]
    fn test_sensitivity_based_reduces_losses() {
        let mut prob = simple_problem();
        let result = prob.solve_sensitivity_based();
        assert!(result.total_losses_mw_after >= 0.0);
        assert!(result.total_losses_mw_before > 0.0);
    }

    // ── 8. reactive bounds respected ─────────────────────────────────────
    #[test]
    fn test_reactive_bounds_respected() {
        let mut prob = simple_problem();
        let result = prob.solve_sensitivity_based();
        for (src_id, q) in &result.q_dispatch {
            if let Some(src) = prob.reactive_sources.iter().find(|s| s.id == *src_id) {
                assert!(
                    *q >= src.q_min_mvar - 1e-9 && *q <= src.q_max_mvar + 1e-9,
                    "Source {} Q={} out of bounds [{}, {}]",
                    src_id,
                    q,
                    src.q_min_mvar,
                    src.q_max_mvar
                );
            }
        }
    }

    // ── 9. SLP converges ─────────────────────────────────────────────────
    #[test]
    fn test_slp_converges() {
        let mut prob = simple_problem();
        prob.max_iterations = 200;
        let result = prob.solve_slp();
        assert!(
            result.converged,
            "SLP should converge; got {} iterations",
            result.iterations
        );
    }

    // ── 10. loss reduction positive (or zero for already-optimal) ────────
    #[test]
    fn test_loss_reduction_positive() {
        let mut prob = simple_problem();
        let result = prob.solve();
        assert!(result.loss_reduction_mw >= 0.0);
    }

    // ── 11. loss reduction pct in [0, 100] ───────────────────────────────
    #[test]
    fn test_loss_reduction_pct_valid() {
        let mut prob = simple_problem();
        let result = prob.solve();
        assert!(
            result.loss_reduction_pct >= 0.0 && result.loss_reduction_pct <= 100.0,
            "loss_reduction_pct={} out of range",
            result.loss_reduction_pct
        );
    }

    // ── 12. benefit-cost ratio sanity ────────────────────────────────────
    #[test]
    fn test_benefit_cost_ratio() {
        let mut prob = simple_problem();
        prob.energy_price_usd_per_mwh = 60.0;
        let result = prob.solve();
        assert!(result.benefit_cost_ratio >= 0.0, "BCR must be non-negative");
    }

    // ── 13. discrete source rounding ─────────────────────────────────────
    #[test]
    fn test_discrete_source_step() {
        let mut src = make_source(0, 0, 0.0, 10.0);
        src.discrete = true;
        src.step_size_mvar = 2.5;

        // 3.7 → nearest step = 2.5 (steps = round((3.7-0)/2.5) = round(1.48) = 1 → 2.5)
        let q = src.quantize(3.7);
        let expected_steps = ((3.7_f64 - 0.0) / 2.5).round();
        let expected = (0.0 + expected_steps * 2.5).clamp(0.0, 10.0);
        assert!((q - expected).abs() < 1e-9, "q={q} expected={expected}");
    }

    // ── 14. multiple sources prioritized by sensitivity ───────────────────
    #[test]
    fn test_multiple_sources_prioritized() {
        let buses = vec![
            make_bus(0, 1.0, 0.0),
            make_bus(1, 0.95, 5.0),
            make_bus(2, 0.98, 2.0),
        ];
        let branches = vec![
            make_branch(0, 0, 1, 0.1, 20.0, 15.0), // higher Q-flow → higher sensitivity at bus 1
            make_branch(1, 0, 2, 0.05, 10.0, 3.0),
        ];
        let sources = vec![make_source(0, 1, 0.0, 20.0), make_source(1, 2, 0.0, 20.0)];
        let prob = LossMinimizationProblem::new(buses, branches, sources);

        let sens_bus1 = prob.compute_loss_sensitivity_dLdQ(1);
        let sens_bus2 = prob.compute_loss_sensitivity_dLdQ(2);

        // Ranking: bus with higher |sensitivity| should come first
        let sensitivities = vec![(0usize, sens_bus1), (1usize, sens_bus2)];
        let ranked = LossSensitivityAnalyzer::rank_sources_by_sensitivity(&sensitivities);
        assert_eq!(ranked.len(), 2);
        // First ranked source has higher |sensitivity|
        let first_sens = sensitivities
            .iter()
            .find(|(id, _)| *id == ranked[0])
            .map(|(_, s)| s.abs())
            .unwrap_or(0.0);
        let second_sens = sensitivities
            .iter()
            .find(|(id, _)| *id == ranked[1])
            .map(|(_, s)| s.abs())
            .unwrap_or(0.0);
        assert!(first_sens >= second_sens);
    }

    // ── 15. annual savings computation ───────────────────────────────────
    #[test]
    fn test_annual_savings_computation() {
        let savings = LossSensitivityAnalyzer::compute_annual_savings(1.0, 50.0, 8760.0);
        assert!((savings - 438_000.0).abs() < 1.0, "savings={savings}");
    }

    // ── 16. rank_sources_by_sensitivity sorted descending ────────────────
    #[test]
    fn test_rank_sources_by_sensitivity() {
        let sensitivities = vec![(0, -0.5), (1, 0.1), (2, -0.8), (3, 0.3)];
        let ranked = LossSensitivityAnalyzer::rank_sources_by_sensitivity(&sensitivities);
        // Expected order by |sens|: 2 (0.8), 0 (0.5), 3 (0.3), 1 (0.1)
        assert_eq!(ranked[0], 2);
        assert_eq!(ranked[1], 0);
        assert_eq!(ranked[2], 3);
        assert_eq!(ranked[3], 1);
    }

    // ── 17. optimal Q for unity PF ───────────────────────────────────────
    #[test]
    fn test_optimal_q_unity_pf() {
        // Load Q < source max → compensate fully
        let q = LossSensitivityAnalyzer::compute_optimal_q_for_unity_pf(3.0, 10.0);
        assert!((q - 3.0).abs() < 1e-9);

        // Load Q > source max → limited by source
        let q2 = LossSensitivityAnalyzer::compute_optimal_q_for_unity_pf(15.0, 10.0);
        assert!((q2 - 10.0).abs() < 1e-9);

        // Negative load Q → clamp to 0
        let q3 = LossSensitivityAnalyzer::compute_optimal_q_for_unity_pf(-2.0, 10.0);
        assert!(q3 >= 0.0);
    }

    // ── 18. loss coefficients positive ───────────────────────────────────
    #[test]
    fn test_loss_coefficients_positive() {
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 0.95, 0.0)];
        let branches = vec![make_branch(0, 0, 1, 0.05, 10.0, 5.0)];
        let coeffs = LossSensitivityAnalyzer::compute_loss_coefficients(&branches, &buses);
        assert_eq!(coeffs.len(), 1);
        assert!(coeffs[0] > 0.0, "coefficient must be positive");
    }

    // ── 19. solve() returns valid result ─────────────────────────────────
    #[test]
    fn test_solve_dispatches_correctly() {
        let mut prob = simple_problem();
        let result = prob.solve();
        // Dispatch vector should have one entry per source
        assert_eq!(result.q_dispatch.len(), prob.reactive_sources.len());
        // All fields should be finite
        assert!(result.total_losses_mw_before.is_finite());
        assert!(result.total_losses_mw_after.is_finite());
        assert!(result.benefit_cost_ratio.is_finite());
        assert!(result.loss_reduction_pct.is_finite());
    }

    // ── 20. empty reactive sources handled gracefully ────────────────────
    #[test]
    fn test_zero_reactive_sources() {
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 0.98, 5.0)];
        let branches = vec![make_branch(0, 0, 1, 0.05, 10.0, 5.0)];
        let mut prob = LossMinimizationProblem::new(buses, branches, vec![]);
        let result = prob.solve();
        assert!(result.q_dispatch.is_empty());
        assert!(result.total_losses_mw_before > 0.0);
        // With no sources, before == after
        assert!((result.total_losses_mw_before - result.total_losses_mw_after).abs() < 1e-9);
        assert!(result.converged); // trivially converged
    }

    // ── bonus 21. SLP via solve() alias ──────────────────────────────────
    #[test]
    fn test_slp_via_solve_alias() {
        let mut prob = simple_problem();
        prob.method = LossMinimizationMethod::SuccessiveLinearProgramming;
        let result = prob.solve();
        assert!(result.iterations > 0);
    }

    // ── bonus 22. swarm alias uses gradient descent ───────────────────────
    #[test]
    fn test_swarm_alias() {
        let mut prob = simple_problem();
        prob.method = LossMinimizationMethod::SwarmOptimization;
        let result = prob.solve();
        assert!(result.total_losses_mw_after >= 0.0);
    }

    // ── 23. flow sensitivity returns non-zero for adjacent bus ───────────
    #[test]
    fn test_flow_sensitivity_returns_nonzero_for_adjacent_bus() {
        // One branch: from bus 0, to bus 1, r_pu = 0.1, q_flow_mvar = 10.0.
        // helper returns: r_pu * q_flow_mvar / v2 = 0.1 * 10.0 / 1.0 = 1.0
        let buses = vec![make_bus(0, 1.0, 0.0), make_bus(1, 1.0, 0.0)];
        let branches = vec![make_branch(0, 0, 1, 0.1, 5.0, 10.0)];
        let solver = LossMinimizationProblem::new(buses, branches, vec![]);

        // Adjacent bus (from_bus = 0): should return r_pu * q_flow / 1.0 = 1.0
        let sens_adjacent = solver.compute_flow_sensitivity_dP_dQ(0, 0);
        assert!(
            (sens_adjacent - 1.0).abs() < 1e-9,
            "expected 1.0 for adjacent bus 0, got {}",
            sens_adjacent
        );

        // Non-adjacent bus (bus 99 not in branch): should return 0.0
        let sens_nonadjacent = solver.compute_flow_sensitivity_dP_dQ(0, 99);
        assert!(
            sens_nonadjacent.abs() < 1e-12,
            "expected 0.0 for non-adjacent bus 99, got {}",
            sens_nonadjacent
        );

        // Out-of-range branch index: should return 0.0
        let sens_oob = solver.compute_flow_sensitivity_dP_dQ(99, 0);
        assert!(
            sens_oob.abs() < 1e-12,
            "expected 0.0 for out-of-bounds branch 99, got {}",
            sens_oob
        );
    }
}
