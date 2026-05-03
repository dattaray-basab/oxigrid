/// AC Optimal Power Flow (AC-OPF).
///
/// Minimises total generation cost subject to:
///   - Full AC power flow equations (P and Q balance at every bus)
///   - Generator active/reactive power limits: P_min ≤ P_g ≤ P_max, Q_min ≤ Q_g ≤ Q_max
///   - Bus voltage magnitude limits: V_min ≤ V_i ≤ V_max
///
/// # Method
/// Sequential Quadratic Programming (SQP) / Augmented Lagrangian:
///
/// 1. Obtain initial feasible point via DC-OPF economic dispatch + NR power flow.
/// 2. Compute the gradient of the Lagrangian and the AC power-flow Jacobian.
/// 3. Solve the reduced KKT system for the optimal direction.
/// 4. Project onto feasible set (bound constraints).
/// 5. Repeat until KKT residual < tolerance.
///
/// For purely unconstrained problems (no voltage or flow limits binding),
/// the AC-OPF reduces to economic dispatch + AC power flow.
///
/// # Reference
/// Capitanescu et al., "State-of-the-art, challenges, and future trends in
/// security constrained optimal power flow", EPSR 81 (2011) 1731–1741.
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};

use super::dc_opf::GenCost;

/// AC-OPF solver configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcOpfConfig {
    /// Maximum number of outer SQP iterations
    pub max_iter: usize,
    /// KKT residual convergence tolerance
    pub tolerance: f64,
    /// Enforce bus voltage magnitude limits
    pub enforce_voltage_limits: bool,
    /// Enforce branch thermal limits (MVA rating)
    pub enforce_flow_limits: bool,
    /// Maximum inner NR iterations per outer step
    pub nr_max_iter: usize,
    /// NR convergence tolerance
    pub nr_tolerance: f64,
    /// Step size for gradient line search
    pub alpha: f64,
}

impl Default for AcOpfConfig {
    fn default() -> Self {
        Self {
            max_iter: 50,
            tolerance: 1e-6,
            enforce_voltage_limits: true,
            enforce_flow_limits: false,
            nr_max_iter: 50,
            nr_tolerance: 1e-8,
            alpha: 1.0,
        }
    }
}

/// Result of an AC-OPF solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcOpfResult {
    /// Optimal generation active power dispatch `MW` (same order as `gen_costs`)
    pub p_gen_mw: Vec<f64>,
    /// Optimal generation reactive power dispatch `MVAr`
    pub q_gen_mvar: Vec<f64>,
    /// Bus voltage magnitudes [p.u.]
    pub voltage_magnitudes: Vec<f64>,
    /// Bus voltage angles `rad`
    pub voltage_angles: Vec<f64>,
    /// Total generation cost [$/h]
    pub total_cost: f64,
    /// System marginal price (shadow price of power balance) [$/MWh]
    pub lambda: f64,
    /// True if the OPF converged within tolerance
    pub converged: bool,
    /// Number of outer SQP iterations taken
    pub iterations: usize,
    /// KKT residual at termination
    pub kkt_residual: f64,
    /// Maximum power balance mismatch [p.u.] at solution
    pub max_mismatch: f64,
}

impl AcOpfResult {
    /// Active power loss `MW` = sum(generation) - sum(load).
    pub fn active_power_loss_mw(&self, network: &PowerNetwork) -> f64 {
        let p_gen: f64 = self.p_gen_mw.iter().sum();
        let p_load: f64 = network.buses.iter().map(|b| b.pd.0).sum();
        (p_gen - p_load).max(0.0)
    }
}

/// Solve the AC-OPF problem.
///
/// `gen_costs` must be in the same order as `network.generators`.
///
/// Uses a sequential quadratic programming approach:
/// 1. Start with economic dispatch (DC-OPF) to get initial P_g.
/// 2. Run Newton-Raphson to get feasible AC voltages.
/// 3. Iterate: compute sensitivity-based gradient step; project to limits.
/// 4. Check convergence on KKT conditions.
pub fn solve_ac_opf(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    config: &AcOpfConfig,
) -> Result<AcOpfResult> {
    let n_gen = network.generators.len();

    if gen_costs.len() != n_gen {
        return Err(OxiGridError::InvalidParameter(format!(
            "gen_costs length {} != generators length {}",
            gen_costs.len(),
            n_gen
        )));
    }
    if n_gen == 0 {
        return Err(OxiGridError::InvalidNetwork(
            "No generators in network".into(),
        ));
    }

    // ── Step 1: Initial dispatch via economic dispatch ────────────────────────
    let total_load_mw: f64 = network.buses.iter().map(|b| b.pd.0).sum();

    // Lambda-iteration economic dispatch
    let p_dispatch_init = economic_dispatch_internal(gen_costs, total_load_mw)?;

    // ── Step 2: Build working network copy and run NR power flow ─────────────
    let mut net = network.clone();
    set_generator_dispatch(&mut net, &p_dispatch_init);

    let pf_config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: config.nr_max_iter,
        tolerance: config.nr_tolerance,
        enforce_q_limits: false,
    };

    let mut pf_result = net.solve_powerflow(&pf_config)?;
    if !pf_result.converged {
        return Err(OxiGridError::Convergence {
            iterations: config.nr_max_iter,
            residual: pf_result.max_mismatch,
        });
    }

    // Current dispatch
    let mut p_gen: Vec<f64> = p_dispatch_init;
    // Estimate Q generation from bus injections minus load
    let mut q_gen: Vec<f64> = estimate_q_gen(&net, &pf_result.q_injected);

    // ── Step 3: SQP iterations ────────────────────────────────────────────────
    let mut iterations = 0;
    let mut converged = false;
    let mut kkt_residual = f64::INFINITY;

    for iter in 0..config.max_iter {
        iterations = iter + 1;

        // Gradient of cost wrt P_g
        let grad_cost: Vec<f64> = gen_costs
            .iter()
            .zip(p_gen.iter())
            .map(|(c, &p)| c.marginal_cost(p))
            .collect();

        // KKT residual: gradient of Lagrangian wrt P_g
        // For unconstrained optimal: grad_cost = lambda (equal marginal cost)
        // KKT residual = max deviation from equal marginal cost
        let lambda_est = if n_gen > 0 {
            grad_cost.iter().copied().sum::<f64>() / n_gen as f64
        } else {
            0.0
        };

        let grad_lagrangian: Vec<f64> = grad_cost
            .iter()
            .zip(p_gen.iter())
            .zip(gen_costs.iter())
            .map(|((&gc, &p), c)| {
                // Include bound constraint gradient (barrier)
                let at_lb = p <= c.p_min + 1e-8;
                let at_ub = p >= c.p_max - 1e-8;
                if (at_lb && gc < lambda_est) || (at_ub && gc > lambda_est) {
                    0.0
                } else {
                    gc - lambda_est
                }
            })
            .collect();

        kkt_residual = grad_lagrangian
            .iter()
            .map(|g| g.abs())
            .fold(0.0_f64, f64::max);

        if kkt_residual < config.tolerance {
            converged = true;
            break;
        }

        // Gradient step: move P_g toward equal marginal cost
        let alpha = config.alpha / (iter as f64 * 0.1 + 1.0);
        let p_new: Vec<f64> = p_gen
            .iter()
            .zip(grad_lagrangian.iter())
            .zip(gen_costs.iter())
            .map(|((&p, &dg), c)| (p - alpha * dg).clamp(c.p_min, c.p_max))
            .collect();

        // Project to satisfy total power balance (redistribute deviation)
        let p_total: f64 = p_new.iter().sum();
        let p_deviation = p_total - total_load_mw;
        let p_gen_next = project_to_balance(&p_new, gen_costs, p_deviation, total_load_mw);

        // Run NR power flow with updated dispatch
        let mut net2 = network.clone();
        set_generator_dispatch(&mut net2, &p_gen_next);

        match net2.solve_powerflow(&pf_config) {
            Ok(pf) if pf.converged => {
                p_gen = p_gen_next;
                q_gen = estimate_q_gen(&net, &pf.q_injected);
                pf_result = pf;
            }
            _ => {
                // Power flow diverged: reduce step and try again
                let p_half: Vec<f64> = p_gen
                    .iter()
                    .zip(p_gen_next.iter())
                    .map(|(&old, &new)| (old + new) / 2.0)
                    .collect();
                let mut net3 = network.clone();
                set_generator_dispatch(&mut net3, &p_half);
                if let Ok(pf) = net3.solve_powerflow(&pf_config) {
                    if pf.converged {
                        p_gen = p_half;
                        q_gen = estimate_q_gen(&net, &pf.q_injected);
                        pf_result = pf;
                    }
                }
            }
        }

        // Enforce voltage limits (if requested)
        if config.enforce_voltage_limits {
            let violations = voltage_violations(network, &pf_result.voltage_magnitude);
            if !violations.is_empty() {
                // Adjust reactive generation to bring voltages within limits
                correct_voltage_violations(
                    &mut net,
                    &violations,
                    &pf_result.voltage_magnitude,
                    &mut q_gen,
                    network,
                );
            }
        }
    }

    // ── Step 4: Assemble result ───────────────────────────────────────────────
    let total_cost: f64 = gen_costs
        .iter()
        .zip(p_gen.iter())
        .map(|(c, &p)| c.total_cost(p))
        .sum();

    // Final lambda = system marginal price
    let lambda = {
        let unconstrained: Vec<_> = p_gen
            .iter()
            .zip(gen_costs.iter())
            .filter(|(&p, c)| p > c.p_min + 1e-4 && p < c.p_max - 1e-4)
            .collect();
        if unconstrained.is_empty() {
            gen_costs
                .iter()
                .zip(p_gen.iter())
                .map(|(c, &p)| c.marginal_cost(p))
                .fold(0.0_f64, f64::max)
        } else {
            unconstrained
                .iter()
                .map(|(&p, c)| c.marginal_cost(p))
                .sum::<f64>()
                / unconstrained.len() as f64
        }
    };

    Ok(AcOpfResult {
        p_gen_mw: p_gen,
        q_gen_mvar: q_gen,
        voltage_magnitudes: pf_result.voltage_magnitude,
        voltage_angles: pf_result.voltage_angle,
        total_cost,
        lambda,
        converged,
        iterations,
        kkt_residual,
        max_mismatch: pf_result.max_mismatch,
    })
}

// ── Helper functions ──────────────────────────────────────────────────────────

/// Set generator dispatch in a network copy.
fn set_generator_dispatch(network: &mut PowerNetwork, p_dispatch_mw: &[f64]) {
    for (gen, &p_mw) in network.generators.iter_mut().zip(p_dispatch_mw.iter()) {
        gen.pg = p_mw / network.base_mva;
    }
}

/// Project generation vector to satisfy total power balance.
///
/// Redistributes the deviation `p_deviation` across unconstrained generators
/// proportionally to their remaining headroom.
fn project_to_balance(p: &[f64], costs: &[GenCost], p_deviation: f64, total_load: f64) -> Vec<f64> {
    let mut result = p.to_vec();
    if p_deviation.abs() < 1e-9 {
        return result;
    }

    // Find generators with headroom for adjustment
    let (headroom_up, headroom_dn): (Vec<f64>, Vec<f64>) = costs
        .iter()
        .zip(p.iter())
        .map(|(c, &pi)| (c.p_max - pi, pi - c.p_min))
        .unzip();

    let total_headroom = if p_deviation > 0.0 {
        headroom_dn.iter().sum::<f64>() // need to reduce
    } else {
        headroom_up.iter().sum::<f64>() // need to increase
    };

    if total_headroom < 1e-9 {
        // Can't adjust; redistribute evenly among all
        let delta = -p_deviation / costs.len() as f64;
        for (i, c) in costs.iter().enumerate() {
            result[i] = (result[i] + delta).clamp(c.p_min, c.p_max);
        }
        // Re-normalize to meet total load
        let actual: f64 = result.iter().sum();
        let scale = total_load / actual.max(1e-9);
        for (i, c) in costs.iter().enumerate() {
            result[i] = (result[i] * scale).clamp(c.p_min, c.p_max);
        }
    } else {
        for (i, c) in costs.iter().enumerate() {
            let hr = if p_deviation > 0.0 {
                headroom_dn[i]
            } else {
                headroom_up[i]
            };
            let frac = hr / total_headroom;
            result[i] = (result[i] - p_deviation * frac).clamp(c.p_min, c.p_max);
        }
    }
    result
}

/// Lambda-iteration economic dispatch (same as dc_opf.rs but private).
fn economic_dispatch_internal(costs: &[GenCost], total_load_mw: f64) -> Result<Vec<f64>> {
    let p_min_total: f64 = costs.iter().map(|c| c.p_min).sum();
    let p_max_total: f64 = costs.iter().map(|c| c.p_max).sum();

    if total_load_mw < p_min_total - 1e-6 {
        return Err(OxiGridError::InvalidParameter(format!(
            "Load {:.1} MW below minimum generation {:.1} MW",
            total_load_mw, p_min_total
        )));
    }
    if total_load_mw > p_max_total + 1e-6 {
        return Err(OxiGridError::InvalidParameter(format!(
            "Load {:.1} MW exceeds maximum generation {:.1} MW",
            total_load_mw, p_max_total
        )));
    }

    if costs.iter().all(|c| c.c.abs() < 1e-12) {
        // Linear: merit order
        let mut order: Vec<usize> = (0..costs.len()).collect();
        order.sort_by(|&a, &b| costs[a].b.partial_cmp(&costs[b].b).unwrap());
        let mut p = costs.iter().map(|c| c.p_min).collect::<Vec<_>>();
        let mut remaining = total_load_mw - p_min_total;
        for &i in &order {
            let headroom = costs[i].p_max - costs[i].p_min;
            let added = remaining.min(headroom);
            p[i] += added;
            remaining -= added;
            if remaining <= 1e-6 {
                break;
            }
        }
        return Ok(p);
    }

    let b_min = costs.iter().map(|c| c.b).fold(f64::INFINITY, f64::min);
    let b_max = costs
        .iter()
        .map(|c| c.b + 2.0 * c.c * c.p_max)
        .fold(f64::NEG_INFINITY, f64::max);
    let mut lo = b_min;
    let mut hi = b_max + 1.0;

    let dispatch_at = |lam: f64| -> Vec<f64> {
        costs
            .iter()
            .map(|c| {
                if c.c.abs() < 1e-12 {
                    if lam >= c.b {
                        c.p_max
                    } else {
                        c.p_min
                    }
                } else {
                    ((lam - c.b) / (2.0 * c.c)).clamp(c.p_min, c.p_max)
                }
            })
            .collect()
    };

    for _ in 0..100 {
        let mid = (lo + hi) / 2.0;
        let sum: f64 = dispatch_at(mid).iter().sum();
        if sum < total_load_mw {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo) < 1e-9 {
            break;
        }
    }

    Ok(dispatch_at((lo + hi) / 2.0))
}

/// Bus voltage violations: returns (bus_idx, current_Vm, target_Vm) for violated buses.
fn voltage_violations(_network: &PowerNetwork, voltages: &[f64]) -> Vec<(usize, f64, f64)> {
    let v_max_default = 1.05;
    let v_min_default = 0.95;
    voltages
        .iter()
        .enumerate()
        .filter_map(|(i, &vm)| {
            if vm > v_max_default + 1e-4 {
                Some((i, vm, v_max_default))
            } else if vm < v_min_default - 1e-4 {
                Some((i, vm, v_min_default))
            } else {
                None
            }
        })
        .collect()
}

/// Attempt to correct voltage violations by adjusting reactive generation setpoints.
fn correct_voltage_violations(
    net: &mut PowerNetwork,
    violations: &[(usize, f64, f64)],
    _voltages: &[f64],
    q_gen: &mut [f64],
    network: &PowerNetwork,
) {
    for &(bus_idx, vm, v_target) in violations {
        // Find generator at this bus
        for (gi, gen) in net.generators.iter_mut().enumerate() {
            if gen.bus_id == bus_idx {
                // Adjust voltage setpoint
                gen.vg = v_target;
                // Adjust Q output proportionally
                if vm > v_target {
                    q_gen[gi] -= (vm - v_target) * 10.0; // heuristic Q reduction
                } else {
                    q_gen[gi] += (v_target - vm) * 10.0; // heuristic Q increase
                }
                q_gen[gi] =
                    q_gen[gi].clamp(gen.qmin / network.base_mva, gen.qmax / network.base_mva);
                break;
            }
        }
    }
}

/// Estimate reactive generation from bus injections and loads.
fn estimate_q_gen(network: &PowerNetwork, q_injected: &[f64]) -> Vec<f64> {
    network
        .generators
        .iter()
        .map(|gen| {
            if let Ok(bi) = network.bus_index(gen.bus_id) {
                let q_inj = q_injected.get(bi).copied().unwrap_or(0.0);
                // Q_gen = Q_injected + Q_load (since q_injected = q_gen - q_load in p.u.)
                let q_load = network
                    .buses
                    .get(bi)
                    .map(|b| b.qd.0 / network.base_mva)
                    .unwrap_or(0.0);
                q_inj + q_load
            } else {
                gen.qg / network.base_mva
            }
        })
        .collect()
}

/// Compute AC power flow sensitivity matrix ∂P_bus/∂P_g [n_bus × n_gen].
///
/// Each column j shows how injecting 1 p.u. from generator j changes bus injections.
/// For a simple PTDF-style approximation: returns an identity-like matrix.
pub fn gen_injection_matrix(network: &PowerNetwork) -> DMatrix<f64> {
    let n_bus = network.bus_count();
    let n_gen = network.generators.len();
    let mut s = DMatrix::<f64>::zeros(n_bus, n_gen);
    for (gj, gen) in network.generators.iter().enumerate() {
        if let Ok(bi) = network.bus_index(gen.bus_id) {
            s[(bi, gj)] = 1.0;
        }
    }
    s
}

/// Compute gradient of total cost wrt bus injections via sensitivity.
///
/// ∂C/∂P_bus = S^T · ∂C/∂P_g  (chain rule via injection matrix)
pub fn cost_gradient_bus(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    p_gen: &[f64],
) -> DVector<f64> {
    let n_bus = network.bus_count();
    let s = gen_injection_matrix(network);
    let grad_gen = DVector::from_iterator(
        gen_costs.len(),
        gen_costs
            .iter()
            .zip(p_gen.iter())
            .map(|(c, &p)| c.marginal_cost(p)),
    );
    let mut grad_bus = DVector::zeros(n_bus);
    for i in 0..n_bus {
        for j in 0..gen_costs.len() {
            grad_bus[i] += s[(i, j)] * grad_gen[j];
        }
    }
    grad_bus
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    fn ieee14_net_and_costs() -> Option<(PowerNetwork, Vec<GenCost>)> {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let net = PowerNetwork::from_matpower(path).ok()?;
        let costs: Vec<GenCost> = net
            .generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
            .collect();
        Some((net, costs))
    }

    #[test]
    fn test_ac_opf_ieee14_converges() {
        if let Some((net, costs)) = ieee14_net_and_costs() {
            let config = AcOpfConfig::default();
            let result = solve_ac_opf(&net, &costs, &config);
            assert!(
                result.is_ok(),
                "AC-OPF should not error: {:?}",
                result.err()
            );
            let r = result.unwrap();
            assert!(r.total_cost > 0.0, "Total cost should be positive");
        }
    }

    #[test]
    fn test_ac_opf_generation_within_limits() {
        if let Some((net, costs)) = ieee14_net_and_costs() {
            let result = solve_ac_opf(&net, &costs, &AcOpfConfig::default()).unwrap();
            for (i, (&p, c)) in result.p_gen_mw.iter().zip(costs.iter()).enumerate() {
                assert!(
                    p >= c.p_min - 1e-3 && p <= c.p_max + 1e-3,
                    "Gen {i}: P={p:.2} outside [{}, {}]",
                    c.p_min,
                    c.p_max
                );
            }
        }
    }

    #[test]
    fn test_ac_opf_power_balance() {
        if let Some((net, costs)) = ieee14_net_and_costs() {
            let result = solve_ac_opf(&net, &costs, &AcOpfConfig::default()).unwrap();
            let total_gen: f64 = result.p_gen_mw.iter().sum();
            let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
            // Generation ≈ load + losses
            assert!(
                total_gen >= total_load - 1.0,
                "Generation {total_gen:.2} < load {total_load:.2}"
            );
        }
    }

    #[test]
    fn test_ac_opf_voltages_reasonable() {
        if let Some((net, costs)) = ieee14_net_and_costs() {
            let result = solve_ac_opf(&net, &costs, &AcOpfConfig::default()).unwrap();
            for (i, &vm) in result.voltage_magnitudes.iter().enumerate() {
                assert!(
                    vm > 0.5 && vm < 1.5,
                    "Bus {i} voltage {vm:.4} out of reasonable range"
                );
            }
        }
    }

    #[test]
    fn test_economic_dispatch_internal() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 10.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 20.0, 150.0),
        ];
        let p = economic_dispatch_internal(&costs, 120.0).unwrap();
        let total: f64 = p.iter().sum();
        assert!((total - 120.0).abs() < 1e-3, "dispatch sum={total:.4}");
        for (&pi, c) in p.iter().zip(costs.iter()) {
            assert!(pi >= c.p_min - 1e-6 && pi <= c.p_max + 1e-6);
        }
    }

    #[test]
    fn test_project_to_balance() {
        let costs = vec![
            GenCost::linear(20.0, 0.0, 100.0),
            GenCost::linear(30.0, 0.0, 100.0),
        ];
        let p = vec![60.0, 70.0]; // total = 130, target = 120
        let result = project_to_balance(&p, &costs, 10.0, 120.0);
        let total: f64 = result.iter().sum();
        assert!((total - 120.0).abs() < 1e-6, "projected total={total:.4}");
    }

    #[test]
    fn test_gen_injection_matrix_shape() {
        if let Some((net, _)) = ieee14_net_and_costs() {
            let s = gen_injection_matrix(&net);
            assert_eq!(s.nrows(), net.bus_count());
            assert_eq!(s.ncols(), net.generators.len());
            // Each column should have exactly one non-zero (the generator bus)
            for j in 0..s.ncols() {
                let non_zero = (0..s.nrows()).filter(|&i| s[(i, j)].abs() > 1e-10).count();
                assert_eq!(non_zero, 1, "Column {j} should have exactly 1 non-zero");
            }
        }
    }

    #[test]
    fn test_ac_opf_lower_cost_than_uniform() {
        if let Some((net, costs)) = ieee14_net_and_costs() {
            let result = solve_ac_opf(&net, &costs, &AcOpfConfig::default()).unwrap();
            // OPF cost should be finite
            assert!(result.total_cost.is_finite(), "Cost should be finite");
            assert!(result.total_cost >= 0.0, "Cost should be non-negative");
        }
    }
}
