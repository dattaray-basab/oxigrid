/// DC Optimal Power Flow (DC-OPF).
///
/// Minimises total generation cost subject to:
///   - DC power flow equations:  B' · θ = P_gen − P_load
///   - Generator output limits:  P_min ≤ P_g ≤ P_max
///
/// Cost function (per generator):
///   C(P) = a + b·P + c·P²    (quadratic; set c=0 for linear)
///
/// The QP/LP is solved by the *lambda-iteration* (equal-incremental-cost)
/// method — exact for the unconstrained economic dispatch problem, then
/// clipped to generator limits.
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use serde::{Deserialize, Serialize};

/// Generator cost function coefficients.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GenCost {
    /// Constant term [$/h]
    pub a: f64,
    /// Linear term [$/MWh]
    pub b: f64,
    /// Quadratic term [$/MW²h]
    pub c: f64,
    /// Minimum output [MW]
    pub p_min: f64,
    /// Maximum output [MW]
    pub p_max: f64,
}

impl GenCost {
    /// Create a linear cost function (c = 0).
    pub fn linear(b: f64, p_min: f64, p_max: f64) -> Self {
        Self {
            a: 0.0,
            b,
            c: 0.0,
            p_min,
            p_max,
        }
    }

    /// Create a quadratic cost function.
    pub fn quadratic(a: f64, b: f64, c: f64, p_min: f64, p_max: f64) -> Self {
        Self {
            a,
            b,
            c,
            p_min,
            p_max,
        }
    }

    /// Marginal cost at output P [MW].
    pub fn marginal_cost(&self, p: f64) -> f64 {
        self.b + 2.0 * self.c * p
    }

    /// Total cost at output P [$/h].
    pub fn total_cost(&self, p: f64) -> f64 {
        self.a + self.b * p + self.c * p * p
    }
}

/// Result of a DC-OPF solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcOpfResult {
    /// Optimal generation dispatch [MW] per generator (same order as `costs`)
    pub p_gen_mw: Vec<f64>,
    /// Total generation cost [$/h]
    pub total_cost: f64,
    /// Bus voltage angles [rad]
    pub voltage_angle: Vec<f64>,
    /// Branch active power flows [MW]
    pub branch_flows_mw: Vec<f64>,
    /// Lambda (system marginal price) [$/MWh]
    pub lambda: f64,
}

/// Solve the DC-OPF for a network with given generator cost functions.
///
/// `gen_costs` must be in the same order as `network.generators`.
/// Uses lambda-iteration (bisection on equal-incremental-cost).
pub fn solve_dc_opf(network: &PowerNetwork, gen_costs: &[GenCost]) -> Result<DcOpfResult> {
    let n_gen = network.generators.len();
    if gen_costs.len() != n_gen {
        return Err(OxiGridError::InvalidParameter(format!(
            "gen_costs length {} != generators length {}",
            gen_costs.len(),
            n_gen
        )));
    }

    // Total load demand [MW]
    let total_load: f64 = network.buses.iter().map(|b| b.pd.0).sum();

    // Dispatch via equal incremental cost (lambda iteration)
    let p_dispatch = economic_dispatch(gen_costs, total_load)?;

    let total_cost: f64 = gen_costs
        .iter()
        .zip(p_dispatch.iter())
        .map(|(c, &p)| c.total_cost(p))
        .sum();

    // Effective lambda at solution
    let lambda = {
        let fully_loaded = p_dispatch
            .iter()
            .zip(gen_costs.iter())
            .find(|(p, c)| **p < c.p_max - 1e-6 && **p > c.p_min + 1e-6);
        if let Some((p, c)) = fully_loaded {
            c.marginal_cost(*p)
        } else {
            gen_costs
                .iter()
                .zip(p_dispatch.iter())
                .map(|(c, &p)| c.marginal_cost(p))
                .fold(0.0_f64, f64::max)
        }
    };

    // Run DC power flow with the dispatched generation
    let mut net = network.clone();
    for (gen, &p) in net.generators.iter_mut().zip(p_dispatch.iter()) {
        gen.pg = p / network.base_mva;
    }

    let pf_config = crate::powerflow::PowerFlowConfig {
        method: crate::powerflow::PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let pf_result = net.solve_powerflow(&pf_config)?;

    let branch_flows_mw: Vec<f64> = pf_result.branch_flows.iter().map(|f| f.p_from_mw).collect();

    Ok(DcOpfResult {
        p_gen_mw: p_dispatch,
        total_cost,
        voltage_angle: pf_result.voltage_angle,
        branch_flows_mw,
        lambda,
    })
}

/// Public re-export of economic dispatch for use by other modules.
pub fn economic_dispatch_pub(costs: &[GenCost], total_load_mw: f64) -> Result<Vec<f64>> {
    economic_dispatch(costs, total_load_mw)
}

/// Economic dispatch via lambda-iteration (bisection on marginal cost).
///
/// Finds λ* such that sum(P_g(λ*)) = P_total_load, where
/// P_g(λ) = clamp((λ - b_g) / (2·c_g), P_min, P_max).
fn economic_dispatch(costs: &[GenCost], total_load_mw: f64) -> Result<Vec<f64>> {
    // Check feasibility
    let p_min_total: f64 = costs.iter().map(|c| c.p_min).sum();
    let p_max_total: f64 = costs.iter().map(|c| c.p_max).sum();
    if total_load_mw < p_min_total {
        return Err(OxiGridError::InvalidParameter(format!(
            "Load {:.1} MW below minimum generation {:.1} MW",
            total_load_mw, p_min_total
        )));
    }
    if total_load_mw > p_max_total {
        return Err(OxiGridError::InvalidParameter(format!(
            "Load {:.1} MW exceeds maximum generation {:.1} MW",
            total_load_mw, p_max_total
        )));
    }

    // For purely linear costs (c = 0): merit-order dispatch
    if costs.iter().all(|c| c.c.abs() < 1e-12) {
        return merit_order_dispatch(costs, total_load_mw);
    }

    // Bisect on lambda
    let b_min = costs.iter().map(|c| c.b).fold(f64::INFINITY, f64::min);
    let b_max = costs
        .iter()
        .map(|c| c.marginal_cost(c.p_max))
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
                    let p_opt = (lam - c.b) / (2.0 * c.c);
                    p_opt.clamp(c.p_min, c.p_max)
                }
            })
            .collect()
    };

    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        let p_sum: f64 = dispatch_at(mid).iter().sum();
        if p_sum < total_load_mw {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo) < 1e-6 {
            break;
        }
    }

    Ok(dispatch_at((lo + hi) / 2.0))
}

/// Merit-order dispatch for linear cost functions.
fn merit_order_dispatch(costs: &[GenCost], total_load_mw: f64) -> Result<Vec<f64>> {
    let mut order: Vec<usize> = (0..costs.len()).collect();
    order.sort_by(|&a, &b| costs[a].b.partial_cmp(&costs[b].b).unwrap());

    let mut p = vec![0.0f64; costs.len()];
    let mut remaining = total_load_mw;

    // First set all to minimum
    for &i in &order {
        p[i] = costs[i].p_min;
        remaining -= costs[i].p_min;
    }

    // Load up cheapest units first
    for &i in &order {
        let headroom = costs[i].p_max - costs[i].p_min;
        let added = remaining.min(headroom);
        p[i] += added;
        remaining -= added;
        if remaining <= 1e-6 {
            break;
        }
    }

    Ok(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_gen_costs() -> Vec<GenCost> {
        vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 10.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 20.0, 150.0),
        ]
    }

    #[test]
    fn test_economic_dispatch_total_equals_load() {
        let costs = two_gen_costs();
        let load = 120.0;
        let p = economic_dispatch(&costs, load).unwrap();
        let total: f64 = p.iter().sum();
        assert!((total - load).abs() < 1e-3, "total={:.4}", total);
    }

    #[test]
    fn test_economic_dispatch_within_limits() {
        let costs = two_gen_costs();
        let p = economic_dispatch(&costs, 100.0).unwrap();
        for (gen, pi) in costs.iter().zip(p.iter()) {
            assert!(
                *pi >= gen.p_min - 1e-6 && *pi <= gen.p_max + 1e-6,
                "pi={:.2}",
                pi
            );
        }
    }

    #[test]
    fn test_marginal_cost_equal_at_solution() {
        // Unconstrained: both generators should have equal marginal cost at optimum
        let costs = two_gen_costs();
        let load = 100.0;
        let p = economic_dispatch(&costs, load).unwrap();
        let mc0 = costs[0].marginal_cost(p[0]);
        let mc1 = costs[1].marginal_cost(p[1]);
        // Allow small error due to limits
        let both_unconstrained = p[0] > costs[0].p_min + 1e-3
            && p[0] < costs[0].p_max - 1e-3
            && p[1] > costs[1].p_min + 1e-3
            && p[1] < costs[1].p_max - 1e-3;
        if both_unconstrained {
            assert!((mc0 - mc1).abs() < 0.1, "mc0={:.4} mc1={:.4}", mc0, mc1);
        }
    }

    #[test]
    fn test_merit_order_linear() {
        let costs = vec![
            GenCost::linear(30.0, 0.0, 100.0), // more expensive
            GenCost::linear(20.0, 0.0, 100.0), // cheaper
        ];
        let p = economic_dispatch(&costs, 80.0).unwrap();
        // Cheaper unit should be fully loaded first
        assert!(p[1] >= p[0] - 1e-6, "p1={:.2} p0={:.2}", p[1], p[0]);
    }

    #[test]
    fn test_infeasible_load_too_high() {
        let costs = vec![GenCost::linear(20.0, 0.0, 50.0)];
        assert!(economic_dispatch(&costs, 100.0).is_err());
    }
}
