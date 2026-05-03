use crate::error::{OxiGridError, Result};
use crate::network::PowerNetwork;
use crate::powerflow::result::BranchFlow;
use crate::powerflow::{PowerFlowConfig, PowerFlowResult, PowerFlowSolver};
use nalgebra::DMatrix;
use nalgebra::DVector;

/// DC power flow solver using the linearised B'·θ = P formulation.
///
/// Lossless approximation: unity voltage magnitudes, no shunts, no reactive power.
///
/// # Examples
///
/// ```rust
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use oxigrid::network::topology::PowerNetwork;
/// use oxigrid::network::bus::{Bus, BusType};
/// use oxigrid::network::branch::Branch;
/// use oxigrid::powerflow::{PowerFlowConfig, PowerFlowSolver};
/// use oxigrid::powerflow::dc_powerflow::DcPowerFlowSolver;
///
/// let mut net = PowerNetwork::new(100.0);
/// net.buses.push(Bus::new(1, BusType::Slack));
/// net.buses.push(Bus::new(2, BusType::PQ));
/// net.branches.push(Branch {
///     from_bus: 1, to_bus: 2,
///     r: 0.0, x: 0.1, b: 0.0,
///     rate_a: 100.0, rate_b: 100.0, rate_c: 100.0,
///     tap: 0.0, shift: 0.0, status: true,
/// });
///
/// let solver = DcPowerFlowSolver;
/// let config = PowerFlowConfig::default();
/// let result = solver.solve(&net, &config)?;
/// assert!(result.converged);
/// # Ok(()) }
/// ```
pub struct DcPowerFlowSolver;

impl PowerFlowSolver for DcPowerFlowSolver {
    fn solve(&self, network: &PowerNetwork, _config: &PowerFlowConfig) -> Result<PowerFlowResult> {
        network.validate()?;

        let n = network.bus_count();
        let slack_idx = network.slack_bus_index()?;

        let mut b_prime = DMatrix::<f64>::zeros(n, n);

        for branch in &network.branches {
            if !branch.status {
                continue;
            }
            let i = network.bus_index(branch.from_bus)?;
            let j = network.bus_index(branch.to_bus)?;
            let tap = branch.effective_tap();

            let bij = 1.0 / (branch.x * tap);
            b_prime[(i, j)] -= bij;
            b_prime[(j, i)] -= bij;
            b_prime[(i, i)] += bij;
            b_prime[(j, j)] += bij;
        }

        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();
        let m = non_slack.len();

        let mut b_red = DMatrix::<f64>::zeros(m, m);
        for (ri, &i) in non_slack.iter().enumerate() {
            for (rj, &j) in non_slack.iter().enumerate() {
                b_red[(ri, rj)] = b_prime[(i, j)];
            }
        }

        let (p_sched, _) = network.net_injection();
        let p_red = DVector::from_vec(non_slack.iter().map(|&i| p_sched[i]).collect::<Vec<_>>());

        let lu = b_red.lu();
        let theta_red = lu
            .solve(&p_red)
            .ok_or_else(|| OxiGridError::LinearAlgebra("B' matrix is singular".to_string()))?;

        let mut v_ang = vec![0.0_f64; n];
        for (ri, &i) in non_slack.iter().enumerate() {
            v_ang[i] = theta_red[ri];
        }

        let v_mag = vec![1.0_f64; n];

        // Compute branch flows (DC: P only)
        let mut p_inj_bus = vec![0.0_f64; n];
        let mut branch_flows = Vec::with_capacity(network.branches.len());

        for (idx, branch) in network.branches.iter().enumerate() {
            if !branch.status {
                branch_flows.push(BranchFlow {
                    branch_index: idx,
                    from_bus: branch.from_bus,
                    to_bus: branch.to_bus,
                    p_from_mw: 0.0,
                    q_from_mvar: 0.0,
                    p_to_mw: 0.0,
                    q_to_mvar: 0.0,
                    p_loss_mw: 0.0,
                    q_loss_mvar: 0.0,
                    loading_pct: 0.0,
                });
                continue;
            }
            let i = network.bus_index(branch.from_bus)?;
            let j = network.bus_index(branch.to_bus)?;
            let tap = branch.effective_tap();
            let p_ij = (v_ang[i] - v_ang[j]) / (branch.x * tap);
            let p_ij_mw = p_ij * network.base_mva;

            p_inj_bus[i] += p_ij_mw;
            p_inj_bus[j] -= p_ij_mw;

            let loading = if branch.rate_a > 0.0 {
                p_ij_mw.abs() / branch.rate_a * 100.0
            } else {
                0.0
            };

            branch_flows.push(BranchFlow {
                branch_index: idx,
                from_bus: branch.from_bus,
                to_bus: branch.to_bus,
                p_from_mw: p_ij_mw,
                q_from_mvar: 0.0,
                p_to_mw: -p_ij_mw, // lossless in DC
                q_to_mvar: 0.0,
                p_loss_mw: 0.0,
                q_loss_mvar: 0.0,
                loading_pct: loading,
            });
        }

        Ok(PowerFlowResult {
            voltage_magnitude: v_mag,
            voltage_angle: v_ang,
            p_injected: p_inj_bus,
            q_injected: vec![0.0; n],
            branch_flows,
            total_p_loss_mw: 0.0,
            total_q_loss_mvar: 0.0,
            converged: true,
            iterations: 1,
            max_mismatch: 0.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::Generator;

    #[test]
    fn test_dc_2bus() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(50.0);
        net.buses.push(bus2);
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 50.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver.solve(&net, &config).unwrap();
        assert!(result.converged);
        assert!((result.voltage_angle[0]).abs() < 1e-10);
        // Branch flow should be ~50 MW
        assert_eq!(result.branch_flows.len(), 1);
        assert!((result.branch_flows[0].p_from_mw - 50.0).abs() < 1.0);
    }
}
