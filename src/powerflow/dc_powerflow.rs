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

    #[test]
    fn test_dc_3bus_power_balance() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(30.0);
        net.buses.push(bus2);
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = crate::units::Power(20.0);
        net.buses.push(bus3);
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
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 3-bus star solve should succeed");
        assert!(result.converged, "solver should report converged");
        assert!(
            result.voltage_angle[0].abs() < 1e-10,
            "slack bus angle must be zero"
        );
        assert!(
            result.voltage_angle[1].is_finite() && result.voltage_angle[1] != 0.0,
            "bus 2 angle should be finite and non-zero"
        );
        assert!(
            result.voltage_angle[2].is_finite() && result.voltage_angle[2] != 0.0,
            "bus 3 angle should be finite and non-zero"
        );
        let p_sum: f64 = result.p_injected.iter().sum();
        assert!(
            p_sum.abs() < 1.0,
            "total injected power should be near zero (lossless), got {}",
            p_sum
        );
    }

    #[test]
    fn test_dc_angle_differences_finite() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(30.0);
        net.buses.push(bus2);
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = crate::units::Power(20.0);
        net.buses.push(bus3);
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
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 3-bus star solve should succeed");
        for (idx, &angle) in result.voltage_angle.iter().enumerate() {
            assert!(
                angle.is_finite(),
                "voltage angle at bus index {} is not finite: {}",
                idx,
                angle
            );
        }
    }

    #[test]
    fn test_dc_flat_start_converges() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(40.0);
        net.buses.push(bus2);
        net.generators.push(Generator {
            bus_id: 1,
            pg: 40.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 2-bus flat-start solve should succeed");
        assert!(
            result.converged && result.iterations == 1,
            "DC power flow must converge in exactly one iteration"
        );
    }

    #[test]
    fn test_dc_branch_flow_sum() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(60.0);
        net.buses.push(bus2);
        net.buses.push(Bus::new(3, BusType::PQ));
        net.generators.push(Generator {
            bus_id: 1,
            pg: 30.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.generators.push(Generator {
            bus_id: 3,
            pg: 30.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.15,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 3-bus ring solve should succeed");
        assert_eq!(
            result.branch_flows.len(),
            3,
            "expected 3 branch flow entries"
        );
        for (idx, flow) in result.branch_flows.iter().enumerate() {
            assert!(
                flow.p_from_mw.is_finite(),
                "branch {} p_from_mw is not finite: {}",
                idx,
                flow.p_from_mw
            );
        }
    }

    #[test]
    fn test_dc_voltage_magnitudes_unity() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(30.0);
        net.buses.push(bus2);
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = crate::units::Power(20.0);
        net.buses.push(bus3);
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
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 3-bus star solve should succeed");
        for (idx, &vm) in result.voltage_magnitude.iter().enumerate() {
            assert!(
                (vm - 1.0).abs() < 1e-10,
                "voltage magnitude at bus index {} should be 1.0 p.u., got {}",
                idx,
                vm
            );
        }
    }

    /// A disabled branch (status = false) must produce a zero-flow BranchFlow entry
    /// and must not affect the voltage angles of the enabled network.
    #[test]
    fn test_dc_disabled_branch_zero_flow() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(40.0);
        net.buses.push(bus2);
        net.generators.push(Generator {
            bus_id: 1,
            pg: 40.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        // Active branch
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        // Disabled branch (parallel path, should carry no flow)
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.05,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: false,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC solve with disabled branch should succeed");

        assert!(result.converged, "solver should converge");
        assert_eq!(result.branch_flows.len(), 2, "expect 2 BranchFlow entries");

        let disabled_flow = &result.branch_flows[1];
        assert_eq!(
            disabled_flow.p_from_mw, 0.0,
            "disabled branch p_from_mw must be 0"
        );
        assert_eq!(
            disabled_flow.p_to_mw, 0.0,
            "disabled branch p_to_mw must be 0"
        );
        assert_eq!(
            disabled_flow.loading_pct, 0.0,
            "disabled branch loading_pct must be 0"
        );
    }

    /// DC power flow is lossless: p_to_mw must equal -p_from_mw for every active branch.
    #[test]
    fn test_dc_lossless_p_to_negates_p_from() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(30.0);
        net.buses.push(bus2);
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = crate::units::Power(20.0);
        net.buses.push(bus3);
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
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 3-bus star solve should succeed");

        for (idx, flow) in result.branch_flows.iter().enumerate() {
            let sum = flow.p_from_mw + flow.p_to_mw;
            assert!(
                sum.abs() < 1e-9,
                "branch {}: p_from_mw + p_to_mw should be 0 (lossless), got {}",
                idx,
                sum
            );
        }
    }

    /// Reactive power and losses must all be zero in the DC approximation.
    #[test]
    fn test_dc_q_injected_and_losses_zero() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(25.0);
        net.buses.push(bus2);
        net.generators.push(Generator {
            bus_id: 1,
            pg: 25.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC 2-bus solve should succeed");

        for (idx, &q) in result.q_injected.iter().enumerate() {
            assert_eq!(q, 0.0, "q_injected[{}] must be 0 in DC approximation", idx);
        }
        assert_eq!(
            result.total_p_loss_mw, 0.0,
            "total_p_loss_mw must be 0 in DC approximation"
        );
        assert_eq!(
            result.total_q_loss_mvar, 0.0,
            "total_q_loss_mvar must be 0 in DC approximation"
        );
        for (idx, flow) in result.branch_flows.iter().enumerate() {
            assert_eq!(flow.p_loss_mw, 0.0, "branch {} p_loss_mw must be 0", idx);
            assert_eq!(
                flow.q_loss_mvar, 0.0,
                "branch {} q_loss_mvar must be 0",
                idx
            );
            assert_eq!(
                flow.q_from_mvar, 0.0,
                "branch {} q_from_mvar must be 0",
                idx
            );
            assert_eq!(flow.q_to_mvar, 0.0, "branch {} q_to_mvar must be 0", idx);
        }
    }

    /// loading_pct = |p_from_mw| / rate_a * 100.  At 50% loading (50 MW on 100 MVA line).
    #[test]
    fn test_dc_loading_pct_calculation() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = crate::units::Power(50.0); // 50 MW load
        net.buses.push(bus2);
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
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0, // 100 MVA rating -> 50 MW = 50%
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        // A second branch with rate_a = 0 to exercise the zero-rating branch
        net.buses.push(Bus::new(3, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 3,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 0.0, // zero rating -> loading_pct should be 0
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC solve should succeed");

        // Branch 0: 50 MW on 100 MVA line = 50%
        let loading0 = result.branch_flows[0].loading_pct;
        assert!(
            (loading0 - 50.0).abs() < 1.0,
            "branch 0 loading_pct should be ~50%, got {}",
            loading0
        );

        // Branch 1: rate_a = 0 -> loading_pct must be 0
        let loading1 = result.branch_flows[1].loading_pct;
        assert_eq!(
            loading1, 0.0,
            "branch with rate_a=0 must have loading_pct=0, got {}",
            loading1
        );
    }

    /// A non-unity transformer tap (e.g. 0.95) scales the susceptance bij = 1/(x*tap).
    /// With a tap < 1 the effective susceptance increases, so the angle difference shrinks.
    #[test]
    fn test_dc_transformer_tap_affects_flow() {
        // Reference network: unity tap
        let build_net = |tap: f64| {
            let mut net = PowerNetwork::new(100.0);
            net.buses.push(Bus::new(1, BusType::Slack));
            let mut bus2 = Bus::new(2, BusType::PQ);
            bus2.pd = crate::units::Power(40.0);
            net.buses.push(bus2);
            net.generators.push(Generator {
                bus_id: 1,
                pg: 40.0,
                qg: 0.0,
                qmax: 999.0,
                qmin: -999.0,
                vg: 1.0,
                mbase: 100.0,
                status: true,
                pmax: 999.0,
                pmin: 0.0,
            });
            net.branches.push(Branch {
                from_bus: 1,
                to_bus: 2,
                r: 0.0,
                x: 0.1,
                b: 0.0,
                rate_a: 200.0,
                rate_b: 200.0,
                rate_c: 200.0,
                tap,
                shift: 0.0,
                status: true,
            });
            net
        };

        let config = PowerFlowConfig::default();

        let net_unity = build_net(0.0); // tap=0 -> effective_tap() returns 1.0
        let result_unity = DcPowerFlowSolver
            .solve(&net_unity, &config)
            .expect("unity-tap solve should succeed");
        let angle_unity = result_unity.voltage_angle[1];

        let net_tap = build_net(0.95); // tap < 1 -> larger bij -> smaller angle difference
        let result_tap = DcPowerFlowSolver
            .solve(&net_tap, &config)
            .expect("0.95-tap solve should succeed");
        let angle_tap = result_tap.voltage_angle[1];

        // With tap=0.95 the effective susceptance is 1/(0.1*0.95) > 1/(0.1*1.0),
        // so the angle difference should be strictly smaller in magnitude.
        assert!(
            angle_tap.abs() < angle_unity.abs(),
            "tap=0.95 angle ({:.6}) should be smaller in magnitude than unity-tap angle ({:.6})",
            angle_tap,
            angle_unity
        );
    }

    /// Zero-load 2-bus network: all PQ buses have pd=0, so the slack supplies nothing
    /// and all voltage angles should be exactly zero.
    #[test]
    fn test_dc_zero_load_flat_angles() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ)); // pd defaults to 0
        net.buses.push(Bus::new(3, BusType::PQ));
        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.0,
            x: 0.15,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("zero-load DC solve should succeed");

        for (idx, &angle) in result.voltage_angle.iter().enumerate() {
            assert!(
                angle.abs() < 1e-10,
                "bus index {} angle should be 0 for zero load, got {}",
                idx,
                angle
            );
        }
        for (idx, flow) in result.branch_flows.iter().enumerate() {
            assert!(
                flow.p_from_mw.abs() < 1e-9,
                "branch {} p_from_mw should be ~0 for zero load, got {}",
                idx,
                flow.p_from_mw
            );
        }
    }

    /// BranchFlow entries must carry correct metadata: branch_index, from_bus, to_bus.
    #[test]
    fn test_dc_branch_flow_metadata() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(10, BusType::Slack)); // non-sequential bus IDs
        let mut bus20 = Bus::new(20, BusType::PQ);
        bus20.pd = crate::units::Power(20.0);
        net.buses.push(bus20);
        let mut bus30 = Bus::new(30, BusType::PQ);
        bus30.pd = crate::units::Power(10.0);
        net.buses.push(bus30);
        net.generators.push(Generator {
            bus_id: 10,
            pg: 30.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 10,
            to_bus: 20,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 10,
            to_bus: 30,
            r: 0.0,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC metadata test solve should succeed");

        assert_eq!(result.branch_flows.len(), 2, "expected 2 branch flows");

        let f0 = &result.branch_flows[0];
        assert_eq!(f0.branch_index, 0, "first flow branch_index should be 0");
        assert_eq!(f0.from_bus, 10, "first flow from_bus should be 10");
        assert_eq!(f0.to_bus, 20, "first flow to_bus should be 20");

        let f1 = &result.branch_flows[1];
        assert_eq!(f1.branch_index, 1, "second flow branch_index should be 1");
        assert_eq!(f1.from_bus, 10, "second flow from_bus should be 10");
        assert_eq!(f1.to_bus, 30, "second flow to_bus should be 30");
    }

    /// A PV bus (generator with non-zero pg at a non-slack bus) should be treated as
    /// a net injector. The solver must still converge and the angles must reflect the
    /// reduced net load at the PV bus.
    #[test]
    fn test_dc_pv_bus_net_injection() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        // Bus 2: PV bus with local generation and local load
        let mut bus2 = Bus::new(2, BusType::PV);
        bus2.pd = crate::units::Power(30.0); // 30 MW load
        net.buses.push(bus2);
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = crate::units::Power(20.0);
        net.buses.push(bus3);
        // Slack supplies the remainder
        net.generators.push(Generator {
            bus_id: 1,
            pg: 10.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        // Local generator at bus 2 covers 20 MW of its own 30 MW load
        net.generators.push(Generator {
            bus_id: 2,
            pg: 20.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let config = PowerFlowConfig::default();
        let result = DcPowerFlowSolver
            .solve(&net, &config)
            .expect("DC PV bus solve should succeed");

        assert!(result.converged, "solver must converge with PV bus");
        assert_eq!(result.voltage_angle[0], 0.0, "slack bus angle must be zero");
        // Net injection at bus 2 = pg - pd = 20 - 30 = -10 MW (net load),
        // so bus 2 angle must be negative (angle decreases toward load).
        assert!(
            result.voltage_angle[1] < 0.0,
            "bus 2 with net load should have negative angle, got {}",
            result.voltage_angle[1]
        );
        // All angles must be finite
        for (idx, &a) in result.voltage_angle.iter().enumerate() {
            assert!(
                a.is_finite(),
                "voltage angle at bus {} is not finite: {}",
                idx,
                a
            );
        }
    }
}
