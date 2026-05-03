//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

/// Solve A·x = b via Gaussian elimination with partial pivoting.
/// Returns `None` if the matrix is singular.
#[allow(clippy::needless_range_loop)]
pub(crate) fn gaussian_elimination(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    if n == 0 {
        return Some(vec![]);
    }
    let mut m: Vec<Vec<f64>> = a
        .iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for row in (col + 1)..n {
            let abs_val = m[row][col].abs();
            if abs_val > max_val {
                max_val = abs_val;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        m.swap(col, max_row);
        let pivot = m[col][col];
        for row in (col + 1)..n {
            let factor = m[row][col] / pivot;
            for k in col..=n {
                let piv_k = m[col][k];
                m[row][k] -= factor * piv_k;
            }
        }
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = m[i][n];
        for j in (i + 1)..n {
            s -= m[i][j] * x[j];
        }
        x[i] = s / m[i][i];
    }
    Some(x)
}
#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    fn make_branch(from: usize, to: usize, r: f64, x: f64) -> Branch {
        Branch {
            from_bus: from,
            to_bus: to,
            r,
            x,
            b: 0.0,
            rate_a: 0.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }
    fn make_bus(id: usize, bus_type: BusType, pd: f64, vm: f64) -> Bus {
        use crate::units::{Power, ReactivePower, Voltage};
        Bus {
            id,
            name: format!("Bus{id}"),
            bus_type,
            base_kv: Voltage(110.0),
            vm,
            va: 0.0,
            pd: Power(pd),
            qd: ReactivePower(0.0),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        }
    }
    #[test]
    fn test_single_vsc_hvdc() {
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 50.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.1));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 30.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve();
        assert!(result.is_ok(), "single VSC HVDC should converge");
        let r = result.unwrap();
        assert!(r.converged);
        assert_eq!(r.dc_voltages.len(), 1);
        assert!((r.dc_voltages[0] - 1.0).abs() < 1e-9);
    }
    #[test]
    fn test_back_to_back_converter() {
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 20.0, 1.0));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: -20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.02,
            q_min_mvar: -30.0,
            q_max_mvar: 30.0,
            rated_mva: 50.0,
        });
        solver.add_vsc(VscConverter {
            id: 1,
            ac_bus: 1,
            dc_bus: 0,
            mode: VscMode::PacVac,
            p_set_mw: 20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.02,
            q_min_mvar: -30.0,
            q_max_mvar: 30.0,
            rated_mva: 50.0,
        });
        let result = solver.solve();
        assert!(
            result.is_ok(),
            "back-to-back should converge: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(r.converged);
        assert_eq!(r.ac_voltages.len(), 2);
    }
    #[test]
    fn test_dc_slack_maintains_voltage() {
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 1,
            n_dc_buses: 2,
            tolerance: 1e-5,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 1,
            v_dc_pu: 0.98,
            p_load_mw: 50.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 1,
            resistance_pu: 0.02,
            rating_mw: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 50.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.02,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve().expect("DC slack test should converge");
        assert!(
            (result.dc_voltages[0] - 1.02).abs() < 1e-9,
            "DC slack voltage should be 1.02 pu, got {}",
            result.dc_voltages[0]
        );
    }
    #[test]
    fn test_converter_losses_reduce_dc_power() {
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 40.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.05));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        let loss_frac = 0.05;
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: -40.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: loss_frac,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve().expect("loss test should converge");
        let p_ac = result.vsc_p_ac_mw[0].abs();
        let p_dc = result.vsc_p_dc_mw[0].abs();
        assert!(
            p_dc <= p_ac + 1e-6,
            "DC power ({p_dc}) should not exceed AC power ({p_ac}) for rectifier with losses"
        );
        assert!(result.total_converter_losses_mw >= 0.0);
    }
    #[test]
    fn test_multi_terminal_dc() {
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 3,
            n_dc_buses: 3,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 30.0, 1.0));
        solver.add_ac_bus(make_bus(3, BusType::PQ, 20.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.05));
        solver.add_ac_branch(make_branch(2, 3, 0.01, 0.05));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 1,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 2,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 1,
            resistance_pu: 0.01,
            rating_mw: 200.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 1,
            to_bus: 2,
            resistance_pu: 0.01,
            rating_mw: 200.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 2,
            resistance_pu: 0.02,
            rating_mw: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 50.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -100.0,
            q_max_mvar: 100.0,
            rated_mva: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 1,
            ac_bus: 1,
            dc_bus: 1,
            mode: VscMode::PdcVdc,
            p_set_mw: -30.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        solver.add_vsc(VscConverter {
            id: 2,
            ac_bus: 2,
            dc_bus: 2,
            mode: VscMode::PdcVdc,
            p_set_mw: -20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve();
        assert!(
            result.is_ok(),
            "multi-terminal DC should converge: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(r.converged);
        assert_eq!(r.dc_voltages.len(), 3);
        assert_eq!(r.dc_line_flows_mw.len(), 3);
    }
    fn two_bus_ac_ybus(x: f64) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let b = 1.0 / x;
        let g = vec![vec![0.0_f64; 2]; 2];
        let bm = vec![vec![b, -b], vec![-b, b]];
        (g, bm)
    }
    fn three_bus_ac_ybus() -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let b = 10.0_f64;
        let g = vec![vec![0.0_f64; 3]; 3];
        let mut bm = vec![vec![0.0_f64; 3]; 3];
        for (i, j) in [(0usize, 1usize), (1, 2), (0, 2)] {
            bm[i][i] += b;
            bm[j][j] += b;
            bm[i][j] -= b;
            bm[j][i] -= b;
        }
        (g, bm)
    }
    #[test]
    fn test_dc_bus_creation() {
        let bus = DcBus::new(3, DcBusType::PQ, 320.0);
        assert_eq!(bus.id, 3);
        assert_eq!(bus.bus_type, DcBusType::PQ);
        assert!((bus.v_dc_kv - 320.0).abs() < 1e-9);
        assert!((bus.v_dc_nom_kv - 320.0).abs() < 1e-9);
        assert_eq!(bus.p_load_mw, 0.0);
    }
    #[test]
    fn test_dc_branch_creation() {
        let br = DcBranch::new(0, 1, 8.0, 150.0);
        assert_eq!(br.from, 0);
        assert_eq!(br.to, 1);
        assert!((br.resistance_ohm - 8.0).abs() < 1e-9);
        assert!((br.length_km - 150.0).abs() < 1e-9);
        let g = br.conductance().expect("conductance ok");
        assert!((g - 0.125).abs() < 1e-9);
    }
    #[test]
    fn test_converter_creation() {
        let conv = AcDcConverter::new(1, 2, 0, ConverterType::PV, 120.0, -10.0, 320.0);
        assert_eq!(conv.id, 1);
        assert_eq!(conv.ac_bus, 2);
        assert_eq!(conv.dc_bus, 0);
        assert!((conv.p_ref_mw - 120.0).abs() < 1e-9);
        assert!((conv.losses_fraction - 0.02).abs() < 1e-9);
        assert!(conv.is_rectifier);
    }
    #[test]
    fn test_ac_dc_network_build() {
        let (g, b) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g, b, vec![conv], dc_buses, vec![]).expect("build ok");
        assert_eq!(net.n_ac_buses, 2);
        assert_eq!(net.n_dc_buses, 1);
        assert_eq!(net.converters.len(), 1);
        assert_eq!(net.dc_g.len(), 1);
    }
    #[test]
    fn test_build_dc_conductance_2bus() {
        let dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g = AcDcNetwork::build_dc_conductance_matrix(&dc_buses, &branches).expect("ok");
        let g_br = 0.1_f64;
        assert!((g[0][0] - g_br).abs() < 1e-12);
        assert!((g[1][1] - g_br).abs() < 1e-12);
        assert!((g[0][1] + g_br).abs() < 1e-12);
        assert!((g[1][0] + g_br).abs() < 1e-12);
    }
    #[test]
    fn test_build_dc_conductance_3bus() {
        let dc_buses: Vec<DcBus> = (0..3)
            .map(|i| DcBus::new(i, DcBusType::PQ, 320.0))
            .collect();
        let branches = vec![
            DcBranch::new(0, 1, 10.0, 100.0),
            DcBranch::new(1, 2, 5.0, 100.0),
            DcBranch::new(0, 2, 20.0, 100.0),
        ];
        let g = AcDcNetwork::build_dc_conductance_matrix(&dc_buses, &branches).expect("ok");
        for (row_i, row) in g.iter().enumerate() {
            let s: f64 = row.iter().sum();
            assert!(s.abs() < 1e-11, "Row {row_i} sum = {s}, expected ~0");
        }
    }
    #[test]
    fn test_dc_mismatch_balanced() {
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let (g, b) = two_bus_ac_ybus(0.1);
        let net = AcDcNetwork::new(2, 1, g, b, vec![], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let dp = pf.compute_dc_mismatches();
        assert!((dp[0]).abs() < 1e-9, "Slack bus mismatch should be zero");
    }
    #[test]
    fn test_dc_mismatch_imbalanced() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 100.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let (g, b) = two_bus_ac_ybus(0.1);
        let net = AcDcNetwork::new(2, 2, g, b, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let dp = pf.compute_dc_mismatches();
        assert!(
            dp[1].abs() > 1e-9,
            "Loaded bus should have non-zero mismatch"
        );
    }
    #[test]
    fn test_linear_system_solver_2x2() {
        let mut a = vec![vec![2.0, 3.0], vec![4.0, 1.0]];
        let mut b = vec![8.0, 6.0];
        let x = AcDcPowerFlow::solve_linear_system(&mut a, &mut b).expect("solved");
        assert!((x[0] - 1.0).abs() < 1e-9, "x[0]={}", x[0]);
        assert!((x[1] - 2.0).abs() < 1e-9, "x[1]={}", x[1]);
    }
    #[test]
    fn test_linear_system_solver_3x3() {
        let mut a = vec![
            vec![1.0, 2.0, -1.0],
            vec![2.0, 1.0, 3.0],
            vec![-1.0, 3.0, 2.0],
        ];
        let mut b = vec![1.0, 13.0, 4.0];
        let x: Vec<f64> = AcDcPowerFlow::solve_linear_system(&mut a, &mut b).expect("solved");
        let res = [
            (1.0 * x[0] + 2.0 * x[1] - 1.0 * x[2] - 1.0).abs(),
            (2.0 * x[0] + 1.0 * x[1] + 3.0 * x[2] - 13.0).abs(),
            (-x[0] + 3.0 * x[1] + 2.0 * x[2] - 4.0).abs(),
        ];
        for r in res {
            assert!(r < 1e-9, "residual={r}");
        }
    }
    #[test]
    fn test_solve_pure_dc_2bus() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 100.0;
        let branches = vec![DcBranch::new(0, 1, 5.0, 100.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![conv], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0], &[0.0], &[3]).expect("solve ok");
        assert!(result.iterations > 0);
        assert_eq!(result.dc_branch_flows.len(), 1);
        assert_eq!(result.dc_voltage.len(), 2);
    }
    #[test]
    fn test_solve_pure_dc_3bus() {
        let mut dc_buses: Vec<DcBus> = (0..3)
            .map(|i| {
                DcBus::new(
                    i,
                    if i == 0 {
                        DcBusType::Slack
                    } else {
                        DcBusType::PQ
                    },
                    320.0,
                )
            })
            .collect();
        dc_buses[1].p_load_mw = 50.0;
        dc_buses[2].p_load_mw = 80.0;
        let branches = vec![
            DcBranch::new(0, 1, 5.0, 100.0),
            DcBranch::new(1, 2, 5.0, 100.0),
            DcBranch::new(0, 2, 10.0, 200.0),
        ];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(1, 3, g_ac, b_ac, vec![conv], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0], &[0.0], &[3]).expect("solve ok");
        assert_eq!(result.dc_branch_flows.len(), 3);
        assert_eq!(result.dc_voltage.len(), 3);
    }
    #[test]
    fn test_solve_2bus_ac_with_converter() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 1, 0, ConverterType::PQ, 50.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, -0.5], &[0.0, -0.2], &[3, 1]).expect("ok");
        assert_eq!(result.ac_voltage_magnitude.len(), 2);
        assert_eq!(result.ac_voltage_angle.len(), 2);
        assert_eq!(result.converter_p_ac.len(), 1);
        assert_eq!(result.converter_p_dc.len(), 1);
    }
    #[test]
    fn test_solve_3bus_hybrid() {
        let (g_ac, b_ac) = three_bus_ac_ybus();
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 80.0;
        let dc_branches = vec![DcBranch::new(0, 1, 8.0, 150.0)];
        let c0 = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let c1 = AcDcConverter::new(1, 2, 1, ConverterType::PQ, -80.0, 0.0, 320.0);
        let net =
            AcDcNetwork::new(3, 2, g_ac, b_ac, vec![c0, c1], dc_buses, dc_branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf
            .solve(&[0.0, -0.3, 0.8], &[0.0, -0.1, 0.0], &[3, 1, 2])
            .expect("ok");
        assert_eq!(result.converter_p_ac.len(), 2);
        assert_eq!(result.converter_q_ac.len(), 2);
        assert_eq!(result.dc_branch_flows.len(), 1);
    }
    #[test]
    fn test_converter_loss_computation() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let mut conv = AcDcConverter::new(0, 0, 0, ConverterType::PQ, 100.0, 0.0, 320.0);
        conv.is_rectifier = true;
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        pf.update_converter_operating_points();
        let c = &pf.network.converters[0];
        assert!((c.p_dc_mw - 98.0).abs() < 1e-9, "p_dc={}", c.p_dc_mw);
    }
    #[test]
    fn test_dc_branch_flows_direction() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 330.0),
            DcBus::new(1, DcBusType::PQ, 310.0),
        ];
        dc_buses[0].v_dc_kv = 330.0;
        dc_buses[1].v_dc_kv = 310.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let flows = pf.compute_dc_flows();
        assert_eq!(flows.len(), 1);
        assert!(
            flows[0] > 0.0,
            "Expected positive flow from higher to lower voltage"
        );
    }
    #[test]
    fn test_total_dc_losses() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 310.0),
        ];
        dc_buses[0].v_dc_kv = 320.0;
        dc_buses[1].v_dc_kv = 310.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let flows = pf.compute_dc_flows();
        let (_, dc_losses, _) = pf.compute_losses(&flows);
        assert!(dc_losses >= 0.0, "DC losses must be non-negative");
    }
    #[test]
    fn test_total_converter_losses() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::PQ, 200.0, 0.0, 320.0);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let (_, _, conv_losses) = pf.compute_losses(&[]);
        assert!(
            (conv_losses - 4.0).abs() < 1e-9,
            "conv_losses={conv_losses}"
        );
    }
    #[test]
    fn test_convergence_tolerance() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.05);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 1, 0, ConverterType::PQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 100,
            tolerance_pu: 1e-10,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, 0.0], &[0.0, 0.0], &[3, 1]);
        assert!(result.is_ok(), "Must not error even with tight tolerance");
        let r = result.unwrap();
        assert!(r.iterations <= 100);
    }
    #[test]
    fn test_acdc_result_struct() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, 0.0], &[0.0, 0.0], &[3, 1]).expect("ok");
        assert_eq!(result.ac_voltage_magnitude.len(), 2);
        assert_eq!(result.ac_voltage_angle.len(), 2);
        assert_eq!(result.dc_voltage.len(), 1);
        assert_eq!(result.converter_p_ac.len(), 1);
        assert_eq!(result.converter_q_ac.len(), 1);
        assert_eq!(result.converter_p_dc.len(), 1);
        assert_eq!(result.dc_branch_flows.len(), 0);
        assert!(result.total_converter_losses_mw >= 0.0);
        assert!(result.max_mismatch >= 0.0);
    }
}
