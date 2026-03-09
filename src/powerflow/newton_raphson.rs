use crate::error::{OxiGridError, Result};
use crate::network::bus::BusType;
use crate::network::PowerNetwork;
#[cfg(not(feature = "parallel"))]
use crate::powerflow::jacobian::build_jacobian;
#[cfg(feature = "parallel")]
use crate::powerflow::jacobian::build_jacobian_parallel as build_jacobian;
use crate::powerflow::{PowerFlowConfig, PowerFlowResult, PowerFlowSolver};
use nalgebra::DVector;
use num_complex::Complex64;
use sprs::CsMat;

use super::result::BranchFlow;

pub struct NewtonRaphsonSolver;

impl PowerFlowSolver for NewtonRaphsonSolver {
    fn solve(&self, network: &PowerNetwork, config: &PowerFlowConfig) -> Result<PowerFlowResult> {
        solve_nr(
            network,
            config,
            &mut network.buses.iter().map(|b| b.bus_type).collect::<Vec<_>>(),
        )
    }
}

fn solve_nr(
    network: &PowerNetwork,
    config: &PowerFlowConfig,
    bus_types: &mut Vec<BusType>,
) -> Result<PowerFlowResult> {
    network.validate()?;

    let n = network.bus_count();
    let ybus = network.admittance_matrix()?;

    // Classify buses using overridden types (supports Q-limit switching)
    let mut pq_indices = Vec::new();
    let mut pv_indices = Vec::new();

    for (i, &bt) in bus_types.iter().enumerate() {
        match bt {
            BusType::PQ => pq_indices.push(i),
            BusType::PV => pv_indices.push(i),
            BusType::Slack => {}
        }
    }

    let mut pvpq_indices: Vec<usize> = pv_indices.clone();
    pvpq_indices.extend_from_slice(&pq_indices);
    pvpq_indices.sort();

    // Initialize voltage from network initial conditions
    let mut v_mag: Vec<f64> = network.buses.iter().map(|b| b.vm).collect();
    let mut v_ang: Vec<f64> = network.buses.iter().map(|b| b.va).collect();

    // Scheduled power injections (p.u.)
    let (p_sched, q_sched) = network.net_injection();

    let mut converged = false;
    let mut iterations = 0;
    let mut max_mismatch = f64::MAX;

    for iter in 0..config.max_iter {
        let (p_calc, q_calc) = calculate_power(&ybus, &v_mag, &v_ang, n);

        // Mismatch: ΔP for pvpq buses, ΔQ for pq buses
        let mut mismatch: Vec<f64> = pvpq_indices
            .iter()
            .map(|&i| p_sched[i] - p_calc[i])
            .collect();
        let dq: Vec<f64> = pq_indices.iter().map(|&i| q_sched[i] - q_calc[i]).collect();
        mismatch.extend_from_slice(&dq);

        max_mismatch = mismatch.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);

        if max_mismatch < config.tolerance {
            converged = true;
            iterations = iter;
            break;
        }

        let jac = build_jacobian(
            &ybus,
            &v_mag,
            &v_ang,
            &p_calc,
            &q_calc,
            &pq_indices,
            &pvpq_indices,
        );

        let rhs = DVector::from_vec(mismatch);
        let lu = jac.lu();
        let dx = lu
            .solve(&rhs)
            .ok_or_else(|| OxiGridError::LinearAlgebra("Jacobian is singular".to_string()))?;

        // Step-size limiting: prevent large updates that could cause divergence
        const MAX_DTHETA: f64 = 0.5; // rad per iteration
        const MAX_DV_REL: f64 = 0.2; // relative voltage change per iteration

        let npvpq = pvpq_indices.len();
        for (col, &i) in pvpq_indices.iter().enumerate() {
            v_ang[i] += dx[col].clamp(-MAX_DTHETA, MAX_DTHETA);
        }
        for (col, &i) in pq_indices.iter().enumerate() {
            let dv_rel = dx[npvpq + col].clamp(-MAX_DV_REL, MAX_DV_REL);
            v_mag[i] *= 1.0 + dv_rel;
        }

        iterations = iter + 1;
    }

    // Q-limit enforcement: switch PV -> PQ if Q is out of range
    if config.enforce_q_limits && converged {
        let (_, q_calc_final) = calculate_power(&ybus, &v_mag, &v_ang, n);
        let mut switched = false;
        // Track (generator_index, forced_q_mvar) for switched buses
        let mut q_fixes: Vec<(usize, f64)> = Vec::new();

        for (gen_idx, gen) in network.generators.iter().enumerate() {
            if !gen.status {
                continue;
            }
            if let Ok(bus_idx) = network.bus_index(gen.bus_id) {
                if bus_types[bus_idx] != BusType::PV {
                    continue;
                }
                let q_gen_pu = q_calc_final[bus_idx];
                let q_gen_mvar = q_gen_pu * network.base_mva;

                if q_gen_mvar > gen.qmax || q_gen_mvar < gen.qmin {
                    bus_types[bus_idx] = BusType::PQ;
                    let q_fixed = if q_gen_mvar > gen.qmax {
                        gen.qmax
                    } else {
                        gen.qmin
                    };
                    q_fixes.push((gen_idx, q_fixed));
                    switched = true;
                }
            }
        }

        if switched {
            // Clone network and fix Q injections for switched generators
            let mut net2 = network.clone();
            for (gen_idx, q_fixed_mvar) in q_fixes {
                net2.generators[gen_idx].qg = q_fixed_mvar;
            }
            let mut config2 = config.clone();
            config2.enforce_q_limits = false;
            return solve_nr(&net2, &config2, bus_types);
        }
    }

    // Final power calculations
    let (p_calc, q_calc) = calculate_power(&ybus, &v_mag, &v_ang, n);

    let p_inj: Vec<f64> = p_calc.iter().map(|&p| p * network.base_mva).collect();
    let q_inj: Vec<f64> = q_calc.iter().map(|&q| q * network.base_mva).collect();

    let total_p_loss = p_inj.iter().sum::<f64>();
    let total_q_loss = q_inj.iter().sum::<f64>();

    let branch_flows = compute_branch_flows(network, &v_mag, &v_ang);

    Ok(PowerFlowResult {
        voltage_magnitude: v_mag,
        voltage_angle: v_ang,
        p_injected: p_inj,
        q_injected: q_inj,
        branch_flows,
        total_p_loss_mw: total_p_loss,
        total_q_loss_mvar: total_q_loss,
        converged,
        iterations,
        max_mismatch,
    })
}

pub fn calculate_power(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    n: usize,
) -> (Vec<f64>, Vec<f64>) {
    let mut p = vec![0.0; n];
    let mut q = vec![0.0; n];

    let v: Vec<Complex64> = v_mag
        .iter()
        .zip(v_ang.iter())
        .map(|(&m, &a)| Complex64::from_polar(m, a))
        .collect();

    for (yij, (i, j)) in ybus.iter() {
        let s = v[i] * (yij * v[j]).conj();
        p[i] += s.re;
        q[i] += s.im;
    }

    (p, q)
}

fn compute_branch_flows(network: &PowerNetwork, v_mag: &[f64], v_ang: &[f64]) -> Vec<BranchFlow> {
    use num_complex::Complex64;

    let v: Vec<Complex64> = v_mag
        .iter()
        .zip(v_ang.iter())
        .map(|(&m, &a)| Complex64::from_polar(m, a))
        .collect();

    let mut flows = Vec::with_capacity(network.branches.len());

    for (idx, branch) in network.branches.iter().enumerate() {
        let Ok(i) = network.bus_index(branch.from_bus) else {
            continue;
        };
        let Ok(j) = network.bus_index(branch.to_bus) else {
            continue;
        };

        if !branch.status {
            flows.push(BranchFlow {
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

        let ys = Complex64::new(branch.r, branch.x).inv();
        let tap = branch.tap_complex();
        let tap_conj = tap.conj();
        let tap_mag_sq = tap.norm_sqr();
        let bc = Complex64::new(0.0, branch.b / 2.0);

        // From-bus current injection: I_ij = (V_i/tap - V_j)*ys
        // S_ij (from) = V_i * conj(I_ij_from)
        // I_from = V_i * ys/|tap|^2 - V_j * ys/tap*  + V_i * jb/2
        let i_from = v[i] * (ys / tap_mag_sq + bc) + v[j] * (-ys / tap_conj);
        let s_from = v[i] * i_from.conj();

        // To-bus current: I_to = -V_i*ys/tap + V_j*(ys + jb/2)
        let i_to = v[i] * (-ys / tap) + v[j] * (ys + bc);
        let s_to = v[j] * i_to.conj();

        let p_from = s_from.re * network.base_mva;
        let q_from = s_from.im * network.base_mva;
        let p_to = s_to.re * network.base_mva;
        let q_to = s_to.im * network.base_mva;

        let s_from_mva = (p_from * p_from + q_from * q_from).sqrt();
        let loading = if branch.rate_a > 0.0 {
            s_from_mva / branch.rate_a * 100.0
        } else {
            0.0
        };

        flows.push(BranchFlow {
            branch_index: idx,
            from_bus: branch.from_bus,
            to_bus: branch.to_bus,
            p_from_mw: p_from,
            q_from_mvar: q_from,
            p_to_mw: p_to,
            q_to_mvar: q_to,
            p_loss_mw: p_from + p_to,
            q_loss_mvar: q_from + q_to,
            loading_pct: loading,
        });
    }

    flows
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::Bus;
    use crate::network::topology::Generator;

    fn make_2bus_net() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b.pd = crate::units::Power(50.0);
            b.qd = crate::units::ReactivePower(20.0);
            b
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.02,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
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
        net
    }

    #[test]
    fn test_2bus_powerflow() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };

        let result = NewtonRaphsonSolver.solve(&net, &config).unwrap();
        assert!(
            result.converged,
            "Did not converge. iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        assert!((result.voltage_magnitude[0] - 1.0).abs() < 1e-6);
        assert!(result.voltage_magnitude[1] < 1.0);
        assert!(result.voltage_magnitude[1] > 0.9);
    }

    #[test]
    fn test_2bus_branch_flows() {
        let net = make_2bus_net();
        let config = PowerFlowConfig::default();
        let result = NewtonRaphsonSolver.solve(&net, &config).unwrap();
        assert!(result.converged);
        assert_eq!(result.branch_flows.len(), 1);
        // Power flowing from bus 1 to bus 2 should be ~50 MW
        let flow = &result.branch_flows[0];
        assert!(flow.p_from_mw > 48.0 && flow.p_from_mw < 55.0);
    }
}
