/// Fast Decoupled Load Flow (FDLF) — Stott & Alsac 1974.
///
/// Uses constant susceptance matrices B' and B'' to decouple the P-θ
/// and Q-V sub-problems, solving them alternately until convergence.
///
/// # Mathematical Background
///
/// The FDLF approximation replaces the full Newton-Raphson Jacobian with
/// two constant matrices:
///
/// - B' (size npvpq × npvpq): for the P equation  B'·Δθ = ΔP/V
/// - B'' (size npq × npq):    for the Q equation  B''·ΔV = ΔQ/V
///
/// B' elements (omitting resistance and line-charging):
///   B'_ij = -1/x_ij,  B'_ii = Σ 1/x_ij
///
/// B'' elements (omitting resistance, line-charging, and transformer taps):
///   B''_ij = -1/x_ij,  B''_ii = Σ 1/x_ij
use crate::error::{OxiGridError, Result};
use crate::network::bus::BusType;
use crate::network::PowerNetwork;
use crate::powerflow::newton_raphson::calculate_power;
use crate::powerflow::result::BranchFlow;
use crate::powerflow::{PowerFlowConfig, PowerFlowResult, PowerFlowSolver};
use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;

pub struct FastDecoupledSolver;

impl PowerFlowSolver for FastDecoupledSolver {
    fn solve(&self, network: &PowerNetwork, config: &PowerFlowConfig) -> Result<PowerFlowResult> {
        network.validate()?;

        let n = network.bus_count();
        let ybus = network.admittance_matrix()?;

        let mut pq_indices = Vec::new();
        let mut pv_indices = Vec::new();

        for (i, bus) in network.buses.iter().enumerate() {
            match bus.bus_type {
                BusType::PQ => pq_indices.push(i),
                BusType::PV => pv_indices.push(i),
                BusType::Slack => {}
            }
        }

        let mut pvpq_indices: Vec<usize> = pv_indices.clone();
        pvpq_indices.extend_from_slice(&pq_indices);
        pvpq_indices.sort();

        let npvpq = pvpq_indices.len();
        let npq = pq_indices.len();

        // Build B' (P-θ matrix, pvpq × pvpq)
        let b_prime = build_b_prime(network, &pvpq_indices, n)?;
        // Build B'' (Q-V matrix, pq × pq)
        let b_double_prime = build_b_double_prime(network, &pq_indices, n)?;

        // Factorize once — constant for all iterations
        let lu_prime = b_prime.lu();
        let lu_dbl = b_double_prime.lu();

        let mut v_mag: Vec<f64> = network.buses.iter().map(|b| b.vm).collect();
        let mut v_ang: Vec<f64> = network.buses.iter().map(|b| b.va).collect();

        let (p_sched, q_sched) = network.net_injection();

        let mut converged = false;
        let mut iterations = 0;
        let mut max_mismatch = f64::MAX;

        for iter in 0..config.max_iter {
            let (p_calc, q_calc) = calculate_power(&ybus, &v_mag, &v_ang, n);

            // --- P iteration ---
            let dp: Vec<f64> = pvpq_indices
                .iter()
                .map(|&i| (p_sched[i] - p_calc[i]) / v_mag[i])
                .collect();
            let dp_rhs = DVector::from_vec(dp);
            let dtheta = lu_prime
                .solve(&dp_rhs)
                .ok_or_else(|| OxiGridError::LinearAlgebra("B' singular".to_string()))?;
            for (k, &i) in pvpq_indices.iter().enumerate() {
                v_ang[i] += dtheta[k];
            }

            // Re-calculate after P correction
            let (p_calc2, q_calc2) = calculate_power(&ybus, &v_mag, &v_ang, n);

            // --- Q iteration ---
            if npq > 0 {
                let dq: Vec<f64> = pq_indices
                    .iter()
                    .map(|&i| (q_sched[i] - q_calc2[i]) / v_mag[i])
                    .collect();
                let dq_rhs = DVector::from_vec(dq);
                let dv = lu_dbl
                    .solve(&dq_rhs)
                    .ok_or_else(|| OxiGridError::LinearAlgebra("B'' singular".to_string()))?;
                for (k, &i) in pq_indices.iter().enumerate() {
                    v_mag[i] += dv[k];
                }
            }

            // Check convergence (max of |ΔP| and |ΔQ| mismatches)
            let (p_fin, q_fin) = calculate_power(&ybus, &v_mag, &v_ang, n);
            let max_dp = pvpq_indices
                .iter()
                .map(|&i| (p_sched[i] - p_fin[i]).abs())
                .fold(0.0_f64, f64::max);
            let max_dq = pq_indices
                .iter()
                .map(|&i| (q_sched[i] - q_fin[i]).abs())
                .fold(0.0_f64, f64::max);
            max_mismatch = max_dp.max(max_dq);

            iterations = iter + 1;

            if max_mismatch < config.tolerance {
                converged = true;
                iterations = iter + 1;
                break;
            }

            // suppress unused warnings
            let _ = (p_calc, q_calc, p_calc2, q_calc2, npvpq, npq);
        }

        let (p_calc, q_calc) = calculate_power(&ybus, &v_mag, &v_ang, n);
        let p_inj: Vec<f64> = p_calc.iter().map(|&p| p * network.base_mva).collect();
        let q_inj: Vec<f64> = q_calc.iter().map(|&q| q * network.base_mva).collect();

        let total_p_loss = p_inj.iter().sum::<f64>();
        let total_q_loss = q_inj.iter().sum::<f64>();
        let branch_flows = compute_fdlf_branch_flows(network, &v_mag, &v_ang);

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
}

fn build_b_prime(network: &PowerNetwork, pvpq_indices: &[usize], n: usize) -> Result<DMatrix<f64>> {
    let npvpq = pvpq_indices.len();
    // Map bus index -> pvpq row/col index
    let mut idx_map = vec![usize::MAX; n];
    for (k, &i) in pvpq_indices.iter().enumerate() {
        idx_map[i] = k;
    }

    let mut b = DMatrix::<f64>::zeros(npvpq, npvpq);

    for branch in &network.branches {
        if !branch.status || branch.x.abs() < 1e-15 {
            continue;
        }
        let i = network.bus_index(branch.from_bus)?;
        let j = network.bus_index(branch.to_bus)?;
        let tap = branch.effective_tap();

        // B' uses series susceptance / tap (ignore resistance, line charging)
        let bij = 1.0 / (branch.x * tap);

        if idx_map[i] != usize::MAX && idx_map[j] != usize::MAX {
            let ri = idx_map[i];
            let rj = idx_map[j];
            b[(ri, rj)] -= bij;
            b[(rj, ri)] -= bij;
            b[(ri, ri)] += bij;
            b[(rj, rj)] += bij;
        } else if idx_map[i] != usize::MAX {
            b[(idx_map[i], idx_map[i])] += bij;
        } else if idx_map[j] != usize::MAX {
            b[(idx_map[j], idx_map[j])] += bij;
        }
    }

    Ok(b)
}

fn build_b_double_prime(
    network: &PowerNetwork,
    pq_indices: &[usize],
    n: usize,
) -> Result<DMatrix<f64>> {
    let npq = pq_indices.len();
    let mut idx_map = vec![usize::MAX; n];
    for (k, &i) in pq_indices.iter().enumerate() {
        idx_map[i] = k;
    }

    let mut b = DMatrix::<f64>::zeros(npq, npq);

    for branch in &network.branches {
        if !branch.status || branch.x.abs() < 1e-15 {
            continue;
        }
        let i = network.bus_index(branch.from_bus)?;
        let j = network.bus_index(branch.to_bus)?;

        // B'' uses series susceptance only (no tap, no r, no charging)
        let bij = 1.0 / branch.x;

        if idx_map[i] != usize::MAX && idx_map[j] != usize::MAX {
            let ri = idx_map[i];
            let rj = idx_map[j];
            b[(ri, rj)] -= bij;
            b[(rj, ri)] -= bij;
            b[(ri, ri)] += bij;
            b[(rj, rj)] += bij;
        } else if idx_map[i] != usize::MAX {
            b[(idx_map[i], idx_map[i])] += bij;
        } else if idx_map[j] != usize::MAX {
            b[(idx_map[j], idx_map[j])] += bij;
        }
    }

    Ok(b)
}

fn compute_fdlf_branch_flows(
    network: &PowerNetwork,
    v_mag: &[f64],
    v_ang: &[f64],
) -> Vec<BranchFlow> {
    let v: Vec<Complex64> = v_mag
        .iter()
        .zip(v_ang.iter())
        .map(|(&m, &a)| Complex64::from_polar(m, a))
        .collect();

    let mut flows = Vec::with_capacity(network.branches.len());

    for (br_idx, branch) in network.branches.iter().enumerate() {
        let Ok(i) = network.bus_index(branch.from_bus) else {
            continue;
        };
        let Ok(j) = network.bus_index(branch.to_bus) else {
            continue;
        };

        if !branch.status {
            flows.push(BranchFlow {
                branch_index: br_idx,
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

        let i_from = v[i] * (ys / tap_mag_sq + bc) + v[j] * (-ys / tap_conj);
        let s_from = v[i] * i_from.conj();
        let i_to = v[i] * (-ys / tap) + v[j] * (ys + bc);
        let s_to = v[j] * i_to.conj();

        let p_from = s_from.re * network.base_mva;
        let q_from = s_from.im * network.base_mva;
        let p_to = s_to.re * network.base_mva;
        let q_to = s_to.im * network.base_mva;

        let s_apparent_from = (p_from * p_from + q_from * q_from).sqrt();
        let loading = if branch.rate_a > 0.0 {
            s_apparent_from / branch.rate_a * 100.0
        } else {
            0.0
        };

        flows.push(BranchFlow {
            branch_index: br_idx,
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
    use crate::network::PowerNetwork;
    use crate::powerflow::PowerFlowMethod;

    #[test]
    fn test_fdlf_ieee14() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let network = PowerNetwork::from_matpower(path).unwrap();
        let config = PowerFlowConfig {
            method: PowerFlowMethod::FastDecoupled,
            max_iter: 50,
            tolerance: 1e-6,
            enforce_q_limits: false,
        };
        let result = FastDecoupledSolver.solve(&network, &config).unwrap();
        assert!(
            result.converged,
            "FDLF did not converge: iters={} mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        // Slack bus voltage fixed
        assert!((result.voltage_magnitude[0] - 1.06).abs() < 1e-6);
    }
}
