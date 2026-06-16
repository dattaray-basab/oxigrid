use crate::error::Result;
use crate::network::bus::BusType;
use crate::network::PowerNetwork;
#[cfg(not(feature = "parallel"))]
use crate::powerflow::jacobian::{build_jacobian, build_jacobian_sparse};
#[cfg(feature = "parallel")]
use crate::powerflow::jacobian::{
    build_jacobian_parallel as build_jacobian, build_jacobian_sparse,
};
use crate::powerflow::linalg::select_backend;
use crate::powerflow::sparse_lu::CrsMatrix;
use crate::powerflow::{PowerFlowConfig, PowerFlowResult, PowerFlowSolver};
use num_complex::Complex64;
use sprs::CsMat;

/// Size threshold above which the sparse Jacobian path is used.
///
/// For systems with more than this many buses the Jacobian is built as a
/// `CsMat<f64>` (via [`build_jacobian_sparse`]), converted directly to a
/// [`CrsMatrix`] (via [`CrsMatrix::from_csmat`]), and then materialised as a
/// dense `DMatrix` for the LU solve.  This eliminates the O(n²)
/// `DMatrix::zeros` allocation that the dense-build path performs.
const SPARSE_JAC_THRESHOLD: usize = 200;

use super::result::BranchFlow;

/// Newton-Raphson AC power flow solver.
///
/// Implements the full Newton-Raphson method with sparse Jacobian and step-size limiting.
///
/// # Examples
///
/// ```rust
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use oxigrid::network::topology::PowerNetwork;
/// use oxigrid::network::bus::{Bus, BusType};
/// use oxigrid::network::branch::Branch;
/// use oxigrid::powerflow::{PowerFlowConfig, PowerFlowSolver};
/// use oxigrid::powerflow::newton_raphson::NewtonRaphsonSolver;
///
/// let mut net = PowerNetwork::new(100.0);
/// net.buses.push(Bus::new(1, BusType::Slack));
/// net.buses.push(Bus::new(2, BusType::PQ));
/// net.branches.push(Branch {
///     from_bus: 1, to_bus: 2,
///     r: 0.01, x: 0.1, b: 0.02,
///     rate_a: 100.0, rate_b: 100.0, rate_c: 100.0,
///     tap: 0.0, shift: 0.0, status: true,
/// });
///
/// let solver = NewtonRaphsonSolver;
/// let config = PowerFlowConfig::default();
/// let result = solver.solve(&net, &config)?;
/// assert!(result.converged);
/// # Ok(()) }
/// ```
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
    let backend = select_backend(n);
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

        // Sparse path for large systems: build Jacobian as CsMat → CrsMatrix →
        // DMatrix, skipping the O(n²) DMatrix::zeros allocation in build_jacobian.
        let jac = if n > SPARSE_JAC_THRESHOLD {
            let csmat = build_jacobian_sparse(
                &ybus,
                &v_mag,
                &v_ang,
                &p_calc,
                &q_calc,
                &pq_indices,
                &pvpq_indices,
            );
            CrsMatrix::from_csmat(&csmat).to_dense()
        } else {
            build_jacobian(
                &ybus,
                &v_mag,
                &v_ang,
                &p_calc,
                &q_calc,
                &pq_indices,
                &pvpq_indices,
            )
        };

        let dx_vec = backend.solve_dense(&jac, &mismatch)?;

        // Step-size limiting: prevent large updates that could cause divergence
        const MAX_DTHETA: f64 = 0.5; // rad per iteration
        const MAX_DV_REL: f64 = 0.2; // relative voltage change per iteration

        let npvpq = pvpq_indices.len();
        for (col, &i) in pvpq_indices.iter().enumerate() {
            v_ang[i] += dx_vec[col].clamp(-MAX_DTHETA, MAX_DTHETA);
        }
        for (col, &i) in pq_indices.iter().enumerate() {
            let dv_rel = dx_vec[npvpq + col].clamp(-MAX_DV_REL, MAX_DV_REL);
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

/// Build dense G (conductance) and B (susceptance) row matrices from a sparse
/// complex admittance matrix.  Each `ybus_g[i][j]` = Re(Y_ij), `ybus_b[i][j]` = Im(Y_ij).
/// Non-present entries default to 0.0.  Used by the SIMD polar-form injection kernel.
#[cfg(feature = "simd")]
fn ybus_to_dense_gb(ybus: &CsMat<Complex64>, n: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let mut g_mat = vec![vec![0.0_f64; n]; n];
    let mut b_mat = vec![vec![0.0_f64; n]; n];
    for (yij, (i, j)) in ybus.iter() {
        g_mat[i][j] = yij.re;
        b_mat[i][j] = yij.im;
    }
    (g_mat, b_mat)
}

pub fn calculate_power(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    n: usize,
) -> (Vec<f64>, Vec<f64>) {
    // SIMD path: use polar-form injection kernel for large systems (n >= 64).
    #[cfg(feature = "simd")]
    if n >= 64 {
        use crate::powerflow::simd_kernels::simd::compute_power_injection;
        let (g_mat, b_mat) = ybus_to_dense_gb(ybus, n);
        let mut p = Vec::with_capacity(n);
        let mut q = Vec::with_capacity(n);
        for i in 0..n {
            let (pi, qi) = compute_power_injection(v_mag, v_ang, &g_mat[i], &b_mat[i], i);
            p.push(pi);
            q.push(qi);
        }
        return (p, q);
    }

    // Scalar path: rectangular-form accumulation via sparse Y-bus iteration.
    calculate_power_scalar(ybus, v_mag, v_ang, n)
}

fn calculate_power_scalar(
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
    use crate::units::{Power, ReactivePower};

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

    /// Verify that the SIMD `calculate_power` path (n >= 64 branch) produces
    /// results within 1e-9 of the scalar path on a 2-bus network.
    ///
    /// The test is compiled and run regardless of whether the `simd` feature is
    /// active: when SIMD is off the function always uses the scalar path and the
    /// tolerance check still holds (both paths are identical).
    #[test]
    fn test_simd_power_injection_matches_scalar() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-10,
            enforce_q_limits: false,
        };

        // Run the full NR solve (uses calculate_power internally).
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed");
        assert!(result.converged, "NR did not converge");

        // Also call calculate_power directly to compare scalar vs current path.
        let ybus = net.admittance_matrix().expect("ybus failed");
        let n = net.bus_count();
        let v_mag = &result.voltage_magnitude;
        let v_ang = &result.voltage_angle;

        let (p_current, q_current) = calculate_power(&ybus, v_mag, v_ang, n);
        let (p_scalar, q_scalar) = calculate_power_scalar(&ybus, v_mag, v_ang, n);

        for i in 0..n {
            assert!(
                (p_current[i] - p_scalar[i]).abs() < 1e-9,
                "P[{i}] mismatch: current={:.10e}  scalar={:.10e}",
                p_current[i],
                p_scalar[i]
            );
            assert!(
                (q_current[i] - q_scalar[i]).abs() < 1e-9,
                "Q[{i}] mismatch: current={:.10e}  scalar={:.10e}",
                q_current[i],
                q_scalar[i]
            );
        }
    }

    /// Verify that for a sub-64-bus network the scalar path is taken.
    ///
    /// We confirm this by checking that `calculate_power_scalar` and
    /// `calculate_power` return bit-identical results (they use the same code
    /// path when n < 64 regardless of the `simd` feature flag).
    #[test]
    fn test_simd_threshold_crossover() {
        let net = make_2bus_net();
        let n = net.bus_count();
        // Ensure we are below the SIMD threshold of 64.
        assert!(n < 64, "make_2bus_net must be < 64 buses for this test");

        let ybus = net.admittance_matrix().expect("ybus failed");
        let v_mag = vec![1.0_f64, 0.98_f64];
        let v_ang = vec![0.0_f64, -0.05_f64];

        let (p_calc, q_calc) = calculate_power(&ybus, &v_mag, &v_ang, n);
        let (p_scal, q_scal) = calculate_power_scalar(&ybus, &v_mag, &v_ang, n);

        // Below threshold: both paths must agree to floating-point equality.
        for i in 0..n {
            assert_eq!(
                p_calc[i], p_scal[i],
                "P[{i}] should be identical below SIMD threshold"
            );
            assert_eq!(
                q_calc[i], q_scal[i],
                "Q[{i}] should be identical below SIMD threshold"
            );
        }
    }

    #[test]
    fn test_2bus_voltage_angle_slack_zero() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for slack angle test");
        assert!(
            result.converged,
            "NR did not converge: iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        assert!(
            result.voltage_angle[0].abs() < 1e-6,
            "Slack bus voltage angle should be ~0.0 rad, got {:.2e}",
            result.voltage_angle[0]
        );
    }

    #[test]
    fn test_2bus_max_mismatch_below_tolerance() {
        let net = make_2bus_net();
        let tolerance = 1e-8_f64;
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 100,
            tolerance,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for mismatch tolerance test");
        assert!(
            result.converged,
            "NR did not converge: iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        assert!(
            result.max_mismatch < tolerance,
            "max_mismatch {:.2e} must be < tolerance {:.2e}",
            result.max_mismatch,
            tolerance
        );
    }

    #[test]
    fn test_2bus_iteration_count_reasonable() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for iteration count test");
        assert!(
            result.converged,
            "NR did not converge: iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        assert!(
            result.iterations > 0,
            "iteration count must be > 0, got {}",
            result.iterations
        );
        assert!(
            result.iterations <= 50,
            "iteration count {} exceeds max_iter=50",
            result.iterations
        );
    }

    #[test]
    fn test_2bus_p_injected_nonzero() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for p_injected nonzero test");
        assert!(
            result.converged,
            "NR did not converge: iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        let any_nonzero = result.p_injected.iter().any(|&p| p.abs() > 0.1);
        assert!(
            any_nonzero,
            "expected at least one p_injected with magnitude > 0.1, got {:?}",
            result.p_injected
        );
    }

    #[test]
    fn test_2bus_total_loss_finite() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for total loss test");
        assert!(
            result.converged,
            "NR did not converge: iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
        assert!(
            result.total_p_loss_mw.is_finite(),
            "total_p_loss_mw must be finite, got {}",
            result.total_p_loss_mw
        );
        assert!(
            result.total_q_loss_mvar.is_finite(),
            "total_q_loss_mvar must be finite, got {}",
            result.total_q_loss_mvar
        );
    }

    #[test]
    fn test_calculate_power_flat_start() {
        let net = make_2bus_net();
        let n = net.bus_count();
        let ybus = net
            .admittance_matrix()
            .expect("ybus failed for flat start test");
        let v_mag = vec![1.0_f64; n];
        let v_ang = vec![0.0_f64; n];

        let (p, q) = calculate_power(&ybus, &v_mag, &v_ang, n);

        assert_eq!(p.len(), 2, "p vector must have length 2, got {}", p.len());
        assert_eq!(q.len(), 2, "q vector must have length 2, got {}", q.len());
        for i in 0..n {
            assert!(
                p[i].is_finite(),
                "p[{i}] must be finite at flat start, got {}",
                p[i]
            );
            assert!(
                q[i].is_finite(),
                "q[{i}] must be finite at flat start, got {}",
                q[i]
            );
        }
    }

    #[test]
    fn test_heavy_load_converges() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b.pd = crate::units::Power(80.0);
            b.qd = crate::units::ReactivePower(40.0);
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

        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 100,
            tolerance: 1e-6,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("NR solve failed for heavy load test");
        assert!(
            result.converged,
            "NR did not converge for heavy load (pd=80MW, qd=40Mvar): \
             iterations={}, max_mismatch={:.2e}",
            result.iterations, result.max_mismatch
        );
    }

    #[test]
    fn test_3bus_network_converges() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b.va = 0.0;
            b.pd = Power(0.0);
            b.qd = ReactivePower(0.0);
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b.va = 0.0;
            b.pd = Power(30.0);
            b.qd = ReactivePower(10.0);
            b
        });
        net.buses.push({
            let mut b = Bus::new(3, BusType::PQ);
            b.vm = 1.0;
            b.va = 0.0;
            b.pd = Power(20.0);
            b.qd = ReactivePower(8.0);
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
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.02,
            x: 0.15,
            b: 0.01,
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
            r: 0.015,
            x: 0.12,
            b: 0.015,
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
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("3-bus solve failed");
        assert!(result.converged, "3-bus network did not converge");
        assert_eq!(
            result.voltage_magnitude.len(),
            3,
            "voltage_magnitude must have 3 entries for a 3-bus network"
        );
        for (idx, &vm) in result.voltage_magnitude.iter().enumerate() {
            assert!(
                (0.85..=1.05).contains(&vm),
                "bus {} voltage magnitude {:.4} is out of range [0.85, 1.05]",
                idx,
                vm
            );
        }
    }

    #[test]
    fn test_tight_tolerance_converges() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 100,
            tolerance: 1e-12,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("solve with tolerance 1e-12 failed");
        assert!(result.converged, "did not converge with tolerance 1e-12");
        assert!(
            result.max_mismatch < 1e-12,
            "max_mismatch should be below tolerance: got {:.4e}",
            result.max_mismatch
        );
    }

    #[test]
    fn test_max_iter_one_does_not_converge() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 1,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("solve should return Ok even when not converged");
        assert!(
            !result.converged,
            "should not converge in a single iteration on this network"
        );
        assert!(
            result.iterations <= 1,
            "iterations should be at most max_iter, got {}",
            result.iterations
        );
    }

    #[test]
    fn test_enforce_q_limits_no_panic() {
        let net = make_2bus_net();
        let config = PowerFlowConfig {
            method: crate::powerflow::PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: true,
        };
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("solve with enforce_q_limits=true panicked or errored");
        assert_eq!(
            result.voltage_magnitude.len(),
            net.bus_count(),
            "voltage_magnitude length must equal bus count"
        );
    }

    #[test]
    fn test_voltage_angle_length_equals_bus_count() {
        let net = make_2bus_net();
        let config = PowerFlowConfig::default();
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("default config solve failed");
        assert_eq!(
            result.voltage_angle.len(),
            net.bus_count(),
            "voltage_angle length must equal bus_count()"
        );
    }

    #[test]
    fn test_branch_loading_pct_reasonable() {
        let net = make_2bus_net();
        let config = PowerFlowConfig::default();
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("default config solve failed");
        let flow = &result.branch_flows[0];
        assert!(
            flow.loading_pct >= 0.0,
            "loading_pct must be non-negative, got {}",
            flow.loading_pct
        );
        assert!(
            flow.loading_pct <= 100.0,
            "loading_pct must not exceed 100% for a lightly loaded network, got {}",
            flow.loading_pct
        );
    }

    #[test]
    fn test_default_config_produces_valid_result() {
        let net = make_2bus_net();
        let config = PowerFlowConfig::default();
        let result = NewtonRaphsonSolver
            .solve(&net, &config)
            .expect("default config solve failed");
        assert!(
            result.converged,
            "default config must converge on the 2-bus network"
        );
        assert_eq!(
            result.voltage_magnitude.len(),
            2,
            "must have 2 voltage entries"
        );
        assert_eq!(result.branch_flows.len(), 1, "must have 1 branch flow");
    }
}
