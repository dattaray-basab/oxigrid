//! Y-bus (nodal admittance matrix) construction.
//!
//! Assembles the sparse complex admittance matrix from branch π-models and
//! bus shunt data.  The result is a CSC `CsMat<Complex64>` suitable for
//! power flow and state estimation solvers.
use crate::error::Result;
use crate::network::topology::PowerNetwork;
use num_complex::Complex64;
use sprs::{CsMat, TriMat};

/// Build the nodal admittance matrix (Y-bus) for `network`.
///
/// Each in-service branch contributes four entries using the π-model:
///
/// ```text
///   Y_ii += ys / |tap|² + jb/2
///   Y_jj += ys + jb/2
///   Y_ij += −ys / tap*
///   Y_ji += −ys / tap
/// ```
///
/// where `ys = 1/(r + jx)` and `tap = t·e^{jφ}` (1.0 for plain lines).
/// Bus shunt elements (`gs + jbs`) are added to the diagonal and scaled by
/// `1/base_mva`.
///
/// Returns a CSC sparse matrix of size `n × n`.
///
/// # Examples
///
/// ```rust
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use oxigrid::network::topology::PowerNetwork;
/// use oxigrid::network::bus::{Bus, BusType};
/// use oxigrid::network::branch::Branch;
/// use oxigrid::network::admittance::build_y_bus;
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
/// let y_bus = build_y_bus(&net)?;
/// assert_eq!(y_bus.rows(), 2);
/// assert_eq!(y_bus.cols(), 2);
/// // Diagonal entries must be non-zero (series + shunt admittance)
/// assert!(y_bus.nnz() > 0);
/// # Ok(()) }
/// ```
pub fn build_y_bus(network: &PowerNetwork) -> Result<CsMat<Complex64>> {
    let n = network.bus_count();
    let mut ybus = TriMat::new((n, n));

    for branch in &network.branches {
        if !branch.status {
            continue;
        }

        let i = network.bus_index(branch.from_bus)?;
        let j = network.bus_index(branch.to_bus)?;

        let ys = Complex64::new(branch.r, branch.x).inv(); // series admittance
        let bc = Complex64::new(0.0, branch.b / 2.0); // line charging

        let tap = branch.tap_complex();
        let tap_conj = tap.conj();
        let tap_mag_sq = tap.norm_sqr();

        // Pi-model with tap
        // Y_ii += ys / |tap|^2 + bc
        // Y_jj += ys + bc
        // Y_ij += -ys / tap*
        // Y_ji += -ys / tap

        let yii = ys / tap_mag_sq + bc;
        let yjj = ys + bc;
        let yij = -ys / tap_conj;
        let yji = -ys / tap;

        ybus.add_triplet(i, i, yii);
        ybus.add_triplet(j, j, yjj);
        ybus.add_triplet(i, j, yij);
        ybus.add_triplet(j, i, yji);
    }

    // Add shunt elements from bus data
    for (i, bus) in network.buses.iter().enumerate() {
        if bus.gs != 0.0 || bus.bs != 0.0 {
            let y_shunt = Complex64::new(bus.gs, bus.bs) / network.base_mva;
            ybus.add_triplet(i, i, y_shunt);
        }
    }

    Ok(ybus.to_csc())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};

    #[test]
    fn test_simple_2bus_ybus() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
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

        let ybus = build_y_bus(&net).unwrap();
        assert_eq!(ybus.rows(), 2);
        assert_eq!(ybus.cols(), 2);

        // Y_12 should be -ys = -(r - jx) / (r^2 + x^2)
        let ys = Complex64::new(0.01, 0.1).inv();
        let y12 = *ybus.get(0, 1).unwrap_or(&Complex64::new(0.0, 0.0));
        assert!((y12 + ys).norm() < 1e-10);
    }

    #[test]
    fn test_ybus_line_admittance() {
        // Single branch r=0.05, x=0.3, b=0.0; no line charging.
        // Off-diagonal Y_ij should equal -ys exactly.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.05,
            x: 0.3,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let ys = Complex64::new(0.05, 0.3).inv();
        let y01 = ybus.get(0, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        // Y_ij = -ys (no tap, no charging)
        assert!((y01 + ys).norm() < 1e-10);
    }

    #[test]
    fn test_ybus_diagonal_includes_charging() {
        // Single branch r=0.01, x=0.1, b=0.02.
        // Diagonal Y_ii = ys + bc where bc = j*(b/2) = j*0.01.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
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

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let ys = Complex64::new(0.01, 0.1).inv();
        let bc = Complex64::new(0.0, 0.02 / 2.0); // j * 0.01
        let expected_diag = ys + bc;
        let y00 = ybus.get(0, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            (y00.re - expected_diag.re).abs() < 1e-10,
            "diagonal real part mismatch: got {}, expected {}",
            y00.re,
            expected_diag.re
        );
        assert!(
            (y00.im - expected_diag.im).abs() < 1e-10,
            "diagonal imag part mismatch: got {}, expected {}",
            y00.im,
            expected_diag.im
        );
    }

    #[test]
    fn test_ybus_off_diagonal_is_negative_series() {
        // Same branch r=0.01, x=0.1, b=0.02; tap=0.0 (plain line → tap=1).
        // Off-diagonal Y_ij = -ys (charging does NOT appear off-diagonal).
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
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

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let ys = Complex64::new(0.01, 0.1).inv();
        let y01 = ybus.get(0, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            (y01.re - (-ys.re)).abs() < 1e-10,
            "off-diagonal real mismatch: got {}, expected {}",
            y01.re,
            -ys.re
        );
        assert!(
            (y01.im - (-ys.im)).abs() < 1e-10,
            "off-diagonal imag mismatch: got {}, expected {}",
            y01.im,
            -ys.im
        );
    }

    #[test]
    fn test_ybus_symmetry_plain_line() {
        // For a plain line (tap=0.0 → effective 1.0), Y_ij == Y_ji.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.02,
            x: 0.15,
            b: 0.04,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let y01 = ybus.get(0, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y10 = ybus.get(1, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            (y01 - y10).norm() < 1e-10,
            "symmetry violated: Y_01={:?}, Y_10={:?}",
            y01,
            y10
        );
    }

    #[test]
    fn test_ybus_open_circuit_branch_zero() {
        // A branch with status=false contributes nothing; nnz should be 0.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
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
            status: false,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        assert_eq!(
            ybus.nnz(),
            0,
            "open-circuit branch should contribute zero entries, got nnz={}",
            ybus.nnz()
        );
    }

    #[test]
    fn test_ybus_three_bus_diagonal_sum() {
        // 3-bus network: bus 0 connected to both bus 1 and bus 2.
        // Y_00 accumulates two branch contributions; Y_11 and Y_22 only one each.
        // |Y_00| must be greater than |Y_11| and |Y_22|.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.buses.push(Bus::new(3, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.02,
            x: 0.2,
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
            r: 0.03,
            x: 0.3,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let y00 = ybus.get(0, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y11 = ybus.get(1, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y22 = ybus.get(2, 2).copied().unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            y00.norm() > y11.norm(),
            "|Y_00|={} should exceed |Y_11|={}",
            y00.norm(),
            y11.norm()
        );
        assert!(
            y00.norm() > y22.norm(),
            "|Y_00|={} should exceed |Y_22|={}",
            y00.norm(),
            y22.norm()
        );
    }

    #[test]
    fn test_ybus_bus_shunt_element() {
        // Bus 0 has gs=0.01, bs=0.05; these should be added to Y[0,0] as gs+j*bs scaled by 1/base_mva.
        let mut net = PowerNetwork::new(100.0);
        let mut bus0 = Bus::new(1, BusType::Slack);
        bus0.gs = 0.01;
        bus0.bs = 0.05;
        net.buses.push(bus0);
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.02,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let ys = Complex64::new(0.02, 0.2).inv();
        // Without shunt, Y[0,0] = ys. With shunt it gets extra (gs+j*bs)/base_mva.
        let shunt = Complex64::new(0.01, 0.05) / 100.0;
        let expected_diag = ys + shunt;
        let y00 = ybus.get(0, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            (y00.re - expected_diag.re).abs() < 1e-12,
            "shunt real part mismatch: got {}, expected {}",
            y00.re,
            expected_diag.re
        );
        assert!(
            (y00.im - expected_diag.im).abs() < 1e-12,
            "shunt imag part mismatch: got {}, expected {}",
            y00.im,
            expected_diag.im
        );
    }

    #[test]
    fn test_ybus_transformer_asymmetric() {
        // For a transformer with tap != 1 (and tap != 0), the π-model produces
        // Y_ij = -ys/tap*  and  Y_ji = -ys/tap  which differ when tap is real != 1.
        // Also diagonal entries differ: Y_ii = ys/|tap|^2 + bc  vs  Y_jj = ys + bc.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        // tap = 1.05 (real, no phase shift) → effective tap = 1.05 + j*0
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.05,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let y01 = ybus.get(0, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y10 = ybus.get(1, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        // For real tap t > 0: Y_ij = -ys/t  and  Y_ji = -ys/t  (conj of real = itself)
        // so Y_ij == Y_ji in the real-tap case — but the diagonal entries differ.
        let y00 = ybus.get(0, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y11 = ybus.get(1, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        // Y_00 = ys / tap^2 (b=0 so no charging); Y_11 = ys
        // Since tap = 1.05, Y_00.re < Y_11.re (dividing by 1.05^2 > 1)
        assert!(
            y00.re < y11.re,
            "transformer diagonal asymmetry: Y_00.re={} should be < Y_11.re={}",
            y00.re,
            y11.re
        );
        // Off-diagonal magnitudes must equal (symmetric off-diagonal for real tap)
        assert!(
            (y01.norm() - y10.norm()).abs() < 1e-12,
            "off-diagonal magnitude mismatch for real tap: |Y_01|={}, |Y_10|={}",
            y01.norm(),
            y10.norm()
        );
    }

    #[test]
    fn test_ybus_parallel_branches_additive() {
        // Two parallel branches between bus 0 and bus 1 with the same parameters.
        // The off-diagonal |Y_01| should be twice that of a single branch.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        let branch = Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.03,
            x: 0.3,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        };
        net.branches.push(branch.clone());
        net.branches.push(branch);

        // Single-branch reference
        let mut net_single = PowerNetwork::new(100.0);
        net_single.buses.push(Bus::new(1, BusType::Slack));
        net_single.buses.push(Bus::new(2, BusType::PQ));
        net_single.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.03,
            x: 0.3,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus_double = build_y_bus(&net).expect("build_y_bus parallel failed");
        let ybus_single = build_y_bus(&net_single).expect("build_y_bus single failed");

        let y01_double = ybus_double
            .get(0, 1)
            .copied()
            .unwrap_or(Complex64::new(0.0, 0.0));
        let y01_single = ybus_single
            .get(0, 1)
            .copied()
            .unwrap_or(Complex64::new(0.0, 0.0));
        assert!(
            (y01_double.norm() - 2.0 * y01_single.norm()).abs() < 1e-12,
            "parallel branches: |Y_01| double={} should be 2x single={}",
            y01_double.norm(),
            y01_single.norm()
        );
    }

    #[test]
    fn test_ybus_kcl_row_sum_no_shunt() {
        // For a plain line with no shunt elements, KCL requires that each row of
        // Y-bus sums to approximately 0 (only line-charging b/2 contributes; with
        // b=0 the sum is exactly 0).
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.04,
            x: 0.4,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        for row in 0..2 {
            let mut row_sum = Complex64::new(0.0, 0.0);
            for col in 0..2 {
                row_sum += ybus
                    .get(row, col)
                    .copied()
                    .unwrap_or(Complex64::new(0.0, 0.0));
            }
            assert!(
                row_sum.norm() < 1e-12,
                "KCL row sum for row {} = {:?} (expected ~0)",
                row,
                row_sum
            );
        }
    }

    #[test]
    fn test_ybus_phase_shift_complex_off_diagonal() {
        // A branch with shift != 0 produces complex tap = t * e^{j*phi}.
        // The off-diagonal Y_ij = -ys/tap*  and  Y_ji = -ys/tap differ in phase.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        let shift_deg: f64 = 5.0;
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: shift_deg,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus failed");
        let y01 = ybus.get(0, 1).copied().unwrap_or(Complex64::new(0.0, 0.0));
        let y10 = ybus.get(1, 0).copied().unwrap_or(Complex64::new(0.0, 0.0));
        // With non-zero phase shift, |Y_ij| == |Y_ji| (both = |ys|/|tap|), but the
        // complex values themselves differ because dividing by tap* vs tap applies
        // opposite rotations of ±φ.
        assert!(
            (y01.norm() - y10.norm()).abs() < 1e-12,
            "phase-shift off-diagonals should have equal magnitude: |Y_01|={}, |Y_10|={}",
            y01.norm(),
            y10.norm()
        );
        // The two entries must NOT be equal (phase shift breaks symmetry).
        assert!(
            (y01 - y10).norm() > 1e-10,
            "phase-shift off-diagonals should differ: Y_01={:?}, Y_10={:?}",
            y01,
            y10
        );
    }

    #[test]
    fn test_ybus_4bus_ring_nnz() {
        // 4-bus ring: 0-1, 1-2, 2-3, 3-0.
        // Expected nnz = 4 diagonal + 8 off-diagonal = 12.
        let mut net = PowerNetwork::new(100.0);
        for id in 1..=4 {
            net.buses.push(Bus::new(id, BusType::PQ));
        }
        let pairs = [(1, 2), (2, 3), (3, 4), (4, 1)];
        for (f, t) in pairs {
            net.branches.push(Branch {
                from_bus: f,
                to_bus: t,
                r: 0.02,
                x: 0.2,
                b: 0.0,
                rate_a: 100.0,
                rate_b: 100.0,
                rate_c: 100.0,
                tap: 0.0,
                shift: 0.0,
                status: true,
            });
        }

        let ybus = build_y_bus(&net).expect("build_y_bus 4-bus ring failed");
        assert_eq!(
            ybus.nnz(),
            12,
            "4-bus ring should have 12 non-zero entries, got {}",
            ybus.nnz()
        );
    }

    #[test]
    fn test_ybus_two_isolated_subgraphs() {
        // Buses 0-1 connected; buses 2-3 connected; no cross-graph edge.
        // Y[0,2] and Y[0,3] and Y[1,2] and Y[1,3] must all be zero.
        let mut net = PowerNetwork::new(100.0);
        for id in 1..=4 {
            net.buses.push(Bus::new(id, BusType::PQ));
        }
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
        net.branches.push(Branch {
            from_bus: 3,
            to_bus: 4,
            r: 0.02,
            x: 0.2,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        let ybus = build_y_bus(&net).expect("build_y_bus two-subgraphs failed");
        // Cross entries between the two subgraphs must be zero (absent in sparse repr)
        for &(r, c) in &[
            (0usize, 2usize),
            (0, 3),
            (1, 2),
            (1, 3),
            (2, 0),
            (3, 0),
            (2, 1),
            (3, 1),
        ] {
            let val = ybus.get(r, c).copied().unwrap_or(Complex64::new(0.0, 0.0));
            assert!(
                val.norm() < 1e-15,
                "cross-subgraph entry Y[{},{}] should be 0, got {:?}",
                r,
                c,
                val
            );
        }
    }

    #[test]
    fn test_ybus_resistive_vs_reactive_diagonal_imag() {
        // A highly resistive line (large r, small x) should have a smaller
        // imaginary diagonal entry than a highly reactive line (small r, large x).
        let params_resistive = (0.4_f64, 0.01_f64); // r >> x
        let params_reactive = (0.01_f64, 0.4_f64); // x >> r

        let make_net = |r: f64, x: f64| {
            let mut net = PowerNetwork::new(100.0);
            net.buses.push(Bus::new(1, BusType::Slack));
            net.buses.push(Bus::new(2, BusType::PQ));
            net.branches.push(Branch {
                from_bus: 1,
                to_bus: 2,
                r,
                x,
                b: 0.0,
                rate_a: 100.0,
                rate_b: 100.0,
                rate_c: 100.0,
                tap: 0.0,
                shift: 0.0,
                status: true,
            });
            net
        };

        let net_r = make_net(params_resistive.0, params_resistive.1);
        let net_x = make_net(params_reactive.0, params_reactive.1);

        let ybus_r = build_y_bus(&net_r).expect("resistive ybus failed");
        let ybus_x = build_y_bus(&net_x).expect("reactive ybus failed");

        // ys = 1/(r+jx); for resistive: Im(ys) is small; for reactive: Im(ys) is large (negative)
        let y00_r = ybus_r
            .get(0, 0)
            .copied()
            .unwrap_or(Complex64::new(0.0, 0.0));
        let y00_x = ybus_x
            .get(0, 0)
            .copied()
            .unwrap_or(Complex64::new(0.0, 0.0));

        // Imaginary part of diagonal = -x/(r^2+x^2).  More reactive → larger |Im|.
        assert!(
            y00_x.im.abs() > y00_r.im.abs(),
            "reactive line diagonal |Im| {} should exceed resistive line diagonal |Im| {}",
            y00_x.im.abs(),
            y00_r.im.abs()
        );
    }
}
