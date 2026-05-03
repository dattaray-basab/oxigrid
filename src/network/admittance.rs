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
}
