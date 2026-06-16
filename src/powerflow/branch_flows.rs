//! Centralized branch power flow and loss computation.
//!
//! Implements the π-model branch flow equations for AC and DC power flow
//! solutions.  The formulas follow the standard π-model (Glover, Sarma &
//! Overbye) with off-nominal tap transformer support:
//!
//! ```text
//! y_series = 1 / (r + jx)
//! tap      = t·e^{jφ}   (t = tap ratio, φ = phase shift)
//!
//! I_from = V_i · (y_s / |tap|²  +  jb/2)  −  V_j · y_s / tap*
//! I_to   = −V_i · y_s / tap               +  V_j · (y_s  +  jb/2)
//!
//! S_from = V_i · conj(I_from)
//! S_to   = V_j · conj(I_to)
//! S_loss = S_from + S_to
//! ```
//!
//! For DC power flow the reactive terms are zero and the lossless
//! approximation holds:
//! ```text
//! P_ij = (θ_i − θ_j) / (x_ij · tap_ij)
//! ```

use crate::network::PowerNetwork;
use crate::powerflow::result::BranchFlow;
use num_complex::Complex64;

/// Compute full AC branch power flows using the π-model.
///
/// # Arguments
/// * `network`      – power network (branches, base MVA, bus index map)
/// * `voltages_pu`  – voltage magnitudes in per-unit, indexed by internal bus order
/// * `angles_rad`   – voltage angles in radians, indexed by internal bus order
///
/// Returns one `BranchFlow` per branch in `network.branches` order.
/// Out-of-service branches get zeroed flows.
pub fn compute_branch_flows(
    network: &PowerNetwork,
    voltages_pu: &[f64],
    angles_rad: &[f64],
) -> Vec<BranchFlow> {
    // Build complex voltage phasor vector
    let v: Vec<Complex64> = voltages_pu
        .iter()
        .zip(angles_rad.iter())
        .map(|(&m, &a)| Complex64::from_polar(m, a))
        .collect();

    let mut flows = Vec::with_capacity(network.branches.len());

    for (branch_index, branch) in network.branches.iter().enumerate() {
        let i = match network.bus_index(branch.from_bus) {
            Ok(idx) => idx,
            Err(_) => continue,
        };
        let j = match network.bus_index(branch.to_bus) {
            Ok(idx) => idx,
            Err(_) => continue,
        };

        if !branch.status {
            flows.push(BranchFlow {
                branch_index,
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

        // Series admittance y_s = 1 / (r + jx)
        let ys = Complex64::new(branch.r, branch.x).inv();
        // Complex tap a = t·e^{jφ}
        let tap = branch.tap_complex();
        let tap_conj = tap.conj();
        let tap_mag_sq = tap.norm_sqr();
        // Half line-charging susceptance as pure imaginary
        let bc = Complex64::new(0.0, branch.b / 2.0);

        // From-bus current:  I_from = V_i·(ys/|tap|² + jb/2) − V_j·ys/tap*
        let i_from = v[i] * (ys / tap_mag_sq + bc) + v[j] * (-ys / tap_conj);
        let s_from = v[i] * i_from.conj();

        // To-bus current:    I_to = −V_i·ys/tap + V_j·(ys + jb/2)
        let i_to = v[i] * (-ys / tap) + v[j] * (ys + bc);
        let s_to = v[j] * i_to.conj();

        let p_from = s_from.re * network.base_mva;
        let q_from = s_from.im * network.base_mva;
        let p_to = s_to.re * network.base_mva;
        let q_to = s_to.im * network.base_mva;

        // Apparent power at the from-end (MVA)
        let s_from_mva = s_from.norm() * network.base_mva;
        let loading_pct = if branch.rate_a > 0.0 {
            s_from_mva / branch.rate_a * 100.0
        } else {
            0.0
        };

        flows.push(BranchFlow {
            branch_index,
            from_bus: branch.from_bus,
            to_bus: branch.to_bus,
            p_from_mw: p_from,
            q_from_mvar: q_from,
            p_to_mw: p_to,
            q_to_mvar: q_to,
            p_loss_mw: p_from + p_to,
            q_loss_mvar: q_from + q_to,
            loading_pct,
        });
    }

    flows
}

/// Compute DC-approximation branch power flows (real power only, lossless).
///
/// Uses the linearised formula:
/// ```text
/// P_ij = (θ_i − θ_j) / (x_ij · tap_ij)   \[p.u.\]
/// ```
/// Reactive power and losses are zero in the DC model.
///
/// # Arguments
/// * `network`    – power network
/// * `angles_rad` – bus voltage angles in radians (indexed by internal bus order)
///
/// Returns one `BranchFlow` per branch in `network.branches` order.
pub fn compute_dc_branch_flows(network: &PowerNetwork, angles_rad: &[f64]) -> Vec<BranchFlow> {
    let mut flows = Vec::with_capacity(network.branches.len());

    for (branch_index, branch) in network.branches.iter().enumerate() {
        let i = match network.bus_index(branch.from_bus) {
            Ok(idx) => idx,
            Err(_) => continue,
        };
        let j = match network.bus_index(branch.to_bus) {
            Ok(idx) => idx,
            Err(_) => continue,
        };

        if !branch.status {
            flows.push(BranchFlow {
                branch_index,
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

        let tap = branch.effective_tap();
        let p_ij = (angles_rad[i] - angles_rad[j]) / (branch.x * tap);
        let p_ij_mw = p_ij * network.base_mva;

        // DC is lossless: power entering from one end exits the other
        let loading_pct = if branch.rate_a > 0.0 {
            p_ij_mw.abs() / branch.rate_a * 100.0
        } else {
            0.0
        };

        flows.push(BranchFlow {
            branch_index,
            from_bus: branch.from_bus,
            to_bus: branch.to_bus,
            p_from_mw: p_ij_mw,
            q_from_mvar: 0.0,
            p_to_mw: -p_ij_mw, // lossless in DC model
            q_to_mvar: 0.0,
            p_loss_mw: 0.0,
            q_loss_mvar: 0.0,
            loading_pct,
        });
    }

    flows
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
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
            b.qd = crate::units::ReactivePower(10.0);
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
    fn test_ac_branch_flows_count() {
        let net = make_2bus_net();
        // Use flat start voltages (1.0 p.u., 0 rad)
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(flows.len(), 1);
        assert_eq!(flows[0].branch_index, 0);
        assert_eq!(flows[0].from_bus, 1);
        assert_eq!(flows[0].to_bus, 2);
    }

    #[test]
    fn test_dc_branch_flows_lossless() {
        let net = make_2bus_net();
        // Simple angle: bus 1 at 0, bus 2 at -0.05 rad
        let angles = vec![0.0_f64, -0.05];
        let flows = compute_dc_branch_flows(&net, &angles);
        assert_eq!(flows.len(), 1);
        let flow = &flows[0];
        // P_12 = (0 − (−0.05)) / 0.1 = 0.5 p.u. = 50 MW
        assert!((flow.p_from_mw - 50.0).abs() < 1e-6);
        // DC is lossless
        assert!((flow.p_loss_mw).abs() < 1e-12);
        assert!((flow.p_from_mw + flow.p_to_mw).abs() < 1e-10);
    }

    #[test]
    fn test_branch_loading_pct_computed() {
        let net = make_2bus_net();
        let angles = vec![0.0_f64, -0.05];
        let flows = compute_dc_branch_flows(&net, &angles);
        // 50 MW on 100 MVA rating = 50 %
        assert!((flows[0].loading_pct - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_open_branch_zeroed() {
        let mut net = make_2bus_net();
        net.branches[0].status = false;
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(flows.len(), 1);
        let f = &flows[0];
        assert_eq!(f.p_from_mw, 0.0);
        assert_eq!(f.loading_pct, 0.0);
    }

    #[test]
    fn test_ac_branch_reactive_flow() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        let f = &flows[0];
        assert!(f.q_from_mvar.is_finite(), "q_from_mvar must be finite");
        assert!(
            f.q_from_mvar != 0.0,
            "q_from_mvar must be non-zero with line charging"
        );
    }

    #[test]
    fn test_dc_branch_reverse_flow() {
        let net = make_2bus_net();
        // Bus 2 leads bus 1 in angle → power flows from bus 2 to bus 1
        let angles = vec![0.0_f64, 0.05];
        let flows = compute_dc_branch_flows(&net, &angles);
        let f = &flows[0];
        assert!(
            f.p_from_mw < 0.0,
            "p_from_mw should be negative (reverse flow)"
        );
        assert!(f.p_to_mw > 0.0, "p_to_mw should be positive (reverse flow)");
        assert_eq!(f.p_loss_mw, 0.0, "DC model must be lossless");
    }

    #[test]
    fn test_ac_loading_nonzero() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64, 0.98];
        let v_ang = vec![0.0_f64, -0.05];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert!(
            flows[0].loading_pct > 0.0,
            "loading_pct must be positive under load"
        );
    }

    #[test]
    fn test_dc_branch_zero_angle_diff_zero_flow() {
        let net = make_2bus_net();
        let angles = vec![0.0_f64, 0.0];
        let flows = compute_dc_branch_flows(&net, &angles);
        let f = &flows[0];
        assert_eq!(f.p_from_mw, 0.0, "zero angle diff must give zero DC flow");
        assert_eq!(
            f.p_to_mw, 0.0,
            "zero angle diff must give zero DC flow at to-end"
        );
    }

    #[test]
    fn test_ac_flow_with_angle_diff() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64, 0.98];
        let v_ang = vec![0.0_f64, -0.05];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        let f = &flows[0];
        assert!(f.p_from_mw > 0.0, "power should flow from bus 1 to bus 2");
    }

    #[test]
    fn test_open_branch_dc_zeroed() {
        let mut net = make_2bus_net();
        net.branches[0].status = false;
        let angles = vec![0.0_f64, -0.05];
        let flows = compute_dc_branch_flows(&net, &angles);
        let f = &flows[0];
        assert_eq!(f.p_from_mw, 0.0, "open branch must have zero DC flow");
        assert_eq!(f.loading_pct, 0.0, "open branch must have zero loading");
    }

    #[test]
    fn test_dc_loading_full_rating() {
        let net = make_2bus_net();
        // P = (θ1 - θ2) / x * base_mva = 0.1 / 0.1 * 100 = 100 MW = rate_a
        let angles = vec![0.1_f64, 0.0];
        let flows = compute_dc_branch_flows(&net, &angles);
        let f = &flows[0];
        assert!(
            (f.loading_pct - 100.0).abs() < 1e-6,
            "loading_pct should be ~100% at full rating, got {}",
            f.loading_pct
        );
    }

    #[test]
    fn test_branch_count_matches_network() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(3, BusType::PQ);
            b.vm = 1.0;
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
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(
            flows.len(),
            2,
            "number of flows must equal number of branches"
        );
        assert_eq!(
            flows[0].branch_index, 0,
            "first flow must have branch_index 0"
        );
        assert_eq!(
            flows[1].branch_index, 1,
            "second flow must have branch_index 1"
        );
    }

    #[test]
    fn test_from_to_bus_fields_match_branch() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(
            flows[0].from_bus, net.branches[0].from_bus,
            "from_bus must match network branch"
        );
        assert_eq!(
            flows[0].to_bus, net.branches[0].to_bus,
            "to_bus must match network branch"
        );
    }

    #[test]
    fn test_ac_flow_magnitudes_finite() {
        let net = make_2bus_net();
        let v_mag = vec![1.02_f64, 0.98_f64];
        let v_ang = vec![0.0_f64, -0.05_f64];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert!(flows[0].p_from_mw.is_finite(), "p_from_mw must be finite");
        assert!(
            flows[0].q_from_mvar.is_finite(),
            "q_from_mvar must be finite"
        );
        assert!(flows[0].p_to_mw.is_finite(), "p_to_mw must be finite");
        assert!(flows[0].q_to_mvar.is_finite(), "q_to_mvar must be finite");
        assert!(
            flows[0].loading_pct.is_finite(),
            "loading_pct must be finite"
        );
    }

    #[test]
    fn test_loading_pct_nonnegative() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64, -0.1_f64];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert!(
            flows[0].loading_pct >= 0.0,
            "loading_pct must be non-negative"
        );
        let angles_dc = vec![0.0_f64, -0.1_f64];
        let flows_dc = compute_dc_branch_flows(&net, &angles_dc);
        assert!(
            flows_dc[0].loading_pct >= 0.0,
            "DC loading_pct must be non-negative"
        );
    }

    #[test]
    fn test_p_loss_nonnegative_ac() {
        let net = make_2bus_net();
        let v_mag = vec![1.0_f64, 0.98_f64];
        let v_ang = vec![0.0_f64, -0.05_f64];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert!(
            flows[0].p_loss_mw >= -1e-9,
            "p_loss_mw must be non-negative (within numerical tolerance) for resistive branch"
        );
    }

    #[test]
    fn test_rate_a_zero_gives_zero_loading() {
        let mut net = make_2bus_net();
        net.branches[0].rate_a = 0.0;
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(
            flows[0].loading_pct, 0.0,
            "loading_pct must be 0.0 when rate_a is zero (no division by zero)"
        );
        let angles_dc = vec![0.0_f64, -0.05_f64];
        let flows_dc = compute_dc_branch_flows(&net, &angles_dc);
        assert_eq!(
            flows_dc[0].loading_pct, 0.0,
            "loading_pct must be 0.0 when rate_a is zero (no division by zero)"
        );
    }

    #[test]
    fn test_disabled_branch_gives_zero_flows() {
        let mut net = make_2bus_net();
        net.branches[0].status = false;
        let v_mag = vec![1.0_f64; net.bus_count()];
        let v_ang = vec![0.0_f64; net.bus_count()];
        let flows = compute_branch_flows(&net, &v_mag, &v_ang);
        assert_eq!(
            flows.len(),
            1,
            "disabled branch must still produce one BranchFlow entry"
        );
        assert_eq!(
            flows[0].p_from_mw, 0.0,
            "p_from_mw must be zero for disabled branch"
        );
        assert_eq!(
            flows[0].q_from_mvar, 0.0,
            "q_from_mvar must be zero for disabled branch"
        );
        assert_eq!(
            flows[0].p_to_mw, 0.0,
            "p_to_mw must be zero for disabled branch"
        );
        assert_eq!(
            flows[0].q_to_mvar, 0.0,
            "q_to_mvar must be zero for disabled branch"
        );
        assert_eq!(
            flows[0].p_loss_mw, 0.0,
            "p_loss_mw must be zero for disabled branch"
        );
        assert_eq!(
            flows[0].loading_pct, 0.0,
            "loading_pct must be zero for disabled branch"
        );
    }
}
