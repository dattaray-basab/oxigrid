//! Distribution network test cases.
//!
//! Provides standard and synthetic distribution network benchmarks:
//!
//! - IEEE 33-bus: Classic 12.66 kV radial distribution system
//! - IEEE 69-bus: 12.66 kV radial system for feeder studies
//! - European LV residential feeder (synthetic, 0.4 kV)
//! - Medium-voltage urban feeder (synthetic, 11 kV)

use crate::error::OxiGridError;
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::testcases::synthetic::Lcg64;
use crate::units::{Power, ReactivePower, Voltage};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

fn make_pq_bus(id: usize, pd_kw: f64, qd_kvar: f64, base_kv: f64) -> Bus {
    Bus {
        id,
        name: format!("Bus {id}"),
        bus_type: BusType::PQ,
        base_kv: Voltage(base_kv),
        vm: 1.0,
        va: 0.0,
        pd: Power(pd_kw / 1000.0), // kW → MW
        qd: ReactivePower(qd_kvar / 1000.0),
        gs: 0.0,
        bs: 0.0,
        zone: None,
    }
}

fn make_slack_bus(id: usize, base_kv: f64) -> Bus {
    Bus {
        id,
        name: format!("Bus {id}"),
        bus_type: BusType::Slack,
        base_kv: Voltage(base_kv),
        vm: 1.0,
        va: 0.0,
        pd: Power(0.0),
        qd: ReactivePower(0.0),
        gs: 0.0,
        bs: 0.0,
        zone: None,
    }
}

fn make_branch(from: usize, to: usize, r_pu: f64, x_pu: f64) -> Branch {
    Branch {
        from_bus: from,
        to_bus: to,
        r: r_pu,
        x: x_pu,
        b: 0.0,
        rate_a: 100.0,
        rate_b: 100.0,
        rate_c: 100.0,
        tap: 0.0,
        shift: 0.0,
        status: true,
    }
}

/// Convert series impedance (Ω) to per-unit at given base.
fn ohm_to_pu(r_ohm: f64, x_ohm: f64, base_kv: f64, base_mva: f64) -> (f64, f64) {
    let z_base = base_kv * base_kv / base_mva;
    (r_ohm / z_base, x_ohm / z_base)
}

// ---------------------------------------------------------------------------
// IEEE 33-Bus Distribution System
// ---------------------------------------------------------------------------

/// IEEE 33-bus radial distribution test case.
///
/// The system is a 12.66 kV, 100 kVA base radial feeder used extensively
/// in distribution system reconfiguration and loss minimisation studies.
///
/// Key parameters:
/// - 33 buses, 32 branch lines (plus 5 tie switches)
/// - Total active load: 3715 kW
/// - Total reactive load: 2300 kVAr
/// - Main feeder + 5 lateral feeders
///
/// # Reference
/// M.E. Baran and F.F. Wu, "Network reconfiguration in distribution systems
/// for loss reduction and load balancing," IEEE Trans. Power Delivery, 1989.
pub fn ieee33() -> Result<PowerNetwork, OxiGridError> {
    let base_kv = 12.66;
    let base_mva = 10.0; // 10 MVA base (common for distribution)

    let mut net = PowerNetwork::new(base_mva);

    // Bus data: id, Pd (kW), Qd (kVAr)
    // Bus 1 is the substation (slack)
    let bus_data: &[(usize, f64, f64)] = &[
        (1, 0.0, 0.0),
        (2, 100.0, 60.0),
        (3, 90.0, 40.0),
        (4, 120.0, 80.0),
        (5, 60.0, 30.0),
        (6, 60.0, 20.0),
        (7, 200.0, 100.0),
        (8, 200.0, 100.0),
        (9, 60.0, 20.0),
        (10, 60.0, 20.0),
        (11, 45.0, 30.0),
        (12, 60.0, 35.0),
        (13, 60.0, 35.0),
        (14, 120.0, 80.0),
        (15, 60.0, 10.0),
        (16, 60.0, 20.0),
        (17, 60.0, 20.0),
        (18, 90.0, 40.0),
        (19, 90.0, 40.0),
        (20, 90.0, 40.0),
        (21, 90.0, 40.0),
        (22, 90.0, 40.0),
        (23, 90.0, 50.0),
        (24, 420.0, 200.0),
        (25, 420.0, 200.0),
        (26, 60.0, 25.0),
        (27, 60.0, 25.0),
        (28, 60.0, 20.0),
        (29, 120.0, 70.0),
        (30, 200.0, 600.0),
        (31, 150.0, 70.0),
        (32, 210.0, 100.0),
        (33, 60.0, 40.0),
    ];

    for &(id, pd, qd) in bus_data {
        if id == 1 {
            net.buses.push(make_slack_bus(id, base_kv));
        } else {
            net.buses.push(make_pq_bus(id, pd, qd, base_kv));
        }
    }

    // Branch data: from, to, r (Ω), x (Ω)
    // Exact data from Baran & Wu 1989
    let branch_data: &[(usize, usize, f64, f64)] = &[
        (1, 2, 0.0922, 0.0470),
        (2, 3, 0.4930, 0.2511),
        (3, 4, 0.3660, 0.1864),
        (4, 5, 0.3811, 0.1941),
        (5, 6, 0.8190, 0.7070),
        (6, 7, 0.1872, 0.6188),
        (7, 8, 1.7114, 1.2351),
        (8, 9, 1.0300, 0.7400),
        (9, 10, 1.0440, 0.7400),
        (10, 11, 0.1966, 0.0650),
        (11, 12, 0.3744, 0.1238),
        (12, 13, 1.4680, 1.1550),
        (13, 14, 0.5416, 0.7129),
        (14, 15, 0.5910, 0.5260),
        (15, 16, 0.7463, 0.5450),
        (16, 17, 1.2890, 1.7210),
        (17, 18, 0.7320, 0.5740),
        (2, 19, 0.1640, 0.1565),
        (19, 20, 1.5042, 1.3554),
        (20, 21, 0.4095, 0.4784),
        (21, 22, 0.7089, 0.9373),
        (3, 23, 0.4512, 0.3083),
        (23, 24, 0.8980, 0.7091),
        (24, 25, 0.8960, 0.7011),
        (6, 26, 0.2030, 0.1034),
        (26, 27, 0.2842, 0.1447),
        (27, 28, 1.0590, 0.9337),
        (28, 29, 0.8042, 0.7006),
        (29, 30, 0.5075, 0.2585),
        (30, 31, 0.9744, 0.9630),
        (31, 32, 0.3105, 0.3619),
        (32, 33, 0.3410, 0.5302),
    ];

    for &(from, to, r_ohm, x_ohm) in branch_data {
        let (r_pu, x_pu) = ohm_to_pu(r_ohm, x_ohm, base_kv, base_mva);
        net.branches.push(make_branch(from, to, r_pu, x_pu));
    }

    // Substation generator (slack)
    net.generators.push(Generator {
        bus_id: 1,
        pg: 0.0, // solved by slack
        qg: 0.0,
        qmax: 50.0,
        qmin: -50.0,
        vg: 1.0,
        mbase: base_mva,
        status: true,
        pmax: 100.0,
        pmin: 0.0,
    });

    Ok(net)
}

// ---------------------------------------------------------------------------
// IEEE 69-Bus Distribution System
// ---------------------------------------------------------------------------

/// IEEE 69-bus radial distribution test case.
///
/// A 12.66 kV, 10 MVA base radial feeder used for loss minimisation and
/// reactive power compensation studies.
///
/// Key parameters:
/// - 69 buses, 68 branches
/// - Total active load ≈ 3802 kW
/// - Total reactive load ≈ 2695 kVAr
///
/// # Reference
/// S. Ghosh and D. Das, "Method for load-flow solution of radial distribution
/// networks," IEE Proc. Generation Transmission Distribution, 1999.
pub fn ieee69() -> Result<PowerNetwork, OxiGridError> {
    let base_kv = 12.66;
    let base_mva = 10.0;

    let mut net = PowerNetwork::new(base_mva);

    // Bus data: id, Pd (kW), Qd (kVAr)
    let bus_data: &[(usize, f64, f64)] = &[
        (1, 0.0, 0.0),
        (2, 0.0, 0.0),
        (3, 0.0, 0.0),
        (4, 0.0, 0.0),
        (5, 0.0, 0.0),
        (6, 2.6, 2.2),
        (7, 40.4, 30.0),
        (8, 75.0, 54.0),
        (9, 30.0, 22.0),
        (10, 28.0, 19.0),
        (11, 145.0, 104.0),
        (12, 145.0, 104.0),
        (13, 8.0, 5.5),
        (14, 8.0, 5.5),
        (15, 0.0, 0.0),
        (16, 45.5, 30.0),
        (17, 60.0, 35.0),
        (18, 0.0, 0.0),
        (19, 1.0, 0.6),
        (20, 114.0, 81.0),
        (21, 5.3, 3.5),
        (22, 0.0, 0.0),
        (23, 28.0, 20.0),
        (24, 0.0, 0.0),
        (25, 0.0, 0.0),
        (26, 14.0, 10.0),
        (27, 14.0, 10.0),
        (28, 26.0, 18.6),
        (29, 26.0, 18.6),
        (30, 0.0, 0.0),
        (31, 0.0, 0.0),
        (32, 0.0, 0.0),
        (33, 14.0, 10.0),
        (34, 19.5, 14.0),
        (35, 6.0, 4.0),
        (36, 26.0, 18.55),
        (37, 26.0, 18.55),
        (38, 0.0, 0.0),
        (39, 24.0, 17.0),
        (40, 24.0, 17.0),
        (41, 1.2, 1.0),
        (42, 0.0, 0.0),
        (43, 6.0, 4.3),
        (44, 0.0, 0.0),
        (45, 39.22, 26.3),
        (46, 39.22, 26.3),
        (47, 0.0, 0.0),
        (48, 79.0, 56.4),
        (49, 384.7, 274.5),
        (50, 384.7, 274.5),
        (51, 40.5, 28.3),
        (52, 3.6, 2.7),
        (53, 4.35, 3.5),
        (54, 26.4, 19.0),
        (55, 24.0, 17.2),
        (56, 0.0, 0.0),
        (57, 0.0, 0.0),
        (58, 0.0, 0.0),
        (59, 100.0, 72.0),
        (60, 0.0, 0.0),
        (61, 1244.0, 888.0),
        (62, 32.0, 23.0),
        (63, 0.0, 0.0),
        (64, 227.0, 162.0),
        (65, 59.0, 42.0),
        (66, 18.0, 13.0),
        (67, 18.0, 13.0),
        (68, 28.0, 20.0),
        (69, 28.0, 20.0),
    ];

    for &(id, pd, qd) in bus_data {
        if id == 1 {
            net.buses.push(make_slack_bus(id, base_kv));
        } else {
            net.buses.push(make_pq_bus(id, pd, qd, base_kv));
        }
    }

    // Branch data: from, to, r (Ω), x (Ω)
    let branch_data: &[(usize, usize, f64, f64)] = &[
        (1, 2, 0.0005, 0.0012),
        (2, 3, 0.0005, 0.0012),
        (3, 4, 0.0015, 0.0036),
        (4, 5, 0.0251, 0.0294),
        (5, 6, 0.3660, 0.1864),
        (6, 7, 0.3811, 0.1941),
        (7, 8, 0.0922, 0.0470),
        (8, 9, 0.0493, 0.0251),
        (9, 10, 0.8190, 0.2707),
        (10, 11, 0.1872, 0.0691),
        (11, 12, 0.7114, 0.2351),
        (12, 13, 1.0300, 0.3400),
        (13, 14, 1.0440, 0.3450),
        (14, 15, 1.0580, 0.3496),
        (15, 16, 0.1966, 0.0650),
        (16, 17, 0.3744, 0.1238),
        (17, 18, 0.0047, 0.0016),
        (18, 19, 0.3276, 0.1083),
        (19, 20, 0.2106, 0.0690),
        (20, 21, 0.3416, 0.1129),
        (21, 22, 0.0140, 0.0046),
        (22, 23, 0.1591, 0.0526),
        (23, 24, 0.3463, 0.1145),
        (24, 25, 0.7488, 0.2475),
        (25, 26, 0.3089, 0.1021),
        (26, 27, 0.1732, 0.0572),
        (3, 28, 0.0044, 0.0108),
        (28, 29, 0.0640, 0.1565),
        (29, 30, 0.3978, 0.1315),
        (30, 31, 0.0702, 0.0232),
        (31, 32, 0.3510, 0.1160),
        (32, 33, 0.8390, 0.2816),
        (33, 34, 1.7080, 0.5646),
        (34, 35, 1.4740, 0.4873),
        (3, 36, 0.0044, 0.0108),
        (36, 37, 0.0640, 0.1565),
        (37, 38, 0.1053, 0.1230),
        (38, 39, 0.0304, 0.0355),
        (39, 40, 0.0018, 0.0021),
        (40, 41, 0.7283, 0.8509),
        (41, 42, 0.3100, 0.3623),
        (42, 43, 0.0410, 0.0478),
        (43, 44, 0.0092, 0.0116),
        (44, 45, 0.1089, 0.1373),
        (45, 46, 0.0009, 0.0012),
        (4, 47, 0.0034, 0.0084),
        (47, 48, 0.0851, 0.2083),
        (48, 49, 0.2898, 0.7091),
        (49, 50, 0.0822, 0.2011),
        (8, 51, 0.0928, 0.0473),
        (51, 52, 0.3319, 0.1114),
        (9, 53, 0.1740, 0.0886),
        (53, 54, 0.2030, 0.1034),
        (54, 55, 0.2842, 0.1447),
        (55, 56, 0.2813, 0.1433),
        (56, 57, 1.5900, 0.5337),
        (57, 58, 0.7837, 0.2630),
        (58, 59, 0.3042, 0.1006),
        (59, 60, 0.3861, 0.1172),
        (60, 61, 0.5075, 0.2585),
        (61, 62, 0.0974, 0.0496),
        (62, 63, 0.1450, 0.0738),
        (63, 64, 0.7105, 0.3619),
        (64, 65, 1.0410, 0.5302),
        (11, 66, 0.2030, 0.1034),
        (66, 67, 0.2842, 0.1447),
        (12, 68, 0.2030, 0.1034),
        (68, 69, 0.0640, 0.1565),
    ];

    for &(from, to, r_ohm, x_ohm) in branch_data {
        let (r_pu, x_pu) = ohm_to_pu(r_ohm, x_ohm, base_kv, base_mva);
        net.branches.push(make_branch(from, to, r_pu, x_pu));
    }

    net.generators.push(Generator {
        bus_id: 1,
        pg: 0.0,
        qg: 0.0,
        qmax: 50.0,
        qmin: -50.0,
        vg: 1.0,
        mbase: base_mva,
        status: true,
        pmax: 100.0,
        pmin: 0.0,
    });

    Ok(net)
}

// ---------------------------------------------------------------------------
// European LV Residential Feeder (Synthetic)
// ---------------------------------------------------------------------------

/// Synthetic European low-voltage residential network.
///
/// Generates a 0.4 kV radial feeder representing a residential street
/// with single-phase connections distributed over multiple segments.
///
/// Each customer has an average peak demand of ≈ 3 kW (residential).
/// Cable parameters reflect typical underground LV cables (XLPE).
///
/// # Arguments
/// - `n_customers`: number of residential connection points (minimum 2)
pub fn lv_european_residential(n_customers: usize) -> Result<PowerNetwork, OxiGridError> {
    let n_customers = n_customers.max(2);
    let base_kv = 0.4; // 400 V LV
    let base_mva = 0.1; // 100 kVA base

    let mut net = PowerNetwork::new(base_mva);
    let mut rng = Lcg64::new(9876543210);

    // LV transformer (slack bus = MV/LV substation)
    net.buses.push(make_slack_bus(1, base_kv));

    // Customer connection buses (2..=n_customers+1)
    for i in 0..n_customers {
        let id = i + 2;
        // Residential demand: ~3 kW average, pf ≈ 0.95
        let pd_kw = 2.0 + rng.next_f64() * 4.0; // 2..6 kW
        let qd_kvar = pd_kw * (1.0 - 0.95_f64 * 0.95_f64).sqrt() / 0.95; // tan(phi) ≈ 0.33
        net.buses.push(make_pq_bus(id, pd_kw, qd_kvar, base_kv));
    }

    // Feeder cable: daisy-chain from substation through customers
    // LV underground cable: r ≈ 0.25 Ω/km, x ≈ 0.08 Ω/km (50 mm² XLPE)
    let segment_length_m = 30.0; // 30 m between customers
    let r_per_m = 0.25 / 1000.0; // Ω/m
    let x_per_m = 0.08 / 1000.0; // Ω/m

    for i in 0..=n_customers {
        let from = i + 1;
        let to = i + 2;
        if to > n_customers + 1 {
            break;
        }
        let len_m = segment_length_m * (0.8 + 0.4 * rng.next_f64());
        let r_ohm = r_per_m * len_m;
        let x_ohm = x_per_m * len_m;
        let (r_pu, x_pu) = ohm_to_pu(r_ohm, x_ohm, base_kv, base_mva);
        net.branches.push(make_branch(from, to, r_pu, x_pu));
    }

    // LV transformer generator (slack)
    let total_load_kw: f64 = net.buses.iter().map(|b| b.pd.0 * 1000.0).sum();
    net.generators.push(Generator {
        bus_id: 1,
        pg: 0.0,
        qg: 0.0,
        qmax: total_load_kw / 1000.0 * 0.5,
        qmin: -total_load_kw / 1000.0 * 0.5,
        vg: 1.0,
        mbase: base_mva,
        status: true,
        pmax: total_load_kw / 1000.0 * 1.5,
        pmin: 0.0,
    });

    Ok(net)
}

// ---------------------------------------------------------------------------
// Medium-Voltage Urban Feeder (Synthetic)
// ---------------------------------------------------------------------------

/// Synthetic medium-voltage urban cable feeder.
///
/// Generates an 11 kV underground cable feeder representative of an urban
/// distribution network with multiple bus-bar sections.  The topology
/// is an open-ring (normally-open switch at the far end) suitable for
/// feeder reconfiguration studies.
///
/// # Arguments
/// - `n_buses`: number of MV/LV substations (minimum 3)
pub fn mv_urban_feeder(n_buses: usize) -> Result<PowerNetwork, OxiGridError> {
    let n_buses = n_buses.max(3);
    let base_kv = 11.0;
    let base_mva = 10.0;

    let mut net = PowerNetwork::new(base_mva);
    let mut rng = Lcg64::new(1122334455);

    // Bus 1 = primary substation (slack)
    net.buses.push(make_slack_bus(1, base_kv));

    for i in 1..n_buses {
        let id = i + 1;
        // MV/LV substation load: 200–800 kVA
        let pd_kw = 200.0 + rng.next_f64() * 600.0;
        let qd_kvar = pd_kw * 0.25;
        net.buses.push(make_pq_bus(id, pd_kw, qd_kvar, base_kv));
    }

    // Series cable segments (normally closed)
    // 11 kV XLPE cable: r ≈ 0.15 Ω/km, x ≈ 0.10 Ω/km (150 mm²)
    let r_per_km = 0.15;
    let x_per_km = 0.10;
    for i in 0..(n_buses - 1) {
        let from = i + 1;
        let to = i + 2;
        let len_km = 0.3 + rng.next_f64() * 0.5; // 300–800 m
        let r_ohm = r_per_km * len_km;
        let x_ohm = x_per_km * len_km;
        let (r_pu, x_pu) = ohm_to_pu(r_ohm, x_ohm, base_kv, base_mva);
        net.branches.push(make_branch(from, to, r_pu, x_pu));
    }

    // Normally-open tie switch: connect last bus back to bus 1 (open in base case)
    // This represents the ring cable; kept open = status false
    let last = n_buses;
    let (r_pu, x_pu) = ohm_to_pu(0.15 * 0.5, 0.10 * 0.5, base_kv, base_mva);
    net.branches.push(Branch {
        from_bus: last,
        to_bus: 1,
        r: r_pu,
        x: x_pu,
        b: 0.0,
        rate_a: 50.0,
        rate_b: 50.0,
        rate_c: 50.0,
        tap: 0.0,
        shift: 0.0,
        status: false, // normally open
    });

    let total_load_mw: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    net.generators.push(Generator {
        bus_id: 1,
        pg: 0.0,
        qg: 0.0,
        qmax: total_load_mw * 0.6,
        qmin: -total_load_mw * 0.4,
        vg: 1.0,
        mbase: base_mva,
        status: true,
        pmax: total_load_mw * 1.5,
        pmin: 0.0,
    });

    Ok(net)
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn slack_count(net: &PowerNetwork) -> usize {
        net.buses
            .iter()
            .filter(|b| b.bus_type == BusType::Slack)
            .count()
    }

    // ── Internal helpers ────────────────────────────────────────────────────

    #[test]
    fn make_pq_bus_converts_kw_to_mw() {
        let b = make_pq_bus(7, 3715.0, 2300.0, 12.66);
        assert_eq!(b.id, 7);
        assert_eq!(b.bus_type, BusType::PQ);
        assert_eq!(b.base_kv.0, 12.66);
        // kW/kVAr are stored internally as MW/MVAr.
        assert!((b.pd.0 - 3.715).abs() < 1e-9);
        assert!((b.qd.0 - 2.300).abs() < 1e-9);
        assert_eq!(b.gs, 0.0);
        assert_eq!(b.bs, 0.0);
        assert_eq!(b.vm, 1.0);
    }

    #[test]
    fn make_slack_bus_has_no_load() {
        let b = make_slack_bus(1, 0.4);
        assert_eq!(b.bus_type, BusType::Slack);
        assert_eq!(b.pd.0, 0.0);
        assert_eq!(b.qd.0, 0.0);
        assert_eq!(b.base_kv.0, 0.4);
    }

    #[test]
    fn make_branch_defaults_are_closed_line() {
        let br = make_branch(3, 4, 0.01, 0.02);
        assert_eq!(br.from_bus, 3);
        assert_eq!(br.to_bus, 4);
        assert_eq!(br.r, 0.01);
        assert_eq!(br.x, 0.02);
        assert_eq!(br.b, 0.0);
        assert_eq!(br.tap, 0.0);
        assert_eq!(br.shift, 0.0);
        assert!(br.status);
        assert_eq!(br.rate_a, 100.0);
    }

    #[test]
    fn ohm_to_pu_uses_kv_squared_over_mva_base() {
        let base_kv = 12.66;
        let base_mva = 0.1;
        let z_base = base_kv * base_kv / base_mva;
        // Impedance equal to the base must map to exactly 1.0 p.u.
        let (r_pu, x_pu) = ohm_to_pu(z_base, z_base, base_kv, base_mva);
        assert!((r_pu - 1.0).abs() < 1e-9);
        assert!((x_pu - 1.0).abs() < 1e-9);
        // And linearity: half the base impedance → 0.5 p.u.
        let (r_half, _) = ohm_to_pu(z_base * 0.5, 0.0, base_kv, base_mva);
        assert!((r_half - 0.5).abs() < 1e-9);
    }

    // ── IEEE 33-bus ─────────────────────────────────────────────────────────

    #[test]
    fn ieee33_structure_and_load() {
        let net = ieee33().unwrap();
        assert_eq!(net.buses.len(), 33);
        assert_eq!(slack_count(&net), 1);
        // 32 series branches form the radial backbone.
        let closed = net.branches.iter().filter(|b| b.status).count();
        assert!(closed >= 32, "expected ≥32 closed branches, got {closed}");
        // Published active load ≈ 3.715 MW.
        assert!(
            (net.total_load_mw() - 3.715).abs() < 0.05,
            "ieee33 load {} MW ≈ 3.715 MW",
            net.total_load_mw()
        );
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn ieee33_newton_raphson_converges() {
        use crate::powerflow::newton_raphson::NewtonRaphsonSolver;
        use crate::powerflow::{PowerFlowConfig, PowerFlowMethod, PowerFlowSolver};

        let net = ieee33().unwrap();
        let cfg = PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 100,
            tolerance: 1e-6,
            enforce_q_limits: false,
        };
        let res = NewtonRaphsonSolver.solve(&net, &cfg).unwrap();
        assert!(
            res.converged,
            "ieee33 NR max_mismatch={:.2e}",
            res.max_mismatch
        );
        // Radial LV feeder: voltages sag below 1.0 but stay within ±10 %.
        for vm in &res.voltage_magnitude {
            assert!((0.85..=1.05).contains(vm), "ieee33 vm {vm} out of range");
        }
    }

    // ── IEEE 69-bus ─────────────────────────────────────────────────────────

    #[test]
    fn ieee69_structure() {
        let net = ieee69().unwrap();
        assert_eq!(net.buses.len(), 69);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    // ── LV European residential feeder ───────────────────────────────────────

    #[test]
    fn lv_feeder_structure_and_voltage_level() {
        let net = lv_european_residential(20).unwrap();
        // 1 substation (slack) + 20 customers.
        assert_eq!(net.buses.len(), 21);
        assert_eq!(slack_count(&net), 1);
        for b in &net.buses {
            assert!((b.base_kv.0 - 0.4).abs() < 1e-9, "LV feeder is 0.4 kV");
        }
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn lv_feeder_clamps_tiny_customer_count() {
        // n_customers is clamped to a minimum of 2 → 3 buses total.
        let net = lv_european_residential(0).unwrap();
        assert_eq!(net.buses.len(), 3);
        assert!(net.is_connected());
    }

    // ── MV urban feeder ──────────────────────────────────────────────────────

    #[test]
    fn mv_feeder_structure_and_open_tie() {
        let net = mv_urban_feeder(10).unwrap();
        assert_eq!(net.buses.len(), 10);
        assert_eq!(slack_count(&net), 1);
        for b in &net.buses {
            assert!((b.base_kv.0 - 11.0).abs() < 1e-9, "MV feeder is 11 kV");
        }
        // The final branch is the normally-open ring tie switch.
        let tie = net.branches.last().unwrap();
        assert!(!tie.status, "ring tie switch must be normally open");
        assert_eq!(tie.from_bus, 10);
        assert_eq!(tie.to_bus, 1);
        net.validate().unwrap();
    }

    #[test]
    fn mv_feeder_clamps_tiny_bus_count() {
        // n_buses is clamped to a minimum of 3.
        let net = mv_urban_feeder(1).unwrap();
        assert_eq!(net.buses.len(), 3);
    }

    #[test]
    fn mv_feeder_is_reproducible() {
        // Fixed internal LCG seed → identical structure across builds.
        let a = mv_urban_feeder(8).unwrap();
        let b = mv_urban_feeder(8).unwrap();
        assert_eq!(a.buses.len(), b.buses.len());
        assert_eq!(a.branches.len(), b.branches.len());
        assert!((a.total_load_mw() - b.total_load_mw()).abs() < 1e-12);
    }
}
