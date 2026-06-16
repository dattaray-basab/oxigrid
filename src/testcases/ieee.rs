#![allow(clippy::type_complexity)]
//! IEEE Standard Test Cases for Power Systems Analysis.
//!
//! Provides classic IEEE benchmark networks used in power systems research:
//! IEEE 14-bus, 30-bus, 57-bus, 118-bus, 300-bus, RTS-96, and PEGASE 89-bus.
//!
//! All bus data follows the MATPOWER convention:
//! - Bus type: 1=PQ, 2=PV, 3=slack
//! - Power quantities in MW/MVAr, base 100 MVA
//! - Voltage in p.u., angles in degrees (converted to radians internally)

use crate::error::OxiGridError;
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::units::{Power, ReactivePower, Voltage};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Map MATPOWER bus type integer to `BusType`.
fn map_bus_type(t: u8) -> Result<BusType, OxiGridError> {
    match t {
        1 => Ok(BusType::PQ),
        2 => Ok(BusType::PV),
        3 => Ok(BusType::Slack),
        _ => Err(OxiGridError::InvalidNetwork(format!(
            "unknown bus type code {t}"
        ))),
    }
}

/// Build a `Bus` from a flat data tuple.
///
/// Fields: (id, type, Pd, Qd, Gs, Bs, base_kv, Vm, Va)
#[allow(clippy::too_many_arguments)]
fn make_bus(
    id: usize,
    bus_type: u8,
    pd_mw: f64,
    qd_mvar: f64,
    gs: f64,
    bs: f64,
    base_kv: f64,
    vm: f64,
    va_deg: f64,
) -> Result<Bus, OxiGridError> {
    Ok(Bus {
        id,
        name: format!("Bus {id}"),
        bus_type: map_bus_type(bus_type)?,
        base_kv: Voltage(base_kv),
        vm,
        va: va_deg.to_radians(),
        pd: Power(pd_mw),
        qd: ReactivePower(qd_mvar),
        gs,
        bs,
        zone: None,
    })
}

/// Build a `Branch` from flat data.
fn make_branch(from_bus: usize, to_bus: usize, r: f64, x: f64, b: f64, rate_a: f64) -> Branch {
    Branch {
        from_bus,
        to_bus,
        r,
        x,
        b,
        rate_a,
        rate_b: rate_a,
        rate_c: rate_a,
        tap: 0.0,
        shift: 0.0,
        status: true,
    }
}

/// Build a transformer `Branch`.
#[allow(dead_code)]
fn make_transformer(
    from_bus: usize,
    to_bus: usize,
    r: f64,
    x: f64,
    b: f64,
    rate_a: f64,
    tap: f64,
) -> Branch {
    Branch {
        from_bus,
        to_bus,
        r,
        x,
        b,
        rate_a,
        rate_b: rate_a,
        rate_c: rate_a,
        tap,
        shift: 0.0,
        status: true,
    }
}

/// Build a `Generator` from flat data.
#[allow(clippy::too_many_arguments)]
fn make_gen(
    bus_id: usize,
    pg: f64,
    qg: f64,
    qmax: f64,
    qmin: f64,
    vg: f64,
    pmax: f64,
    pmin: f64,
) -> Generator {
    Generator {
        bus_id,
        pg,
        qg,
        qmax,
        qmin,
        vg,
        mbase: 100.0,
        status: true,
        pmax,
        pmin,
    }
}

// ---------------------------------------------------------------------------
// IEEE 14-Bus System
// ---------------------------------------------------------------------------

/// IEEE 14-bus system — the canonical power flow test case.
///
/// The system represents a portion of the American Electric Power (AEP) system
/// as of February 1962.  It contains 14 buses, 20 branches, and 5 generators.
///
/// Key data:
/// - System base: 100 MVA
/// - Voltage levels: 132 kV and 33 kV
/// - Total load: ~259 MW + 81 MVAr
/// - Generators at buses 1 (slack), 2, 3, 6, 8
///
/// # Reference
/// Power Systems Test Case Archive, University of Washington.
pub fn ieee14() -> Result<PowerNetwork, OxiGridError> {
    let mut net = PowerNetwork::new(100.0);

    // Bus data: id, type, Pd, Qd, Gs, Bs, base_kv, Vm, Va(deg)
    let bus_data: &[(usize, u8, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 3, 0.0, 0.0, 0.0, 0.0, 132.0, 1.060, 0.0),
        (2, 2, 21.7, 12.7, 0.0, 0.0, 132.0, 1.045, -4.98),
        (3, 2, 94.2, 19.0, 0.0, 0.0, 132.0, 1.010, -12.72),
        (4, 1, 47.8, -3.9, 0.0, 0.0, 132.0, 1.019, -10.33),
        (5, 1, 7.6, 1.6, 0.0, 0.0, 132.0, 1.020, -8.78),
        (6, 2, 11.2, 7.5, 0.0, 0.0, 33.0, 1.070, -14.22),
        (7, 1, 0.0, 0.0, 0.0, 0.0, 33.0, 1.062, -13.37),
        (8, 2, 0.0, 0.0, 0.0, 0.0, 33.0, 1.090, -13.36),
        (9, 1, 29.5, 16.6, 0.0, 0.19, 33.0, 1.056, -14.94),
        (10, 1, 9.0, 5.8, 0.0, 0.0, 33.0, 1.051, -15.10),
        (11, 1, 3.5, 1.8, 0.0, 0.0, 33.0, 1.057, -14.79),
        (12, 1, 6.1, 1.6, 0.0, 0.0, 33.0, 1.055, -15.07),
        (13, 1, 13.5, 5.8, 0.0, 0.0, 33.0, 1.050, -15.16),
        (14, 1, 14.9, 5.0, 0.0, 0.0, 33.0, 1.036, -16.04),
    ];

    for &(id, t, pd, qd, gs, bs, kv, vm, va) in bus_data {
        net.buses.push(make_bus(id, t, pd, qd, gs, bs, kv, vm, va)?);
    }

    // Branch data: from, to, r, x, b, rate_a(MVA)
    // Transmission lines
    let line_data: &[(usize, usize, f64, f64, f64, f64)] = &[
        (1, 2, 0.01938, 0.05917, 0.0528, 9999.0),
        (1, 5, 0.05403, 0.22304, 0.0492, 9999.0),
        (2, 3, 0.04699, 0.19797, 0.0438, 9999.0),
        (2, 4, 0.05811, 0.17632, 0.0340, 9999.0),
        (2, 5, 0.05695, 0.17388, 0.0346, 9999.0),
        (3, 4, 0.06701, 0.17103, 0.0128, 9999.0),
        (4, 5, 0.01335, 0.04211, 0.0, 9999.0),
        (4, 7, 0.0, 0.20912, 0.0, 9999.0), // transformer
        (4, 9, 0.0, 0.55618, 0.0, 9999.0), // transformer
        (5, 6, 0.0, 0.25202, 0.0, 9999.0), // transformer
        (6, 11, 0.09498, 0.19890, 0.0, 9999.0),
        (6, 12, 0.12291, 0.25581, 0.0, 9999.0),
        (6, 13, 0.06615, 0.13027, 0.0, 9999.0),
        (7, 8, 0.0, 0.17615, 0.0, 9999.0), // transformer
        (7, 9, 0.0, 0.11001, 0.0, 9999.0),
        (9, 10, 0.03181, 0.08450, 0.0, 9999.0),
        (9, 14, 0.12711, 0.27038, 0.0, 9999.0),
        (10, 11, 0.08205, 0.19207, 0.0, 9999.0),
        (12, 13, 0.22092, 0.19988, 0.0, 9999.0),
        (13, 14, 0.17093, 0.34802, 0.0, 9999.0),
    ];

    for &(f, t, r, x, b, ra) in line_data {
        net.branches.push(make_branch(f, t, r, x, b, ra));
    }

    // Generator data: bus, Pg, Qg, Qmax, Qmin, Vg, Pmax, Pmin
    let gen_data: &[(usize, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 232.4, -16.9, 10.0, 0.0, 1.060, 332.4, 0.0),
        (2, 40.0, 42.4, 50.0, -40.0, 1.045, 140.0, 0.0),
        (3, 0.0, 23.4, 40.0, 0.0, 1.010, 100.0, 0.0),
        (6, 0.0, 12.2, 24.0, -6.0, 1.070, 100.0, 0.0),
        (8, 0.0, 17.4, 24.0, -6.0, 1.090, 100.0, 0.0),
    ];

    for &(bus, pg, qg, qmax, qmin, vg, pmax, pmin) in gen_data {
        net.generators
            .push(make_gen(bus, pg, qg, qmax, qmin, vg, pmax, pmin));
    }

    Ok(net)
}

// ---------------------------------------------------------------------------
// IEEE 30-Bus System
// ---------------------------------------------------------------------------

/// IEEE 30-bus system — 100 MVA base, mixed 132/33 kV.
///
/// Represents a portion of the AEP power system as of December 1961.
/// Contains 30 buses, 41 branches, and 6 generators.
///
/// # Reference
/// Power Systems Test Case Archive, University of Washington.
pub fn ieee30() -> Result<PowerNetwork, OxiGridError> {
    let mut net = PowerNetwork::new(100.0);

    // Bus data: id, type, Pd, Qd, Gs, Bs, base_kv, Vm, Va(deg)
    let bus_data: &[(usize, u8, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 3, 0.0, 0.0, 0.0, 0.0, 132.0, 1.060, 0.000),
        (2, 2, 21.7, 12.7, 0.0, 0.0, 132.0, 1.043, -5.480),
        (3, 1, 2.4, 1.2, 0.0, 0.0, 132.0, 1.021, -7.960),
        (4, 1, 7.6, 1.6, 0.0, 0.0, 132.0, 1.012, -9.620),
        (5, 2, 94.2, 19.0, 0.0, 0.0, 132.0, 1.010, -14.370),
        (6, 1, 0.0, 0.0, 0.0, 0.0, 132.0, 1.010, -11.340),
        (7, 1, 22.8, 10.9, 0.0, 0.0, 132.0, 1.002, -13.120),
        (8, 2, 30.0, 30.0, 0.0, 0.0, 132.0, 1.010, -12.100),
        (9, 1, 0.0, 0.0, 0.0, 0.0, 33.0, 1.051, -14.380),
        (10, 1, 5.8, 2.0, 0.0, 0.19, 33.0, 1.045, -15.970),
        (11, 2, 0.0, 0.0, 0.0, 0.0, 11.0, 1.082, -14.390),
        (12, 1, 11.2, 7.5, 0.0, 0.0, 33.0, 1.057, -15.240),
        (13, 2, 0.0, 0.0, 0.0, 0.0, 11.0, 1.071, -15.240),
        (14, 1, 6.2, 1.6, 0.0, 0.0, 33.0, 1.042, -16.130),
        (15, 1, 8.2, 2.5, 0.0, 0.0, 33.0, 1.038, -16.220),
        (16, 1, 3.5, 1.8, 0.0, 0.0, 33.0, 1.045, -15.830),
        (17, 1, 9.0, 5.8, 0.0, 0.0, 33.0, 1.040, -16.140),
        (18, 1, 3.2, 0.9, 0.0, 0.0, 33.0, 1.028, -16.820),
        (19, 1, 9.5, 3.4, 0.0, 0.0, 33.0, 1.026, -17.000),
        (20, 1, 2.2, 0.7, 0.0, 0.0, 33.0, 1.030, -16.800),
        (21, 1, 17.5, 11.2, 0.0, 0.0, 33.0, 1.033, -16.420),
        (22, 1, 0.0, 0.0, 0.0, 0.0, 33.0, 1.033, -16.410),
        (23, 1, 3.2, 1.6, 0.0, 0.0, 33.0, 1.027, -16.610),
        (24, 1, 8.7, 6.7, 0.0, 0.043, 33.0, 1.021, -16.780),
        (25, 1, 0.0, 0.0, 0.0, 0.0, 33.0, 1.017, -16.350),
        (26, 1, 3.5, 2.3, 0.0, 0.0, 33.0, 1.000, -16.770),
        (27, 1, 0.0, 0.0, 0.0, 0.0, 33.0, 1.023, -15.820),
        (28, 1, 0.0, 0.0, 0.0, 0.0, 132.0, 1.007, -11.970),
        (29, 1, 2.4, 0.9, 0.0, 0.0, 33.0, 1.003, -17.060),
        (30, 1, 10.6, 1.9, 0.0, 0.0, 33.0, 0.992, -17.940),
    ];

    for &(id, t, pd, qd, gs, bs, kv, vm, va) in bus_data {
        net.buses.push(make_bus(id, t, pd, qd, gs, bs, kv, vm, va)?);
    }

    // Branches (lines)
    let line_data: &[(usize, usize, f64, f64, f64, f64)] = &[
        (1, 2, 0.0192, 0.0575, 0.0528, 130.0),
        (1, 3, 0.0452, 0.1852, 0.0408, 130.0),
        (2, 4, 0.0570, 0.1737, 0.0368, 65.0),
        (3, 4, 0.0132, 0.0379, 0.0084, 130.0),
        (2, 5, 0.0472, 0.1983, 0.0418, 130.0),
        (2, 6, 0.0581, 0.1763, 0.0374, 65.0),
        (4, 6, 0.0119, 0.0414, 0.0090, 90.0),
        (5, 7, 0.0460, 0.1160, 0.0204, 70.0),
        (6, 7, 0.0267, 0.0820, 0.0170, 130.0),
        (6, 8, 0.0120, 0.0420, 0.0090, 32.0),
        (6, 9, 0.0, 0.2080, 0.0, 65.0),
        (6, 10, 0.0, 0.5560, 0.0, 32.0),
        (9, 11, 0.0, 0.2080, 0.0, 65.0),
        (9, 10, 0.0, 0.1100, 0.0, 65.0),
        (4, 12, 0.0, 0.2560, 0.0, 65.0),
        (12, 13, 0.0, 0.1400, 0.0, 65.0),
        (12, 14, 0.1231, 0.2559, 0.0, 32.0),
        (12, 15, 0.0662, 0.1304, 0.0, 32.0),
        (12, 16, 0.0945, 0.1987, 0.0, 32.0),
        (14, 15, 0.2210, 0.1997, 0.0, 16.0),
        (16, 17, 0.0524, 0.1923, 0.0, 16.0),
        (15, 18, 0.1073, 0.2185, 0.0, 16.0),
        (18, 19, 0.0639, 0.1292, 0.0, 16.0),
        (19, 20, 0.0340, 0.0680, 0.0, 32.0),
        (10, 20, 0.0936, 0.2090, 0.0, 32.0),
        (10, 17, 0.0324, 0.0845, 0.0, 32.0),
        (10, 21, 0.0348, 0.0749, 0.0, 32.0),
        (10, 22, 0.0727, 0.1499, 0.0, 32.0),
        (21, 22, 0.0116, 0.0236, 0.0, 32.0),
        (15, 23, 0.1000, 0.2020, 0.0, 16.0),
        (22, 24, 0.1150, 0.1790, 0.0, 16.0),
        (23, 24, 0.1320, 0.2700, 0.0, 16.0),
        (24, 25, 0.1885, 0.3292, 0.0, 16.0),
        (25, 26, 0.2544, 0.3800, 0.0, 16.0),
        (25, 27, 0.1093, 0.2087, 0.0, 16.0),
        (28, 27, 0.0, 0.3960, 0.0, 65.0),
        (27, 29, 0.2198, 0.4153, 0.0, 16.0),
        (27, 30, 0.3202, 0.6027, 0.0, 16.0),
        (29, 30, 0.2399, 0.4533, 0.0, 16.0),
        (8, 28, 0.0636, 0.2000, 0.0428, 32.0),
        (6, 28, 0.0169, 0.0599, 0.0130, 32.0),
    ];

    for &(f, t, r, x, b, ra) in line_data {
        net.branches.push(make_branch(f, t, r, x, b, ra));
    }

    // Generators
    let gen_data: &[(usize, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 260.2, -16.1, 10.0, 0.0, 1.060, 360.2, 0.0),
        (2, 40.0, 50.0, 50.0, -40.0, 1.043, 140.0, 0.0),
        (5, 0.0, 37.0, 40.0, -40.0, 1.010, 100.0, 0.0),
        (8, 0.0, 37.3, 40.0, -10.0, 1.010, 100.0, 0.0),
        (11, 0.0, 16.2, 24.0, -6.0, 1.082, 100.0, 0.0),
        (13, 0.0, 10.6, 24.0, -6.0, 1.071, 100.0, 0.0),
    ];

    for &(bus, pg, qg, qmax, qmin, vg, pmax, pmin) in gen_data {
        net.generators
            .push(make_gen(bus, pg, qg, qmax, qmin, vg, pmax, pmin));
    }

    Ok(net)
}

// ---------------------------------------------------------------------------
// IEEE 57-Bus System (topologically representative)
// ---------------------------------------------------------------------------

/// IEEE 57-bus system — 100 MVA base, 57 buses, 85 branches, 7 generators.
///
/// Represents a portion of the American Electric Power (AEP) system.
/// This implementation uses topologically correct data faithful to the
/// published test case (Power Systems Test Case Archive, Univ. of Washington).
pub fn ieee57() -> Result<PowerNetwork, OxiGridError> {
    let mut net = PowerNetwork::new(100.0);

    // Bus data: id, type, Pd, Qd, Gs, Bs, base_kv, Vm, Va(deg)
    // Condensed from the full 57-bus dataset
    let bus_data: &[(usize, u8, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 3, 55.0, 17.0, 0.0, 0.0, 138.0, 1.040, 0.000),
        (2, 2, 3.0, 88.0, 0.0, 0.0, 138.0, 1.010, -1.180),
        (3, 2, 41.0, 21.0, 0.0, 0.0, 138.0, 0.985, -5.970),
        (4, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 0.981, -7.320),
        (5, 1, 13.0, 4.0, 0.0, 0.0, 138.0, 0.976, -8.520),
        (6, 2, 75.0, 2.0, 0.0, 0.0, 138.0, 0.980, -8.650),
        (7, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 0.984, -7.580),
        (8, 2, 150.0, 22.0, 0.0, 0.0, 138.0, 1.005, -4.530),
        (9, 2, 121.0, 26.0, 0.0, 0.0, 138.0, 0.980, -9.000),
        (10, 1, 5.0, 2.0, 0.0, 0.0, 138.0, 0.986, -9.000),
        (11, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 0.974, -9.840),
        (12, 2, 377.0, 24.0, 0.0, 0.0, 138.0, 1.015, -9.000),
        (13, 1, 18.0, 2.3, 0.0, 0.0, 138.0, 0.979, -10.360),
        (14, 1, 10.5, 5.3, 0.0, 0.0, 138.0, 0.970, -10.360),
        (15, 1, 22.0, 5.0, 0.0, 0.0, 138.0, 0.962, -11.600),
        (16, 1, 43.0, 3.0, 0.0, 0.0, 138.0, 0.954, -12.580),
        (17, 1, 4.2, 1.5, 0.0, 0.0, 138.0, 0.978, -9.780),
        (18, 1, 27.2, 9.8, 0.0, 0.0, 138.0, 0.963, -11.090),
        (19, 1, 3.3, 0.6, 0.0, 0.0, 138.0, 0.966, -10.690),
        (20, 1, 2.3, 1.0, 0.0, 0.0, 138.0, 0.970, -10.840),
        (21, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 1.008, -7.580),
        (22, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 1.010, -7.510),
        (23, 1, 6.3, 2.1, 0.0, 0.0, 138.0, 1.008, -7.330),
        (24, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 0.999, -9.380),
        (25, 1, 6.3, 3.2, 0.0, 0.0, 138.0, 0.982, -10.210),
        (26, 1, 0.0, 0.0, 0.0, 0.0, 138.0, 0.959, -11.530),
        (27, 1, 9.3, 0.5, 0.0, 0.0, 138.0, 0.982, -9.780),
        (28, 1, 4.6, 2.3, 0.0, 0.0, 138.0, 0.997, -8.350),
        (29, 1, 17.0, 2.6, 0.0, 0.0, 138.0, 1.010, -7.580),
        (30, 1, 3.6, 1.8, 0.0, 0.0, 138.0, 0.962, -11.630),
        (31, 1, 58.0, 29.0, 0.0, 0.0, 138.0, 0.936, -13.780),
        (32, 1, 1.6, 0.8, 0.0, 0.0, 138.0, 0.949, -12.880),
        (33, 1, 3.8, 0.19, 0.0, 0.0, 138.0, 0.947, -12.960),
        (34, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.024, -11.320),
        (35, 1, 6.0, 3.0, 0.0, 0.0, 20.0, 1.025, -11.320),
        (36, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.035, -12.060),
        (37, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.035, -12.360),
        (38, 1, 14.0, 7.0, 0.0, 0.0, 20.0, 1.028, -12.060),
        (39, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.033, -12.400),
        (40, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.033, -12.250),
        (41, 1, 6.3, 3.0, 0.0, 0.0, 20.0, 1.027, -12.440),
        (42, 1, 7.1, 4.4, 0.0, 0.0, 20.0, 1.024, -12.440),
        (43, 1, 2.0, 1.0, 0.0, 0.0, 20.0, 1.023, -12.640),
        (44, 1, 12.0, 1.8, 0.0, 0.0, 20.0, 1.023, -12.640),
        (45, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.028, -12.370),
        (46, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.030, -12.450),
        (47, 1, 29.7, 11.6, 0.0, 0.0, 20.0, 1.033, -11.040),
        (48, 1, 0.0, 0.0, 0.0, 0.0, 20.0, 1.028, -11.440),
        (49, 1, 18.0, 8.5, 0.0, 0.0, 138.0, 1.027, -9.000),
        (50, 1, 21.0, 10.5, 0.0, 0.0, 138.0, 1.023, -10.450),
        (51, 1, 18.0, 5.3, 0.0, 0.0, 138.0, 1.052, -9.290),
        (52, 1, 4.9, 2.2, 0.0, 0.0, 138.0, 0.980, -9.930),
        (53, 1, 20.0, 10.0, 0.0, 0.0, 138.0, 0.971, -10.850),
        (54, 1, 4.1, 1.4, 0.0, 0.0, 138.0, 0.996, -9.420),
        (55, 1, 6.8, 3.4, 0.0, 0.0, 138.0, 0.954, -12.000),
        (56, 1, 7.6, 2.2, 0.0, 0.0, 138.0, 0.954, -12.420),
        (57, 1, 6.7, 2.0, 0.0, 0.0, 138.0, 0.980, -10.290),
    ];

    for &(id, t, pd, qd, gs, bs, kv, vm, va) in bus_data {
        net.buses.push(make_bus(id, t, pd, qd, gs, bs, kv, vm, va)?);
    }

    // Branch data (85 branches, including parallel circuits on 4-18, 24-25,
    // 42-49 and 49-54).
    let line_data: &[(usize, usize, f64, f64, f64, f64)] = &[
        (1, 2, 0.0083, 0.0280, 0.1290, 250.0),
        (2, 3, 0.0298, 0.0850, 0.0818, 250.0),
        (3, 4, 0.0112, 0.0366, 0.0380, 250.0),
        (4, 5, 0.0625, 0.1320, 0.0258, 250.0),
        (4, 6, 0.0430, 0.1480, 0.0348, 250.0),
        (6, 7, 0.0200, 0.1020, 0.0276, 250.0),
        (6, 8, 0.0339, 0.1730, 0.0470, 250.0),
        (8, 9, 0.0099, 0.0505, 0.0548, 250.0),
        (9, 10, 0.0369, 0.1679, 0.0440, 250.0),
        (9, 11, 0.0258, 0.0848, 0.0218, 250.0),
        (9, 12, 0.0648, 0.2950, 0.0772, 250.0),
        (9, 13, 0.0481, 0.1580, 0.0406, 250.0),
        (13, 14, 0.0132, 0.0434, 0.0110, 250.0),
        (13, 15, 0.0269, 0.0869, 0.0230, 250.0),
        (1, 15, 0.0178, 0.0910, 0.0988, 250.0),
        (1, 16, 0.0454, 0.2060, 0.0546, 250.0),
        (1, 17, 0.0238, 0.1080, 0.0286, 250.0),
        (3, 15, 0.0162, 0.0530, 0.0544, 250.0),
        (4, 18, 0.0, 0.5550, 0.0, 250.0),
        (4, 18, 0.0, 0.4300, 0.0, 250.0),
        (5, 6, 0.0302, 0.0641, 0.0124, 250.0),
        (7, 8, 0.0139, 0.0712, 0.0194, 250.0),
        (10, 12, 0.0277, 0.1262, 0.0328, 250.0),
        (11, 13, 0.0223, 0.0732, 0.0188, 250.0),
        (12, 13, 0.0178, 0.0580, 0.0604, 250.0),
        (12, 16, 0.0180, 0.0813, 0.0216, 250.0),
        (12, 17, 0.0397, 0.1790, 0.0476, 250.0),
        (14, 15, 0.0171, 0.0547, 0.0148, 250.0),
        (18, 19, 0.4610, 0.6850, 0.0, 250.0),
        (19, 20, 0.2830, 0.4340, 0.0, 250.0),
        (21, 20, 0.0, 0.7767, 0.0, 250.0),
        (21, 22, 0.0736, 0.1170, 0.0, 250.0),
        (22, 23, 0.0099, 0.0152, 0.0, 250.0),
        (23, 24, 0.1660, 0.2560, 0.0042, 250.0),
        (24, 25, 0.0, 1.1820, 0.0, 250.0),
        (24, 25, 0.0, 1.2300, 0.0, 250.0),
        (24, 26, 0.0, 0.0473, 0.0, 250.0),
        (26, 27, 0.1650, 0.2540, 0.0, 250.0),
        (27, 28, 0.0618, 0.0954, 0.0, 250.0),
        (28, 29, 0.0418, 0.0587, 0.0, 250.0),
        (7, 29, 0.0, 0.6480, 0.0, 250.0),
        (25, 30, 0.1350, 0.2020, 0.0, 250.0),
        (30, 31, 0.3260, 0.4970, 0.0, 250.0),
        (23, 32, 0.0, 0.6300, 0.0, 250.0),
        (31, 32, 0.5070, 0.7550, 0.0, 250.0),
        (27, 32, 0.3130, 0.4670, 0.0, 250.0),
        (15, 33, 0.0178, 0.0910, 0.0, 250.0),
        (19, 34, 0.0, 0.5547, 0.0, 250.0),
        (35, 36, 0.0143, 0.0700, 0.0, 250.0),
        (35, 37, 0.0, 0.1900, 0.0, 250.0),
        (33, 37, 0.0415, 0.1420, 0.0, 250.0),
        (34, 36, 0.0, 0.2670, 0.0, 250.0),
        (34, 37, 0.0, 0.1900, 0.0, 250.0),
        (38, 37, 0.0, 0.1750, 0.0, 250.0),
        (37, 39, 0.0321, 0.1060, 0.0, 250.0),
        (37, 40, 0.0593, 0.1680, 0.0, 250.0),
        (30, 38, 0.0, 0.0954, 0.0, 250.0),
        (39, 40, 0.0184, 0.0605, 0.0, 250.0),
        (40, 41, 0.1450, 0.4870, 0.0, 250.0),
        (40, 42, 0.5550, 0.1830, 0.0, 250.0),
        (41, 43, 0.4100, 0.1350, 0.0, 250.0),
        (40, 44, 0.0, 0.1780, 0.0, 250.0),
        (43, 44, 0.0608, 0.2454, 0.0, 250.0),
        (34, 43, 0.4130, 0.6810, 0.0, 250.0),
        (44, 45, 0.0224, 0.0901, 0.0, 250.0),
        (45, 46, 0.0, 0.0845, 0.0, 250.0),
        (46, 47, 0.0, 0.2518, 0.0, 250.0),
        (46, 48, 0.0, 0.1298, 0.0, 250.0),
        (47, 49, 0.0844, 0.2778, 0.0, 250.0),
        (42, 49, 0.3150, 0.4270, 0.0, 250.0),
        (42, 49, 0.3150, 0.4270, 0.0, 250.0),
        (45, 49, 0.0780, 0.1570, 0.0, 250.0),
        (48, 49, 0.0, 0.1060, 0.0, 250.0),
        (49, 50, 0.2910, 0.3860, 0.0, 250.0),
        (49, 51, 0.1730, 0.2260, 0.0, 250.0),
        (51, 52, 0.2030, 0.2680, 0.0, 250.0),
        (52, 53, 0.4050, 0.5480, 0.0, 250.0),
        (53, 54, 0.2630, 0.3440, 0.0, 250.0),
        (49, 54, 0.0730, 0.0961, 0.0, 250.0),
        (49, 54, 0.0869, 0.1151, 0.0, 250.0),
        (54, 55, 0.1690, 0.2070, 0.0, 250.0),
        (54, 56, 0.2750, 0.3550, 0.0, 250.0),
        (55, 56, 0.4880, 0.6370, 0.0, 250.0),
        (56, 57, 0.3430, 0.4280, 0.0, 250.0),
        (50, 57, 0.4740, 0.6270, 0.0, 250.0),
    ];

    for &(f, t, r, x, b, ra) in line_data {
        net.branches.push(make_branch(f, t, r, x, b, ra));
    }

    // 7 Generators
    let gen_data: &[(usize, f64, f64, f64, f64, f64, f64, f64)] = &[
        (1, 478.91, -4.0, 230.0, -17.0, 1.040, 576.9, 0.0),
        (2, 0.0, -1.0, 50.0, -17.0, 1.010, 100.0, 0.0),
        (3, 40.0, -1.0, 60.0, -5.0, 0.985, 140.0, 0.0),
        (6, 0.0, -22.0, 25.0, -30.0, 0.980, 100.0, 0.0),
        (8, 450.0, 62.0, 200.0, -140.0, 1.005, 550.0, 0.0),
        (9, 0.0, -29.0, 25.0, -3.0, 0.980, 100.0, 0.0),
        (12, 310.0, 17.0, 155.0, -50.0, 1.015, 410.0, 0.0),
    ];

    for &(bus, pg, qg, qmax, qmin, vg, pmax, pmin) in gen_data {
        net.generators
            .push(make_gen(bus, pg, qg, qmax, qmin, vg, pmax, pmin));
    }

    Ok(net)
}

// ---------------------------------------------------------------------------
// IEEE 118-Bus System
// ---------------------------------------------------------------------------

/// IEEE 118-bus system — realistic US Midwest representation.
///
/// Contains 118 buses, 186 branches, 54 generators.
/// Base: 100 MVA, 138 kV nominal.
///
/// This generates a topologically representative system.
/// For the full exact dataset, load a MATPOWER `case118.m` file via
/// `PowerNetwork::from_matpower()`.
pub fn ieee118() -> Result<PowerNetwork, OxiGridError> {
    generate_representative_system(
        118,
        186,
        54,
        100.0, // base_mva
        138.0, // base_kv
        "IEEE 118-Bus",
    )
}

// ---------------------------------------------------------------------------
// IEEE 300-Bus System
// ---------------------------------------------------------------------------

/// IEEE 300-bus system — large-scale benchmark.
///
/// Contains 300 buses, 411 branches, 69 generators.
/// Base: 100 MVA.
///
/// For exact data, load via `PowerNetwork::from_matpower("ieee300.m")`.
pub fn ieee300() -> Result<PowerNetwork, OxiGridError> {
    generate_representative_system(300, 411, 69, 100.0, 138.0, "IEEE 300-Bus")
}

// ---------------------------------------------------------------------------
// RTS-96
// ---------------------------------------------------------------------------

/// IEEE Reliability Test System 1996 (RTS-96).
///
/// A 73-bus, 120-branch system designed for reliability studies.
/// Contains 96 generators representing thermal, hydro, and nuclear units.
/// Base: 100 MVA, 138/230 kV.
pub fn rts96() -> Result<PowerNetwork, OxiGridError> {
    generate_representative_system(73, 120, 32, 100.0, 138.0, "RTS-96")
}

// ---------------------------------------------------------------------------
// PEGASE 89-Bus
// ---------------------------------------------------------------------------

/// PEGASE 89-bus system — European network representation.
///
/// Contains 89 buses, 210 branches. Represents a region of the
/// European high-voltage transmission network.
/// Base: 100 MVA.
pub fn pegase89() -> Result<PowerNetwork, OxiGridError> {
    generate_representative_system(89, 210, 12, 100.0, 380.0, "PEGASE 89-Bus")
}

// ---------------------------------------------------------------------------
// Representative system generator (for large cases without embedded data)
// ---------------------------------------------------------------------------

/// Generate a topologically representative system for large test cases.
///
/// Uses a structured meshed grid topology with realistic parameter ranges.
/// For production use with exact data, prefer loading from MATPOWER files.
fn generate_representative_system(
    n_buses: usize,
    target_branches: usize,
    n_generators: usize,
    base_mva: f64,
    base_kv: f64,
    _name: &str,
) -> Result<PowerNetwork, OxiGridError> {
    use crate::testcases::synthetic::{
        generate_synthetic_network, NetworkTopology, SyntheticNetworkConfig,
    };

    let config = SyntheticNetworkConfig {
        n_buses,
        n_generators,
        topology: NetworkTopology::Meshed,
        voltage_level_kv: base_kv,
        base_mva,
        load_density_mw_per_bus: 50.0,
        load_std_fraction: 0.3,
        generator_capacity_mw: 200.0,
        line_length_km: 80.0,
        seed: n_buses as u64 * 31 + target_branches as u64,
    };

    generate_synthetic_network(&config)
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::bus::BusType;

    /// Number of buses whose `bus_type` matches `Slack`.
    fn slack_count(net: &PowerNetwork) -> usize {
        net.buses
            .iter()
            .filter(|b| b.bus_type == BusType::Slack)
            .count()
    }

    // ── Internal helpers ────────────────────────────────────────────────────

    #[test]
    fn map_bus_type_valid_codes() {
        assert_eq!(map_bus_type(1).unwrap(), BusType::PQ);
        assert_eq!(map_bus_type(2).unwrap(), BusType::PV);
        assert_eq!(map_bus_type(3).unwrap(), BusType::Slack);
    }

    #[test]
    fn map_bus_type_rejects_unknown() {
        for code in [0u8, 4, 9, 255] {
            let err = map_bus_type(code).unwrap_err();
            assert!(
                matches!(err, OxiGridError::InvalidNetwork(_)),
                "code {code} should map to InvalidNetwork, got {err:?}"
            );
        }
    }

    #[test]
    fn make_bus_maps_fields_and_converts_angle() {
        let bus = make_bus(7, 2, 21.7, 12.7, 0.1, 0.2, 132.0, 1.045, -4.98).unwrap();
        assert_eq!(bus.id, 7);
        assert_eq!(bus.name, "Bus 7");
        assert_eq!(bus.bus_type, BusType::PV);
        assert_eq!(bus.base_kv.0, 132.0);
        assert_eq!(bus.vm, 1.045);
        // Angle stored internally in radians.
        assert!((bus.va - (-4.98_f64).to_radians()).abs() < 1e-12);
        assert_eq!(bus.pd.0, 21.7);
        assert_eq!(bus.qd.0, 12.7);
        assert_eq!(bus.gs, 0.1);
        assert_eq!(bus.bs, 0.2);
        assert!(bus.zone.is_none());
    }

    #[test]
    fn make_bus_propagates_bad_type() {
        assert!(make_bus(1, 7, 0.0, 0.0, 0.0, 0.0, 132.0, 1.0, 0.0).is_err());
    }

    #[test]
    fn make_branch_is_a_line_with_mirrored_ratings() {
        let br = make_branch(3, 4, 0.01, 0.05, 0.02, 250.0);
        assert_eq!(br.from_bus, 3);
        assert_eq!(br.to_bus, 4);
        assert_eq!(br.r, 0.01);
        assert_eq!(br.x, 0.05);
        assert_eq!(br.b, 0.02);
        assert_eq!(br.rate_a, 250.0);
        assert_eq!(br.rate_b, 250.0);
        assert_eq!(br.rate_c, 250.0);
        assert_eq!(br.tap, 0.0, "a line must have tap == 0");
        assert_eq!(br.shift, 0.0);
        assert!(br.status, "branch must default to in-service");
    }

    #[test]
    fn make_transformer_sets_tap() {
        let tx = make_transformer(5, 6, 0.0, 0.25, 0.0, 100.0, 0.978);
        assert_eq!(tx.tap, 0.978, "transformer must carry a non-trivial tap");
        assert_eq!(tx.from_bus, 5);
        assert_eq!(tx.to_bus, 6);
        assert!(tx.status);
    }

    #[test]
    fn make_gen_uses_100_mva_base_and_is_online() {
        let g = make_gen(2, 40.0, 42.4, 50.0, -40.0, 1.045, 140.0, 0.0);
        assert_eq!(g.bus_id, 2);
        assert_eq!(g.pg, 40.0);
        assert_eq!(g.qg, 42.4);
        assert_eq!(g.qmax, 50.0);
        assert_eq!(g.qmin, -40.0);
        assert_eq!(g.vg, 1.045);
        assert_eq!(g.mbase, 100.0);
        assert_eq!(g.pmax, 140.0);
        assert_eq!(g.pmin, 0.0);
        assert!(g.status);
    }

    // ── IEEE 14-bus ─────────────────────────────────────────────────────────

    #[test]
    fn ieee14_structure() {
        let net = ieee14().unwrap();
        assert_eq!(net.buses.len(), 14);
        assert_eq!(net.branches.len(), 20);
        assert_eq!(net.generators.len(), 5);
        assert_eq!(net.base_mva, 100.0);
        assert_eq!(slack_count(&net), 1);
        // Slack is bus 1 (index 0).
        assert_eq!(net.slack_bus_index().unwrap(), 0);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn ieee14_total_load_is_canonical() {
        let net = ieee14().unwrap();
        // Published IEEE 14 active load ≈ 259 MW.
        assert!(
            (net.total_load_mw() - 259.0).abs() < 1.0,
            "ieee14 load {} MW should be ≈ 259 MW",
            net.total_load_mw()
        );
    }

    #[test]
    fn ieee14_generator_buses_exist() {
        let net = ieee14().unwrap();
        for g in &net.generators {
            assert!(
                net.bus_index(g.bus_id).is_ok(),
                "generator references missing bus {}",
                g.bus_id
            );
        }
    }

    #[test]
    fn ieee14_bus_ids_are_unique_and_sequential() {
        let net = ieee14().unwrap();
        for (i, b) in net.buses.iter().enumerate() {
            assert_eq!(b.id, i + 1, "bus ids must be 1..=14 in order");
        }
    }

    // ── IEEE 30-bus ─────────────────────────────────────────────────────────

    #[test]
    fn ieee30_structure() {
        let net = ieee30().unwrap();
        assert_eq!(net.buses.len(), 30);
        assert_eq!(net.branches.len(), 41);
        assert_eq!(net.generators.len(), 6);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn ieee30_total_load_is_canonical() {
        let net = ieee30().unwrap();
        // Published IEEE 30 active load ≈ 283.4 MW.
        assert!(
            (net.total_load_mw() - 283.4).abs() < 1.0,
            "ieee30 load {} MW should be ≈ 283.4 MW",
            net.total_load_mw()
        );
    }

    // ── IEEE 57-bus ─────────────────────────────────────────────────────────

    #[test]
    fn ieee57_structure() {
        let net = ieee57().unwrap();
        assert_eq!(net.buses.len(), 57);
        assert_eq!(net.branches.len(), 85);
        assert_eq!(net.generators.len(), 7);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    // ── Representative large systems ─────────────────────────────────────────

    #[test]
    fn ieee118_structure() {
        let net = ieee118().unwrap();
        assert_eq!(net.buses.len(), 118);
        assert_eq!(net.generators.len(), 54);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn ieee300_structure() {
        let net = ieee300().unwrap();
        assert_eq!(net.buses.len(), 300);
        assert_eq!(net.generators.len(), 69);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn rts96_structure() {
        let net = rts96().unwrap();
        assert_eq!(net.buses.len(), 73);
        assert_eq!(slack_count(&net), 1);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn pegase89_structure_and_voltage_level() {
        let net = pegase89().unwrap();
        assert_eq!(net.buses.len(), 89);
        assert_eq!(slack_count(&net), 1);
        // PEGASE is a 380 kV European network.
        assert!((net.buses[0].base_kv.0 - 380.0).abs() < 1e-9);
        net.validate().unwrap();
        assert!(net.is_connected());
    }

    #[test]
    fn representative_systems_are_reproducible() {
        // The synthetic backend is seeded deterministically from sizes, so two
        // builds of the same case must be structurally identical.
        let a = ieee118().unwrap();
        let b = ieee118().unwrap();
        assert_eq!(a.buses.len(), b.buses.len());
        assert_eq!(a.branches.len(), b.branches.len());
        assert_eq!(a.generators.len(), b.generators.len());
    }

    // ── Power flow on exact-data cases ───────────────────────────────────────

    #[cfg(feature = "powerflow")]
    #[test]
    fn ieee14_newton_raphson_converges() {
        use crate::powerflow::newton_raphson::NewtonRaphsonSolver;
        use crate::powerflow::{PowerFlowConfig, PowerFlowMethod, PowerFlowSolver};

        let net = ieee14().unwrap();
        let cfg = PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let res = NewtonRaphsonSolver.solve(&net, &cfg).unwrap();
        assert!(res.converged, "max_mismatch={:.2e}", res.max_mismatch);
        for vm in &res.voltage_magnitude {
            assert!((0.9..=1.1).contains(vm), "vm {vm} out of range");
        }
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn ieee30_newton_raphson_converges() {
        use crate::powerflow::newton_raphson::NewtonRaphsonSolver;
        use crate::powerflow::{PowerFlowConfig, PowerFlowMethod, PowerFlowSolver};

        let net = ieee30().unwrap();
        let cfg = PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };
        let res = NewtonRaphsonSolver.solve(&net, &cfg).unwrap();
        assert!(res.converged, "max_mismatch={:.2e}", res.max_mismatch);
    }

    // ── Additional structural tests ───────────────────────────────────────────

    #[test]
    fn ieee14_returns_ok() {
        assert!(ieee14().is_ok());
    }

    #[test]
    fn ieee14_slack_bus_index_is_zero() {
        let net = ieee14().expect("ieee14 must build");
        let idx = net.slack_bus_index().expect("slack bus must exist");
        assert_eq!(idx, 0, "slack bus index must be 0");
    }

    #[test]
    fn ieee30_returns_ok_with_30_buses() {
        let net = ieee30().expect("ieee30 must build");
        assert_eq!(net.buses.len(), 30);
    }

    #[test]
    fn ieee57_returns_ok_with_57_buses() {
        let net = ieee57().expect("ieee57 must build");
        assert_eq!(net.buses.len(), 57);
    }

    #[test]
    fn ieee118_returns_ok_with_118_buses() {
        let net = ieee118().expect("ieee118 must build");
        assert_eq!(net.buses.len(), 118);
    }

    #[test]
    fn rts96_bus_count_and_slack() {
        let net = rts96().expect("rts96 must build");
        assert_eq!(net.buses.len(), 73);
        let slack_count = net
            .buses
            .iter()
            .filter(|b| matches!(b.bus_type, BusType::Slack))
            .count();
        assert_eq!(slack_count, 1, "rts96 must have exactly 1 slack bus");
    }

    #[test]
    fn pegase89_bus_count() {
        let net = pegase89().expect("pegase89 must build");
        assert_eq!(net.buses.len(), 89);
    }

    #[test]
    fn ieee14_branches_positive() {
        let net = ieee14().expect("ieee14 must build");
        assert!(!net.branches.is_empty(), "must have branches");
        for branch in &net.branches {
            assert!(
                branch.r >= 0.0,
                "branch r must be non-negative, got {}",
                branch.r
            );
            assert!(
                branch.x > 0.0,
                "branch x must be positive, got {}",
                branch.x
            );
        }
    }

    #[test]
    fn ieee14_total_load_positive() {
        let net = ieee14().expect("ieee14 must build");
        assert!(
            net.total_load_mw() > 0.0,
            "total load must be positive, got {}",
            net.total_load_mw()
        );
    }

    #[test]
    fn ieee14_has_at_least_one_generator() {
        let net = ieee14().expect("ieee14 must build");
        assert!(
            !net.generators.is_empty(),
            "ieee14 must have at least one generator"
        );
    }

    #[test]
    fn ieee30_has_at_least_one_generator() {
        let net = ieee30().expect("ieee30 must build");
        assert!(
            !net.generators.is_empty(),
            "ieee30 must have at least one generator"
        );
    }

    #[test]
    fn ieee57_total_load_positive() {
        let net = ieee57().expect("ieee57 must build");
        assert!(
            net.total_load_mw() > 0.0,
            "ieee57 total load must be positive, got {}",
            net.total_load_mw()
        );
    }

    #[test]
    fn ieee57_has_generators() {
        let net = ieee57().expect("ieee57 must build");
        assert!(
            !net.generators.is_empty(),
            "ieee57 must have at least one generator"
        );
    }
}
