/// OxiGrid–pandapower JSON format parser.
///
/// Parses a simplified JSON format compatible with pandapower network exports.
/// Supports the core network elements needed for power flow analysis.
///
/// # Format
/// ```json
/// {
///   "version": "1.0",
///   "baseMVA": 100.0,
///   "f_hz": 60.0,
///   "bus": [
///     {"id": 0, "type": "slack", "vn_kv": 138.0, "vm_pu": 1.06, "va_deg": 0.0}
///   ],
///   "gen": [
///     {"bus": 0, "p_mw": 50.0, "vm_pu": 1.06, "qmax_mvar": 200.0, "qmin_mvar": -50.0,
///      "pmax_mw": 200.0, "pmin_mw": 0.0, "in_service": true}
///   ],
///   "load": [
///     {"bus": 1, "p_mw": 21.7, "q_mvar": 12.7, "in_service": true}
///   ],
///   "line": [
///     {"from_bus": 0, "to_bus": 1, "r_pu": 0.01938, "x_pu": 0.05917,
///      "b_pu": 0.0528, "rate_mva": 0.0, "in_service": true}
///   ],
///   "trafo": [
///     {"from_bus": 4, "to_bus": 7, "r_pu": 0.0, "x_pu": 0.20912,
///      "b_pu": 0.0, "tap": 0.978, "shift_deg": 0.0, "in_service": true}
///   ]
/// }
/// ```
///
/// Bus types: `"slack"` / `"ref"`, `"pv"` / `"gen"`, `"pq"` / `"load"` / `"b"`.
use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::units::{Power, ReactivePower, Voltage};
use serde::{Deserialize, Serialize};
use serde_json::Value;

// ── Internal JSON structs ────────────────────────────────────────────────────

#[derive(Debug, Deserialize, Serialize)]
struct PpBus {
    id: usize,
    #[serde(rename = "type")]
    bus_type: String,
    vn_kv: f64,
    #[serde(default = "default_one")]
    vm_pu: f64,
    #[serde(default)]
    va_deg: f64,
    #[serde(default = "default_one")]
    vmax_pu: f64,
    #[serde(default = "default_094")]
    vmin_pu: f64,
    #[serde(default = "default_true")]
    in_service: bool,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    zone: Option<u32>,
}

#[derive(Debug, Deserialize, Serialize)]
struct PpGen {
    bus: usize,
    #[serde(default)]
    p_mw: f64,
    #[serde(default = "default_one")]
    vm_pu: f64,
    #[serde(default = "default_9999")]
    qmax_mvar: f64,
    #[serde(default = "default_neg9999")]
    qmin_mvar: f64,
    #[serde(default = "default_9999")]
    pmax_mw: f64,
    #[serde(default)]
    pmin_mw: f64,
    #[serde(default = "default_true")]
    in_service: bool,
}

#[derive(Debug, Deserialize, Serialize)]
struct PpLoad {
    bus: usize,
    #[serde(default)]
    p_mw: f64,
    #[serde(default)]
    q_mvar: f64,
    #[serde(default = "default_true")]
    in_service: bool,
}

#[derive(Debug, Deserialize, Serialize)]
struct PpLine {
    from_bus: usize,
    to_bus: usize,
    r_pu: f64,
    x_pu: f64,
    #[serde(default)]
    b_pu: f64,
    #[serde(default)]
    rate_mva: f64,
    #[serde(default = "default_true")]
    in_service: bool,
}

#[derive(Debug, Deserialize, Serialize)]
struct PpTrafo {
    from_bus: usize,
    to_bus: usize,
    #[serde(default)]
    r_pu: f64,
    x_pu: f64,
    #[serde(default)]
    b_pu: f64,
    #[serde(default = "default_one")]
    tap: f64,
    #[serde(default)]
    shift_deg: f64,
    #[serde(default)]
    rate_mva: f64,
    #[serde(default = "default_true")]
    in_service: bool,
}

#[derive(Debug, Deserialize, Serialize)]
struct PpExtGrid {
    bus: usize,
    #[serde(default = "default_one")]
    vm_pu: f64,
    #[serde(default)]
    va_deg: f64,
    #[serde(default = "default_9999")]
    max_p_mw: f64,
    #[serde(default = "default_neg9999")]
    min_p_mw: f64,
    #[serde(default = "default_9999")]
    max_q_mvar: f64,
    #[serde(default = "default_neg9999")]
    min_q_mvar: f64,
    #[serde(default = "default_true")]
    in_service: bool,
}

fn default_one() -> f64 {
    1.0
}
fn default_094() -> f64 {
    0.94
}
fn default_9999() -> f64 {
    9999.0
}
fn default_neg9999() -> f64 {
    -9999.0
}
fn default_true() -> bool {
    true
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Parse an OxiGrid–pandapower JSON file.
pub fn parse_pandapower_file(path: &str) -> Result<PowerNetwork> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| OxiGridError::ParseError(format!("Failed to read {path}: {e}")))?;
    parse_pandapower_string(&content)
}

/// Parse an OxiGrid–pandapower JSON string.
pub fn parse_pandapower_string(content: &str) -> Result<PowerNetwork> {
    let v: Value = serde_json::from_str(content)
        .map_err(|e| OxiGridError::ParseError(format!("Invalid JSON: {e}")))?;

    let base_mva = v["baseMVA"].as_f64().unwrap_or(100.0);

    // ── Buses ────────────────────────────────────────────────────────────────
    let pp_buses: Vec<PpBus> = serde_json::from_value(v["bus"].clone().into_array_or_empty())
        .map_err(|e| OxiGridError::ParseError(format!("bus parse error: {e}")))?;

    let mut buses: Vec<Bus> = Vec::with_capacity(pp_buses.len());
    for pb in &pp_buses {
        if !pb.in_service {
            continue;
        }
        let bus_type = match pb.bus_type.to_lowercase().as_str() {
            "slack" | "ref" | "3" => BusType::Slack,
            "pv" | "gen" | "2" => BusType::PV,
            _ => BusType::PQ,
        };
        buses.push(Bus {
            id: pb.id,
            name: pb.name.clone().unwrap_or_else(|| format!("Bus {}", pb.id)),
            bus_type,
            base_kv: Voltage(pb.vn_kv),
            vm: pb.vm_pu,
            va: pb.va_deg.to_radians(),
            pd: Power(0.0),
            qd: ReactivePower(0.0),
            gs: 0.0,
            bs: 0.0,
            zone: pb.zone,
        });
    }

    // ── Loads (add to bus pd/qd) ─────────────────────────────────────────────
    let pp_loads: Vec<PpLoad> = serde_json::from_value(v["load"].clone().into_array_or_empty())
        .map_err(|e| OxiGridError::ParseError(format!("load parse error: {e}")))?;
    for pl in &pp_loads {
        if !pl.in_service {
            continue;
        }
        if let Some(bus) = buses.iter_mut().find(|b| b.id == pl.bus) {
            bus.pd.0 += pl.p_mw;
            bus.qd.0 += pl.q_mvar;
        }
    }

    // ── External grid (slack bus generator) ──────────────────────────────────
    let pp_ext: Vec<PpExtGrid> =
        serde_json::from_value(v["ext_grid"].clone().into_array_or_empty())
            .map_err(|e| OxiGridError::ParseError(format!("ext_grid parse error: {e}")))?;

    let mut generators: Vec<Generator> = Vec::new();
    for eg in &pp_ext {
        if !eg.in_service {
            continue;
        }
        // Mark bus as slack
        if let Some(bus) = buses.iter_mut().find(|b| b.id == eg.bus) {
            bus.bus_type = BusType::Slack;
            bus.vm = eg.vm_pu;
            bus.va = eg.va_deg.to_radians();
        }
        generators.push(Generator {
            bus_id: eg.bus,
            pg: 0.0,
            qg: 0.0,
            qmax: eg.max_q_mvar,
            qmin: eg.min_q_mvar,
            vg: eg.vm_pu,
            mbase: base_mva,
            status: true,
            pmax: eg.max_p_mw,
            pmin: eg.min_p_mw,
        });
    }

    // ── Generators ───────────────────────────────────────────────────────────
    let pp_gens: Vec<PpGen> = serde_json::from_value(v["gen"].clone().into_array_or_empty())
        .map_err(|e| OxiGridError::ParseError(format!("gen parse error: {e}")))?;
    for pg in &pp_gens {
        if !pg.in_service {
            continue;
        }
        // Mark bus as PV if not already Slack
        if let Some(bus) = buses.iter_mut().find(|b| b.id == pg.bus) {
            if bus.bus_type == BusType::PQ {
                bus.bus_type = BusType::PV;
            }
            bus.vm = pg.vm_pu;
        }
        generators.push(Generator {
            bus_id: pg.bus,
            pg: pg.p_mw,
            qg: 0.0,
            qmax: pg.qmax_mvar,
            qmin: pg.qmin_mvar,
            vg: pg.vm_pu,
            mbase: base_mva,
            status: true,
            pmax: pg.pmax_mw,
            pmin: pg.pmin_mw,
        });
    }

    // ── Lines ────────────────────────────────────────────────────────────────
    let pp_lines: Vec<PpLine> = serde_json::from_value(v["line"].clone().into_array_or_empty())
        .map_err(|e| OxiGridError::ParseError(format!("line parse error: {e}")))?;
    let mut branches: Vec<Branch> = Vec::with_capacity(pp_lines.len());
    for pl in &pp_lines {
        if !pl.in_service {
            continue;
        }
        branches.push(Branch {
            from_bus: pl.from_bus,
            to_bus: pl.to_bus,
            r: pl.r_pu,
            x: if pl.x_pu.abs() < 1e-10 { 1e-6 } else { pl.x_pu },
            b: pl.b_pu,
            rate_a: pl.rate_mva,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
    }

    // ── Transformers ─────────────────────────────────────────────────────────
    let pp_trafos: Vec<PpTrafo> = serde_json::from_value(v["trafo"].clone().into_array_or_empty())
        .map_err(|e| OxiGridError::ParseError(format!("trafo parse error: {e}")))?;
    for pt in &pp_trafos {
        if !pt.in_service {
            continue;
        }
        branches.push(Branch {
            from_bus: pt.from_bus,
            to_bus: pt.to_bus,
            r: pt.r_pu,
            x: if pt.x_pu.abs() < 1e-10 { 1e-6 } else { pt.x_pu },
            b: pt.b_pu,
            rate_a: pt.rate_mva,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: pt.tap,
            shift: pt.shift_deg.to_radians(),
            status: true,
        });
    }

    if buses.is_empty() {
        return Err(OxiGridError::ParseError(
            "No bus data found in pandapower JSON".into(),
        ));
    }

    let mut network = PowerNetwork::new(base_mva);
    network.buses = buses;
    network.branches = branches;
    network.generators = generators;
    network.validate()?;
    Ok(network)
}

/// Serialise a PowerNetwork to OxiGrid–pandapower JSON string.
pub fn to_pandapower_json(network: &PowerNetwork) -> Result<String> {
    let buses: Vec<Value> = network
        .buses
        .iter()
        .map(|b| {
            let btype = match b.bus_type {
                BusType::Slack => "slack",
                BusType::PV => "pv",
                BusType::PQ => "pq",
            };
            serde_json::json!({
                "id": b.id,
                "type": btype,
                "vn_kv": b.base_kv.0,
                "vm_pu": b.vm,
                "va_deg": b.va.to_degrees(),
                "name": b.name,
                "in_service": true
            })
        })
        .collect();

    let gens: Vec<Value> = network
        .generators
        .iter()
        .map(|g| {
            serde_json::json!({
                "bus": g.bus_id,
                "p_mw": g.pg,
                "vm_pu": g.vg,
                "qmax_mvar": g.qmax,
                "qmin_mvar": g.qmin,
                "pmax_mw": g.pmax,
                "pmin_mw": g.pmin,
                "in_service": g.status
            })
        })
        .collect();

    let loads: Vec<Value> = network
        .buses
        .iter()
        .filter(|b| b.pd.0 != 0.0 || b.qd.0 != 0.0)
        .map(|b| {
            serde_json::json!({
                "bus": b.id,
                "p_mw": b.pd.0,
                "q_mvar": b.qd.0,
                "in_service": true
            })
        })
        .collect();

    let lines: Vec<Value> = network
        .branches
        .iter()
        .filter(|br| br.tap == 0.0)
        .map(|br| {
            serde_json::json!({
                "from_bus": br.from_bus,
                "to_bus": br.to_bus,
                "r_pu": br.r,
                "x_pu": br.x,
                "b_pu": br.b,
                "rate_mva": br.rate_a,
                "in_service": br.status
            })
        })
        .collect();

    let trafos: Vec<Value> = network
        .branches
        .iter()
        .filter(|br| br.tap != 0.0)
        .map(|br| {
            serde_json::json!({
                "from_bus": br.from_bus,
                "to_bus": br.to_bus,
                "r_pu": br.r,
                "x_pu": br.x,
                "b_pu": br.b,
                "tap": br.tap,
                "shift_deg": br.shift.to_degrees(),
                "rate_mva": br.rate_a,
                "in_service": br.status
            })
        })
        .collect();

    let v = serde_json::json!({
        "version": "1.0",
        "baseMVA": network.base_mva,
        "bus": buses,
        "gen": gens,
        "load": loads,
        "line": lines,
        "trafo": trafos
    });

    serde_json::to_string_pretty(&v)
        .map_err(|e| OxiGridError::ParseError(format!("JSON serialisation error: {e}")))
}

// ── Helper trait ─────────────────────────────────────────────────────────────

trait IntoArrayOrEmpty {
    fn into_array_or_empty(self) -> Value;
}

impl IntoArrayOrEmpty for Value {
    fn into_array_or_empty(self) -> Value {
        if self.is_array() {
            self
        } else {
            Value::Array(vec![])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_PP: &str = r#"{
        "version": "1.0",
        "baseMVA": 100.0,
        "bus": [
            {"id": 1, "type": "slack", "vn_kv": 138.0, "vm_pu": 1.06},
            {"id": 2, "type": "pv",    "vn_kv": 138.0, "vm_pu": 1.045},
            {"id": 3, "type": "pq",    "vn_kv": 138.0}
        ],
        "gen": [
            {"bus": 2, "p_mw": 40.0, "vm_pu": 1.045, "qmax_mvar": 300.0, "qmin_mvar": -300.0}
        ],
        "load": [
            {"bus": 3, "p_mw": 94.2, "q_mvar": 19.0}
        ],
        "line": [
            {"from_bus": 1, "to_bus": 2, "r_pu": 0.01938, "x_pu": 0.05917, "b_pu": 0.0528},
            {"from_bus": 1, "to_bus": 3, "r_pu": 0.05403, "x_pu": 0.22304, "b_pu": 0.0492}
        ],
        "trafo": []
    }"#;

    #[test]
    fn test_parse_sample() {
        let net = parse_pandapower_string(SAMPLE_PP).unwrap();
        assert_eq!(net.bus_count(), 3);
        assert_eq!(net.branch_count(), 2);
        assert_eq!(net.base_mva, 100.0);
    }

    #[test]
    fn test_bus_types() {
        let net = parse_pandapower_string(SAMPLE_PP).unwrap();
        assert_eq!(net.buses[0].bus_type, BusType::Slack);
        assert_eq!(net.buses[1].bus_type, BusType::PV);
        assert_eq!(net.buses[2].bus_type, BusType::PQ);
    }

    #[test]
    fn test_load_absorbed_into_bus() {
        let net = parse_pandapower_string(SAMPLE_PP).unwrap();
        let bus3 = net.buses.iter().find(|b| b.id == 3).unwrap();
        assert!((bus3.pd.0 - 94.2).abs() < 1e-6);
        assert!((bus3.qd.0 - 19.0).abs() < 1e-6);
    }

    #[test]
    fn test_branch_impedance() {
        let net = parse_pandapower_string(SAMPLE_PP).unwrap();
        let br = &net.branches[0];
        assert!((br.r - 0.01938).abs() < 1e-8);
        assert!((br.x - 0.05917).abs() < 1e-8);
    }

    #[test]
    fn test_roundtrip_json() {
        let net = parse_pandapower_string(SAMPLE_PP).unwrap();
        let json_str = to_pandapower_json(&net).unwrap();
        let net2 = parse_pandapower_string(&json_str).unwrap();
        assert_eq!(net.bus_count(), net2.bus_count());
        assert_eq!(net.branch_count(), net2.branch_count());
    }

    #[test]
    fn test_empty_arrays_ok() {
        let minimal = r#"{
            "baseMVA": 100.0,
            "bus": [{"id": 0, "type": "slack", "vn_kv": 138.0}],
            "gen": [{"bus": 0, "p_mw": 100.0, "vm_pu": 1.0}],
            "load": [],
            "line": [],
            "trafo": []
        }"#;
        let net = parse_pandapower_string(minimal).unwrap();
        assert_eq!(net.bus_count(), 1);
        assert_eq!(net.branch_count(), 0);
    }

    #[test]
    fn test_missing_optional_fields() {
        // JSON with minimal fields — defaults should kick in
        let json = r#"{
            "baseMVA": 100.0,
            "bus": [
                {"id": 1, "type": "slack", "vn_kv": 138.0},
                {"id": 2, "type": "pq",    "vn_kv": 138.0}
            ],
            "gen": [{"bus": 1, "p_mw": 50.0}],
            "load": [{"bus": 2, "p_mw": 50.0}],
            "line": [{"from_bus": 1, "to_bus": 2, "r_pu": 0.01, "x_pu": 0.05}]
        }"#;
        let net = parse_pandapower_string(json).unwrap();
        assert_eq!(net.bus_count(), 2);
        assert_eq!(net.branch_count(), 1);
    }
}
