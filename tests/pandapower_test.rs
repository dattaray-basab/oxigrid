#![cfg(feature = "powerflow")]
use oxigrid::network::bus::BusType;
use oxigrid::network::PowerNetwork;

// ── Minimal 2-bus JSON ────────────────────────────────────────────────────────

fn two_bus_json() -> &'static str {
    r#"{
        "version": "1.0",
        "baseMVA": 100.0,
        "f_hz": 60.0,
        "bus": [
            {"id": 0, "type": "slack", "vn_kv": 138.0, "vm_pu": 1.05, "va_deg": 0.0},
            {"id": 1, "type": "pq",    "vn_kv": 138.0, "vm_pu": 1.0,  "va_deg": 0.0}
        ],
        "gen": [
            {"bus": 0, "p_mw": 100.0, "vm_pu": 1.05, "qmax_mvar": 200.0,
             "qmin_mvar": -50.0, "pmax_mw": 300.0, "pmin_mw": 0.0, "in_service": true}
        ],
        "load": [
            {"bus": 1, "p_mw": 50.0, "q_mvar": 20.0, "in_service": true}
        ],
        "line": [
            {"from_bus": 0, "to_bus": 1, "r_pu": 0.01938, "x_pu": 0.05917,
             "b_pu": 0.0528, "rate_mva": 0.0, "in_service": true}
        ],
        "trafo": [],
        "ext_grid": []
    }"#
}

fn ieee14_like_json() -> &'static str {
    r#"{
        "version": "1.0",
        "baseMVA": 100.0,
        "f_hz": 60.0,
        "bus": [
            {"id": 0, "type": "slack", "vn_kv": 138.0, "vm_pu": 1.06},
            {"id": 1, "type": "pv",    "vn_kv": 138.0, "vm_pu": 1.045},
            {"id": 2, "type": "pv",    "vn_kv": 138.0, "vm_pu": 1.01},
            {"id": 3, "type": "pq",    "vn_kv": 138.0},
            {"id": 4, "type": "pq",    "vn_kv": 138.0}
        ],
        "gen": [
            {"bus": 0, "p_mw": 232.4, "vm_pu": 1.06, "qmax_mvar": 10.0, "qmin_mvar": 0.0,
             "pmax_mw": 500.0, "pmin_mw": 0.0, "in_service": true},
            {"bus": 1, "p_mw": 40.0,  "vm_pu": 1.045, "qmax_mvar": 50.0, "qmin_mvar": -40.0,
             "pmax_mw": 100.0, "pmin_mw": 0.0, "in_service": true},
            {"bus": 2, "p_mw": 0.0,   "vm_pu": 1.01, "qmax_mvar": 40.0, "qmin_mvar": 0.0,
             "pmax_mw": 0.0, "pmin_mw": 0.0, "in_service": true}
        ],
        "load": [
            {"bus": 1, "p_mw": 21.7, "q_mvar": 12.7, "in_service": true},
            {"bus": 2, "p_mw": 94.2, "q_mvar": 19.0, "in_service": true},
            {"bus": 3, "p_mw": 47.8, "q_mvar": -3.9, "in_service": true},
            {"bus": 4, "p_mw": 7.6,  "q_mvar": 1.6,  "in_service": true}
        ],
        "line": [
            {"from_bus": 0, "to_bus": 1, "r_pu": 0.01938, "x_pu": 0.05917, "b_pu": 0.0528},
            {"from_bus": 0, "to_bus": 3, "r_pu": 0.05403, "x_pu": 0.22304, "b_pu": 0.0492},
            {"from_bus": 1, "to_bus": 2, "r_pu": 0.04699, "x_pu": 0.19797, "b_pu": 0.0438},
            {"from_bus": 1, "to_bus": 3, "r_pu": 0.05811, "x_pu": 0.17632, "b_pu": 0.0340},
            {"from_bus": 2, "to_bus": 4, "r_pu": 0.05695, "x_pu": 0.17388, "b_pu": 0.0346}
        ],
        "trafo": [
            {"from_bus": 3, "to_bus": 4, "x_pu": 0.20912, "tap": 0.978, "in_service": true}
        ],
        "ext_grid": []
    }"#
}

// ── Parse tests ───────────────────────────────────────────────────────────────

#[test]
fn test_parse_two_bus_json() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).expect("Should parse 2-bus JSON");
    assert_eq!(net.bus_count(), 2);
}

#[test]
fn test_parse_bus_types() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let slack = net.buses.iter().find(|b| b.id == 0).unwrap();
    let pq = net.buses.iter().find(|b| b.id == 1).unwrap();
    assert_eq!(slack.bus_type, BusType::Slack);
    assert_eq!(pq.bus_type, BusType::PQ);
}

#[test]
fn test_parse_generators() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    assert_eq!(net.generators.len(), 1);
    let gen = &net.generators[0];
    assert_eq!(gen.bus_id, 0);
    // pandapower parser stores pg in raw MW (not per-unit)
    assert!((gen.pg - 100.0).abs() < 1e-9, "pg = 100 MW, got {}", gen.pg);
    assert!((gen.pmax - 300.0).abs() < 1e-9, "pmax = 300 MW");
}

#[test]
fn test_parse_loads_applied_to_buses() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let bus1 = net.buses.iter().find(|b| b.id == 1).unwrap();
    assert!((bus1.pd.0 - 50.0).abs() < 1e-9, "PD = 50 MW");
    assert!((bus1.qd.0 - 20.0).abs() < 1e-9, "QD = 20 MVAr");
}

#[test]
fn test_parse_branches() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    assert_eq!(net.branch_count(), 1);
    let br = &net.branches[0];
    assert!((br.r - 0.01938).abs() < 1e-9);
    assert!((br.x - 0.05917).abs() < 1e-9);
}

#[test]
fn test_parse_base_mva() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    assert!((net.base_mva - 100.0).abs() < 1e-9);
}

#[test]
fn test_parse_slack_voltage() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let slack = net
        .buses
        .iter()
        .find(|b| b.bus_type == BusType::Slack)
        .unwrap();
    assert!((slack.vm - 1.05).abs() < 1e-9, "Slack vm = 1.05 p.u.");
}

// ── Multi-bus / transformer tests ─────────────────────────────────────────────

#[test]
fn test_parse_five_bus_count() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    assert_eq!(net.bus_count(), 5);
}

#[test]
fn test_parse_five_bus_generators() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    assert_eq!(net.generators.len(), 3);
}

#[test]
fn test_parse_transformer_as_branch() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    // 5 lines + 1 trafo = 6 branches total
    assert_eq!(net.branch_count(), 6);
}

#[test]
fn test_parse_transformer_tap() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    // Transformer has tap = 0.978
    let trafo_branch = net.branches.iter().find(|b| (b.tap - 0.978).abs() < 0.01);
    assert!(
        trafo_branch.is_some(),
        "Transformer with tap=0.978 should be present"
    );
}

#[test]
fn test_parse_pv_bus_type() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    let pv_buses: Vec<_> = net
        .buses
        .iter()
        .filter(|b| b.bus_type == BusType::PV)
        .collect();
    assert_eq!(pv_buses.len(), 2, "Should have 2 PV buses");
}

#[test]
fn test_has_slack_bus() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let slack = net.slack_bus_index();
    assert!(slack.is_ok(), "Should have exactly one slack bus");
}

// ── Validate parsed network ───────────────────────────────────────────────────

#[test]
fn test_validate_two_bus() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    assert!(net.validate().is_ok(), "2-bus network should be valid");
}

#[test]
fn test_validate_five_bus() {
    let net = PowerNetwork::from_pandapower_str(ieee14_like_json()).unwrap();
    assert!(net.validate().is_ok(), "5-bus network should be valid");
}

// ── Power flow on parsed network ──────────────────────────────────────────────

#[test]
fn test_power_flow_two_bus_converges() {
    use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = net.solve_powerflow(&config);
    assert!(
        result.is_ok(),
        "Power flow should succeed: {:?}",
        result.err()
    );
    let r = result.unwrap();
    assert!(r.converged, "Power flow should converge");
}

#[test]
fn test_admittance_matrix_two_bus() {
    let net = PowerNetwork::from_pandapower_str(two_bus_json()).unwrap();
    let ybus = net.admittance_matrix();
    assert!(ybus.is_ok(), "Y-bus construction should succeed");
    let y = ybus.unwrap();
    assert_eq!(y.rows(), 2);
    assert_eq!(y.cols(), 2);
}

// ── Out-of-service elements ───────────────────────────────────────────────────

#[test]
fn test_out_of_service_bus_excluded() {
    let json = r#"{
        "baseMVA": 100.0,
        "bus": [
            {"id": 0, "type": "slack", "vn_kv": 138.0, "in_service": true},
            {"id": 1, "type": "pq",    "vn_kv": 138.0, "in_service": false}
        ],
        "gen": [], "load": [], "line": [], "trafo": [], "ext_grid": []
    }"#;
    let net = PowerNetwork::from_pandapower_str(json).unwrap();
    assert_eq!(net.bus_count(), 1, "Out-of-service bus should be excluded");
}

#[test]
fn test_out_of_service_load_excluded() {
    let json = r#"{
        "baseMVA": 100.0,
        "bus": [
            {"id": 0, "type": "slack", "vn_kv": 138.0},
            {"id": 1, "type": "pq",    "vn_kv": 138.0}
        ],
        "gen": [],
        "load": [
            {"bus": 1, "p_mw": 100.0, "q_mvar": 50.0, "in_service": false}
        ],
        "line": [
            {"from_bus": 0, "to_bus": 1, "r_pu": 0.01, "x_pu": 0.1, "in_service": true}
        ],
        "trafo": [], "ext_grid": []
    }"#;
    let net = PowerNetwork::from_pandapower_str(json).unwrap();
    let bus1 = net.buses.iter().find(|b| b.id == 1).unwrap();
    assert!(
        (bus1.pd.0).abs() < 1e-9,
        "Out-of-service load should not affect bus PD"
    );
}

// ── Invalid JSON ──────────────────────────────────────────────────────────────

#[test]
fn test_invalid_json_returns_error() {
    let result = PowerNetwork::from_pandapower_str("not valid json");
    assert!(result.is_err(), "Invalid JSON should return error");
}

#[test]
fn test_missing_bus_field_uses_defaults() {
    // Minimal JSON with only required fields
    let json = r#"{
        "baseMVA": 100.0,
        "bus": [{"id": 0, "type": "slack", "vn_kv": 138.0}],
        "gen": [], "load": [], "line": [], "trafo": [], "ext_grid": []
    }"#;
    let net = PowerNetwork::from_pandapower_str(json);
    assert!(
        net.is_ok(),
        "Minimal JSON with defaults should parse: {:?}",
        net.err()
    );
    assert_eq!(net.unwrap().bus_count(), 1);
}

// ── Type aliases / alternate bus types ───────────────────────────────────────

#[test]
fn test_bus_type_aliases() {
    let json = r#"{
        "baseMVA": 100.0,
        "bus": [
            {"id": 0, "type": "ref",  "vn_kv": 138.0},
            {"id": 1, "type": "gen",  "vn_kv": 138.0},
            {"id": 2, "type": "load", "vn_kv": 138.0},
            {"id": 3, "type": "b",    "vn_kv": 138.0}
        ],
        "gen": [], "load": [], "line": [], "trafo": [], "ext_grid": []
    }"#;
    let net = PowerNetwork::from_pandapower_str(json).unwrap();
    assert_eq!(
        net.buses.iter().find(|b| b.id == 0).unwrap().bus_type,
        BusType::Slack
    );
    assert_eq!(
        net.buses.iter().find(|b| b.id == 1).unwrap().bus_type,
        BusType::PV
    );
    assert_eq!(
        net.buses.iter().find(|b| b.id == 2).unwrap().bus_type,
        BusType::PQ
    );
    assert_eq!(
        net.buses.iter().find(|b| b.id == 3).unwrap().bus_type,
        BusType::PQ
    );
}
