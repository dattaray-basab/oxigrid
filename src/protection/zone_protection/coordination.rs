use super::*;

/// Helper: build a zone coordinator with two relays on the same line.
fn make_coordinator(z_line: f64, z_adj: f64) -> ZoneCoordinator {
    let dummy_map = ProtectionZoneMap::new("TestSubstation");
    let coordinator = ZoneCoordinator::new(dummy_map, DEFAULT_CTI_S);
    let _ = (z_line, z_adj); // used in tests directly
    coordinator
}

/// Helper: create a relay with auto-set zones.
fn make_relay_with_zones(relay_id: usize, z_line: f64, z_adj: f64) -> DistanceRelay {
    let coord = make_coordinator(z_line, z_adj);
    let mut relay = DistanceRelay::new(relay_id, 0, relay_id, z_line);
    let zones = coord.auto_set_zones(z_line, z_adj);
    for z in zones {
        relay.add_zone(z);
    }
    relay
}

// ── Zone reach tests ──────────────────────────────────────────────────────

#[test]
fn test_zone1_reach() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z1 = zones
        .iter()
        .find(|z| z.zone_num == 1)
        .expect("Zone 1 must exist");
    let expected = 0.8 * 0.1;
    assert!(
        (z1.reach_pu - expected).abs() < 1e-10,
        "Zone 1 reach={} expected={}",
        z1.reach_pu,
        expected
    );
}

#[test]
fn test_zone2_reach_gt_zone1() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z1 = zones
        .iter()
        .find(|z| z.zone_num == 1)
        .expect("Zone 1")
        .reach_pu;
    let z2 = zones
        .iter()
        .find(|z| z.zone_num == 2)
        .expect("Zone 2")
        .reach_pu;
    assert!(
        z2 > z1,
        "Zone 2 reach {} must exceed Zone 1 reach {}",
        z2,
        z1
    );
}

#[test]
fn test_zone3_reach_gt_zone2() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z2 = zones
        .iter()
        .find(|z| z.zone_num == 2)
        .expect("Zone 2")
        .reach_pu;
    let z3 = zones
        .iter()
        .find(|z| z.zone_num == 3)
        .expect("Zone 3")
        .reach_pu;
    assert!(
        z3 > z2,
        "Zone 3 reach {} must exceed Zone 2 reach {}",
        z3,
        z2
    );
}

#[test]
fn test_zone1_delay_zero() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z1 = zones.iter().find(|z| z.zone_num == 1).expect("Zone 1");
    assert_eq!(z1.time_delay_s, 0.0, "Zone 1 must have zero time delay");
}

#[test]
fn test_zone2_delay_gte_03() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z2 = zones.iter().find(|z| z.zone_num == 2).expect("Zone 2");
    assert!(
        z2.time_delay_s >= 0.3,
        "Zone 2 delay {} must be >= 0.3 s",
        z2.time_delay_s
    );
}

#[test]
fn test_zone3_delay_gte_06() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let z3 = zones.iter().find(|z| z.zone_num == 3).expect("Zone 3");
    assert!(
        z3.time_delay_s >= 0.6,
        "Zone 3 delay {} must be >= 0.6 s",
        z3.time_delay_s
    );
}

// ── Mho characteristic tests ──────────────────────────────────────────────

#[test]
fn test_mho_fault_at_50pct_inside_zone1() {
    // Z_line = 0.1 pu, Zone 1 reach = 0.08 pu
    // Fault at 50% of line: Z_app = 0.5 * 0.1 = 0.05 pu (well within Zone 1)
    let z_line = 0.1_f64;
    let z1_reach = 0.8 * z_line; // 0.08
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    let d = 0.5_f64;
    let r_app = d * z_line * angle_rad.cos();
    let x_app = d * z_line * angle_rad.sin();
    assert!(
        is_inside_mho((r_app, x_app), z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "Fault at 50% should be inside Zone 1 Mho circle"
    );
}

#[test]
fn test_mho_fault_beyond_zone1_not_in_zone1() {
    // Fault at 95% of line, Zone 1 reach = 80%: should NOT be in Zone 1
    let z_line = 0.1_f64;
    let z1_reach = 0.8 * z_line;
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    let d = 0.95_f64;
    let r_app = d * z_line * angle_rad.cos();
    let x_app = d * z_line * angle_rad.sin();
    assert!(
        !is_inside_mho((r_app, x_app), z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "Fault at 95% should NOT be inside Zone 1 Mho circle"
    );
}

#[test]
fn test_mho_fault_at_90pct_in_zone2_not_zone1() {
    let z_line = 0.1_f64;
    let z_adj = 0.08_f64;
    let relay = make_relay_with_zones(1, z_line, z_adj);
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    let d = 0.90_f64;
    let r_app = d * z_line * angle_rad.cos();
    let x_app = d * z_line * angle_rad.sin();

    let z1 = relay
        .zones
        .iter()
        .find(|z| z.zone_num == 1)
        .expect("Zone 1");
    let z2 = relay
        .zones
        .iter()
        .find(|z| z.zone_num == 2)
        .expect("Zone 2");

    let in_z1 = is_inside_mho((r_app, x_app), z1.reach_pu, DEFAULT_LINE_ANGLE_DEG);
    let in_z2 = is_inside_mho((r_app, x_app), z2.reach_pu, DEFAULT_LINE_ANGLE_DEG);

    assert!(!in_z1, "90% fault should NOT be in Zone 1");
    assert!(in_z2, "90% fault should be in Zone 2");
}

// ── Differential protection tests ─────────────────────────────────────────

#[test]
fn test_differential_internal_fault_trips() {
    // Internal fault: both currents flow in same direction (inflow)
    let mut dz = DifferentialZone::new(1, DifferentialZoneType::BusDifferential, "Bus A".into());
    dz.pickup_pu = 0.2;
    dz.slope_pct = 30.0;
    dz.i_min_operate_pu = 0.1;

    // I_in=1.0, I_out=0.0 → I_diff=1.0, I_restraint=0.5 → pickup=max(0.1, 0.15)=0.15 < 1.0 → TRIP
    let result = dz.check_operation(&[(1.0, true), (0.0, false)]);
    assert!(result, "Internal fault should trip differential protection");
}

#[test]
fn test_differential_through_fault_restrains() {
    let mut dz = DifferentialZone::new(
        2,
        DifferentialZoneType::TransformerDifferential,
        "TX1".into(),
    );
    dz.pickup_pu = 0.2;
    dz.slope_pct = 30.0;
    dz.i_min_operate_pu = 0.1;

    // Through-fault: I_in = I_out in magnitude → I_diff = 0 → should NOT trip
    let result = dz.check_operation(&[(1.0, true), (1.0, false)]);
    assert!(
        !result,
        "Through-fault should restrain differential protection"
    );
}

// ── auto_set_zones tests ──────────────────────────────────────────────────

#[test]
fn test_auto_set_zones_returns_three() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    assert_eq!(zones.len(), 3, "auto_set_zones must return exactly 3 zones");
}

// ── Coordination tests ────────────────────────────────────────────────────

#[test]
fn test_coordination_margin_zone2_gte_cti() {
    let relay1 = make_relay_with_zones(1, 0.1, 0.08);
    let relay2 = make_relay_with_zones(2, 0.08, 0.06);

    let map = ProtectionZoneMap::new("Test");
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);

    let margin = coord.compute_coordination_margin(&relay1, &relay2);
    // Zone 2 delay (0.4) - Zone 1 delay (0.0) = 0.4 ≥ 0.3
    assert!(
        margin >= DEFAULT_CTI_S,
        "Coordination margin {} must be >= CTI {}",
        margin,
        DEFAULT_CTI_S
    );
}

#[test]
fn test_insufficient_time_margin_detected() {
    // Create two relays where backup operates too fast relative to primary
    let mut relay1 = DistanceRelay::new(1, 0, 10, 0.1);
    relay1.add_zone(DistanceZone::new(
        1,
        0.08,
        0.0,
        ZoneDirectional::Forward,
        80.0,
    ));
    relay1.add_zone(DistanceZone::new(
        2,
        0.14,
        0.4,
        ZoneDirectional::Forward,
        140.0,
    ));

    let mut relay2 = DistanceRelay::new(2, 1, 11, 0.08);
    // Backup zone 2 with only 0.1 s delay (violation: margin = 0.1 < 0.3)
    relay2.add_zone(DistanceZone::new(
        2,
        0.12,
        0.1,
        ZoneDirectional::Forward,
        150.0,
    ));

    let mut map = ProtectionZoneMap::new("BadCoord");
    map.distance_relays.push(relay1);
    map.distance_relays.push(relay2);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let result = coord.check_coordination();

    let has_cti_issue = result
        .issues
        .iter()
        .any(|i| matches!(i, CoordinationIssue::InsufficientTimeMargin { .. }));
    assert!(has_cti_issue, "Should detect InsufficientTimeMargin issue");
    assert!(!result.is_coordinated, "Should be NOT coordinated");
}

#[test]
fn test_zone1_too_long_detected() {
    let mut relay = DistanceRelay::new(1, 0, 10, 0.1);
    // Zone 1 coverage > 85% → violation
    relay.add_zone(DistanceZone::new(
        1,
        0.09,
        0.0,
        ZoneDirectional::Forward,
        90.0,
    ));

    let mut map = ProtectionZoneMap::new("BadReach");
    map.distance_relays.push(relay);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let issues = coord.find_gaps_in_coverage();

    let has_z1_long = issues
        .iter()
        .any(|i| matches!(i, CoordinationIssue::Zone1TooLong { .. }));
    assert!(has_z1_long, "Should detect Zone1TooLong issue");
}

// ── check_differential_operation tests ───────────────────────────────────

#[test]
fn test_check_differential_operation_true() {
    let mut dz = DifferentialZone::new(5, DifferentialZoneType::LineDifferential, "Line AB".into());
    dz.pickup_pu = 0.2;
    dz.slope_pct = 20.0;
    dz.i_min_operate_pu = 0.05;

    let mut map = ProtectionZoneMap::new("Sub");
    map.differential_zones.push(dz);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    // Large internal fault current
    assert!(
        coord.check_differential_operation(5, 2.0, 0.0),
        "Should trip for large internal fault"
    );
}

#[test]
fn test_check_differential_operation_false() {
    let mut dz = DifferentialZone::new(
        6,
        DifferentialZoneType::TransformerDifferential,
        "TX2".into(),
    );
    dz.pickup_pu = 0.2;
    dz.slope_pct = 30.0;
    dz.i_min_operate_pu = 0.1;

    let mut map = ProtectionZoneMap::new("Sub");
    map.differential_zones.push(dz);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    // Through-fault: equal in/out
    assert!(
        !coord.check_differential_operation(6, 1.0, 1.0),
        "Should restrain for through-fault"
    );
}

// ── evaluate_fault tests ──────────────────────────────────────────────────

#[test]
fn test_evaluate_fault_zone1_close_in() {
    // Fault at 30% of line should be in Zone 1 (Zone 1 reach = 80%)
    let relay = make_relay_with_zones(1, 0.1, 0.08);
    let mut map = ProtectionZoneMap::new("Sub");
    map.distance_relays.push(relay);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let fault = FaultLocation {
        per_unit_distance: 0.30,
        fault_type: ProtFaultType::ThreePhase,
        fault_resistance_pu: 0.0,
    };
    let perf = coord.evaluate_fault(&fault, 1);
    assert_eq!(
        perf.operating_zone, 1,
        "Close-in fault at 30% should operate Zone 1"
    );
    assert!(perf.is_correct_operation, "Should be correct operation");
}

#[test]
fn test_evaluate_fault_zone2_far_end() {
    // Fault at 85% of line: beyond Zone 1 (80%) but within Zone 2
    let relay = make_relay_with_zones(1, 0.1, 0.08);
    let mut map = ProtectionZoneMap::new("Sub");
    map.distance_relays.push(relay);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let fault = FaultLocation {
        per_unit_distance: 0.85,
        fault_type: ProtFaultType::SingleLineGround,
        fault_resistance_pu: 0.0,
    };
    let perf = coord.evaluate_fault(&fault, 1);
    assert_eq!(
        perf.operating_zone, 2,
        "Far-end fault at 85% should operate Zone 2"
    );
}

#[test]
fn test_zone1_clearing_time_zero() {
    let relay = make_relay_with_zones(1, 0.1, 0.08);
    let mut map = ProtectionZoneMap::new("Sub");
    map.distance_relays.push(relay);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let fault = FaultLocation {
        per_unit_distance: 0.30,
        fault_type: ProtFaultType::ThreePhase,
        fault_resistance_pu: 0.0,
    };
    let perf = coord.evaluate_fault(&fault, 1);
    assert_eq!(
        perf.clearing_time_s, 0.0,
        "Zone 1 operation must have zero clearing time"
    );
}

#[test]
fn test_backup_clearing_time_gt_primary() {
    // Two relays on line 1: primary operates Zone 1 (0 s), backup in Zone 2 (0.4 s)
    let relay1 = make_relay_with_zones(1, 0.1, 0.08);
    let relay2 = make_relay_with_zones(2, 0.1, 0.08);
    let mut map = ProtectionZoneMap::new("Sub");
    map.distance_relays.push(relay1);
    map.distance_relays.push(relay2);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let fault = FaultLocation {
        per_unit_distance: 0.30,
        fault_type: ProtFaultType::ThreePhase,
        fault_resistance_pu: 0.0,
    };
    let perf = coord.evaluate_fault(&fault, 1);

    if let Some(bt) = perf.backup_clearing_time_s {
        assert!(
            bt > perf.clearing_time_s,
            "Backup clearing time {} must exceed primary clearing time {}",
            bt,
            perf.clearing_time_s
        );
    }
    // If no backup relay, the primary relay itself provides higher-zone backup
}

// ── Enum variant accessibility tests ─────────────────────────────────────

#[test]
fn test_differential_zone_type_variants() {
    let _bus = DifferentialZoneType::BusDifferential;
    let _tx = DifferentialZoneType::TransformerDifferential;
    let _line = DifferentialZoneType::LineDifferential;
    let _gen = DifferentialZoneType::GeneratorDifferential;
    // All variants must be accessible
    assert_eq!(_bus, DifferentialZoneType::BusDifferential);
}

#[test]
fn test_zone_directional_forward() {
    let dir = ZoneDirectional::Forward;
    assert_eq!(dir, ZoneDirectional::Forward);
    let _rev = ZoneDirectional::Reverse;
    let _non = ZoneDirectional::Nondirectional;
}

#[test]
fn test_summary_report_non_empty() {
    let relay = make_relay_with_zones(1, 0.1, 0.08);
    let mut map = ProtectionZoneMap::new("TestSubstation");
    map.distance_relays.push(relay);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let report = coord.summary_report();
    assert!(
        !report.is_empty(),
        "summary_report must return a non-empty string"
    );
    assert!(
        report.contains("TestSubstation"),
        "Report must include substation name"
    );
}

// ── Additional coverage tests ─────────────────────────────────────────────

#[test]
fn test_prot_fault_type_variants() {
    let _tp = ProtFaultType::ThreePhase;
    let _slg = ProtFaultType::SingleLineGround;
    let _ll = ProtFaultType::LineToLine;
    let _dlg = ProtFaultType::DoubleLineGround;
    assert_eq!(_tp, ProtFaultType::ThreePhase);
}

#[test]
fn test_apparent_impedance_calculation() {
    let relay = DistanceRelay::new(1, 0, 10, 0.1);
    // V=1.0 pu, I=10.0 pu, angle=75°
    let (r, x) = relay.apparent_impedance(1.0, 10.0, 75.0);
    let expected_mag = 1.0 / 10.0; // 0.1 pu
    let actual_mag = (r * r + x * x).sqrt();
    assert!(
        (actual_mag - expected_mag).abs() < 1e-10,
        "Impedance magnitude {} should be {}",
        actual_mag,
        expected_mag
    );
}

#[test]
fn test_differential_zone_add_ct() {
    let mut dz = DifferentialZone::new(10, DifferentialZoneType::BusDifferential, "Bus B".into());
    dz.add_ct(1, 600.0);
    dz.add_ct(2, 400.0);
    assert_eq!(dz.boundary_cts.len(), 2);
    assert_eq!(dz.boundary_cts[0], (1, 600.0));
}

#[test]
fn test_relay_add_zone() {
    let mut relay = DistanceRelay::new(1, 0, 10, 0.1);
    assert_eq!(relay.zones.len(), 0);
    relay.add_zone(DistanceZone::new(
        1,
        0.08,
        0.0,
        ZoneDirectional::Forward,
        80.0,
    ));
    relay.add_zone(DistanceZone::new(
        2,
        0.14,
        0.4,
        ZoneDirectional::Forward,
        140.0,
    ));
    assert_eq!(relay.zones.len(), 2);
}

#[test]
fn test_is_inside_mho_public() {
    let map = ProtectionZoneMap::new("Sub");
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let zone = DistanceZone::new(1, 0.08, 0.0, ZoneDirectional::Forward, 80.0);
    let char = DistanceCharacteristic::Mho {
        mho_angle_deg: 75.0,
    };
    // Fault at 50% of 0.1 pu line = 0.05 pu inside Zone 1 (0.08 pu)
    let angle_rad = 75_f64.to_radians();
    let r = 0.05 * angle_rad.cos();
    let x = 0.05 * angle_rad.sin();
    assert!(coord.is_inside_mho((r, x), &zone, &char));
}

#[test]
fn test_protection_zone_map_new() {
    let map = ProtectionZoneMap::new("Substation Alpha");
    assert_eq!(map.substation_name, "Substation Alpha");
    assert!(map.buses.is_empty());
    assert!(map.distance_relays.is_empty());
    assert!(map.differential_zones.is_empty());
}

#[test]
fn test_coordination_result_is_coordinated_for_good_relays() {
    // Two well-coordinated relays
    let relay1 = make_relay_with_zones(1, 0.1, 0.08);
    let relay2 = make_relay_with_zones(2, 0.08, 0.06);
    let mut map = ProtectionZoneMap::new("GoodCoord");
    map.distance_relays.push(relay1);
    map.distance_relays.push(relay2);

    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let result = coord.check_coordination();
    // Both relays auto-set: Zone 1 at 0.0 s, Zone 2 at 0.4 s
    // margin = 0.4 - 0.0 = 0.4 ≥ 0.3 → coordinated
    assert!(result.is_coordinated, "Well-coordinated relays should pass");
}

#[test]
fn test_all_three_zones_strictly_ordered_reach() {
    // Use different impedances from existing tests to provide additional coverage
    let coord = make_coordinator(0.15, 0.10);
    let zones = coord.auto_set_zones(0.15, 0.10);
    let z1 = zones
        .iter()
        .find(|z| z.zone_num == 1)
        .expect("Zone 1 must exist")
        .reach_pu;
    let z2 = zones
        .iter()
        .find(|z| z.zone_num == 2)
        .expect("Zone 2 must exist")
        .reach_pu;
    let z3 = zones
        .iter()
        .find(|z| z.zone_num == 3)
        .expect("Zone 3 must exist")
        .reach_pu;
    assert!(
        z1 < z2,
        "Zone 1 reach {} must be less than Zone 2 reach {}",
        z1,
        z2
    );
    assert!(
        z2 < z3,
        "Zone 2 reach {} must be less than Zone 3 reach {}",
        z2,
        z3
    );
}

#[test]
fn test_fault_inside_zone_trips_outside_does_not() {
    // Zone 1 reach = 0.08 pu at 75°: a fault at exactly 79% (inside) and 82% (outside)
    let z_line = 0.1_f64;
    let z1_reach = 0.8 * z_line; // 0.08 pu
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    // 79% of line — inside Zone 1
    let d_in = 0.79_f64;
    let r_in = d_in * z_line * angle_rad.cos();
    let x_in = d_in * z_line * angle_rad.sin();
    // 82% of line — outside Zone 1
    let d_out = 0.82_f64;
    let r_out = d_out * z_line * angle_rad.cos();
    let x_out = d_out * z_line * angle_rad.sin();

    assert!(
        is_inside_mho((r_in, x_in), z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "Fault at 79% of line should be inside Zone 1 (reach 80%)"
    );
    assert!(
        !is_inside_mho((r_out, x_out), z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "Fault at 82% of line should be outside Zone 1 (reach 80%)"
    );
}

#[test]
fn test_time_grading_z1_lt_z2_lt_z3() {
    let coord = make_coordinator(0.1, 0.08);
    let zones = coord.auto_set_zones(0.1, 0.08);
    let t1 = zones
        .iter()
        .find(|z| z.zone_num == 1)
        .expect("Zone 1 must exist")
        .time_delay_s;
    let t2 = zones
        .iter()
        .find(|z| z.zone_num == 2)
        .expect("Zone 2 must exist")
        .time_delay_s;
    let t3 = zones
        .iter()
        .find(|z| z.zone_num == 3)
        .expect("Zone 3 must exist")
        .time_delay_s;
    assert!(
        t1 < t2,
        "Zone 1 delay {} must be strictly less than Zone 2 delay {}",
        t1,
        t2
    );
    assert!(
        t2 < t3,
        "Zone 2 delay {} must be strictly less than Zone 3 delay {}",
        t2,
        t3
    );
}

#[test]
fn test_apparent_impedance_r_x_components() {
    // V=1.0 pu, I=5.0 pu, angle=75° → Z_mag=0.2, R=0.2*cos75°, X=0.2*sin75°
    let relay = DistanceRelay::new(1, 0, 10, 0.2);
    let (r, x) = relay.apparent_impedance(1.0, 5.0, 75.0);
    let angle_rad = 75_f64.to_radians();
    let expected_r = 0.2 * angle_rad.cos();
    let expected_x = 0.2 * angle_rad.sin();
    assert!(
        (r - expected_r).abs() < 1e-10,
        "R component {} should be {} (V=1.0, I=5.0, angle=75°)",
        r,
        expected_r
    );
    assert!(
        (x - expected_x).abs() < 1e-10,
        "X component {} should be {} (V=1.0, I=5.0, angle=75°)",
        x,
        expected_x
    );
}

#[test]
fn test_reverse_fault_not_inside_forward_mho_zone() {
    // Reverse fault: impedance is in the third quadrant (negative R, negative X)
    // A forward-looking Mho zone (centred along positive R,X direction) should not include it
    let z_line = 0.1_f64;
    let _z1_reach = 0.08_f64;
    // Reverse fault at same distance but negative direction
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    let r_rev = -0.5 * z_line * angle_rad.cos();
    let x_rev = -0.5 * z_line * angle_rad.sin();
    assert!(
        !is_inside_mho((r_rev, x_rev), _z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "Reverse fault (negative R,X) must not be detected by forward Mho zone"
    );
}

#[test]
fn test_power_swing_large_resistance_outside_mho() {
    // A power swing trajectory often appears as high-resistance impedance far off the line angle.
    // A fault with large R and small X should be outside the Mho circle.
    // Zone 1 reach = 0.08 pu at 75°; centre = (0.08*cos75°/2, 0.08*sin75°/2), radius = 0.04
    let z1_reach = 0.08_f64;
    // High resistance, near-zero reactance — typical power swing locus
    let r_swing = 1.0_f64;
    let x_swing = 0.01_f64;
    assert!(
        !is_inside_mho((r_swing, x_swing), z1_reach, DEFAULT_LINE_ANGLE_DEG),
        "High-resistance power swing impedance must be outside Mho circle"
    );
}

#[test]
fn test_relay_coordination_margin_meets_cti() {
    // Primary relay: Zone 1 at 0.0 s, Zone 2 at 0.4 s
    // Backup relay: Zone 2 at 0.4 s
    // Margin = backup Zone2 delay - primary Zone1 delay = 0.4 - 0.0 = 0.4 >= CTI(0.3)
    let primary = make_relay_with_zones(1, 0.1, 0.08);
    let backup = make_relay_with_zones(2, 0.1, 0.08);
    let map = ProtectionZoneMap::new("CoordSub");
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let margin = coord.compute_coordination_margin(&primary, &backup);
    assert!(
        margin >= DEFAULT_CTI_S,
        "Coordination margin {} s must be >= CTI {} s",
        margin,
        DEFAULT_CTI_S
    );
}

#[test]
fn test_quadrilateral_characteristic_in_and_out() {
    // Quadrilateral: r_reach=0.05, x_reach=0.10
    let mut relay = DistanceRelay::new(1, 0, 10, 0.12);
    relay.characteristic = DistanceCharacteristic::Quadrilateral {
        r_reach_pu: 0.05,
        x_reach_pu: 0.10,
        angle_deg: 75.0,
    };
    let zone = DistanceZone::new(1, 0.12, 0.0, ZoneDirectional::Forward, 80.0);
    relay.add_zone(zone);

    // Fault inside quadrilateral: R=0.03 < 0.05, X=0.07 < 0.10 → should operate
    let inside = relay.operating_zone((0.03, 0.07));
    assert!(
        inside.is_some(),
        "Fault inside quadrilateral reach should trigger Zone 1"
    );

    // Fault outside quadrilateral: R=0.06 > 0.05 → should not operate
    let outside = relay.operating_zone((0.06, 0.07));
    assert!(
        outside.is_none(),
        "Fault outside quadrilateral R reach must not operate"
    );
}

#[test]
fn test_evaluate_fault_no_relay_returns_incorrect() {
    // No relay in the map protects line_id 9999 → is_correct_operation must be false
    let map = ProtectionZoneMap::new("EmptySubstation");
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let fault = FaultLocation {
        per_unit_distance: 0.5,
        fault_type: ProtFaultType::ThreePhase,
        fault_resistance_pu: 0.0,
    };
    let perf = coord.evaluate_fault(&fault, 9999);
    assert!(
        !perf.is_correct_operation,
        "evaluate_fault with no matching relay must return is_correct_operation=false"
    );
}

#[test]
fn test_apparent_impedance_zero_current_returns_infinity() {
    // When i_relay < 1e-12 the relay returns (∞, ∞) to avoid division by zero
    let relay = DistanceRelay::new(1, 0, 10, 0.1);
    let (r, x) = relay.apparent_impedance(1.0, 0.0, 75.0);
    assert!(
        r.is_infinite(),
        "R component must be infinity when current is zero, got {}",
        r
    );
    assert!(
        x.is_infinite(),
        "X component must be infinity when current is zero, got {}",
        x
    );
}

#[test]
fn test_operating_zone_out_of_reach_returns_none() {
    // Relay with z_line=0.1, Zone 1 reach=0.08 pu at 75°
    // Impedance at (10.0, 10.0) is far outside the Mho circle → None
    let mut relay = DistanceRelay::new(1, 0, 10, 0.1);
    let zone = DistanceZone::new(1, 0.08, 0.0, ZoneDirectional::Forward, 80.0);
    relay.add_zone(zone);
    let result = relay.operating_zone((10.0, 10.0));
    assert!(
        result.is_none(),
        "Impedance (10.0, 10.0) is far outside Zone 1 reach 0.08 pu and must return None"
    );
}

#[test]
fn test_check_differential_operation_unknown_zone_returns_false() {
    // zone_id 999 is not in the zone map → check_differential_operation must return false
    let map = ProtectionZoneMap::new("DiffSub");
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);
    let result = coord.check_differential_operation(999, 1.0, 0.5);
    assert!(
        !result,
        "check_differential_operation with unknown zone_id must return false"
    );
}

#[test]
fn test_distance_zone_new_stores_fields_correctly() {
    // DistanceZone::new must store all five fields exactly as supplied
    let zone = DistanceZone::new(2, 0.15, 0.4, ZoneDirectional::Reverse, 120.0);
    assert_eq!(zone.zone_num, 2, "zone_num must be 2");
    assert!(
        (zone.reach_pu - 0.15).abs() < 1e-12,
        "reach_pu must be 0.15, got {}",
        zone.reach_pu
    );
    assert!(
        (zone.time_delay_s - 0.4).abs() < 1e-12,
        "time_delay_s must be 0.4, got {}",
        zone.time_delay_s
    );
    assert!(
        matches!(zone.directional, ZoneDirectional::Reverse),
        "directional must be Reverse"
    );
    assert!(
        (zone.coverage_pct - 120.0).abs() < 1e-12,
        "coverage_pct must be 120.0, got {}",
        zone.coverage_pct
    );
}

#[test]
fn test_lens_characteristic_smaller_than_mho() {
    // For the same reach, the Lens characteristic covers 70% of the Mho reach.
    // A fault at ~75% of line (0.075 pu) along the 75° line angle:
    //   - Inside a Mho relay with reach 0.08 pu
    //   - Outside a Lens relay with effective reach 0.08 * 0.7 = 0.056 pu
    let reach = 0.08_f64;
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    // Fault at 72% of reach (0.0576 pu) — inside Mho (0.08) but OUTSIDE Lens (0.056)
    let fault_dist = 0.72 * reach;
    let r_f = fault_dist * angle_rad.cos();
    let x_f = fault_dist * angle_rad.sin();

    // Mho relay: expects operating_zone to find the zone
    let mut mho_relay = DistanceRelay::new(1, 0, 10, 0.1);
    mho_relay.characteristic = DistanceCharacteristic::Mho {
        mho_angle_deg: DEFAULT_LINE_ANGLE_DEG,
    };
    let mho_zone = DistanceZone::new(1, reach, 0.0, ZoneDirectional::Forward, 80.0);
    mho_relay.add_zone(mho_zone);

    // Lens relay: same reach but effective coverage = 0.7 * reach = 0.056 pu
    let mut lens_relay = DistanceRelay::new(2, 0, 20, 0.1);
    lens_relay.characteristic = DistanceCharacteristic::Lens;
    let lens_zone = DistanceZone::new(1, reach, 0.0, ZoneDirectional::Forward, 80.0);
    lens_relay.add_zone(lens_zone);

    assert!(
        mho_relay.operating_zone((r_f, x_f)).is_some(),
        "Fault at 72% of Mho reach must be inside the Mho circle"
    );
    assert!(
        lens_relay.operating_zone((r_f, x_f)).is_none(),
        "Fault at 72% of Mho reach (= 103% of Lens reach 0.056) must be outside Lens"
    );
}

#[test]
fn test_zone_coverage_backup_zones_field() {
    // ZoneCoverage.backup_zones must store all supplied backup zone IDs
    let coverage = ZoneCoverage {
        zone_id: 5,
        protected_equipment: vec!["Line-A".to_string()],
        backup_zones: vec![6, 7, 8],
        coverage_overlap: 0.2,
    };
    assert!(
        !coverage.backup_zones.is_empty(),
        "backup_zones must not be empty"
    );
    assert_eq!(
        coverage.backup_zones.len(),
        3,
        "backup_zones must contain 3 entries"
    );
    assert_eq!(coverage.backup_zones[0], 6, "first backup zone must be 6");
    assert_eq!(coverage.backup_zones[1], 7, "second backup zone must be 7");
    assert_eq!(coverage.backup_zones[2], 8, "third backup zone must be 8");
}

#[test]
fn test_evaluate_fault_high_resistance_bolted_comparison() {
    // For the same per_unit_distance, a resistive arc fault (r_f > 0) causes a larger
    // apparent impedance than a bolted fault (r_f = 0).
    // Setup: relay on line 50, Zone 1 reach large enough to see a fault at 30% with resistance.
    let z_line = 0.2_f64;
    let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();
    let d = 0.3_f64;
    let r_f_arc = 0.05_f64;

    // Bolted: r_app = d*z*cos(θ), x_app = d*z*sin(θ)
    let r_bolted = d * z_line * angle_rad.cos();
    let x_bolted = d * z_line * angle_rad.sin();
    let z_bolted = (r_bolted * r_bolted + x_bolted * x_bolted).sqrt();

    // Arc: r_app = d*z*cos(θ) + r_f, x same
    let r_arc = r_bolted + r_f_arc;
    let z_arc = (r_arc * r_arc + x_bolted * x_bolted).sqrt();

    assert!(
        z_arc > z_bolted,
        "Arc fault impedance {} must exceed bolted fault impedance {} for same distance",
        z_arc,
        z_bolted
    );

    // Verify via evaluate_fault: use a wide Zone 1 (full line reach) to ensure both faults
    // are detected, then compare measured_impedance_pu
    let mut relay = DistanceRelay::new(1, 0, 50, z_line);
    let zone = DistanceZone::new(1, z_line * 1.2, 0.0, ZoneDirectional::Forward, 100.0);
    relay.add_zone(zone);
    let mut map = ProtectionZoneMap::new("ResistanceSub");
    map.distance_relays.push(relay);
    let coord = ZoneCoordinator::new(map, DEFAULT_CTI_S);

    let fault_bolted = FaultLocation {
        per_unit_distance: d,
        fault_type: ProtFaultType::SingleLineGround,
        fault_resistance_pu: 0.0,
    };
    let fault_arc = FaultLocation {
        per_unit_distance: d,
        fault_type: ProtFaultType::SingleLineGround,
        fault_resistance_pu: r_f_arc,
    };

    let perf_bolted = coord.evaluate_fault(&fault_bolted, 50);
    let perf_arc = coord.evaluate_fault(&fault_arc, 50);

    assert!(
        perf_arc.measured_impedance_pu >= perf_bolted.measured_impedance_pu,
        "Arc fault measured impedance {} must be >= bolted fault impedance {}",
        perf_arc.measured_impedance_pu,
        perf_bolted.measured_impedance_pu
    );
}
