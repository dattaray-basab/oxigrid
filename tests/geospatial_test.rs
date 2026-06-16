use oxigrid::network::geospatial::{
    GeoBoundingBox, GeoNode, GeoNodeType, GeoPoint, SpatialAnalysis,
};

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

fn make_substation_node(id: usize, lat: f64, lon: f64) -> GeoNode {
    GeoNode {
        id,
        name: format!("node_{id}"),
        location: GeoPoint::new(lat, lon),
        node_type: GeoNodeType::TransmissionSubstation,
        voltage_kv: 110.0,
        rated_mva: 100.0,
    }
}

// ---------------------------------------------------------------------------
// 1. Haversine symmetry
// ---------------------------------------------------------------------------

#[test]
fn test_haversine_symmetry() {
    let paris = GeoPoint::new(48.8566, 2.3522);
    let tokyo = GeoPoint::new(35.6762, 139.6503);
    let diff = (paris.haversine_distance(&tokyo) - tokyo.haversine_distance(&paris)).abs();
    assert!(diff < 1.0, "haversine not symmetric: diff = {diff} m");
}

// ---------------------------------------------------------------------------
// 2. Haversine self-distance is zero
// ---------------------------------------------------------------------------

#[test]
fn test_haversine_self_distance_zero() {
    let point = GeoPoint::new(48.8566, 2.3522);
    let d = point.haversine_distance(&point).abs();
    assert!(d < 1e-6, "self-distance should be ~0, got {d}");
}

// ---------------------------------------------------------------------------
// 3. London-Paris known distance (330–350 km)
// ---------------------------------------------------------------------------

#[test]
fn test_haversine_london_paris_km() {
    let london = GeoPoint::new(51.5074, -0.1278);
    let paris = GeoPoint::new(48.8566, 2.3522);
    let d_km = london.haversine_distance(&paris) / 1000.0;
    assert!(
        d_km > 330.0 && d_km < 350.0,
        "London-Paris = {d_km:.1} km, expected 330–350 km"
    );
}

// ---------------------------------------------------------------------------
// 4. Bearing southward is 180°
// ---------------------------------------------------------------------------

#[test]
fn test_bearing_south() {
    let north = GeoPoint::new(10.0, 0.0);
    let south = GeoPoint::new(0.0, 0.0);
    let b = north.bearing_to(&south);
    assert!(
        (b - 180.0).abs() < 1.0,
        "bearing south should be ~180°, got {b}"
    );
}

// ---------------------------------------------------------------------------
// 5. BoundingBox expand increases bounds
// ---------------------------------------------------------------------------

#[test]
fn test_bbox_expand_increases_bounds() {
    let bbox = GeoBoundingBox::new(10.0, 20.0, 30.0, 40.0);
    let expanded = bbox.expand(1.0);
    assert!(
        expanded.min_lat < 10.0,
        "expanded.min_lat ({}) should be < 10.0",
        expanded.min_lat
    );
    assert!(
        expanded.max_lat > 20.0,
        "expanded.max_lat ({}) should be > 20.0",
        expanded.max_lat
    );
    assert!(
        expanded.min_lon < 30.0,
        "expanded.min_lon ({}) should be < 30.0",
        expanded.min_lon
    );
    assert!(
        expanded.max_lon > 40.0,
        "expanded.max_lon ({}) should be > 40.0",
        expanded.max_lon
    );
}

// ---------------------------------------------------------------------------
// 6. BoundingBox center is midpoint
// ---------------------------------------------------------------------------

#[test]
fn test_bbox_center_midpoint() {
    let bbox = GeoBoundingBox::new(0.0, 10.0, 0.0, 10.0);
    let center = bbox.center();
    assert!(
        (center.lat - 5.0).abs() < 0.1,
        "center.lat = {}, expected ~5.0",
        center.lat
    );
    assert!(
        (center.lon - 5.0).abs() < 0.1,
        "center.lon = {}, expected ~5.0",
        center.lon
    );
}

// ---------------------------------------------------------------------------
// 7. SpatialAnalysis::nearest_node returns index of closest node
// ---------------------------------------------------------------------------

#[test]
fn test_spatial_analysis_nearest_node() {
    let nodes = vec![
        make_substation_node(0, 0.0, 0.0),
        make_substation_node(1, 1.0, 1.0),
        make_substation_node(2, 5.0, 5.0),
    ];
    let query = GeoPoint::new(0.1, 0.1);
    let idx = SpatialAnalysis::nearest_node(&nodes, &query).expect("should find a node");
    assert_eq!(idx, 0, "nearest node index should be 0, got {idx}");
}

// ---------------------------------------------------------------------------
// 8. geographic_load_center returns None when no LoadCenter nodes exist
// ---------------------------------------------------------------------------

#[test]
fn test_spatial_analysis_no_load_center_returns_none() {
    let nodes = vec![
        make_substation_node(0, 48.0, 2.0),
        make_substation_node(1, 51.0, 0.0),
        make_substation_node(2, 45.0, 10.0),
    ];
    let result = SpatialAnalysis::geographic_load_center(&nodes);
    assert!(
        result.is_none(),
        "expected None for nodes with no LoadCenter"
    );
}
