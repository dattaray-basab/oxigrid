//! Geospatial coordinates and routing for power network mapping.
//!
//! Provides geographic data structures for power system visualization,
//! line routing, distance computation, and GeoJSON export.
//!
//! # Coordinate System
//!
//! All coordinates use WGS84 (EPSG:4326) by default:
//! - `latitude_deg`: −90 to +90 (negative = south)
//! - `longitude_deg`: −180 to +180 (negative = west)
//! - `elevation_m`: metres above mean sea level \[m\]
//!
//! # Haversine Distance
//!
//! Geodesic distance between two points is computed via the Haversine formula:
//!
//! ```text
//! a = sin²(Δlat/2) + cos(lat₁)·cos(lat₂)·sin²(Δlon/2)
//! c = 2·atan2(√a, √(1−a))
//! d = R·c   (R = 6371.0 km)
//! ```

use thiserror::Error;

// ---------------------------------------------------------------------------
// Error
// ---------------------------------------------------------------------------

/// Errors from geospatial network operations.
#[derive(Debug, Error)]
pub enum GeoError {
    /// The requested branch ID does not exist.
    #[error("branch {0} not found in geospatial network")]
    BranchNotFound(usize),
    /// The referenced bus ID for a branch end does not exist.
    #[error("bus {0} referenced in branch {1} not found")]
    BusNotFound(usize, usize),
    /// A coordinate value is out of valid range.
    #[error("invalid coordinate: {0}")]
    InvalidCoordinate(String),
}

// ---------------------------------------------------------------------------
// Geographic primitives
// ---------------------------------------------------------------------------

/// A geographic point in WGS84 coordinates.
#[derive(Debug, Clone, PartialEq)]
pub struct GeoCoordinate {
    /// Latitude in decimal degrees \[deg\]; range [−90, +90].
    pub latitude_deg: f64,
    /// Longitude in decimal degrees \[deg\]; range [−180, +180].
    pub longitude_deg: f64,
    /// Elevation above mean sea level \[m\].
    pub elevation_m: f64,
}

impl GeoCoordinate {
    /// Create a coordinate at sea level.
    pub fn new(latitude_deg: f64, longitude_deg: f64) -> Self {
        Self {
            latitude_deg,
            longitude_deg,
            elevation_m: 0.0,
        }
    }

    /// Create a coordinate with explicit elevation \[m\].
    pub fn with_elevation(latitude_deg: f64, longitude_deg: f64, elevation_m: f64) -> Self {
        Self {
            latitude_deg,
            longitude_deg,
            elevation_m,
        }
    }
}

// ---------------------------------------------------------------------------
// Bus and substation types
// ---------------------------------------------------------------------------

/// Substation voltage class classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubstationType {
    /// Bulk transmission: 230 \[kV\] and above.
    Transmission,
    /// Sub-transmission: 69–115 \[kV\].
    Subtransmission,
    /// Primary distribution: 4–35 \[kV\].
    Distribution,
    /// Customer service: below 4 \[kV\].
    CustomerService,
}

impl SubstationType {
    fn as_str(self) -> &'static str {
        match self {
            SubstationType::Transmission => "Transmission",
            SubstationType::Subtransmission => "Subtransmission",
            SubstationType::Distribution => "Distribution",
            SubstationType::CustomerService => "CustomerService",
        }
    }
}

/// A bus with geographic location information.
#[derive(Debug, Clone)]
pub struct GeoBus {
    /// Corresponding power flow bus identifier.
    pub bus_id: usize,
    /// Human-readable substation name.
    pub name: String,
    /// Geographic location of the substation.
    pub coord: GeoCoordinate,
    /// Nominal voltage \[kV\].
    pub voltage_kv: f64,
    /// Substation voltage class.
    pub substation_type: SubstationType,
}

// ---------------------------------------------------------------------------
// Branch and line types
// ---------------------------------------------------------------------------

/// Transmission/distribution line technology.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransmissionLineType {
    /// Above-ground AC line.
    OverheadAc,
    /// Above-ground DC line (bipole or monopole).
    OverheadDc,
    /// Buried AC cable.
    UndergroundAc,
    /// Buried DC cable.
    UndergroundDc,
    /// Undersea cable (AC or DC).
    Submarine,
}

impl TransmissionLineType {
    fn as_str(self) -> &'static str {
        match self {
            TransmissionLineType::OverheadAc => "OverheadAC",
            TransmissionLineType::OverheadDc => "OverheadDC",
            TransmissionLineType::UndergroundAc => "UndergroundAC",
            TransmissionLineType::UndergroundDc => "UndergroundDC",
            TransmissionLineType::Submarine => "Submarine",
        }
    }
}

/// A transmission/distribution branch with geographic routing.
#[derive(Debug, Clone)]
pub struct GeoBranch {
    /// Corresponding power flow branch identifier.
    pub branch_id: usize,
    /// From-bus identifier.
    pub from_bus: usize,
    /// To-bus identifier.
    pub to_bus: usize,
    /// Intermediate waypoints along the route (not including endpoints).
    pub waypoints: Vec<GeoCoordinate>,
    /// Physical line technology.
    pub line_type: TransmissionLineType,
    /// Nominal voltage \[kV\].
    pub voltage_kv: f64,
    /// Total route length \[km\] (recomputed from coordinates; 0.0 until validated).
    pub length_km: f64,
}

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------

/// A geospatial power network with buses and branches.
#[derive(Debug, Clone)]
pub struct GeoNetwork {
    /// All substations/buses with geographic coordinates.
    pub buses: Vec<GeoBus>,
    /// All branches with routing waypoints.
    pub branches: Vec<GeoBranch>,
    /// Name of the coordinate reference system (e.g. `"WGS84"`).
    pub coordinate_system: String,
}

impl Default for GeoNetwork {
    fn default() -> Self {
        Self::new()
    }
}

impl GeoNetwork {
    /// Create an empty geospatial network using WGS84 coordinates.
    pub fn new() -> Self {
        Self {
            buses: Vec::new(),
            branches: Vec::new(),
            coordinate_system: "WGS84".to_string(),
        }
    }

    /// Add a geo-tagged bus to the network.
    pub fn add_bus(&mut self, bus: GeoBus) {
        self.buses.push(bus);
    }

    /// Add a branch with automatic length computation from coordinates.
    ///
    /// Returns [`GeoError::BusNotFound`] if either endpoint bus does not exist.
    pub fn add_branch(&mut self, mut branch: GeoBranch) -> Result<(), GeoError> {
        let from_coord = self
            .buses
            .iter()
            .find(|b| b.bus_id == branch.from_bus)
            .map(|b| b.coord.clone())
            .ok_or(GeoError::BusNotFound(branch.from_bus, branch.branch_id))?;
        let to_coord = self
            .buses
            .iter()
            .find(|b| b.bus_id == branch.to_bus)
            .map(|b| b.coord.clone())
            .ok_or(GeoError::BusNotFound(branch.to_bus, branch.branch_id))?;

        // Compute total length: from → waypoints → to
        let mut total_km = 0.0f64;
        let mut prev = &from_coord;
        // We need to avoid borrow conflicts — build waypoint refs temporarily
        let wps: Vec<&GeoCoordinate> = branch.waypoints.iter().collect();
        for wp in &wps {
            total_km += Self::haversine_km(prev, wp);
            prev = wp;
        }
        total_km += Self::haversine_km(prev, &to_coord);
        branch.length_km = total_km;

        self.branches.push(branch);
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Geodesic computation
    // -----------------------------------------------------------------------

    /// Compute the geodesic (great-circle) distance between two WGS84 points
    /// using the Haversine formula \[km\].
    ///
    /// Earth radius: 6371.0 \[km\] (mean radius).
    pub fn haversine_km(a: &GeoCoordinate, b: &GeoCoordinate) -> f64 {
        const R_EARTH_KM: f64 = 6371.0;
        let lat1 = a.latitude_deg.to_radians();
        let lat2 = b.latitude_deg.to_radians();
        let dlat = (b.latitude_deg - a.latitude_deg).to_radians();
        let dlon = (b.longitude_deg - a.longitude_deg).to_radians();

        let sin_dlat = (dlat / 2.0).sin();
        let sin_dlon = (dlon / 2.0).sin();
        let hav = sin_dlat * sin_dlat + lat1.cos() * lat2.cos() * sin_dlon * sin_dlon;
        let c = 2.0 * hav.sqrt().atan2((1.0 - hav).sqrt());
        R_EARTH_KM * c
    }

    /// Compute the total route length of a branch from its waypoints \[km\].
    ///
    /// Includes the segment from the from-bus endpoint to the first waypoint,
    /// all intermediate waypoint segments, and the final segment to the to-bus.
    pub fn branch_length_km(&self, branch_id: usize) -> Result<f64, GeoError> {
        let branch = self
            .branches
            .iter()
            .find(|br| br.branch_id == branch_id)
            .ok_or(GeoError::BranchNotFound(branch_id))?;

        let from_coord = self
            .buses
            .iter()
            .find(|b| b.bus_id == branch.from_bus)
            .map(|b| &b.coord)
            .ok_or(GeoError::BusNotFound(branch.from_bus, branch_id))?;
        let to_coord = self
            .buses
            .iter()
            .find(|b| b.bus_id == branch.to_bus)
            .map(|b| &b.coord)
            .ok_or(GeoError::BusNotFound(branch.to_bus, branch_id))?;

        let mut total = 0.0f64;
        let mut prev_lat = from_coord.latitude_deg;
        let mut prev_lon = from_coord.longitude_deg;

        for wp in &branch.waypoints {
            let seg = Self::haversine_km(&GeoCoordinate::new(prev_lat, prev_lon), wp);
            total += seg;
            prev_lat = wp.latitude_deg;
            prev_lon = wp.longitude_deg;
        }
        total += Self::haversine_km(&GeoCoordinate::new(prev_lat, prev_lon), to_coord);
        Ok(total)
    }

    // -----------------------------------------------------------------------
    // Spatial queries
    // -----------------------------------------------------------------------

    /// Return the IDs of all buses whose coordinates lie within `radius_km` of
    /// `center` (Haversine distance).
    pub fn buses_within_radius(&self, center: &GeoCoordinate, radius_km: f64) -> Vec<usize> {
        self.buses
            .iter()
            .filter(|b| Self::haversine_km(center, &b.coord) <= radius_km)
            .map(|b| b.bus_id)
            .collect()
    }

    /// Compute the geographic centroid of all buses (arithmetic mean of lat/lon).
    ///
    /// Returns a coordinate at elevation 0 \[m\] if the network has no buses.
    pub fn network_centroid(&self) -> GeoCoordinate {
        if self.buses.is_empty() {
            return GeoCoordinate::new(0.0, 0.0);
        }
        let n = self.buses.len() as f64;
        let lat_sum: f64 = self.buses.iter().map(|b| b.coord.latitude_deg).sum();
        let lon_sum: f64 = self.buses.iter().map(|b| b.coord.longitude_deg).sum();
        GeoCoordinate::new(lat_sum / n, lon_sum / n)
    }

    // -----------------------------------------------------------------------
    // Branch crossing detection
    // -----------------------------------------------------------------------

    /// Find pairs of branches whose straight-line (from→to) segments cross in
    /// 2D (latitude/longitude plane).
    ///
    /// Returns a list of `(branch_id_a, branch_id_b)` pairs.
    /// Waypoints are ignored — only from/to endpoints are used for this check.
    pub fn find_crossing_branches(&self) -> Vec<(usize, usize)> {
        let nb = self.branches.len();
        let mut crossings = Vec::new();
        for i in 0..nb {
            for j in (i + 1)..nb {
                let a = &self.branches[i];
                let b = &self.branches[j];
                if let (Some(af), Some(at), Some(bf), Some(bt)) = (
                    self.bus_coord(a.from_bus),
                    self.bus_coord(a.to_bus),
                    self.bus_coord(b.from_bus),
                    self.bus_coord(b.to_bus),
                ) {
                    if segments_intersect_2d(af, at, bf, bt) {
                        crossings.push((a.branch_id, b.branch_id));
                    }
                }
            }
        }
        crossings
    }

    fn bus_coord(&self, bus_id: usize) -> Option<&GeoCoordinate> {
        self.buses
            .iter()
            .find(|b| b.bus_id == bus_id)
            .map(|b| &b.coord)
    }

    // -----------------------------------------------------------------------
    // GeoJSON export
    // -----------------------------------------------------------------------

    /// Export the network as a GeoJSON `FeatureCollection` string.
    ///
    /// Buses are exported as `Point` features; branches as `LineString` features.
    pub fn to_geojson(&self) -> String {
        let mut features: Vec<String> = Vec::new();

        // Buses → Points
        for bus in &self.buses {
            let props = format!(
                r#"{{"bus_id":{},"name":"{}","voltage_kv":{},"type":"{}"}}"#,
                bus.bus_id,
                escape_json(&bus.name),
                bus.voltage_kv,
                bus.substation_type.as_str()
            );
            let geom = format!(
                r#"{{"type":"Point","coordinates":[{},{}]}}"#,
                bus.coord.longitude_deg, bus.coord.latitude_deg
            );
            features.push(format!(
                r#"{{"type":"Feature","geometry":{},"properties":{}}}"#,
                geom, props
            ));
        }

        // Branches → LineStrings
        for branch in &self.branches {
            let from_coord = self.bus_coord(branch.from_bus);
            let to_coord = self.bus_coord(branch.to_bus);
            if let (Some(fc), Some(tc)) = (from_coord, to_coord) {
                let mut coords = Vec::new();
                // from endpoint
                coords.push(format!("[{},{}]", fc.longitude_deg, fc.latitude_deg));
                // waypoints
                for wp in &branch.waypoints {
                    coords.push(format!("[{},{}]", wp.longitude_deg, wp.latitude_deg));
                }
                // to endpoint
                coords.push(format!("[{},{}]", tc.longitude_deg, tc.latitude_deg));

                let coord_str = coords.join(",");
                let geom = format!(r#"{{"type":"LineString","coordinates":[{}]}}"#, coord_str);
                let props = format!(
                    r#"{{"branch_id":{},"from_bus":{},"to_bus":{},"voltage_kv":{},"length_km":{:.3},"line_type":"{}"}}"#,
                    branch.branch_id,
                    branch.from_bus,
                    branch.to_bus,
                    branch.voltage_kv,
                    branch.length_km,
                    branch.line_type.as_str()
                );
                features.push(format!(
                    r#"{{"type":"Feature","geometry":{},"properties":{}}}"#,
                    geom, props
                ));
            }
        }

        let features_str = features.join(",");
        format!(
            r#"{{"type":"FeatureCollection","features":[{}]}}"#,
            features_str
        )
    }

    // -----------------------------------------------------------------------
    // Line parameter estimation
    // -----------------------------------------------------------------------

    /// Estimate electrical parameters for a branch from its length and type.
    ///
    /// Returns `(resistance_ohm, reactance_ohm, susceptance_siemens)` using
    /// typical per-unit-length values for each line technology and voltage class.
    ///
    /// # Typical values used
    ///
    /// | Type          | r \[Ω/km\] | x \[Ω/km\] | b \[μS/km\] |
    /// |---------------|-----------|-----------|------------|
    /// | OverheadAc    | 0.05      | 0.40      | 3.0        |
    /// | OverheadDc    | 0.04      | 0.00      | 0.0        |
    /// | UndergroundAc | 0.10      | 0.15      | 200.0      |
    /// | UndergroundDc | 0.08      | 0.00      | 0.0        |
    /// | Submarine     | 0.12      | 0.18      | 150.0      |
    pub fn estimate_line_parameters(&self, branch_id: usize) -> Result<(f64, f64, f64), GeoError> {
        let branch = self
            .branches
            .iter()
            .find(|br| br.branch_id == branch_id)
            .ok_or(GeoError::BranchNotFound(branch_id))?;

        // Per-km parameters (r [Ω/km], x [Ω/km], b [S/km])
        let (r_km, x_km, b_km_us) = match branch.line_type {
            TransmissionLineType::OverheadAc => (0.05, 0.40, 3.0e-6),
            TransmissionLineType::OverheadDc => (0.04, 0.00, 0.0),
            TransmissionLineType::UndergroundAc => (0.10, 0.15, 200.0e-6),
            TransmissionLineType::UndergroundDc => (0.08, 0.00, 0.0),
            TransmissionLineType::Submarine => (0.12, 0.18, 150.0e-6),
        };

        let len = branch.length_km;
        Ok((r_km * len, x_km * len, b_km_us * len))
    }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// Test whether line segment (p1→p2) crosses (p3→p4) in 2D (lon/lat plane).
///
/// Uses the parametric cross-product method.
fn segments_intersect_2d(
    p1: &GeoCoordinate,
    p2: &GeoCoordinate,
    p3: &GeoCoordinate,
    p4: &GeoCoordinate,
) -> bool {
    // Represent as 2D vectors (use lon as x, lat as y)
    let d1x = p2.longitude_deg - p1.longitude_deg;
    let d1y = p2.latitude_deg - p1.latitude_deg;
    let d2x = p4.longitude_deg - p3.longitude_deg;
    let d2y = p4.latitude_deg - p3.latitude_deg;

    let denom = d1x * d2y - d1y * d2x;
    if denom.abs() < 1e-12 {
        // Parallel or collinear
        return false;
    }

    let dx = p3.longitude_deg - p1.longitude_deg;
    let dy = p3.latitude_deg - p1.latitude_deg;

    let t = (dx * d2y - dy * d2x) / denom;
    let s = (dx * d1y - dy * d1x) / denom;

    // Exclude endpoint touches (strictly interior crossing)
    t > 1e-10 && t < (1.0 - 1e-10) && s > 1e-10 && s < (1.0 - 1e-10)
}

/// Escape a string for embedding in JSON.
fn escape_json(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
        .replace('\r', "\\r")
        .replace('\t', "\\t")
}

// ===========================================================================
// Extended geospatial analysis API
// ===========================================================================

// ---------------------------------------------------------------------------
// GeoPoint
// ---------------------------------------------------------------------------

/// Geographic coordinate in WGS84 datum (latitude / longitude / elevation).
#[derive(Clone, Debug, PartialEq)]
pub struct GeoPoint {
    /// Latitude in decimal degrees, −90 … +90.
    pub lat: f64,
    /// Longitude in decimal degrees, −180 … +180.
    pub lon: f64,
    /// Elevation above mean sea level \[m\].
    pub elevation_m: f64,
}

impl GeoPoint {
    /// Create a point at sea level.
    pub fn new(lat: f64, lon: f64) -> Self {
        Self {
            lat,
            lon,
            elevation_m: 0.0,
        }
    }

    /// Create a point with explicit elevation \[m\].
    pub fn new_with_elevation(lat: f64, lon: f64, elevation_m: f64) -> Self {
        Self {
            lat,
            lon,
            elevation_m,
        }
    }

    /// Haversine great-circle distance to `other` \[m\].
    ///
    /// Earth mean radius R = 6 371 000 m.
    pub fn haversine_distance(&self, other: &GeoPoint) -> f64 {
        const R: f64 = 6_371_000.0;
        let dlat = (other.lat - self.lat).to_radians();
        let dlon = (other.lon - self.lon).to_radians();
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        R * c
    }

    /// Initial bearing (clockwise from North) to `other` \[0, 360).
    pub fn bearing_to(&self, other: &GeoPoint) -> f64 {
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let dlon = (other.lon - self.lon).to_radians();
        let y = dlon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
        let bearing = y.atan2(x).to_degrees();
        (bearing + 360.0) % 360.0
    }

    /// Spherical midpoint between `self` and `other`.
    pub fn midpoint(&self, other: &GeoPoint) -> GeoPoint {
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let lon1 = self.lon.to_radians();
        let dlon = (other.lon - self.lon).to_radians();
        let bx = lat2.cos() * dlon.cos();
        let by = lat2.cos() * dlon.sin();
        let lat_m =
            (lat1.sin() + lat2.sin()).atan2(((lat1.cos() + bx).powi(2) + by.powi(2)).sqrt());
        let lon_m = lon1 + by.atan2(lat1.cos() + bx);
        let elev = (self.elevation_m + other.elevation_m) / 2.0;
        GeoPoint::new_with_elevation(lat_m.to_degrees(), lon_m.to_degrees(), elev)
    }

    /// Destination point given `distance_m` \[m\] and `bearing_deg` (clockwise from North).
    pub fn destination(&self, distance_m: f64, bearing_deg: f64) -> GeoPoint {
        const R: f64 = 6_371_000.0;
        let delta = distance_m / R;
        let theta = bearing_deg.to_radians();
        let lat1 = self.lat.to_radians();
        let lon1 = self.lon.to_radians();
        let lat2 = (lat1.sin() * delta.cos() + lat1.cos() * delta.sin() * theta.cos()).asin();
        let lon2 = lon1
            + (theta.sin() * delta.sin() * lat1.cos()).atan2(delta.cos() - lat1.sin() * lat2.sin());
        GeoPoint::new(lat2.to_degrees(), lon2.to_degrees())
    }
}

// ---------------------------------------------------------------------------
// GeoBoundingBox
// ---------------------------------------------------------------------------

/// Axis-aligned geographic bounding box.
#[derive(Clone, Debug)]
pub struct GeoBoundingBox {
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
}

impl GeoBoundingBox {
    pub fn new(min_lat: f64, max_lat: f64, min_lon: f64, max_lon: f64) -> Self {
        Self {
            min_lat,
            max_lat,
            min_lon,
            max_lon,
        }
    }

    /// Returns `true` if `point` is inside or on the boundary.
    pub fn contains(&self, point: &GeoPoint) -> bool {
        point.lat >= self.min_lat
            && point.lat <= self.max_lat
            && point.lon >= self.min_lon
            && point.lon <= self.max_lon
    }

    /// Expand all edges by `margin_deg` degrees.
    pub fn expand(&self, margin_deg: f64) -> GeoBoundingBox {
        GeoBoundingBox {
            min_lat: self.min_lat - margin_deg,
            max_lat: self.max_lat + margin_deg,
            min_lon: self.min_lon - margin_deg,
            max_lon: self.max_lon + margin_deg,
        }
    }

    /// Centre point.
    pub fn center(&self) -> GeoPoint {
        GeoPoint::new(
            (self.min_lat + self.max_lat) / 2.0,
            (self.min_lon + self.max_lon) / 2.0,
        )
    }

    /// Great-circle distance between SW and NE corners \[km\].
    pub fn diagonal_distance_km(&self) -> f64 {
        let sw = GeoPoint::new(self.min_lat, self.min_lon);
        let ne = GeoPoint::new(self.max_lat, self.max_lon);
        sw.haversine_distance(&ne) / 1000.0
    }

    /// Tightest bounding box for `points`; `None` if the slice is empty.
    pub fn from_points(points: &[GeoPoint]) -> Option<GeoBoundingBox> {
        let mut iter = points.iter();
        let first = iter.next()?;
        let mut min_lat = first.lat;
        let mut max_lat = first.lat;
        let mut min_lon = first.lon;
        let mut max_lon = first.lon;
        for p in iter {
            if p.lat < min_lat {
                min_lat = p.lat;
            }
            if p.lat > max_lat {
                max_lat = p.lat;
            }
            if p.lon < min_lon {
                min_lon = p.lon;
            }
            if p.lon > max_lon {
                max_lon = p.lon;
            }
        }
        Some(GeoBoundingBox {
            min_lat,
            max_lat,
            min_lon,
            max_lon,
        })
    }
}

// ---------------------------------------------------------------------------
// GeoNodeType / GeoNode
// ---------------------------------------------------------------------------

/// Classification of a network node in geographic context.
#[derive(Clone, Debug)]
pub enum GeoNodeType {
    TransmissionSubstation,
    DistributionSubstation,
    GenerationPlant { fuel_type: String },
    LoadCenter { customer_count: u32 },
    WindFarm { capacity_mw: f64 },
    SolarFarm { capacity_mw: f64 },
    HvdcTerminal,
}

/// A geospatially-aware power network node.
#[derive(Clone, Debug)]
pub struct GeoNode {
    pub id: usize,
    pub name: String,
    pub location: GeoPoint,
    pub node_type: GeoNodeType,
    pub voltage_kv: f64,
    pub rated_mva: f64,
}

// ---------------------------------------------------------------------------
// LineType / GeoLine
// ---------------------------------------------------------------------------

/// Physical medium of a transmission line.
#[derive(Clone, Debug, PartialEq)]
pub enum LineType {
    Overhead,
    UndergroundCable,
    OffshoreSubmarineCable,
}

/// A geospatially-aware transmission line with waypoints.
#[derive(Clone, Debug)]
pub struct GeoLine {
    pub id: usize,
    pub from_node: usize,
    pub to_node: usize,
    /// Ordered waypoints **including** both terminal endpoints.
    pub waypoints: Vec<GeoPoint>,
    pub voltage_kv: f64,
    pub capacity_mw: f64,
    pub line_type: LineType,
}

impl GeoLine {
    /// Total route length as the sum of segment haversine distances \[km\].
    pub fn total_length_km(&self) -> f64 {
        if self.waypoints.len() < 2 {
            return 0.0;
        }
        self.waypoints
            .windows(2)
            .map(|seg| seg[0].haversine_distance(&seg[1]) / 1000.0)
            .sum()
    }

    /// Bounding box of the full line corridor; `None` if waypoints is empty.
    pub fn corridor_bbox(&self) -> Option<GeoBoundingBox> {
        GeoBoundingBox::from_points(&self.waypoints)
    }

    /// Returns `true` if the line passes within `radius_km` of `point`.
    pub fn passes_near(&self, point: &GeoPoint, radius_km: f64) -> bool {
        if self.waypoints.is_empty() {
            return false;
        }
        for wp in &self.waypoints {
            if wp.haversine_distance(point) / 1000.0 <= radius_km {
                return true;
            }
        }
        for seg in self.waypoints.windows(2) {
            if geo_point_to_segment_dist_km(point, &seg[0], &seg[1]) <= radius_km {
                return true;
            }
        }
        false
    }
}

/// Approximate perpendicular distance from `p` to segment \[a, b\] \[km\].
fn geo_point_to_segment_dist_km(p: &GeoPoint, a: &GeoPoint, b: &GeoPoint) -> f64 {
    let ab_lat = b.lat - a.lat;
    let ab_lon = b.lon - a.lon;
    let ap_lat = p.lat - a.lat;
    let ap_lon = p.lon - a.lon;
    let ab_sq = ab_lat * ab_lat + ab_lon * ab_lon;
    let t = if ab_sq < f64::EPSILON {
        0.0_f64
    } else {
        ((ap_lat * ab_lat + ap_lon * ab_lon) / ab_sq).clamp(0.0, 1.0)
    };
    let proj = GeoPoint::new(a.lat + t * ab_lat, a.lon + t * ab_lon);
    p.haversine_distance(&proj) / 1000.0
}

// ---------------------------------------------------------------------------
// Geographic k-means clustering
// ---------------------------------------------------------------------------

/// Input configuration for geographic clustering.
pub struct GeographicClustering {
    pub nodes: Vec<GeoNode>,
    pub n_clusters: usize,
}

/// Output of a clustering run.
pub struct ClusterResult {
    /// Cluster assignment for each node (indexed by position in the nodes vec).
    pub assignments: Vec<usize>,
    /// Geographic centroid of each cluster.
    pub centroids: Vec<GeoPoint>,
    /// Maximum distance from any node to its cluster centroid \[km\].
    pub cluster_radii_km: Vec<f64>,
    /// Indices (into the `lines` slice) of lines internal to each cluster.
    pub intra_cluster_lines: Vec<Vec<usize>>,
    /// Indices (into the `lines` slice) of lines crossing cluster boundaries.
    pub inter_cluster_lines: Vec<usize>,
}

impl GeographicClustering {
    pub fn new(nodes: Vec<GeoNode>, n_clusters: usize) -> Self {
        Self { nodes, n_clusters }
    }

    /// Geographic k-means on the sphere (maximally-spread initialisation).
    pub fn cluster_kmeans(
        &mut self,
        lines: &[GeoLine],
        max_iter: usize,
    ) -> Result<ClusterResult, crate::error::OxiGridError> {
        let n = self.nodes.len();
        let k = self.n_clusters;
        if k == 0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "n_clusters must be ≥ 1".into(),
            ));
        }
        if n == 0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "nodes list is empty".into(),
            ));
        }
        let k_eff = k.min(n);

        // Greedy maximally-spread initialisation.
        let mut centroid_indices: Vec<usize> = vec![0];
        while centroid_indices.len() < k_eff {
            let mut best_idx = 0usize;
            let mut best_dist = -1.0_f64;
            for (i, node) in self.nodes.iter().enumerate() {
                if centroid_indices.contains(&i) {
                    continue;
                }
                let min_d = centroid_indices
                    .iter()
                    .map(|&ci| node.location.haversine_distance(&self.nodes[ci].location))
                    .fold(f64::MAX, f64::min);
                if min_d > best_dist {
                    best_dist = min_d;
                    best_idx = i;
                }
            }
            centroid_indices.push(best_idx);
        }

        let mut centroids: Vec<GeoPoint> = centroid_indices
            .iter()
            .map(|&i| self.nodes[i].location.clone())
            .collect();
        let mut assignments = vec![0usize; n];

        for _ in 0..max_iter {
            let mut changed = false;
            for (i, node) in self.nodes.iter().enumerate() {
                let nearest = centroids
                    .iter()
                    .enumerate()
                    .map(|(ci, c)| (ci, node.location.haversine_distance(c)))
                    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                    .map(|(ci, _)| ci)
                    .unwrap_or(0);
                if assignments[i] != nearest {
                    assignments[i] = nearest;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
            #[allow(clippy::needless_range_loop)]
            for c in 0..k_eff {
                let pts: Vec<GeoPoint> = self
                    .nodes
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| assignments[*i] == c)
                    .map(|(_, node)| node.location.clone())
                    .collect();
                if !pts.is_empty() {
                    centroids[c] = Self::spherical_centroid(&pts);
                }
            }
        }

        let mut cluster_radii_km = vec![0.0_f64; k_eff];
        for (i, node) in self.nodes.iter().enumerate() {
            let c = assignments[i];
            let d = node.location.haversine_distance(&centroids[c]) / 1000.0;
            if d > cluster_radii_km[c] {
                cluster_radii_km[c] = d;
            }
        }

        let id_to_cluster: std::collections::HashMap<usize, usize> = self
            .nodes
            .iter()
            .enumerate()
            .map(|(i, node)| (node.id, assignments[i]))
            .collect();

        let mut intra_cluster_lines: Vec<Vec<usize>> = vec![Vec::new(); k_eff];
        let mut inter_cluster_lines: Vec<usize> = Vec::new();
        for (li, line) in lines.iter().enumerate() {
            match (
                id_to_cluster.get(&line.from_node),
                id_to_cluster.get(&line.to_node),
            ) {
                (Some(&fc), Some(&tc)) if fc == tc => intra_cluster_lines[fc].push(li),
                _ => inter_cluster_lines.push(li),
            }
        }

        Ok(ClusterResult {
            assignments,
            centroids,
            cluster_radii_km,
            intra_cluster_lines,
            inter_cluster_lines,
        })
    }

    /// Spherical centroid via unit-vector Cartesian averaging.
    pub fn spherical_centroid(points: &[GeoPoint]) -> GeoPoint {
        if points.is_empty() {
            return GeoPoint::new(0.0, 0.0);
        }
        let (mut sx, mut sy, mut sz) = (0.0_f64, 0.0_f64, 0.0_f64);
        for p in points {
            let lat = p.lat.to_radians();
            let lon = p.lon.to_radians();
            sx += lat.cos() * lon.cos();
            sy += lat.cos() * lon.sin();
            sz += lat.sin();
        }
        let n = points.len() as f64;
        sx /= n;
        sy /= n;
        sz /= n;
        let hyp = (sx * sx + sy * sy).sqrt();
        GeoPoint::new(sz.atan2(hyp).to_degrees(), sy.atan2(sx).to_degrees())
    }
}

// ---------------------------------------------------------------------------
// TransmissionRouter
// ---------------------------------------------------------------------------

/// Routes transmission lines around geographic exclusion zones.
pub struct TransmissionRouter {
    pub terrain_resolution_km: f64,
    pub exclusion_zones: Vec<GeoBoundingBox>,
    /// Cost multiplier per km for overhead lines \[million EUR/km\].
    pub cost_per_km_overhead: f64,
    /// Cost multiplier per km for underground/submarine cables \[million EUR/km\].
    pub cost_per_km_underground: f64,
}

/// Result of a routing computation.
pub struct RoutingResult {
    /// Ordered waypoints (including start and end).
    pub waypoints: Vec<GeoPoint>,
    /// Total route length \[km\].
    pub total_length_km: f64,
    /// Total estimated capital cost \[million EUR\].
    pub total_cost_million_eur: f64,
    /// Number of exclusion zones that the final route still crosses.
    pub n_exclusion_zone_crossings: usize,
    /// Heuristic terrain complexity in \[0, 1\].
    pub terrain_complexity: f64,
}

impl TransmissionRouter {
    pub fn new(
        terrain_resolution_km: f64,
        exclusion_zones: Vec<GeoBoundingBox>,
        cost_per_km_overhead: f64,
        cost_per_km_underground: f64,
    ) -> Self {
        Self {
            terrain_resolution_km,
            exclusion_zones,
            cost_per_km_overhead,
            cost_per_km_underground,
        }
    }

    /// Route between `from` and `to`, inserting detour waypoints around exclusion zones.
    pub fn route(&self, from: &GeoPoint, to: &GeoPoint, line_type: &LineType) -> RoutingResult {
        let mut waypoints: Vec<GeoPoint> = vec![from.clone(), to.clone()];
        let max_passes = self.exclusion_zones.len() * 2 + 1;

        for _ in 0..max_passes {
            let mut inserted = false;
            let mut new_wps: Vec<GeoPoint> = Vec::new();
            let n = waypoints.len();
            for seg_i in 0..n.saturating_sub(1) {
                new_wps.push(waypoints[seg_i].clone());
                let a = &waypoints[seg_i];
                let b = &waypoints[seg_i + 1];
                for zone in &self.exclusion_zones {
                    if Self::segment_crosses_bbox(a, b, zone) {
                        // Expand by 1.0° so the corner waypoint is well outside
                        // the original (unexpanded) zone boundary.
                        let exp = zone.expand(1.0);
                        new_wps.push(Self::best_corner_detour(a, b, &exp));
                        inserted = true;
                        break;
                    }
                }
            }
            if let Some(last) = waypoints.last() {
                new_wps.push(last.clone());
            }
            waypoints = new_wps;
            if !inserted {
                break;
            }
        }

        waypoints.dedup_by(|a, b| (a.lat - b.lat).abs() < 1e-9 && (a.lon - b.lon).abs() < 1e-9);

        let mut crossings = 0usize;
        let n = waypoints.len();
        for seg_i in 0..n.saturating_sub(1) {
            for zone in &self.exclusion_zones {
                if Self::segment_crosses_bbox(&waypoints[seg_i], &waypoints[seg_i + 1], zone) {
                    crossings += 1;
                }
            }
        }

        let total_length_km: f64 = waypoints
            .windows(2)
            .map(|seg| seg[0].haversine_distance(&seg[1]) / 1000.0)
            .sum();

        let direct_km = from.haversine_distance(to) / 1000.0;
        let terrain_complexity = if direct_km > 1e-6 {
            (1.0 - direct_km / total_length_km.max(direct_km)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let cost_rate = match line_type {
            LineType::Overhead => self.cost_per_km_overhead,
            LineType::UndergroundCable | LineType::OffshoreSubmarineCable => {
                self.cost_per_km_underground
            }
        };

        RoutingResult {
            waypoints,
            total_length_km,
            total_cost_million_eur: total_length_km * cost_rate,
            n_exclusion_zone_crossings: crossings,
            terrain_complexity,
        }
    }

    /// Sample-based check: does the linear segment [a, b] cross `bbox`?
    fn segment_crosses_bbox(a: &GeoPoint, b: &GeoPoint, bbox: &GeoBoundingBox) -> bool {
        const SAMPLES: usize = 20;
        if bbox.contains(a) || bbox.contains(b) {
            return true;
        }
        for i in 1..SAMPLES {
            let t = i as f64 / SAMPLES as f64;
            if bbox.contains(&GeoPoint::new(
                a.lat + t * (b.lat - a.lat),
                a.lon + t * (b.lon - a.lon),
            )) {
                return true;
            }
        }
        false
    }

    /// Return the bbox corner that minimises the total detour distance.
    fn best_corner_detour(a: &GeoPoint, b: &GeoPoint, exp: &GeoBoundingBox) -> GeoPoint {
        let corners = [
            GeoPoint::new(exp.min_lat, exp.min_lon),
            GeoPoint::new(exp.min_lat, exp.max_lon),
            GeoPoint::new(exp.max_lat, exp.min_lon),
            GeoPoint::new(exp.max_lat, exp.max_lon),
        ];
        corners
            .into_iter()
            .min_by(|c1, c2| {
                let d1 = a.haversine_distance(c1) + c1.haversine_distance(b);
                let d2 = a.haversine_distance(c2) + c2.haversine_distance(b);
                d1.partial_cmp(&d2).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or_else(|| GeoPoint::new(exp.min_lat, exp.min_lon))
    }
}

// ---------------------------------------------------------------------------
// SpatialAnalysis
// ---------------------------------------------------------------------------

/// Stateless spatial analysis utilities.
pub struct SpatialAnalysis;

impl SpatialAnalysis {
    /// Indices (into `nodes`) of nodes within `radius_km` of `center`.
    pub fn nodes_within_radius(nodes: &[GeoNode], center: &GeoPoint, radius_km: f64) -> Vec<usize> {
        nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| node.location.haversine_distance(center) / 1000.0 <= radius_km)
            .map(|(i, _)| i)
            .collect()
    }

    /// Index of the node nearest to `point`; `None` if the slice is empty.
    pub fn nearest_node(nodes: &[GeoNode], point: &GeoPoint) -> Option<usize> {
        nodes
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = a.location.haversine_distance(point);
                let db = b.location.haversine_distance(point);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
    }

    /// Network density: nodes per 100 km² of coverage area.
    pub fn network_density(nodes: &[GeoNode]) -> f64 {
        if nodes.is_empty() {
            return 0.0;
        }
        let area = Self::coverage_area_km2(nodes);
        if area < 1e-6 {
            return 0.0;
        }
        nodes.len() as f64 / area * 100.0
    }

    fn cross2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
        (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    }

    /// Returns `true` if 2-D segments \[a1, a2\] and \[b1, b2\] intersect.
    pub fn lines_cross(a1: &GeoPoint, a2: &GeoPoint, b1: &GeoPoint, b2: &GeoPoint) -> bool {
        let d1 = Self::cross2d(b1.lon, b1.lat, b2.lon, b2.lat, a1.lon, a1.lat);
        let d2 = Self::cross2d(b1.lon, b1.lat, b2.lon, b2.lat, a2.lon, a2.lat);
        let d3 = Self::cross2d(a1.lon, a1.lat, a2.lon, a2.lat, b1.lon, b1.lat);
        let d4 = Self::cross2d(a1.lon, a1.lat, a2.lon, a2.lat, b2.lon, b2.lat);
        if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
            && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
        {
            return true;
        }
        if d1.abs() < f64::EPSILON && Self::on_seg_2d(b1, b2, a1) {
            return true;
        }
        if d2.abs() < f64::EPSILON && Self::on_seg_2d(b1, b2, a2) {
            return true;
        }
        if d3.abs() < f64::EPSILON && Self::on_seg_2d(a1, a2, b1) {
            return true;
        }
        if d4.abs() < f64::EPSILON && Self::on_seg_2d(a1, a2, b2) {
            return true;
        }
        false
    }

    fn on_seg_2d(a: &GeoPoint, b: &GeoPoint, p: &GeoPoint) -> bool {
        p.lon >= a.lon.min(b.lon)
            && p.lon <= a.lon.max(b.lon)
            && p.lat >= a.lat.min(b.lat)
            && p.lat <= a.lat.max(b.lat)
    }

    /// Find all pairs (i, j), i < j, of `GeoLine`s whose segments cross.
    pub fn find_crossing_lines(lines: &[GeoLine]) -> Vec<(usize, usize)> {
        let mut crossings = Vec::new();
        for i in 0..lines.len() {
            for j in (i + 1)..lines.len() {
                if Self::geo_lines_intersect(&lines[i], &lines[j]) {
                    crossings.push((i, j));
                }
            }
        }
        crossings
    }

    fn geo_lines_intersect(la: &GeoLine, lb: &GeoLine) -> bool {
        for sa in la.waypoints.windows(2) {
            for sb in lb.waypoints.windows(2) {
                if Self::lines_cross(&sa[0], &sa[1], &sb[0], &sb[1]) {
                    return true;
                }
            }
        }
        false
    }

    /// Weighted geographic centroid of `LoadCenter` nodes.
    ///
    /// Returns `None` if no load nodes exist.
    pub fn geographic_load_center(nodes: &[GeoNode]) -> Option<GeoPoint> {
        let mut pts: Vec<GeoPoint> = Vec::new();
        for node in nodes {
            if matches!(node.node_type, GeoNodeType::LoadCenter { .. }) {
                let weight = (node.rated_mva.max(1.0) as usize).min(1000);
                for _ in 0..weight {
                    pts.push(node.location.clone());
                }
            }
        }
        if pts.is_empty() {
            return None;
        }
        Some(GeographicClustering::spherical_centroid(&pts))
    }

    /// Approximate coverage area \[km²\] via shoelace on angle-sorted nodes.
    pub fn coverage_area_km2(nodes: &[GeoNode]) -> f64 {
        if nodes.len() < 3 {
            return 0.0;
        }
        let cx: f64 = nodes.iter().map(|n| n.location.lon).sum::<f64>() / nodes.len() as f64;
        let cy: f64 = nodes.iter().map(|n| n.location.lat).sum::<f64>() / nodes.len() as f64;
        let mut pts: Vec<(f64, f64)> = nodes
            .iter()
            .map(|n| (n.location.lon, n.location.lat))
            .collect();
        pts.sort_by(|a, b| {
            let ang_a = (a.1 - cy).atan2(a.0 - cx);
            let ang_b = (b.1 - cy).atan2(b.0 - cx);
            ang_a
                .partial_cmp(&ang_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let n = pts.len();
        let mut area_deg2 = 0.0_f64;
        for i in 0..n {
            let j = (i + 1) % n;
            area_deg2 += pts[i].0 * pts[j].1;
            area_deg2 -= pts[j].0 * pts[i].1;
        }
        area_deg2 = area_deg2.abs() / 2.0;
        let cos_lat = cy.to_radians().cos();
        area_deg2 * 111.32 * 111.32 * cos_lat
    }
}

// ---------------------------------------------------------------------------
// Extended-API tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod geo_tests {
    use super::*;

    fn make_node(id: usize, lat: f64, lon: f64) -> GeoNode {
        GeoNode {
            id,
            name: format!("node_{id}"),
            location: GeoPoint::new(lat, lon),
            node_type: GeoNodeType::TransmissionSubstation,
            voltage_kv: 110.0,
            rated_mva: 100.0,
        }
    }

    // --- GeoPoint ---

    #[test]
    fn test_geopoint_haversine_known_distance() {
        // London to Paris ≈ 340 km.
        let london = GeoPoint::new(51.5074, -0.1278);
        let paris = GeoPoint::new(48.8566, 2.3522);
        let d_km = london.haversine_distance(&paris) / 1000.0;
        assert!(d_km > 330.0 && d_km < 350.0, "London-Paris = {d_km:.1} km");
    }

    #[test]
    fn test_geopoint_bearing_north() {
        let origin = GeoPoint::new(51.0, 0.0);
        let north = GeoPoint::new(55.0, 0.0);
        let b = origin.bearing_to(&north);
        assert!(b < 1.0 || (b - 360.0).abs() < 1.0, "bearing = {b}");
    }

    #[test]
    fn test_geopoint_bearing_east() {
        let origin = GeoPoint::new(0.0, 0.0);
        let east = GeoPoint::new(0.0, 10.0);
        let b = origin.bearing_to(&east);
        assert!((b - 90.0).abs() < 1.0, "bearing = {b}");
    }

    #[test]
    fn test_geopoint_midpoint() {
        let a = GeoPoint::new(0.0, 0.0);
        let b = GeoPoint::new(0.0, 10.0);
        let m = a.midpoint(&b);
        assert!(m.lat.abs() < 0.01);
        assert!((m.lon - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_geopoint_destination() {
        // 1 000 000 m due north from equator ≈ 8.99° latitude.
        let origin = GeoPoint::new(0.0, 0.0);
        let dest = origin.destination(1_000_000.0, 0.0);
        assert!(dest.lat > 8.0 && dest.lat < 10.0, "lat = {}", dest.lat);
        assert!(dest.lon.abs() < 0.5, "lon = {}", dest.lon);
    }

    // --- GeoBoundingBox ---

    #[test]
    fn test_bounding_box_contains() {
        let bbox = GeoBoundingBox::new(48.0, 52.0, -1.0, 3.0);
        assert!(bbox.contains(&GeoPoint::new(50.0, 1.0)));
    }

    #[test]
    fn test_bounding_box_not_contains() {
        let bbox = GeoBoundingBox::new(48.0, 52.0, -1.0, 3.0);
        assert!(!bbox.contains(&GeoPoint::new(53.0, 1.0)));
    }

    #[test]
    fn test_bounding_box_from_points() {
        let pts = vec![
            GeoPoint::new(48.0, -1.0),
            GeoPoint::new(52.0, 3.0),
            GeoPoint::new(50.0, 1.0),
        ];
        let bbox = GeoBoundingBox::from_points(&pts).expect("should produce bbox");
        assert_eq!(bbox.min_lat, 48.0);
        assert_eq!(bbox.max_lat, 52.0);
        assert_eq!(bbox.min_lon, -1.0);
        assert_eq!(bbox.max_lon, 3.0);
    }

    #[test]
    fn test_bounding_box_diagonal() {
        let bbox = GeoBoundingBox::new(0.0, 1.0, 0.0, 1.0);
        let d = bbox.diagonal_distance_km();
        assert!(d > 130.0 && d < 170.0, "diagonal = {d:.1} km");
    }

    // --- GeoLine ---

    #[test]
    fn test_geoline_length() {
        let line = GeoLine {
            id: 0,
            from_node: 0,
            to_node: 1,
            waypoints: vec![
                GeoPoint::new(48.8566, 2.3522),
                GeoPoint::new(51.5074, -0.1278),
            ],
            voltage_kv: 400.0,
            capacity_mw: 1000.0,
            line_type: LineType::Overhead,
        };
        let len = line.total_length_km();
        assert!(len > 330.0 && len < 350.0, "length = {len:.1} km");
    }

    #[test]
    fn test_geoline_passes_near() {
        let line = GeoLine {
            id: 1,
            from_node: 0,
            to_node: 1,
            waypoints: vec![GeoPoint::new(48.0, 0.0), GeoPoint::new(52.0, 0.0)],
            voltage_kv: 400.0,
            capacity_mw: 1000.0,
            line_type: LineType::Overhead,
        };
        assert!(line.passes_near(&GeoPoint::new(50.0, 0.05), 10.0));
        assert!(!line.passes_near(&GeoPoint::new(50.0, 5.0), 10.0));
    }

    #[test]
    fn test_geoline_corridor_bbox() {
        let line = GeoLine {
            id: 2,
            from_node: 0,
            to_node: 2,
            waypoints: vec![
                GeoPoint::new(48.0, 2.0),
                GeoPoint::new(50.0, 5.0),
                GeoPoint::new(52.0, 3.0),
            ],
            voltage_kv: 220.0,
            capacity_mw: 500.0,
            line_type: LineType::Overhead,
        };
        let bbox = line.corridor_bbox().expect("should have bbox");
        assert_eq!(bbox.min_lat, 48.0);
        assert_eq!(bbox.max_lat, 52.0);
        assert_eq!(bbox.min_lon, 2.0);
        assert_eq!(bbox.max_lon, 5.0);
    }

    // --- Clustering ---

    #[test]
    fn test_clustering_2_clusters() {
        let nodes = vec![
            make_node(0, 48.0, 2.0),
            make_node(1, 51.0, 0.0),
            make_node(2, 40.0, -74.0),
            make_node(3, 45.0, -75.0),
        ];
        let mut gc = GeographicClustering::new(nodes, 2);
        let res = gc.cluster_kmeans(&[], 50).expect("should succeed");
        assert_eq!(res.assignments[0], res.assignments[1]);
        assert_eq!(res.assignments[2], res.assignments[3]);
        assert_ne!(res.assignments[0], res.assignments[2]);
    }

    #[test]
    fn test_clustering_3_clusters() {
        let nodes = vec![
            make_node(0, 48.0, 2.0),
            make_node(1, 49.0, 3.0),
            make_node(2, 40.0, -74.0),
            make_node(3, 41.0, -73.0),
            make_node(4, 35.0, 139.0),
            make_node(5, 36.0, 140.0),
        ];
        let mut gc = GeographicClustering::new(nodes, 3);
        let res = gc.cluster_kmeans(&[], 50).expect("should succeed");
        assert_eq!(res.assignments[0], res.assignments[1]);
        assert_eq!(res.assignments[2], res.assignments[3]);
        assert_eq!(res.assignments[4], res.assignments[5]);
    }

    #[test]
    fn test_spherical_centroid() {
        let pts = vec![GeoPoint::new(0.0, 0.0), GeoPoint::new(0.0, 10.0)];
        let c = GeographicClustering::spherical_centroid(&pts);
        assert!(c.lat.abs() < 0.1, "lat = {}", c.lat);
        assert!((c.lon - 5.0).abs() < 0.1, "lon = {}", c.lon);
    }

    // --- TransmissionRouter ---

    #[test]
    fn test_transmission_router_direct() {
        let router = TransmissionRouter::new(1.0, vec![], 1.0, 3.0);
        let from = GeoPoint::new(48.0, 2.0);
        let to = GeoPoint::new(51.0, 0.0);
        let res = router.route(&from, &to, &LineType::Overhead);
        assert!(res.total_length_km > 300.0);
        assert_eq!(res.n_exclusion_zone_crossings, 0);
        assert!(
            (res.terrain_complexity).abs() < 1e-6,
            "complexity = {}",
            res.terrain_complexity
        );
    }

    #[test]
    fn test_transmission_router_with_exclusion() {
        let zone = GeoBoundingBox::new(49.0, 50.0, 0.5, 1.5);
        let router = TransmissionRouter::new(1.0, vec![zone], 1.0, 3.0);
        let from = GeoPoint::new(48.0, 1.0);
        let to = GeoPoint::new(52.0, 1.0);
        let res = router.route(&from, &to, &LineType::Overhead);
        assert_eq!(res.n_exclusion_zone_crossings, 0);
        let direct = from.haversine_distance(&to) / 1000.0;
        assert!(
            res.total_length_km >= direct - 1.0,
            "routed={} direct={direct}",
            res.total_length_km
        );
    }

    // --- SpatialAnalysis ---

    #[test]
    fn test_nodes_within_radius() {
        let nodes = vec![
            make_node(0, 51.5, -0.1),
            make_node(1, 48.9, 2.4),
            make_node(2, 51.6, -0.2),
        ];
        let center = GeoPoint::new(51.5, -0.1);
        let within = SpatialAnalysis::nodes_within_radius(&nodes, &center, 50.0);
        assert!(within.contains(&0));
        assert!(within.contains(&2));
        assert!(!within.contains(&1));
    }

    #[test]
    fn test_nearest_node() {
        let nodes = vec![make_node(0, 51.5, -0.1), make_node(1, 48.9, 2.4)];
        let nearest = SpatialAnalysis::nearest_node(&nodes, &GeoPoint::new(51.4, -0.0))
            .expect("should find nearest");
        assert_eq!(nearest, 0);
    }

    #[test]
    fn test_lines_cross_detection() {
        let a1 = GeoPoint::new(0.0, 0.0);
        let a2 = GeoPoint::new(2.0, 2.0);
        let b1 = GeoPoint::new(0.0, 2.0);
        let b2 = GeoPoint::new(2.0, 0.0);
        assert!(SpatialAnalysis::lines_cross(&a1, &a2, &b1, &b2));

        let c1 = GeoPoint::new(0.0, 0.0);
        let c2 = GeoPoint::new(0.0, 2.0);
        let d1 = GeoPoint::new(1.0, 0.0);
        let d2 = GeoPoint::new(1.0, 2.0);
        assert!(!SpatialAnalysis::lines_cross(&c1, &c2, &d1, &d2));
    }

    #[test]
    fn test_coverage_area() {
        let nodes = vec![
            make_node(0, 0.0, 0.0),
            make_node(1, 0.0, 1.0),
            make_node(2, 1.0, 1.0),
            make_node(3, 1.0, 0.0),
        ];
        let area = SpatialAnalysis::coverage_area_km2(&nodes);
        assert!(area > 10_000.0 && area < 15_000.0, "area = {area:.0} km²");
    }

    #[test]
    fn test_geographic_load_center() {
        let nodes = vec![
            GeoNode {
                id: 0,
                name: "load_a".into(),
                location: GeoPoint::new(50.0, 0.0),
                node_type: GeoNodeType::LoadCenter {
                    customer_count: 1000,
                },
                voltage_kv: 20.0,
                rated_mva: 10.0,
            },
            GeoNode {
                id: 1,
                name: "load_b".into(),
                location: GeoPoint::new(50.0, 10.0),
                node_type: GeoNodeType::LoadCenter {
                    customer_count: 2000,
                },
                voltage_kv: 20.0,
                rated_mva: 10.0,
            },
            GeoNode {
                id: 2,
                name: "gen".into(),
                location: GeoPoint::new(48.0, 5.0),
                node_type: GeoNodeType::GenerationPlant {
                    fuel_type: "gas".into(),
                },
                voltage_kv: 110.0,
                rated_mva: 200.0,
            },
        ];
        let c = SpatialAnalysis::geographic_load_center(&nodes).expect("should find load center");
        assert!(c.lon > 3.0 && c.lon < 7.0, "lon = {}", c.lon);
        assert!((c.lat - 50.0).abs() < 1.0, "lat = {}", c.lat);
    }

    #[test]
    fn test_find_crossing_lines() {
        let la = GeoLine {
            id: 0,
            from_node: 0,
            to_node: 1,
            waypoints: vec![GeoPoint::new(0.0, 0.0), GeoPoint::new(2.0, 2.0)],
            voltage_kv: 110.0,
            capacity_mw: 300.0,
            line_type: LineType::Overhead,
        };
        let lb = GeoLine {
            id: 1,
            from_node: 2,
            to_node: 3,
            waypoints: vec![GeoPoint::new(0.0, 2.0), GeoPoint::new(2.0, 0.0)],
            voltage_kv: 110.0,
            capacity_mw: 300.0,
            line_type: LineType::Overhead,
        };
        let lc = GeoLine {
            id: 2,
            from_node: 4,
            to_node: 5,
            waypoints: vec![GeoPoint::new(5.0, 0.0), GeoPoint::new(5.0, 2.0)],
            voltage_kv: 110.0,
            capacity_mw: 300.0,
            line_type: LineType::Overhead,
        };
        let crossings = SpatialAnalysis::find_crossing_lines(&[la, lb, lc]);
        assert!(crossings.contains(&(0, 1)));
        assert!(!crossings.contains(&(0, 2)));
    }

    #[test]
    fn test_geopoint_elevation() {
        let p = GeoPoint::new_with_elevation(47.0, 8.0, 450.0);
        assert_eq!(p.elevation_m, 450.0);
    }

    #[test]
    fn test_bounding_box_expand() {
        let exp = GeoBoundingBox::new(49.0, 51.0, 1.0, 3.0).expand(0.5);
        assert!((exp.min_lat - 48.5).abs() < 1e-9);
        assert!((exp.max_lat - 51.5).abs() < 1e-9);
    }

    #[test]
    fn test_bounding_box_center() {
        let c = GeoBoundingBox::new(48.0, 52.0, 0.0, 4.0).center();
        assert!((c.lat - 50.0).abs() < 1e-9);
        assert!((c.lon - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_clustering_error_empty_nodes() {
        let mut gc = GeographicClustering::new(vec![], 2);
        assert!(gc.cluster_kmeans(&[], 10).is_err());
    }

    #[test]
    fn test_clustering_k_larger_than_nodes() {
        let nodes = vec![make_node(0, 48.0, 2.0), make_node(1, 51.0, 0.0)];
        let mut gc = GeographicClustering::new(nodes, 5);
        let res = gc.cluster_kmeans(&[], 10).expect("should succeed");
        assert_eq!(res.assignments.len(), 2);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn new_bus(id: usize, lat: f64, lon: f64, kv: f64) -> GeoBus {
        GeoBus {
            bus_id: id,
            name: format!("Bus{id}"),
            coord: GeoCoordinate::new(lat, lon),
            voltage_kv: kv,
            substation_type: SubstationType::Transmission,
        }
    }

    fn new_branch(id: usize, from: usize, to: usize, waypoints: Vec<GeoCoordinate>) -> GeoBranch {
        GeoBranch {
            branch_id: id,
            from_bus: from,
            to_bus: to,
            waypoints,
            line_type: TransmissionLineType::OverheadAc,
            voltage_kv: 345.0,
            length_km: 0.0,
        }
    }

    #[test]
    fn test_haversine_new_york_to_london() {
        // New York: 40.7128°N, 74.0060°W
        // London:   51.5074°N, 0.1278°W
        // Known geodesic ≈ 5570 km
        let ny = GeoCoordinate::new(40.7128, -74.0060);
        let london = GeoCoordinate::new(51.5074, -0.1278);
        let d = GeoNetwork::haversine_km(&ny, &london);
        assert!(
            (d - 5570.0).abs() < 50.0,
            "NY-London distance should be ~5570 km, got {d:.1} km"
        );
    }

    #[test]
    fn test_haversine_same_point_is_zero() {
        let p = GeoCoordinate::new(48.8566, 2.3522); // Paris
        let d = GeoNetwork::haversine_km(&p, &p);
        assert!(d < 1e-9, "same-point distance should be 0, got {d}");
    }

    #[test]
    fn test_branch_length_with_waypoints() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 40.0, -74.0, 345.0));
        net.add_bus(new_bus(2, 41.0, -73.0, 345.0));

        // Direct distance ≈ 130 km; add a waypoint at (40.5, -73.5)
        let mut br = new_branch(10, 1, 2, vec![GeoCoordinate::new(40.5, -73.5)]);
        net.add_branch(br.clone()).unwrap();
        br.branch_id = 10; // already pushed

        let len = net.branch_length_km(10).unwrap();
        // Should be longer than direct (waypoint adds detour)
        let direct = GeoNetwork::haversine_km(
            &GeoCoordinate::new(40.0, -74.0),
            &GeoCoordinate::new(41.0, -73.0),
        );
        assert!(
            len >= direct,
            "waypoint length {len:.2} should be >= direct {direct:.2}"
        );
        assert!(len > 50.0, "length should be non-trivial");
    }

    #[test]
    fn test_branch_length_no_waypoints() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 51.5, 0.0, 132.0));
        net.add_bus(new_bus(2, 52.5, 1.0, 132.0));
        net.add_branch(new_branch(5, 1, 2, vec![])).unwrap();
        let len = net.branch_length_km(5).unwrap();
        let expected = GeoNetwork::haversine_km(
            &GeoCoordinate::new(51.5, 0.0),
            &GeoCoordinate::new(52.5, 1.0),
        );
        assert!((len - expected).abs() < 0.1, "no-waypoint len mismatch");
    }

    #[test]
    fn test_buses_within_radius() {
        let mut net = GeoNetwork::new();
        // Paris
        net.add_bus(new_bus(1, 48.8566, 2.3522, 400.0));
        // Lyon (≈ 392 km from Paris)
        net.add_bus(new_bus(2, 45.7640, 4.8357, 225.0));
        // Bordeaux (≈ 499 km)
        net.add_bus(new_bus(3, 44.8378, -0.5792, 225.0));
        // Berlin (≈ 1050 km)
        net.add_bus(new_bus(4, 52.5200, 13.4050, 380.0));

        let paris = GeoCoordinate::new(48.8566, 2.3522);
        // Within 450 km: Paris + Lyon
        let nearby = net.buses_within_radius(&paris, 450.0);
        assert!(nearby.contains(&1), "Paris should be within 0 km of itself");
        assert!(nearby.contains(&2), "Lyon should be within 450 km of Paris");
        assert!(!nearby.contains(&3), "Bordeaux should NOT be within 450 km");
        assert!(!nearby.contains(&4), "Berlin should NOT be within 450 km");
    }

    #[test]
    fn test_network_centroid() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 0.0, 0.0, 230.0));
        net.add_bus(new_bus(2, 10.0, 10.0, 230.0));
        let c = net.network_centroid();
        assert!((c.latitude_deg - 5.0).abs() < 1e-9);
        assert!((c.longitude_deg - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_centroid_empty_network() {
        let net = GeoNetwork::new();
        let c = net.network_centroid();
        assert_eq!(c.latitude_deg, 0.0);
        assert_eq!(c.longitude_deg, 0.0);
    }

    #[test]
    fn test_geojson_export_structure() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 40.0, -74.0, 345.0));
        net.add_bus(new_bus(2, 41.0, -73.0, 345.0));
        net.add_branch(new_branch(1, 1, 2, vec![])).unwrap();
        let json = net.to_geojson();
        assert!(json.contains(r#""type":"FeatureCollection""#));
        assert!(json.contains(r#""type":"Feature""#));
        assert!(json.contains(r#""type":"Point""#));
        assert!(json.contains(r#""type":"LineString""#));
        assert!(json.contains("bus_id"));
        assert!(json.contains("branch_id"));
    }

    #[test]
    fn test_geojson_is_valid_json() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 48.8566, 2.3522, 400.0));
        net.add_bus(new_bus(2, 51.5074, -0.1278, 400.0));
        net.add_branch(new_branch(1, 1, 2, vec![])).unwrap();
        let json = net.to_geojson();
        // Must start and end with braces
        let trimmed = json.trim();
        assert!(trimmed.starts_with('{'), "GeoJSON must start with {{");
        assert!(trimmed.ends_with('}'), "GeoJSON must end with }}");
        // Basic brace balance check
        let opens: usize = json.chars().filter(|&c| c == '{').count();
        let closes: usize = json.chars().filter(|&c| c == '}').count();
        assert_eq!(opens, closes, "Unbalanced braces in GeoJSON output");
    }

    #[test]
    fn test_estimate_line_parameters_overhead_ac() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 0.0, 0.0, 345.0));
        net.add_bus(new_bus(2, 0.0, 1.0, 345.0)); // ~111 km apart
        net.add_branch(new_branch(1, 1, 2, vec![])).unwrap();
        let (r, x, b) = net.estimate_line_parameters(1).unwrap();
        assert!(r > 0.0, "resistance should be positive");
        assert!(x > 0.0, "reactance should be positive for OverheadAc");
        assert!(b > 0.0, "susceptance should be positive for OverheadAc");
        // For ~111 km: r ≈ 5.5 Ω, x ≈ 44 Ω
        assert!(
            (r - 5.5).abs() < 1.0,
            "resistance ~5.5 Ω for 111 km overhead AC"
        );
    }

    #[test]
    fn test_estimate_line_parameters_overhead_dc_no_reactance() {
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 0.0, 0.0, 500.0));
        net.add_bus(new_bus(2, 0.0, 1.0, 500.0));
        let mut br = new_branch(2, 1, 2, vec![]);
        br.line_type = TransmissionLineType::OverheadDc;
        net.add_branch(br).unwrap();
        let (_r, x, _b) = net.estimate_line_parameters(2).unwrap();
        assert_eq!(x, 0.0, "DC line has no reactance");
    }

    #[test]
    fn test_find_crossing_branches() {
        // Build an X-shaped crossing: branch 1 goes NW→SE, branch 2 goes NE→SW
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 10.0, 0.0, 345.0)); // north-west
        net.add_bus(new_bus(2, 0.0, 10.0, 345.0)); // south-east
        net.add_bus(new_bus(3, 10.0, 10.0, 345.0)); // north-east
        net.add_bus(new_bus(4, 0.0, 0.0, 345.0)); // south-west
        net.add_branch(new_branch(1, 1, 2, vec![])).unwrap(); // NW→SE
        net.add_branch(new_branch(2, 3, 4, vec![])).unwrap(); // NE→SW
        let crossings = net.find_crossing_branches();
        assert_eq!(crossings.len(), 1, "X-shaped branches should cross once");
        assert!(crossings.contains(&(1, 2)) || crossings.contains(&(2, 1)));
    }

    #[test]
    fn test_find_no_crossings_parallel() {
        // Two parallel N→S branches should not cross
        let mut net = GeoNetwork::new();
        net.add_bus(new_bus(1, 10.0, 0.0, 230.0));
        net.add_bus(new_bus(2, 0.0, 0.0, 230.0));
        net.add_bus(new_bus(3, 10.0, 5.0, 230.0));
        net.add_bus(new_bus(4, 0.0, 5.0, 230.0));
        net.add_branch(new_branch(1, 1, 2, vec![])).unwrap();
        net.add_branch(new_branch(2, 3, 4, vec![])).unwrap();
        let crossings = net.find_crossing_branches();
        assert!(crossings.is_empty(), "Parallel lines should not cross");
    }
}
