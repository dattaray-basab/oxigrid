/// Distance (impedance) relay models for transmission line protection.
///
/// Implements:
/// - **Mho** (circular) characteristic — traditional impedance relay
/// - **Quadrilateral** characteristic — modern numerical relay
/// - **Load encroachment blinder** — prevents unwanted trip during heavy load
/// - Zone 1/2/3 coordination with reach settings
/// - Phase-to-phase and phase-to-ground distance elements
///
/// # Method
/// The apparent impedance seen by the relay is:
///   Z_app = V_relay / I_relay  (in R+jX coordinates)
///
/// The relay trips when Z_app falls inside the operating characteristic.
///
/// # References
/// - Anderson, P.M., "Power System Protection", IEEE Press, 1999
/// - Blackburn, J.L., Domin, T.J., "Protective Relaying", 3rd Ed., CRC Press
/// - IEC 60255-121:2014 — Measuring relays: distance protection
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Relay zone
// ─────────────────────────────────────────────────────────────────────────────

/// Protection zone identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Zone {
    /// Zone 1 — instantaneous, ~80% of protected line
    Zone1,
    /// Zone 2 — time-delayed, ~120% of protected line
    Zone2,
    /// Zone 3 — backup, ~180–220% of protected line
    Zone3,
}

impl Zone {
    /// Default time delay for each zone in seconds.
    pub fn default_delay_s(&self) -> f64 {
        match self {
            Zone::Zone1 => 0.0,
            Zone::Zone2 => 0.3,
            Zone::Zone3 => 0.6,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Mho characteristic
// ─────────────────────────────────────────────────────────────────────────────

/// Mho (circular) distance relay characteristic.
///
/// The Mho circle passes through the origin in the R-X plane.
/// Operating condition: the apparent impedance falls inside the circle.
///
/// Centre of circle: `Z_reach / 2` (at angle `line_angle_deg`)
/// Radius: `|Z_reach| / 2`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MhoCharacteristic {
    /// Reach impedance [p.u. or Ω — must be consistent with measured Z]
    pub z_reach: Complex64,
    /// Line impedance angle `degrees` (positive sequence)
    pub line_angle_deg: f64,
    /// Zone identifier
    pub zone: Zone,
    /// Trip time delay `s`
    pub time_delay_s: f64,
}

impl MhoCharacteristic {
    /// Create a Mho relay for a given reach and line angle.
    pub fn new(z_reach: Complex64, line_angle_deg: f64, zone: Zone) -> Self {
        Self {
            z_reach,
            line_angle_deg,
            zone,
            time_delay_s: zone.default_delay_s(),
        }
    }

    /// Centre of the Mho circle in R-X plane.
    pub fn centre(&self) -> Complex64 {
        self.z_reach / 2.0
    }

    /// Radius of the Mho circle.
    pub fn radius(&self) -> f64 {
        self.z_reach.norm() / 2.0
    }

    /// Check if apparent impedance `z_app` falls within the Mho characteristic.
    ///
    /// Geometric condition: `|Z_app − centre| ≤ radius`
    pub fn is_inside(&self, z_app: Complex64) -> bool {
        (z_app - self.centre()).norm() <= self.radius()
    }

    /// Compute operating signal (0 = boundary, >0 inside, <0 outside).
    /// Normalised: positive = trip margin, negative = no-trip margin.
    pub fn operate_signal(&self, z_app: Complex64) -> f64 {
        self.radius() - (z_app - self.centre()).norm()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Quadrilateral characteristic
// ─────────────────────────────────────────────────────────────────────────────

/// Quadrilateral distance relay characteristic.
///
/// The operating region is a parallelogram in the R-X plane defined by:
/// - **Reactance reach** (X direction): `x_reach` (top boundary)
/// - **Resistive reach** (R direction): `r_reach` (left/right boundaries — blinders)
/// - **Reactance offset** (for ground faults): `x_offset` (bottom boundary, typically slightly negative)
/// - **Line angle** (tilt of left/right blinders)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuadrilateralCharacteristic {
    /// Forward reactance reach [p.u.]
    pub x_reach: f64,
    /// Resistive reach (half-width of quad on each side) [p.u.]
    pub r_reach: f64,
    /// Reactance offset (lower boundary — slightly below zero for ground faults)
    pub x_offset: f64,
    /// Line impedance angle `degrees` — tilts the resistance blinders
    pub line_angle_deg: f64,
    /// Zone identifier
    pub zone: Zone,
    /// Trip time delay `s`
    pub time_delay_s: f64,
}

impl QuadrilateralCharacteristic {
    /// Create a quadrilateral characteristic.
    pub fn new(x_reach: f64, r_reach: f64, line_angle_deg: f64, zone: Zone) -> Self {
        Self {
            x_reach,
            r_reach,
            x_offset: -0.05 * x_reach, // small offset below zero
            line_angle_deg,
            zone,
            time_delay_s: zone.default_delay_s(),
        }
    }

    /// Check if apparent impedance `z_app` falls within the quadrilateral.
    ///
    /// Uses rotated frame aligned with line angle to check blinder boundaries,
    /// plus the reactance reach and offset boundaries.
    pub fn is_inside(&self, z_app: Complex64) -> bool {
        let r = z_app.re;
        let x = z_app.im;

        // 1. Check reactance reach (horizontal boundaries)
        if x > self.x_reach || x < self.x_offset {
            return false;
        }

        // 2. Check resistance blinders (rotated by line angle)
        // The blinders are parallel to the line impedance vector.
        // Project z_app onto the direction perpendicular to the line angle.
        let phi = self.line_angle_deg.to_radians();
        // Perpendicular direction to line: (-sin(phi), cos(phi))
        let proj = r * (-phi.sin()) + x * phi.cos();

        // The blinder reach is r_reach projected
        proj.abs() <= self.r_reach
    }

    /// Margin to the characteristic boundary (positive = inside).
    /// Returns the minimum distance to any of the four boundaries.
    pub fn margin(&self, z_app: Complex64) -> f64 {
        let r = z_app.re;
        let x = z_app.im;
        let phi = self.line_angle_deg.to_radians();
        let proj = r * (-phi.sin()) + x * phi.cos();

        let margins = [
            self.x_reach - x,
            x - self.x_offset,
            self.r_reach - proj.abs(),
        ];
        margins.iter().cloned().fold(f64::INFINITY, f64::min)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Load encroachment blinder
// ─────────────────────────────────────────────────────────────────────────────

/// Load encroachment blinder — prevents distance relay from tripping on heavy load.
///
/// Defines a "load region" in the R-X plane: impedances with small angle
/// (high power factor) and large magnitude (heavy load, but not fault).
/// If Z_app falls in the load region, the relay is blocked.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadBlinder {
    /// Minimum load impedance magnitude [p.u.] — loads have |Z| > Z_load_min
    pub z_load_min: f64,
    /// Maximum load impedance angle `degrees` — loads have |angle| < phi_load_max
    pub phi_load_max_deg: f64,
}

impl LoadBlinder {
    /// Typical transmission system load blinder (|Z| > 0.8 p.u., angle < 30°).
    pub fn typical_transmission() -> Self {
        Self {
            z_load_min: 0.8,
            phi_load_max_deg: 30.0,
        }
    }

    /// Typical distribution system load blinder.
    pub fn typical_distribution() -> Self {
        Self {
            z_load_min: 0.5,
            phi_load_max_deg: 40.0,
        }
    }

    /// Returns `true` if the impedance is in the load region (relay should be blocked).
    pub fn is_load_region(&self, z_app: Complex64) -> bool {
        let mag = z_app.norm();
        let angle_deg = z_app.im.atan2(z_app.re).to_degrees().abs();
        mag >= self.z_load_min && angle_deg <= self.phi_load_max_deg
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Distance relay state machine
// ─────────────────────────────────────────────────────────────────────────────

/// Measurement input to the distance relay.
#[derive(Debug, Clone)]
pub struct RelayMeasurement {
    /// Relay voltage [p.u.]
    pub v_relay: Complex64,
    /// Relay current [p.u.]
    pub i_relay: Complex64,
    /// Zero-sequence compensation factor k0 (for ground distance)
    pub k0: Complex64,
}

impl RelayMeasurement {
    /// Compute apparent impedance for phase-to-phase element.
    pub fn z_apparent_phase(&self) -> Complex64 {
        if self.i_relay.norm() < 1e-9 {
            Complex64::new(f64::INFINITY, 0.0)
        } else {
            self.v_relay / self.i_relay
        }
    }

    /// Compute apparent impedance for ground distance element (with k0 compensation).
    ///
    /// Z = V / (I + k0 * I0)  where I0 = I_relay / 3 (simplified for loop)
    pub fn z_apparent_ground(&self, i_zero_seq: Complex64) -> Complex64 {
        let i_comp = self.i_relay + self.k0 * i_zero_seq;
        if i_comp.norm() < 1e-9 {
            Complex64::new(f64::INFINITY, 0.0)
        } else {
            self.v_relay / i_comp
        }
    }
}

/// Distance relay trip decision.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DistanceTripDecision {
    /// No trip — impedance outside all zones
    NoTrip,
    /// Trip initiated — zone and time delay
    Trip { zone: Zone, delay_s: f64 },
    /// Blocked by load encroachment blinder
    Blocked,
}

/// Full distance relay model with up to 3 zones.
///
/// Uses quadrilateral characteristic for all zones (more flexible than Mho
/// for numerical relays). A load encroachment blinder is applied before
/// checking zone membership.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceRelay {
    /// Zone 1 characteristic
    pub zone1: QuadrilateralCharacteristic,
    /// Zone 2 characteristic
    pub zone2: QuadrilateralCharacteristic,
    /// Zone 3 characteristic (optional)
    pub zone3: Option<QuadrilateralCharacteristic>,
    /// Load encroachment blinder
    pub load_blinder: LoadBlinder,
    /// Minimum operating current [p.u.] — below this level, relay is blocked
    pub i_min_pu: f64,
}

impl DistanceRelay {
    /// Create a distance relay for a line with known impedance.
    ///
    /// Reach settings:
    /// - Zone 1: 80% of line impedance
    /// - Zone 2: 120% of line impedance
    /// - Zone 3: 200% of line impedance (backup)
    pub fn from_line_impedance(z_line: Complex64, line_angle_deg: f64) -> Self {
        let x_line = z_line.norm() * line_angle_deg.to_radians().sin();
        let r_line = z_line.norm() * line_angle_deg.to_radians().cos();

        let zone1 = QuadrilateralCharacteristic::new(
            0.80 * x_line,
            0.80 * r_line.max(0.2 * x_line),
            line_angle_deg,
            Zone::Zone1,
        );
        let zone2 = QuadrilateralCharacteristic::new(
            1.20 * x_line,
            1.20 * r_line.max(0.25 * x_line),
            line_angle_deg,
            Zone::Zone2,
        );
        let zone3 = QuadrilateralCharacteristic::new(
            2.00 * x_line,
            2.00 * r_line.max(0.3 * x_line),
            line_angle_deg,
            Zone::Zone3,
        );

        Self {
            zone1,
            zone2,
            zone3: Some(zone3),
            load_blinder: LoadBlinder::typical_transmission(),
            i_min_pu: 0.05,
        }
    }

    /// Evaluate apparent impedance against all zones.
    ///
    /// Returns the innermost zone that contains the impedance point,
    /// subject to load blinder and current threshold checks.
    pub fn evaluate(&self, z_app: Complex64, i_mag: f64) -> DistanceTripDecision {
        // Current supervision
        if i_mag < self.i_min_pu {
            return DistanceTripDecision::NoTrip;
        }

        // Load encroachment check — block if in load region
        if self.load_blinder.is_load_region(z_app) {
            return DistanceTripDecision::Blocked;
        }

        // Check zones (innermost = fastest trip)
        if self.zone1.is_inside(z_app) {
            return DistanceTripDecision::Trip {
                zone: Zone::Zone1,
                delay_s: self.zone1.time_delay_s,
            };
        }
        if self.zone2.is_inside(z_app) {
            return DistanceTripDecision::Trip {
                zone: Zone::Zone2,
                delay_s: self.zone2.time_delay_s,
            };
        }
        if let Some(ref z3) = self.zone3 {
            if z3.is_inside(z_app) {
                return DistanceTripDecision::Trip {
                    zone: Zone::Zone3,
                    delay_s: z3.time_delay_s,
                };
            }
        }

        DistanceTripDecision::NoTrip
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// R-X plane utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Compute apparent impedance from phasor quantities.
pub fn apparent_impedance(v: Complex64, i: Complex64) -> Complex64 {
    if i.norm() < 1e-12 {
        Complex64::new(f64::INFINITY, 0.0)
    } else {
        v / i
    }
}

/// Compute the Mho circle boundary points for plotting.
///
/// Returns `n` points on the Mho circle in R-X coordinates.
pub fn mho_circle_points(mho: &MhoCharacteristic, n: usize) -> Vec<(f64, f64)> {
    let centre = mho.centre();
    let r = mho.radius();
    (0..=n)
        .map(|k| {
            let theta = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            (centre.re + r * theta.cos(), centre.im + r * theta.sin())
        })
        .collect()
}

/// Distance between fault point and relay (as impedance fraction of line).
///
/// Returns the per-unit distance `d` in [0, 1] where 0 = relay and 1 = remote end.
/// Uses Z_line as the total line impedance.
pub fn fault_distance_pu(z_apparent: Complex64, z_line: Complex64) -> f64 {
    // |Z_app| / |Z_line| * cos(angle difference) — project onto line direction
    let z_line_mag = z_line.norm();
    if z_line_mag < 1e-12 {
        return 0.0;
    }
    // Project z_apparent onto z_line unit vector
    let z_line_unit = z_line / z_line_mag;
    let projection = (z_apparent * z_line_unit.conj()).re;
    (projection / z_line_mag).clamp(0.0, 1.0)
}

/// Convert per-unit impedance to secondary ohms (for relay settings).
///
/// Z_secondary = Z_pu × (Z_base_primary / CT_ratio) × (VT_ratio / 1)
pub fn pu_to_secondary_ohms(z_pu: f64, z_base_ohm: f64, ct_ratio: f64, vt_ratio: f64) -> f64 {
    z_pu * z_base_ohm * vt_ratio / ct_ratio
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn line_z() -> Complex64 {
        // Typical 132 kV line: Z = 0.05 + j0.40 p.u.
        Complex64::new(0.05, 0.40)
    }

    // ── Mho tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_mho_inside_at_origin() {
        // Origin should be on the boundary (Mho passes through origin)
        let mho = MhoCharacteristic::new(line_z(), 83.0, Zone::Zone1);
        // Origin is exactly on the boundary: |0 - centre| = radius
        let sig = mho.operate_signal(Complex64::new(0.0, 0.0));
        assert!(
            sig.abs() < 1e-10,
            "Origin should be on Mho boundary: sig={sig:.2e}"
        );
    }

    #[test]
    fn test_mho_fault_inside_reach() {
        let z_reach = line_z() * 0.8;
        let mho = MhoCharacteristic::new(z_reach, 83.0, Zone::Zone1);
        // Fault at 50% of line: Z_app = 0.5 * z_reach (inside)
        let z_fault = z_reach * 0.5;
        assert!(
            mho.is_inside(z_fault),
            "50% fault should be inside Zone 1 Mho"
        );
    }

    #[test]
    fn test_mho_fault_outside_reach() {
        let z_reach = line_z() * 0.8;
        let mho = MhoCharacteristic::new(z_reach, 83.0, Zone::Zone1);
        // Fault at 120% of reach (outside Zone 1)
        let z_fault = z_reach * 1.2;
        assert!(
            !mho.is_inside(z_fault),
            "120% reach should be outside Zone 1 Mho"
        );
    }

    #[test]
    fn test_mho_circle_points_count() {
        let mho = MhoCharacteristic::new(line_z(), 80.0, Zone::Zone2);
        let pts = mho_circle_points(&mho, 36);
        assert_eq!(pts.len(), 37); // n+1 for closed circle
    }

    #[test]
    fn test_mho_radius_positive() {
        let mho = MhoCharacteristic::new(line_z(), 80.0, Zone::Zone1);
        assert!(mho.radius() > 0.0);
        assert!((mho.radius() - line_z().norm() / 2.0).abs() < 1e-10);
    }

    // ── Quadrilateral tests ──────────────────────────────────────────────────

    #[test]
    fn test_quad_fault_inside() {
        let quad = QuadrilateralCharacteristic::new(0.35, 0.15, 83.0, Zone::Zone1);
        // A fault with X=0.2, R=0.05 should be inside (X < 0.35, |proj| < 0.15)
        let z_app = Complex64::new(0.04, 0.20);
        assert!(
            quad.is_inside(z_app),
            "Fault should be inside quadrilateral"
        );
    }

    #[test]
    fn test_quad_fault_outside_x() {
        let quad = QuadrilateralCharacteristic::new(0.35, 0.15, 83.0, Zone::Zone1);
        // X > x_reach: outside
        let z_app = Complex64::new(0.04, 0.40);
        assert!(!quad.is_inside(z_app), "X > x_reach should be outside");
    }

    #[test]
    fn test_quad_fault_outside_r() {
        let quad = QuadrilateralCharacteristic::new(0.35, 0.05, 83.0, Zone::Zone1);
        // Large R: outside resistance blinder
        let z_app = Complex64::new(0.50, 0.20);
        assert!(!quad.is_inside(z_app), "Large R should be outside blinder");
    }

    #[test]
    fn test_quad_margin_positive_inside() {
        let quad = QuadrilateralCharacteristic::new(0.35, 0.15, 83.0, Zone::Zone1);
        let z_app = Complex64::new(0.01, 0.10);
        assert!(
            quad.margin(z_app) > 0.0,
            "Inside point should have positive margin"
        );
    }

    #[test]
    fn test_quad_zone_delays() {
        let z1 = QuadrilateralCharacteristic::new(0.3, 0.1, 80.0, Zone::Zone1);
        let z2 = QuadrilateralCharacteristic::new(0.4, 0.15, 80.0, Zone::Zone2);
        assert_eq!(z1.time_delay_s, 0.0);
        assert!((z2.time_delay_s - 0.3).abs() < 1e-10);
    }

    // ── Load blinder tests ───────────────────────────────────────────────────

    #[test]
    fn test_blinder_heavy_load_blocked() {
        let blinder = LoadBlinder::typical_transmission();
        // Heavy load: Z = 1.5∠20° (high magnitude, low angle) → load region
        let phi = 20_f64.to_radians();
        let z_load = Complex64::new(1.5 * phi.cos(), 1.5 * phi.sin());
        assert!(
            blinder.is_load_region(z_load),
            "Heavy load should be in load region"
        );
    }

    #[test]
    fn test_blinder_fault_not_blocked() {
        let blinder = LoadBlinder::typical_transmission();
        // Close-in fault: Z = 0.1∠80° (small magnitude, high angle) → not load
        let phi = 80_f64.to_radians();
        let z_fault = Complex64::new(0.1 * phi.cos(), 0.1 * phi.sin());
        assert!(
            !blinder.is_load_region(z_fault),
            "Fault should not be in load region"
        );
    }

    // ── Full relay tests ─────────────────────────────────────────────────────

    #[test]
    fn test_relay_zone1_trip() {
        let relay = DistanceRelay::from_line_impedance(line_z(), 83.0);
        // Fault at 50% of Zone 1 reach
        let z_app = line_z() * 0.8 * 0.5;
        let dec = relay.evaluate(z_app, 2.0);
        assert_eq!(
            dec,
            DistanceTripDecision::Trip {
                zone: Zone::Zone1,
                delay_s: 0.0
            }
        );
    }

    #[test]
    fn test_relay_zone2_trip() {
        let relay = DistanceRelay::from_line_impedance(line_z(), 83.0);
        // Fault between Zone 1 and Zone 2 reach
        let z_app = line_z() * 0.95; // 95% of line: past Zone1 (80%), inside Zone2 (120%)
        let dec = relay.evaluate(z_app, 2.0);
        assert_eq!(
            dec,
            DistanceTripDecision::Trip {
                zone: Zone::Zone2,
                delay_s: 0.3
            }
        );
    }

    #[test]
    fn test_relay_no_trip_outside_all_zones() {
        let relay = DistanceRelay::from_line_impedance(line_z(), 83.0);
        // Impedance far beyond Zone 3
        let z_app = line_z() * 5.0;
        let dec = relay.evaluate(z_app, 2.0);
        assert_eq!(dec, DistanceTripDecision::NoTrip);
    }

    #[test]
    fn test_relay_blocked_by_load_blinder() {
        let mut relay = DistanceRelay::from_line_impedance(line_z(), 83.0);
        // Make blinder very aggressive so any large-angle, large-magnitude Z is load
        relay.load_blinder = LoadBlinder {
            z_load_min: 0.01,
            phi_load_max_deg: 89.0,
        };
        let z_load = Complex64::new(1.5, 0.1); // large R, small X → low angle
        let dec = relay.evaluate(z_load, 2.0);
        assert_eq!(dec, DistanceTripDecision::Blocked);
    }

    #[test]
    fn test_relay_current_supervision() {
        let relay = DistanceRelay::from_line_impedance(line_z(), 83.0);
        // Current below threshold → no trip even if Z_app is inside Zone 1
        let z_app = line_z() * 0.3;
        let dec = relay.evaluate(z_app, 0.001); // i_mag << i_min
        assert_eq!(dec, DistanceTripDecision::NoTrip);
    }

    #[test]
    fn test_fault_distance_pu_midline() {
        // Fault at 50% of line
        let z_line = line_z();
        let z_app = z_line * 0.5;
        let d = fault_distance_pu(z_app, z_line);
        assert!((d - 0.5).abs() < 1e-6, "d={d:.4}");
    }

    #[test]
    fn test_fault_distance_pu_clamped() {
        let z_line = line_z();
        let z_remote = z_line * 3.0; // beyond line
        let d = fault_distance_pu(z_remote, z_line);
        assert!(d <= 1.0, "d should be clamped to 1.0");
    }

    #[test]
    fn test_pu_to_secondary_ohms() {
        // z_pu=0.1, z_base=264Ω (132kV, 100MVA), CT=600/5=120, VT=132kV/110V≈1200
        let z_sec = pu_to_secondary_ohms(0.1, 264.0, 120.0, 1200.0);
        assert!(z_sec > 0.0);
        let expected = 0.1 * 264.0 * 1200.0 / 120.0; // = 264
        assert!((z_sec - expected).abs() < 1e-6, "z_sec={z_sec:.4}");
    }

    #[test]
    fn test_apparent_impedance_zero_current() {
        let v = Complex64::new(1.0, 0.0);
        let i = Complex64::new(0.0, 0.0);
        let z = apparent_impedance(v, i);
        assert!(z.re.is_infinite());
    }

    #[test]
    fn test_zone_delays() {
        assert_eq!(Zone::Zone1.default_delay_s(), 0.0);
        assert!((Zone::Zone2.default_delay_s() - 0.3).abs() < 1e-10);
        assert!((Zone::Zone3.default_delay_s() - 0.6).abs() < 1e-10);
    }
}
