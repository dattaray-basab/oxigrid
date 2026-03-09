//! Protection zone coordination for transmission systems.
//!
//! Implements distance relay zones (Zone 1/2/3), differential protection zones
//! (bus/transformer/line/generator), and coordination of overlapping protection
//! boundaries per IEC 60255 / IEEE C37.113 standards.
//!
//! # Overview
//! - [`DistanceRelay`] — impedance-based protection with Mho/Quadrilateral/Lens characteristics
//! - [`DifferentialZone`] — current differential protection for buses, transformers, lines
//! - [`ZoneCoordinator`] — coordination checker and auto-setting engine
//!
//! # References
//! - IEC 60255-121:2014 — Distance protection
//! - IEEE C37.113-2015 — Guide for Protective Relay Applications to Transmission Lines
//! - Anderson, P.M., "Power System Protection", IEEE Press, 1999

use serde::{Deserialize, Serialize};
use std::fmt::Write as FmtWrite;

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Default line impedance angle (degrees). Typical transmission line: 75–85°.
const DEFAULT_LINE_ANGLE_DEG: f64 = 75.0;

/// Default Zone 1 reach limit (fraction of line impedance).
const DEFAULT_ZONE1_REACH: f64 = 0.80;

/// Default Zone 2 reach (fraction of line + fraction of adjacent).
const DEFAULT_ZONE2_LINE_FACTOR: f64 = 1.20;
const DEFAULT_ZONE2_ADJ_FACTOR: f64 = 0.50;

/// Default Zone 3 reach factors.
const DEFAULT_ZONE3_LINE_FACTOR: f64 = 1.00;
const DEFAULT_ZONE3_ADJ_FACTOR: f64 = 1.00;

/// Default time delays (seconds).
const ZONE1_DELAY_S: f64 = 0.0;
const ZONE2_DELAY_S: f64 = 0.40;
const ZONE3_DELAY_S: f64 = 0.80;

/// Minimum coordination time interval (seconds).
#[allow(dead_code)]
const DEFAULT_CTI_S: f64 = 0.30;

/// Zone 1 maximum reach limit (85% of protected line).
const ZONE1_MAX_REACH_PCT: f64 = 85.0;

/// Zone 2 minimum reach requirement (120% of protected line).
const ZONE2_MIN_REACH_PCT: f64 = 120.0;

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// Directionality setting of a distance relay zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ZoneDirectional {
    /// Zone operates for faults in the forward (line) direction.
    Forward,
    /// Zone operates for faults in the reverse (bus) direction.
    Reverse,
    /// Zone operates regardless of fault direction.
    Nondirectional,
}

/// Distance relay operating characteristic shape.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DistanceCharacteristic {
    /// Circular Mho characteristic (most common for transmission).
    ///
    /// The Mho circle passes through the origin and has its diameter along the
    /// line impedance angle.
    Mho {
        /// Line (Mho) angle in degrees; typically 75–85°.
        mho_angle_deg: f64,
    },
    /// Quadrilateral characteristic (better coverage of resistive faults).
    Quadrilateral {
        /// Resistive reach (R-axis) in per-unit.
        r_reach_pu: f64,
        /// Reactive reach (X-axis) in per-unit.
        x_reach_pu: f64,
        /// Characteristic tilt angle in degrees.
        angle_deg: f64,
    },
    /// Lens (lenticular) characteristic — limited resistive reach.
    Lens,
}

/// Type of differential protection zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DifferentialZoneType {
    /// Bus differential protection (87B).
    BusDifferential,
    /// Transformer differential protection (87T).
    TransformerDifferential,
    /// Line differential protection (87L).
    LineDifferential,
    /// Generator differential protection (87G).
    GeneratorDifferential,
}

/// Fault type enumeration for zone protection analysis.
///
/// Named `ProtFaultType` to avoid collision with `crate::protection::fault::FaultType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProtFaultType {
    /// Balanced three-phase fault.
    ThreePhase,
    /// Single-line-to-ground (most common, ~70% of faults).
    SingleLineGround,
    /// Line-to-line fault.
    LineToLine,
    /// Double-line-to-ground fault.
    DoubleLineGround,
}

// ─────────────────────────────────────────────────────────────────────────────
// Distance relay structures
// ─────────────────────────────────────────────────────────────────────────────

/// Distance relay protection zone with reach and time-delay settings.
///
/// Per IEC 60255-121: Zone 1 provides high-speed primary protection (≤80%),
/// Zone 2 provides delayed primary + backup (≥120%), Zone 3 provides remote backup.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceZone {
    /// Zone number: 1 (instantaneous), 2 (delayed primary), or 3 (remote backup).
    pub zone_num: u8,
    /// Impedance reach setting in per-unit (positive-sequence).
    pub reach_pu: f64,
    /// Operating time delay in seconds.
    /// Zone 1 ≈ 0 s, Zone 2 ≈ 0.3–0.5 s, Zone 3 ≈ 0.6–1.0 s.
    pub time_delay_s: f64,
    /// Directional control for this zone.
    pub directional: ZoneDirectional,
    /// Fraction of protected line covered by this zone (e.g., 80.0 for Zone 1).
    pub coverage_pct: f64,
}

impl DistanceZone {
    /// Create a new distance zone with explicit settings.
    pub fn new(
        zone_num: u8,
        reach_pu: f64,
        time_delay_s: f64,
        directional: ZoneDirectional,
        coverage_pct: f64,
    ) -> Self {
        Self {
            zone_num,
            reach_pu,
            time_delay_s,
            directional,
            coverage_pct,
        }
    }
}

/// Distance relay protecting a transmission line.
///
/// The relay measures apparent impedance `Z_app = V/I` at its terminal and
/// compares it against zone reach settings to determine if a fault is within
/// its protection coverage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceRelay {
    /// Unique relay identifier.
    pub relay_id: usize,
    /// Bus ID where the relay is installed.
    pub bus_id: usize,
    /// ID of the line protected by this relay.
    pub protected_line_id: usize,
    /// Positive-sequence impedance magnitude of the protected line in per-unit.
    pub line_impedance_pu: f64,
    /// Distance zones configured on this relay (typically Zones 1, 2, 3).
    pub zones: Vec<DistanceZone>,
    /// Relay operating characteristic (Mho, Quadrilateral, or Lens).
    pub characteristic: DistanceCharacteristic,
    /// Current transformer ratio (primary:secondary).
    pub ct_ratio: f64,
    /// Voltage transformer ratio (primary:secondary).
    pub vt_ratio: f64,
}

impl DistanceRelay {
    /// Create a new distance relay with default Mho characteristic and no zones.
    ///
    /// Zones must be added via [`Self::add_zone`] or [`ZoneCoordinator::auto_set_zones`].
    pub fn new(relay_id: usize, bus_id: usize, line_id: usize, z_line: f64) -> Self {
        Self {
            relay_id,
            bus_id,
            protected_line_id: line_id,
            line_impedance_pu: z_line,
            zones: Vec::new(),
            characteristic: DistanceCharacteristic::Mho {
                mho_angle_deg: DEFAULT_LINE_ANGLE_DEG,
            },
            ct_ratio: 1.0,
            vt_ratio: 1.0,
        }
    }

    /// Append a distance zone to this relay.
    pub fn add_zone(&mut self, zone: DistanceZone) {
        self.zones.push(zone);
    }

    /// Compute apparent impedance seen by the relay from terminal voltage and current.
    ///
    /// Returns `(R_pu, X_pu)` in per-unit on the system base.
    ///
    /// # Parameters
    /// - `v_relay` — voltage magnitude at relay terminal (pu)
    /// - `i_relay` — current magnitude at relay terminal (pu)
    /// - `angle_diff` — angle between V and I in degrees (V leads I by this angle)
    pub fn apparent_impedance(&self, v_relay: f64, i_relay: f64, angle_diff: f64) -> (f64, f64) {
        if i_relay.abs() < 1e-12 {
            return (f64::INFINITY, f64::INFINITY);
        }
        let z_mag = (v_relay / i_relay) * (self.ct_ratio / self.vt_ratio);
        let angle_rad = angle_diff.to_radians();
        let r = z_mag * angle_rad.cos();
        let x = z_mag * angle_rad.sin();
        (r, x)
    }

    /// Determine which distance zone operates for a given apparent impedance.
    ///
    /// Returns a reference to the fastest (lowest zone number) zone whose
    /// characteristic contains the measured impedance point.
    pub fn operating_zone(&self, z_apparent: (f64, f64)) -> Option<&DistanceZone> {
        // Sort zones by zone_num to find the fastest operating zone first
        let mut sorted: Vec<&DistanceZone> = self.zones.iter().collect();
        sorted.sort_by_key(|z| z.zone_num);

        sorted
            .into_iter()
            .find(|&zone| self.is_inside_characteristic(z_apparent, zone))
            .map(|v| v as _)
    }

    /// Check if the impedance point is inside the zone's operating characteristic.
    fn is_inside_characteristic(&self, z_fault: (f64, f64), zone: &DistanceZone) -> bool {
        match &self.characteristic {
            DistanceCharacteristic::Mho { mho_angle_deg } => {
                is_inside_mho(z_fault, zone.reach_pu, *mho_angle_deg)
            }
            DistanceCharacteristic::Quadrilateral {
                r_reach_pu,
                x_reach_pu,
                angle_deg: _,
            } => {
                let (r, x) = z_fault;
                r >= 0.0 && r <= *r_reach_pu && x >= 0.0 && x <= *x_reach_pu
            }
            DistanceCharacteristic::Lens => {
                // Lens characteristic: smaller coverage than Mho
                // Approximate as Mho with 70% of reach
                is_inside_mho(z_fault, zone.reach_pu * 0.7, DEFAULT_LINE_ANGLE_DEG)
            }
        }
    }
}

/// Check if an impedance point lies inside a Mho circle.
///
/// The Mho circle has its diameter along the line angle from origin to Z_reach.
/// A fault impedance Z_f is inside the Mho circle if:
/// `|Z_f - Z_reach/2| < |Z_reach/2|`
///
/// This is equivalent to: `Re{Z_f * conj(Z_reach)} > |Z_f|^2`
/// (dot product condition for a circle passing through the origin).
fn is_inside_mho(z_fault: (f64, f64), reach_pu: f64, mho_angle_deg: f64) -> bool {
    let (r_f, x_f) = z_fault;
    let angle_rad = mho_angle_deg.to_radians();
    // Centre of Mho circle: Z_reach / 2
    let cx = reach_pu * angle_rad.cos() / 2.0;
    let cy = reach_pu * angle_rad.sin() / 2.0;
    // Radius = |Z_reach / 2|
    let radius = (cx * cx + cy * cy).sqrt();
    // Distance from fault point to centre
    let dx = r_f - cx;
    let dy = x_f - cy;
    let dist = (dx * dx + dy * dy).sqrt();
    dist < radius
}

// ─────────────────────────────────────────────────────────────────────────────
// Differential protection
// ─────────────────────────────────────────────────────────────────────────────

/// Differential protection zone using percentage differential characteristic (87).
///
/// Compares the vector sum of boundary currents against a restraint quantity.
/// An internal fault produces a large differential current while a through-fault
/// produces cancelling boundary currents and minimal differential current.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DifferentialZone {
    /// Unique zone identifier.
    pub zone_id: usize,
    /// Type of equipment protected by this differential zone.
    pub zone_type: DifferentialZoneType,
    /// Human-readable name of the protected equipment.
    pub equipment_name: String,
    /// Current transformers at zone boundaries: `(bus_id, ct_ratio)`.
    pub boundary_cts: Vec<(usize, f64)>,
    /// Pickup current in per-unit of the minimum boundary CT rating.
    pub pickup_pu: f64,
    /// Percentage differential slope (e.g., 30.0 for 30% slope).
    pub slope_pct: f64,
    /// Minimum operating current in per-unit (sensitivity threshold).
    pub i_min_operate_pu: f64,
}

impl DifferentialZone {
    /// Create a new differential zone with default settings.
    ///
    /// Default pickup = 0.2 pu, slope = 30%, minimum operate = 0.1 pu.
    pub fn new(zone_id: usize, zone_type: DifferentialZoneType, equipment: String) -> Self {
        Self {
            zone_id,
            zone_type,
            equipment_name: equipment,
            boundary_cts: Vec::new(),
            pickup_pu: 0.20,
            slope_pct: 30.0,
            i_min_operate_pu: 0.10,
        }
    }

    /// Add a boundary current transformer to this differential zone.
    pub fn add_ct(&mut self, bus_id: usize, ct_ratio: f64) {
        self.boundary_cts.push((bus_id, ct_ratio));
    }

    /// Determine if this differential zone should operate for the given boundary currents.
    ///
    /// # Parameters
    /// - `currents` — slice of `(magnitude_pu, is_inflow)` for each boundary CT.
    ///   `is_inflow = true` means current flowing into the protected zone.
    ///
    /// # Returns
    /// `true` if the differential element should trip.
    pub fn check_operation(&self, currents: &[(f64, bool)]) -> bool {
        if currents.is_empty() {
            return false;
        }
        // Differential current: algebraic sum (inflow positive, outflow negative)
        let i_diff_signed: f64 = currents
            .iter()
            .map(|(mag, is_in)| if *is_in { *mag } else { -*mag })
            .sum();
        let i_diff = i_diff_signed.abs();

        // Restraint current: sum of magnitudes / 2
        let i_restraint: f64 = currents.iter().map(|(mag, _)| *mag).sum::<f64>() / 2.0;

        let pickup_threshold = self
            .i_min_operate_pu
            .max((self.slope_pct / 100.0) * i_restraint);

        i_diff > pickup_threshold
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Zone map and coverage
// ─────────────────────────────────────────────────────────────────────────────

/// Protection zone coverage descriptor for one zone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZoneCoverage {
    /// Zone identifier (matches `DistanceRelay::relay_id` or `DifferentialZone::zone_id`).
    pub zone_id: usize,
    /// Names of equipment protected by this zone.
    pub protected_equipment: Vec<String>,
    /// IDs of zones providing backup protection for the same equipment.
    pub backup_zones: Vec<usize>,
    /// Overlap with adjacent zone protection in per-unit of line length.
    /// Overlap is intentional for Zone 2/3 (remote backup).
    pub coverage_overlap: f64,
}

/// Protection zone map for a substation or protection area.
///
/// Aggregates all distance relays and differential zones associated with
/// a substation, and tracks their coverage assignments.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtectionZoneMap {
    /// Substation or protection area name.
    pub substation_name: String,
    /// Bus IDs associated with this protection zone map.
    pub buses: Vec<usize>,
    /// Distance relays installed at this substation.
    pub distance_relays: Vec<DistanceRelay>,
    /// Differential protection zones (bus/transformer/line/generator).
    pub differential_zones: Vec<DifferentialZone>,
    /// Coverage assignments and overlap information.
    pub coverage: Vec<ZoneCoverage>,
}

impl ProtectionZoneMap {
    /// Create a new empty protection zone map for a substation.
    pub fn new(substation_name: impl Into<String>) -> Self {
        Self {
            substation_name: substation_name.into(),
            buses: Vec::new(),
            distance_relays: Vec::new(),
            differential_zones: Vec::new(),
            coverage: Vec::new(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Coordination results
// ─────────────────────────────────────────────────────────────────────────────

/// A detected coordination problem between protection zones.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CoordinationIssue {
    /// Backup relay operates too close in time to primary relay (CTI violation).
    InsufficientTimeMargin {
        /// Relay ID of the primary (faster) relay.
        primary_relay: usize,
        /// Relay ID of the backup (slower) relay.
        backup_relay: usize,
        /// Actual time margin between primary and backup in seconds.
        margin_s: f64,
    },
    /// Two adjacent zones overlap excessively (potential selectivity loss).
    OverlapTooLarge {
        /// First zone ID.
        zone_a: usize,
        /// Second zone ID.
        zone_b: usize,
        /// Overlap fraction of line length (pu).
        overlap_pct: f64,
    },
    /// A section of line has insufficient protection coverage.
    GapInCoverage {
        /// Location on the line (per-unit distance from relay, 0–1).
        location: f64,
        /// Fraction of line covered at this location.
        coverage_pct: f64,
    },
    /// Zone 1 reach exceeds the recommended 85% limit.
    Zone1TooLong {
        /// Relay ID with the oversized Zone 1.
        relay_id: usize,
        /// Actual Zone 1 coverage percentage.
        coverage_pct: f64,
    },
    /// A piece of equipment has no backup protection zone assigned.
    MissingBackup {
        /// Name of the equipment lacking backup protection.
        equipment: String,
    },
}

/// Result of a protection coordination study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationResult {
    /// `true` if all relays are properly coordinated without CTI violations.
    pub is_coordinated: bool,
    /// Minimum time margin found between any primary/backup relay pair (seconds).
    pub coordination_margin_s: f64,
    /// List of detected coordination problems.
    pub issues: Vec<CoordinationIssue>,
    /// Total fault clearing time for the worst-case scenario (seconds).
    pub total_clearing_time_s: f64,
    /// Backup relay zone reach in per-unit.
    pub backup_reach_pu: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Fault location and performance
// ─────────────────────────────────────────────────────────────────────────────

/// Location and characteristics of a fault for protection performance evaluation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultLocation {
    /// Distance from the sending-end relay to the fault, expressed as a
    /// fraction of total line length (0.0 = relay bus, 1.0 = remote bus).
    pub per_unit_distance: f64,
    /// Type of fault.
    pub fault_type: ProtFaultType,
    /// Fault arc/contact resistance in per-unit. Zero for a bolted fault.
    pub fault_resistance_pu: f64,
}

/// Protection system performance metrics for a specific fault scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtectionPerformance {
    /// Fault being evaluated.
    pub fault_location: FaultLocation,
    /// ID of the relay that clears the fault.
    pub operating_relay_id: usize,
    /// Zone number that operated (1, 2, or 3).
    pub operating_zone: u8,
    /// Time from fault inception to fault clearing in seconds.
    pub clearing_time_s: f64,
    /// Impedance seen by the operating relay in per-unit (magnitude).
    pub measured_impedance_pu: f64,
    /// `true` if the correct relay operated for this fault location.
    pub is_correct_operation: bool,
    /// Relay ID providing backup protection, if available.
    pub backup_relay_id: Option<usize>,
    /// Backup relay clearing time in seconds, if backup relay exists.
    pub backup_clearing_time_s: Option<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Zone coordinator
// ─────────────────────────────────────────────────────────────────────────────

/// Protection zone coordinator for a transmission substation.
///
/// Verifies coordination between distance relays and differential zones,
/// checks CTI (coordination time interval) margins, auto-sets zone reaches,
/// and evaluates protection performance for fault scenarios.
///
/// # Usage
/// ```no_run
/// use oxigrid::protection::zone_protection::*;
///
/// let map = ProtectionZoneMap::new("Substation A");
/// let coordinator = ZoneCoordinator::new(map, 0.3);
/// let result = coordinator.check_coordination();
/// println!("{}", coordinator.summary_report());
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZoneCoordinator {
    /// Protection zone map containing all relays and differential zones.
    pub zone_map: ProtectionZoneMap,
    /// Minimum coordination time interval (CTI) in seconds.
    /// Typical value: 0.3 s (IEC/IEEE standard).
    pub cti_s: f64,
    /// Maximum allowed Zone 1 reach as a percentage of line impedance.
    /// Default: 80% (conservative) — must never exceed 85%.
    pub zone1_max_reach_pct: f64,
    /// Minimum required Zone 2 reach as a percentage of line impedance.
    /// Default: 120% (ensures overlap with Zone 1 of remote relay).
    pub zone2_min_reach_pct: f64,
}

impl ZoneCoordinator {
    /// Create a new zone coordinator.
    ///
    /// # Parameters
    /// - `zone_map` — protection zone map with relays and differential zones
    /// - `cti_s` — minimum coordination time interval in seconds (typically 0.3 s)
    pub fn new(zone_map: ProtectionZoneMap, cti_s: f64) -> Self {
        Self {
            zone_map,
            cti_s,
            zone1_max_reach_pct: ZONE1_MAX_REACH_PCT,
            zone2_min_reach_pct: ZONE2_MIN_REACH_PCT,
        }
    }

    /// Check coordination between all distance relays in the zone map.
    ///
    /// Verifies CTI margins, Zone 1 reach limits, and identifies gaps or
    /// overlaps in protection coverage. Returns a detailed [`CoordinationResult`].
    pub fn check_coordination(&self) -> CoordinationResult {
        let relays = &self.zone_map.distance_relays;
        let mut issues = Vec::new();
        let mut min_margin = f64::INFINITY;
        let mut max_clearing_time: f64 = 0.0;
        let mut max_backup_reach: f64 = 0.0;

        // Check Zone 1 reach limits
        for relay in relays {
            for zone in &relay.zones {
                if zone.zone_num == 1 {
                    let cov = zone.coverage_pct;
                    if cov > self.zone1_max_reach_pct {
                        issues.push(CoordinationIssue::Zone1TooLong {
                            relay_id: relay.relay_id,
                            coverage_pct: cov,
                        });
                    }
                    let delay = zone.time_delay_s;
                    if delay > max_clearing_time {
                        max_clearing_time = delay;
                    }
                }
                if zone.zone_num >= 2 {
                    if zone.reach_pu > max_backup_reach {
                        max_backup_reach = zone.reach_pu;
                    }
                    if zone.time_delay_s > max_clearing_time {
                        max_clearing_time = zone.time_delay_s;
                    }
                }
            }
        }

        // Check CTI margins between relay pairs (primary relay zone 1/2, backup zone 2/3)
        for i in 0..relays.len() {
            for j in 0..relays.len() {
                if i == j {
                    continue;
                }
                let primary = &relays[i];
                let backup = &relays[j];
                let margin = self.compute_coordination_margin(primary, backup);
                if margin < f64::INFINITY {
                    if margin < min_margin {
                        min_margin = margin;
                    }
                    if margin < self.cti_s {
                        issues.push(CoordinationIssue::InsufficientTimeMargin {
                            primary_relay: primary.relay_id,
                            backup_relay: backup.relay_id,
                            margin_s: margin,
                        });
                    }
                }
            }
        }

        // Check coverage gaps and backup assignment
        let gap_issues = self.find_gaps_in_coverage();
        issues.extend(gap_issues);

        let effective_min_margin = if min_margin == f64::INFINITY {
            self.cti_s
        } else {
            min_margin
        };

        let is_coordinated = !issues
            .iter()
            .any(|i| matches!(i, CoordinationIssue::InsufficientTimeMargin { .. }));

        CoordinationResult {
            is_coordinated,
            coordination_margin_s: effective_min_margin,
            issues,
            total_clearing_time_s: max_clearing_time,
            backup_reach_pu: max_backup_reach,
        }
    }

    /// Evaluate protection system performance for a specific fault on a line.
    ///
    /// Simulates the apparent impedance seen by each relay and determines which
    /// zone operates, the clearing time, and whether backup protection is available.
    ///
    /// # Parameters
    /// - `fault` — fault location and characteristics
    /// - `line_id` — ID of the line on which the fault occurs
    pub fn evaluate_fault(&self, fault: &FaultLocation, line_id: usize) -> ProtectionPerformance {
        let d = fault.per_unit_distance.clamp(0.0, 1.0);
        let r_f = fault.fault_resistance_pu;

        // Find all relays protecting this line
        let line_relays: Vec<&DistanceRelay> = self
            .zone_map
            .distance_relays
            .iter()
            .filter(|r| r.protected_line_id == line_id)
            .collect();

        // Default: no relay found
        if line_relays.is_empty() {
            return ProtectionPerformance {
                fault_location: fault.clone(),
                operating_relay_id: usize::MAX,
                operating_zone: 0,
                clearing_time_s: f64::INFINITY,
                measured_impedance_pu: f64::INFINITY,
                is_correct_operation: false,
                backup_relay_id: None,
                backup_clearing_time_s: None,
            };
        }

        // Use 75° line angle for impedance calculation
        let angle_rad = DEFAULT_LINE_ANGLE_DEG.to_radians();

        // Primary: find the relay and zone that operates fastest
        let mut best_relay_id = usize::MAX;
        let mut best_zone_num = 0u8;
        let mut best_clearing = f64::INFINITY;
        let mut best_z_app = 0.0f64;

        for relay in &line_relays {
            let z_line = relay.line_impedance_pu;
            // Apparent impedance: R = d*Z*cos(θ) + Rf, X = d*Z*sin(θ)
            let r_app = d * z_line * angle_rad.cos() + r_f;
            let x_app = d * z_line * angle_rad.sin();
            let z_app_mag = (r_app * r_app + x_app * x_app).sqrt();

            if let Some(zone) = relay.operating_zone((r_app, x_app)) {
                if zone.time_delay_s < best_clearing {
                    best_clearing = zone.time_delay_s;
                    best_relay_id = relay.relay_id;
                    best_zone_num = zone.zone_num;
                    best_z_app = z_app_mag;
                }
            }
        }

        let is_correct = best_relay_id != usize::MAX;

        // Backup: find another relay that also sees the fault in a higher zone
        let mut backup_relay_id = None;
        let mut backup_clearing = None;

        if is_correct {
            // Look for backup from relays on adjacent lines or same line with higher zones
            for relay in &self.zone_map.distance_relays {
                if relay.relay_id == best_relay_id {
                    continue;
                }
                // Compute apparent impedance including the primary line reach
                let z_line = relay.line_impedance_pu;
                let r_app = d * z_line * angle_rad.cos() + r_f;
                let x_app = d * z_line * angle_rad.sin();

                for zone in &relay.zones {
                    if zone.zone_num > best_zone_num
                        && relay.is_inside_characteristic((r_app, x_app), zone)
                    {
                        let ct = zone.time_delay_s;
                        if backup_clearing.map_or(true, |bc: f64| ct < bc) {
                            backup_relay_id = Some(relay.relay_id);
                            backup_clearing = Some(ct);
                        }
                    }
                }
            }

            // Also consider backup from same relay in a higher zone if primary operated in Zone 1
            if backup_clearing.is_none() {
                if let Some(primary_relay) =
                    line_relays.iter().find(|r| r.relay_id == best_relay_id)
                {
                    for zone in &primary_relay.zones {
                        if zone.zone_num > best_zone_num {
                            let bt = zone.time_delay_s;
                            if backup_clearing.map_or(true, |bc: f64| bt < bc) {
                                backup_relay_id = Some(primary_relay.relay_id);
                                backup_clearing = Some(bt);
                            }
                        }
                    }
                }
            }
        }

        ProtectionPerformance {
            fault_location: fault.clone(),
            operating_relay_id: best_relay_id,
            operating_zone: best_zone_num,
            clearing_time_s: best_clearing,
            measured_impedance_pu: best_z_app,
            is_correct_operation: is_correct,
            backup_relay_id,
            backup_clearing_time_s: backup_clearing,
        }
    }

    /// Auto-set Zone 1/2/3 reach and time-delay settings per IEC 60255 / IEEE C37.113.
    ///
    /// # Parameters
    /// - `line_impedance_pu` — positive-sequence impedance of the protected line
    /// - `adjacent_impedance_pu` — positive-sequence impedance of the adjacent line
    ///
    /// # Returns
    /// A vector of three [`DistanceZone`] settings (Zone 1, 2, 3).
    pub fn auto_set_zones(
        &self,
        line_impedance_pu: f64,
        adjacent_impedance_pu: f64,
    ) -> Vec<DistanceZone> {
        let z1_reach = DEFAULT_ZONE1_REACH * line_impedance_pu;
        let z2_reach = DEFAULT_ZONE2_LINE_FACTOR * line_impedance_pu
            + DEFAULT_ZONE2_ADJ_FACTOR * adjacent_impedance_pu;
        let z3_reach = DEFAULT_ZONE3_LINE_FACTOR * line_impedance_pu
            + DEFAULT_ZONE3_ADJ_FACTOR * adjacent_impedance_pu;

        let z1_cov = (z1_reach / line_impedance_pu) * 100.0;
        let z2_cov = (z2_reach / line_impedance_pu) * 100.0;
        let z3_cov = (z3_reach / line_impedance_pu) * 100.0;

        vec![
            DistanceZone::new(1, z1_reach, ZONE1_DELAY_S, ZoneDirectional::Forward, z1_cov),
            DistanceZone::new(2, z2_reach, ZONE2_DELAY_S, ZoneDirectional::Forward, z2_cov),
            DistanceZone::new(3, z3_reach, ZONE3_DELAY_S, ZoneDirectional::Forward, z3_cov),
        ]
    }

    /// Check whether a specific differential zone should operate given boundary currents.
    ///
    /// Looks up the differential zone by `zone_id` and evaluates the trip condition
    /// using a single inflow/outflow current pair.
    ///
    /// # Returns
    /// `true` if the differential element should trip; `false` if it should restrain.
    pub fn check_differential_operation(&self, zone_id: usize, i_in: f64, i_out: f64) -> bool {
        if let Some(dz) = self
            .zone_map
            .differential_zones
            .iter()
            .find(|z| z.zone_id == zone_id)
        {
            dz.check_operation(&[(i_in, true), (i_out, false)])
        } else {
            false
        }
    }

    /// Determine if a fault impedance lies inside the Mho circle of a given zone.
    ///
    /// Dispatches based on the relay's [`DistanceCharacteristic`].
    pub fn is_inside_mho(
        &self,
        z_fault: (f64, f64),
        zone: &DistanceZone,
        char: &DistanceCharacteristic,
    ) -> bool {
        match char {
            DistanceCharacteristic::Mho { mho_angle_deg } => {
                is_inside_mho(z_fault, zone.reach_pu, *mho_angle_deg)
            }
            DistanceCharacteristic::Quadrilateral {
                r_reach_pu,
                x_reach_pu,
                angle_deg: _,
            } => {
                let (r, x) = z_fault;
                r >= 0.0 && r <= *r_reach_pu && x >= 0.0 && x <= *x_reach_pu
            }
            DistanceCharacteristic::Lens => {
                is_inside_mho(z_fault, zone.reach_pu * 0.7, DEFAULT_LINE_ANGLE_DEG)
            }
        }
    }

    /// Compute the coordination time margin between a primary and backup relay pair.
    ///
    /// The margin is the difference between the backup relay's operating time
    /// and the primary relay's operating time. Returns `f64::INFINITY` if no
    /// overlapping zone pair exists between the two relays.
    pub fn compute_coordination_margin(
        &self,
        primary: &DistanceRelay,
        backup: &DistanceRelay,
    ) -> f64 {
        // Find primary zone 1 or 2 delay
        let primary_delay = primary
            .zones
            .iter()
            .filter(|z| z.zone_num <= 2)
            .map(|z| z.time_delay_s)
            .fold(f64::INFINITY, f64::min);

        // Find backup zone 2 or 3 delay
        let backup_delay = backup
            .zones
            .iter()
            .filter(|z| z.zone_num >= 2)
            .map(|z| z.time_delay_s)
            .fold(f64::INFINITY, f64::min);

        if primary_delay == f64::INFINITY || backup_delay == f64::INFINITY {
            return f64::INFINITY;
        }

        backup_delay - primary_delay
    }

    /// Identify gaps and missing backup coverage in the zone map.
    ///
    /// Returns a list of [`CoordinationIssue`] entries for:
    /// - Zone 1 exceeding the maximum reach limit
    /// - Equipment with no backup zones assigned
    pub fn find_gaps_in_coverage(&self) -> Vec<CoordinationIssue> {
        let mut issues = Vec::new();

        // Check Zone 1 reach for each relay
        for relay in &self.zone_map.distance_relays {
            for zone in &relay.zones {
                if zone.zone_num == 1 && zone.coverage_pct > self.zone1_max_reach_pct {
                    issues.push(CoordinationIssue::Zone1TooLong {
                        relay_id: relay.relay_id,
                        coverage_pct: zone.coverage_pct,
                    });
                }
            }
        }

        // Check coverage entries for missing backup
        for cov in &self.zone_map.coverage {
            if cov.backup_zones.is_empty() {
                for equip in &cov.protected_equipment {
                    issues.push(CoordinationIssue::MissingBackup {
                        equipment: equip.clone(),
                    });
                }
            }
        }

        // Check differential zones for missing backup
        for dz in &self.zone_map.differential_zones {
            let has_backup = self.zone_map.coverage.iter().any(|c| {
                c.protected_equipment.contains(&dz.equipment_name) && !c.backup_zones.is_empty()
            });
            if !has_backup && !self.zone_map.coverage.is_empty() {
                // Only flag if a coverage map exists but doesn't include this equipment
                let is_covered = self
                    .zone_map
                    .coverage
                    .iter()
                    .any(|c| c.protected_equipment.contains(&dz.equipment_name));
                if !is_covered {
                    issues.push(CoordinationIssue::MissingBackup {
                        equipment: dz.equipment_name.clone(),
                    });
                }
            }
        }

        issues
    }

    /// Generate a human-readable coordination summary report.
    ///
    /// Includes substation name, relay/zone counts, coordination status, and
    /// any detected issues.
    pub fn summary_report(&self) -> String {
        let mut out = String::new();
        let result = self.check_coordination();

        let _ = writeln!(out, "=== Protection Zone Coordination Report ===");
        let _ = writeln!(out, "Substation: {}", self.zone_map.substation_name);
        let _ = writeln!(
            out,
            "Distance relays: {}",
            self.zone_map.distance_relays.len()
        );
        let _ = writeln!(
            out,
            "Differential zones: {}",
            self.zone_map.differential_zones.len()
        );
        let _ = writeln!(out, "CTI setting: {:.2} s", self.cti_s);
        let _ = writeln!(
            out,
            "Zone 1 max reach: {:.1}% | Zone 2 min reach: {:.1}%",
            self.zone1_max_reach_pct, self.zone2_min_reach_pct
        );
        let _ = writeln!(out);

        let status = if result.is_coordinated {
            "COORDINATED"
        } else {
            "NOT COORDINATED"
        };
        let _ = writeln!(out, "Coordination status: {}", status);
        let _ = writeln!(
            out,
            "Minimum CTI margin: {:.3} s",
            result.coordination_margin_s
        );
        let _ = writeln!(
            out,
            "Total clearing time: {:.3} s",
            result.total_clearing_time_s
        );
        let _ = writeln!(out, "Max backup reach: {:.4} pu", result.backup_reach_pu);

        if result.issues.is_empty() {
            let _ = writeln!(out, "\nNo coordination issues detected.");
        } else {
            let _ = writeln!(out, "\nCoordination issues ({}):", result.issues.len());
            for issue in &result.issues {
                match issue {
                    CoordinationIssue::InsufficientTimeMargin {
                        primary_relay,
                        backup_relay,
                        margin_s,
                    } => {
                        let _ = writeln!(
                            out,
                            "  [CTI] Relay {} (primary) → Relay {} (backup): margin={:.3} s < {:.3} s",
                            primary_relay, backup_relay, margin_s, self.cti_s
                        );
                    }
                    CoordinationIssue::Zone1TooLong {
                        relay_id,
                        coverage_pct,
                    } => {
                        let _ = writeln!(
                            out,
                            "  [REACH] Relay {}: Zone 1 coverage {:.1}% > {:.1}% limit",
                            relay_id, coverage_pct, self.zone1_max_reach_pct
                        );
                    }
                    CoordinationIssue::OverlapTooLarge {
                        zone_a,
                        zone_b,
                        overlap_pct,
                    } => {
                        let _ = writeln!(
                            out,
                            "  [OVERLAP] Zones {} and {}: overlap={:.1}%",
                            zone_a, zone_b, overlap_pct
                        );
                    }
                    CoordinationIssue::GapInCoverage {
                        location,
                        coverage_pct,
                    } => {
                        let _ = writeln!(
                            out,
                            "  [GAP] Coverage gap at {:.2} pu: only {:.1}% covered",
                            location, coverage_pct
                        );
                    }
                    CoordinationIssue::MissingBackup { equipment } => {
                        let _ = writeln!(out, "  [BACKUP] No backup protection for: {}", equipment);
                    }
                }
            }
        }

        // Per-relay zone summary
        if !self.zone_map.distance_relays.is_empty() {
            let _ = writeln!(out, "\nDistance relay zones:");
            for relay in &self.zone_map.distance_relays {
                let _ = writeln!(
                    out,
                    "  Relay {} (bus {}, line {}): Z_line={:.4} pu",
                    relay.relay_id, relay.bus_id, relay.protected_line_id, relay.line_impedance_pu
                );
                for zone in &relay.zones {
                    let _ = writeln!(
                        out,
                        "    Zone {}: reach={:.4} pu, delay={:.2} s, coverage={:.1}%",
                        zone.zone_num, zone.reach_pu, zone.time_delay_s, zone.coverage_pct
                    );
                }
            }
        }

        out
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
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
        let mut dz =
            DifferentialZone::new(1, DifferentialZoneType::BusDifferential, "Bus A".into());
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
        let mut dz =
            DifferentialZone::new(5, DifferentialZoneType::LineDifferential, "Line AB".into());
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
        let mut dz =
            DifferentialZone::new(10, DifferentialZoneType::BusDifferential, "Bus B".into());
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
}
