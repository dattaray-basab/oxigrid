//! Advanced protection coordination module.
//!
//! Provides:
//! - [`Iec60909Calculator`] — simplified Thevenin-based IEC 60909 short-circuit calculation
//! - [`OcCoordinationTool`] — IDMT overcurrent relay coordination
//! - [`DifferentialProtection87`] — dual-slope 87T/87B transformer & bus differential relay
//! - [`DistanceZoneSettings`] — 21-relay zone coordination (Mho/Quad/Lens/Offset)
//! - [`BusProtection87B`] — multi-feeder bus differential protection
//! - [`ProtectionReport`] — generated coordination report
//!
//! # References
//! - IEC 60909-0:2016 "Short-circuit currents in three-phase AC systems"
//! - IEC 60255-151:2009 "Functional requirements for overcurrent relays"
//! - IEC 60255-12:2021 "Transformer differential protection"
//! - IEEE Std C37.113-2015 "Distance relay guide"

use num_complex::Complex;
use serde::{Deserialize, Serialize};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. IEC 60909 Short-Circuit Calculator (simplified Thevenin form)
// ═══════════════════════════════════════════════════════════════════════════════

/// Simplified IEC 60909 short-circuit calculator using a pre-computed Thevenin
/// equivalent at the fault bus.
///
/// The voltage factor `c` (1.05 for maximum, 0.95 for minimum) scales the
/// equivalent driving voltage per IEC 60909-0 §4.3.1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Iec60909Calculator {
    /// Voltage factor `c` (1.05 for max SC, 0.95 for min SC).
    pub c_factor: f64,
    /// System MVA base \[MVA\].
    pub base_mva: f64,
    /// System nominal voltage (line-to-line) \[kV\].
    pub base_kv: f64,
    /// Thevenin positive-sequence impedance at the fault bus \[pu\].
    pub system_impedance_pu: Complex<f64>,
}

impl Iec60909Calculator {
    /// Compute the base impedance \[Ω\] for pu → physical conversion.
    #[inline]
    fn z_base_ohm(&self) -> f64 {
        // Z_base = kV² / MVA
        (self.base_kv * self.base_kv) / self.base_mva
    }

    /// Three-phase symmetrical fault current \[kA\] (IEC 60909-0 Eq. 29).
    ///
    /// `I″k3 = c · Un / (√3 · |Z1|)`
    ///
    /// where `Un` is the nominal line-to-line voltage and `|Z1|` is the
    /// magnitude of the Thevenin impedance in physical \[Ω\].
    pub fn three_phase_fault_current_ka(&self, v_pre_pu: f64) -> f64 {
        let z_ohm = self.system_impedance_pu * self.z_base_ohm();
        let z_abs = z_ohm.norm();
        if z_abs < 1e-12 {
            return 0.0;
        }
        // Equivalent voltage source = c · v_pre · Un / √3  [kV phase]
        let v_kv_phase = self.c_factor * v_pre_pu * self.base_kv / 3.0_f64.sqrt();
        // I [kA] = V_phase [kV] / Z [Ω]
        v_kv_phase / z_abs
    }

    /// Single-line-to-ground fault current \[kA\] (IEC 60909-0 Eq. 52).
    ///
    /// `I″k1 = √3 · c · Un / (|Z1 + Z2 + Z0|)`
    ///
    /// All sequence impedances are in physical \[Ω\].
    pub fn single_line_to_ground_ka(
        &self,
        z1: Complex<f64>,
        z2: Complex<f64>,
        z0: Complex<f64>,
    ) -> f64 {
        let z_sum = z1 + z2 + z0;
        let z_abs = z_sum.norm();
        if z_abs < 1e-12 {
            return 0.0;
        }
        // I″k1 = √3 · c · Un / |Z1+Z2+Z0|,  Un in kV → result in kA
        3.0_f64.sqrt() * self.c_factor * self.base_kv / z_abs
    }

    /// Peak factor κ from X/R ratio (IEC 60909-0 §4.3.1.1).
    ///
    /// `κ = 1.02 + 0.98 · exp(−3 / (X/R))`
    ///
    /// Range: κ ∈ \[1.02, 2.0\].
    pub fn peak_factor_kappa(&self, x_over_r: f64) -> f64 {
        if x_over_r < 1e-12 {
            // Purely resistive: κ → 1.02
            return 1.02;
        }
        1.02 + 0.98 * (-3.0 / x_over_r).exp()
    }

    /// Peak (instantaneous) fault current \[kA\].
    ///
    /// `ip = κ · √2 · Irms`
    pub fn peak_fault_current_ka(&self, i_rms_ka: f64, x_over_r: f64) -> f64 {
        self.peak_factor_kappa(x_over_r) * 2.0_f64.sqrt() * i_rms_ka
    }

    /// Breaking (interrupting) fault current \[kA\] at contact separation (IEC 60909-0 §4.5).
    ///
    /// `Ib = Isym · μ`
    ///
    /// where `μ = 0.84 + 0.26 · exp(−0.26 · t_break / τ)` and `τ` is the DC
    /// decay time constant (typically `τ = X / (ω · R)`) \[s\].
    pub fn break_fault_current_ka(&self, i_sym_ka: f64, t_break_s: f64, tau_s: f64) -> f64 {
        let mu = if tau_s > 1e-12 {
            0.84 + 0.26 * (-0.26 * t_break_s / tau_s).exp()
        } else {
            // Very short DC time constant — μ → 0.84
            0.84
        };
        i_sym_ka * mu
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. Overcurrent Relay Coordination Tool
// ═══════════════════════════════════════════════════════════════════════════════

/// IEC/IEEE IDMT curve type.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdmtCurve {
    /// IEC Standard Inverse (k=0.14, α=0.02).
    Si,
    /// IEC Very Inverse (k=13.5, α=1.0).
    Vi,
    /// IEC Extremely Inverse (k=80.0, α=2.0).
    Ei,
    /// IEC Long-Time Very Inverse (k=120.0, α=1.0).
    LtVi,
    /// IEEE Moderately Inverse (k=0.0515, α=0.02).
    IeeeMi,
    /// IEEE Very Inverse (k=19.61, α=2.0).
    IeeeVi,
    /// IEEE Extremely Inverse (k=28.2, α=2.0).
    IeeeEi,
}

impl IdmtCurve {
    /// Return `(k, alpha)` coefficients for `t = TMS · k / ((I/Is)^alpha − 1)`.
    pub fn coefficients(&self) -> (f64, f64) {
        match self {
            IdmtCurve::Si => (0.14, 0.02),
            IdmtCurve::Vi => (13.5, 1.0),
            IdmtCurve::Ei => (80.0, 2.0),
            IdmtCurve::LtVi => (120.0, 1.0),
            IdmtCurve::IeeeMi => (0.0515, 0.02),
            IdmtCurve::IeeeVi => (19.61, 2.0),
            IdmtCurve::IeeeEi => (28.2, 2.0),
        }
    }
}

/// Settings for a single overcurrent relay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OcRelaySettings {
    /// Relay unique identifier.
    pub id: usize,
    /// Human-readable relay name.
    pub name: String,
    /// Physical location description.
    pub location: String,
    /// IDMT curve type.
    pub curve_type: IdmtCurve,
    /// Time multiplier setting (TMS).
    pub time_multiplier: f64,
    /// Pickup current \[A\] (Is).
    pub pickup_current_a: f64,
    /// Optional instantaneous element setting \[A\] (None if disabled).
    pub instantaneous_a: Option<f64>,
    /// Current transformer ratio (e.g. 400:1 → 400.0).
    pub ct_ratio: f64,
}

/// A primary→backup coordination pair evaluated at a specific fault current.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationPair {
    /// Primary relay id.
    pub primary: usize,
    /// Backup relay id.
    pub backup: usize,
    /// Fault current at which coordination is evaluated \[A\].
    pub fault_current_a: f64,
}

/// A detected coordination violation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationViolation {
    /// Primary relay id.
    pub primary: usize,
    /// Backup relay id.
    pub backup: usize,
    /// Fault current at which the violation was detected \[A\].
    pub fault_current_a: f64,
    /// Actual CTI = t_backup − t_primary \[s\].
    pub actual_cti_s: f64,
    /// Required minimum CTI \[s\].
    pub required_cti_s: f64,
}

/// Overcurrent relay coordination tool.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OcCoordinationTool {
    /// All relay settings in the protection scheme.
    pub relays: Vec<OcRelaySettings>,
    /// Coordination pairs (primary, backup, fault_current).
    pub ctis: Vec<CoordinationPair>,
    /// Target coordination time interval \[s\] (default 0.3 s).
    pub target_cti_s: f64,
}

impl OcCoordinationTool {
    /// Compute IDMT operating time \[s\] for a relay at a given fault current \[A\].
    ///
    /// Uses `t = TMS · k / ((I/Is)^α − 1)`.
    ///
    /// Returns `f64::INFINITY` when `I ≤ Is` (relay does not pick up).
    pub fn operating_time(&self, relay: &OcRelaySettings, fault_current_a: f64) -> f64 {
        if relay.pickup_current_a < 1e-10 {
            return f64::INFINITY;
        }
        let m = fault_current_a / relay.pickup_current_a;
        if m <= 1.0 {
            return f64::INFINITY;
        }
        // Check instantaneous element
        if let Some(inst_a) = relay.instantaneous_a {
            if fault_current_a >= inst_a {
                return 0.0;
            }
        }
        let (k, alpha) = relay.curve_type.coefficients();
        let denom = m.powf(alpha) - 1.0;
        if denom < 1e-12 {
            return f64::INFINITY;
        }
        relay.time_multiplier * k / denom
    }

    /// Check coordination for all registered pairs.
    ///
    /// Returns a list of [`CoordinationViolation`] where `t_backup − t_primary < CTI`.
    pub fn check_coordination(&self) -> Vec<CoordinationViolation> {
        let mut violations = Vec::new();
        for pair in &self.ctis {
            let relay_primary = self.relays.iter().find(|r| r.id == pair.primary);
            let relay_backup = self.relays.iter().find(|r| r.id == pair.backup);
            let (Some(rp), Some(rb)) = (relay_primary, relay_backup) else {
                continue;
            };
            let t_primary = self.operating_time(rp, pair.fault_current_a);
            let t_backup = self.operating_time(rb, pair.fault_current_a);
            let actual_cti = t_backup - t_primary;
            if actual_cti < self.target_cti_s {
                violations.push(CoordinationViolation {
                    primary: pair.primary,
                    backup: pair.backup,
                    fault_current_a: pair.fault_current_a,
                    actual_cti_s: actual_cti,
                    required_cti_s: self.target_cti_s,
                });
            }
        }
        violations
    }

    /// Find the minimum TMS for `relay_id` such that for all provided
    /// `(primary_relay_id, fault_current_a)` pairs the backup-to-primary CTI
    /// is at least `target_cti_s`.
    ///
    /// Uses binary search in \[0.01, 10.0\].
    pub fn suggest_tms(&mut self, relay_id: usize, coordination_pairs: &[(usize, f64)]) -> f64 {
        // Binary search bounds
        let mut lo = 0.01_f64;
        let mut hi = 10.0_f64;
        let iterations = 60;

        for _ in 0..iterations {
            let mid = (lo + hi) / 2.0;
            // Temporarily apply mid TMS to the target relay
            if self.tms_satisfies(relay_id, mid, coordination_pairs) {
                hi = mid;
            } else {
                lo = mid;
            }
        }

        // Apply the found TMS to the relay
        if let Some(relay) = self.relays.iter_mut().find(|r| r.id == relay_id) {
            relay.time_multiplier = hi;
        }
        hi
    }

    /// Internal: check whether a given TMS satisfies all coordination pairs.
    fn tms_satisfies(&self, relay_id: usize, tms: f64, pairs: &[(usize, f64)]) -> bool {
        for &(primary_id, fault_a) in pairs {
            let relay_primary = self.relays.iter().find(|r| r.id == primary_id);
            let relay_backup = self.relays.iter().find(|r| r.id == relay_id);
            let (Some(rp), Some(rb)) = (relay_primary, relay_backup) else {
                return false;
            };
            let t_primary = self.operating_time(rp, fault_a);
            // Compute backup time with the trial TMS
            let mut rb_trial = rb.clone();
            rb_trial.time_multiplier = tms;
            let t_backup = self.operating_time(&rb_trial, fault_a);
            if t_backup - t_primary < self.target_cti_s {
                return false;
            }
        }
        true
    }

    /// Generate a grading chart: for each fault current in the range, compute
    /// the operating time for every relay.
    ///
    /// Returns `Vec<(fault_current_A, Vec<operating_time_s>)>` with `n_points`
    /// logarithmically spaced current values in `(i_min, i_max)` \[A\].
    pub fn grading_chart(
        &self,
        fault_current_range: (f64, f64),
        n_points: usize,
    ) -> Vec<(f64, Vec<f64>)> {
        if n_points == 0 {
            return Vec::new();
        }
        let (i_min, i_max) = fault_current_range;
        let log_min = i_min.max(1e-6).ln();
        let log_max = i_max.max(i_min + 1.0).ln();
        (0..n_points)
            .map(|idx| {
                let frac = if n_points == 1 {
                    0.0
                } else {
                    idx as f64 / (n_points - 1) as f64
                };
                let current = (log_min + frac * (log_max - log_min)).exp();
                let times: Vec<f64> = self
                    .relays
                    .iter()
                    .map(|r| self.operating_time(r, current))
                    .collect();
                (current, times)
            })
            .collect()
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. Differential Protection (87T / 87B)
// ═══════════════════════════════════════════════════════════════════════════════

/// Device type for differential protection.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DiffDeviceType {
    /// Power transformer (87T).
    Transformer,
    /// Busbar (87B).
    Busbar,
    /// Synchronous generator (87G).
    Generator,
    /// Motor (87M).
    Motor,
}

/// Outcome of the differential relay decision logic.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DiffRelayDecision {
    /// Differential element is below threshold — no trip.
    NoTrip,
    /// Differential element exceeds threshold — trip.
    Operate,
    /// Trip blocked due to 2nd harmonic (magnetising inrush).
    BlockedInrush,
    /// Trip blocked due to 5th harmonic (overexcitation).
    BlockedOverexcitation,
}

/// Dual-slope percentage-bias differential protection relay (87T / 87B).
///
/// The operate/restrain characteristic is:
/// - Below `breakpoint1_pu`: `I_diff_pickup = base_diff_current_pct · I_rated`
/// - From `breakpoint1_pu` to `breakpoint2_pu`: `I_diff_pickup = slope1 · I_restrain`
/// - Above `breakpoint2_pu`: `I_diff_pickup = slope2 · I_restrain`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DifferentialProtection87 {
    /// Protected device type.
    pub device_type: DiffDeviceType,
    /// Rated current of the protected device \[A\].
    pub rated_current_a: f64,
    /// Minimum differential pickup as a fraction of rated (e.g. 0.20 = 20 \[%\]).
    pub base_diff_current_pct: f64,
    /// First slope (low-restraint region), e.g. 0.25.
    pub slope1_pct: f64,
    /// Second slope (high-restraint region), e.g. 0.50.
    pub slope2_pct: f64,
    /// First knee-point of restraint characteristic \[pu\], e.g. 1.0.
    pub breakpoint1_pu: f64,
    /// Second knee-point of restraint characteristic \[pu\], e.g. 2.5.
    pub breakpoint2_pu: f64,
    /// Enable harmonic restraint blocking.
    pub harmonic_restraint: bool,
    /// 2nd harmonic block threshold as fraction of differential (e.g. 0.15 = 15 \[%\]).
    pub second_harmonic_pct: f64,
    /// 5th harmonic block threshold as fraction of differential (e.g. 0.35 = 35 \[%\]).
    pub fifth_harmonic_pct: f64,
}

impl DifferentialProtection87 {
    /// Restrain (bias) current \[pu of rated\].
    ///
    /// IEEE convention: `Ir = (|I1| + |I2|) / 2`.
    pub fn restraint_current(&self, i1_pu: f64, i2_pu: f64) -> f64 {
        (i1_pu.abs() + i2_pu.abs()) / 2.0
    }

    /// Differential (operate) current \[pu of rated\].
    ///
    /// `Id = |I1 + I2|`
    pub fn differential_current(&self, i1_pu: f64, i2_pu: f64) -> f64 {
        (i1_pu + i2_pu).abs()
    }

    /// Pickup threshold \[pu of rated\] for a given restraint current.
    ///
    /// Dual-slope characteristic:
    /// - `I_restrain ≤ breakpoint1`: threshold = `base_diff_current_pct`
    /// - `breakpoint1 < I_restrain ≤ breakpoint2`: threshold = `slope1 · I_restrain`
    /// - `I_restrain > breakpoint2`: threshold = `slope2 · I_restrain`
    pub fn pickup_threshold(&self, i_restraint_pu: f64) -> f64 {
        let base = self.base_diff_current_pct;
        if i_restraint_pu <= self.breakpoint1_pu {
            base
        } else if i_restraint_pu <= self.breakpoint2_pu {
            let slope_thresh = self.slope1_pct * i_restraint_pu;
            slope_thresh.max(base)
        } else {
            let slope_thresh = self.slope2_pct * i_restraint_pu;
            slope_thresh.max(base)
        }
    }

    /// Full relay evaluation.
    ///
    /// # Arguments
    /// * `i1_pu`, `i2_pu` — winding currents (normalised to rated).
    /// * `second_harmonic_pct` — 2nd harmonic as fraction of fundamental differential.
    /// * `fifth_harmonic_pct`  — 5th harmonic as fraction of fundamental differential.
    pub fn evaluate(
        &self,
        i1_pu: f64,
        i2_pu: f64,
        second_harmonic_pct: f64,
        fifth_harmonic_pct: f64,
    ) -> DiffRelayDecision {
        let i_diff = self.differential_current(i1_pu, i2_pu);
        let i_rst = self.restraint_current(i1_pu, i2_pu);
        let threshold = self.pickup_threshold(i_rst);

        if i_diff <= threshold {
            return DiffRelayDecision::NoTrip;
        }

        // Operate condition met — check harmonic blocking
        if self.harmonic_restraint {
            if second_harmonic_pct >= self.second_harmonic_pct {
                return DiffRelayDecision::BlockedInrush;
            }
            if fifth_harmonic_pct >= self.fifth_harmonic_pct {
                return DiffRelayDecision::BlockedOverexcitation;
            }
        }

        DiffRelayDecision::Operate
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. Distance Protection (21) Zone Coordination
// ═══════════════════════════════════════════════════════════════════════════════

/// Distance relay zone characteristic shape.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ZoneCharacteristic {
    /// Mho (circular) — inherent load-rejection capability.
    Mho,
    /// Quadrilateral — independent R and X reach settings.
    Quad,
    /// Lens — reduced load encroachment for long lines.
    Lens,
    /// Offset Mho — extends into the 3rd quadrant for close-in faults.
    Offset,
}

/// A single distance protection zone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceZone {
    /// Zone number (1 = fastest/innermost).
    pub zone_number: usize,
    /// Reach as a percentage of line impedance \[%\].
    pub reach_pct: f64,
    /// Trip time delay \[ms\].
    pub time_delay_ms: f64,
    /// Zone shape characteristic.
    pub characteristic: ZoneCharacteristic,
}

/// Distance relay zone settings for a protected line.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceZoneSettings {
    /// All configured zones (ordered by zone_number ascending).
    pub zones: Vec<DistanceZone>,
    /// Positive-sequence impedance of the protected line \[pu\].
    pub line_impedance_pu: Complex<f64>,
    /// Physical line length \[km\].
    pub line_length_km: f64,
}

impl DistanceZoneSettings {
    /// Construct a standard three-zone distance relay.
    ///
    /// | Zone | Reach | Delay |
    /// |------|-------|-------|
    /// | 1    | 80 \[%\]  | 0 ms  |
    /// | 2    | 120 \[%\] | 300 ms |
    /// | 3    | 200 \[%\] | 600 ms |
    pub fn new_three_zone(line_z: Complex<f64>, length_km: f64) -> Self {
        Self {
            zones: vec![
                DistanceZone {
                    zone_number: 1,
                    reach_pct: 80.0,
                    time_delay_ms: 0.0,
                    characteristic: ZoneCharacteristic::Mho,
                },
                DistanceZone {
                    zone_number: 2,
                    reach_pct: 120.0,
                    time_delay_ms: 300.0,
                    characteristic: ZoneCharacteristic::Mho,
                },
                DistanceZone {
                    zone_number: 3,
                    reach_pct: 200.0,
                    time_delay_ms: 600.0,
                    characteristic: ZoneCharacteristic::Mho,
                },
            ],
            line_impedance_pu: line_z,
            line_length_km: length_km,
        }
    }

    /// Determine which zone the fault impedance falls into.
    ///
    /// Returns `Some(zone_number)` for the innermost enclosing zone, or `None`
    /// if the fault is beyond all configured zones.
    ///
    /// Detection uses the magnitude of the fault impedance relative to each
    /// zone's reach impedance (Mho-equivalent check):
    /// `|Z_fault| ≤ reach_pct/100 · |Z_line|`.
    pub fn zone_for_fault(&self, fault_impedance: Complex<f64>) -> Option<usize> {
        let z_line_mag = self.line_impedance_pu.norm();
        let z_fault_mag = fault_impedance.norm();
        // Also check that fault is in the forward direction (positive R+jX quadrant)
        // and angle deviation is within ±30° of line angle.
        let line_angle = self.line_impedance_pu.im.atan2(self.line_impedance_pu.re);
        let fault_angle = fault_impedance.im.atan2(fault_impedance.re);
        let angle_diff = (fault_angle - line_angle).abs();
        // Allow ±30° mismatch (typical Mho relay angular acceptance)
        if angle_diff > std::f64::consts::PI / 6.0 && z_fault_mag > 1e-10 {
            return None;
        }

        let mut sorted_zones = self.zones.clone();
        sorted_zones.sort_by(|a, b| a.zone_number.cmp(&b.zone_number));

        for zone in &sorted_zones {
            let reach_z = (zone.reach_pct / 100.0) * z_line_mag;
            if z_fault_mag <= reach_z {
                return Some(zone.zone_number);
            }
        }
        None
    }

    /// Operating time \[ms\] for a fault at the given impedance.
    ///
    /// Returns the time delay of the enclosing zone, or `None` if no zone covers the fault.
    pub fn operating_time_ms(&self, fault_z: Complex<f64>) -> Option<f64> {
        self.zone_for_fault(fault_z)
            .and_then(|zn| self.zones.iter().find(|z| z.zone_number == zn))
            .map(|z| z.time_delay_ms)
    }

    /// Check whether a load impedance encroaches on Zone 3.
    ///
    /// Returns `true` if the load impedance magnitude is within the Zone 3 reach.
    /// Zone 3 is identified as the zone with the largest reach.
    pub fn check_load_encroachment(&self, load_impedance: Complex<f64>) -> bool {
        let z_line_mag = self.line_impedance_pu.norm();
        let z_load_mag = load_impedance.norm();
        let max_reach_pct = self
            .zones
            .iter()
            .map(|z| z.reach_pct)
            .fold(f64::NEG_INFINITY, f64::max);
        let zone3_reach = (max_reach_pct / 100.0) * z_line_mag;
        z_load_mag <= zone3_reach
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. Bus Protection (87B) with Multiple Feeders
// ═══════════════════════════════════════════════════════════════════════════════

/// Result of bus protection evaluation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum BusProtResult {
    /// Differential current within threshold — busbar is secure.
    Secure,
    /// Differential current exceeds threshold — trip all feeders.
    Operate,
}

/// Current transformer on a single feeder.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederCt {
    /// Feeder identifier.
    pub feeder_id: usize,
    /// Rated primary current \[A\].
    pub rated_current_a: f64,
    /// CT ratio (primary:secondary, e.g. 400 → 400:1).
    pub ct_ratio: f64,
    /// Current polarity: +1 if current flows into bus, −1 if out of bus.
    pub polarity: f64,
}

/// Multi-feeder bus differential protection (87B).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusProtection87B {
    /// Bus identifier.
    pub bus_id: usize,
    /// All feeder CTs connected to this bus zone.
    pub feeders: Vec<FeederCt>,
    /// Differential pickup threshold \[pu of the largest feeder rating\].
    pub diff_threshold_pu: f64,
    /// Enable zone-assignment cross-check (bus zone integrity check).
    pub check_zone_assignment: bool,
}

impl BusProtection87B {
    /// Compute the differential (operate) current \[pu\].
    ///
    /// `I_diff = |Σ (polarity_k · I_k / rated_k)|`
    ///
    /// In normal load the algebraic sum should be ≈ 0; a fault on the bus
    /// causes a large net imbalance.
    pub fn differential_current_pu(&self, feeder_currents_a: &[f64]) -> f64 {
        if self.feeders.is_empty() {
            return 0.0;
        }
        let sum: f64 = self
            .feeders
            .iter()
            .zip(feeder_currents_a.iter())
            .map(|(ct, &i_a)| {
                let i_rated = ct.rated_current_a.max(1e-6);
                ct.polarity * i_a / i_rated
            })
            .sum();
        sum.abs()
    }

    /// Compute the restrain current \[pu\].
    ///
    /// `I_restrain = Σ |I_k / I_rated_k| / n_feeders`
    pub fn restraint_current_pu(&self, feeder_currents_a: &[f64]) -> f64 {
        if self.feeders.is_empty() {
            return 0.0;
        }
        let sum: f64 = self
            .feeders
            .iter()
            .zip(feeder_currents_a.iter())
            .map(|(ct, &i_a)| {
                let i_rated = ct.rated_current_a.max(1e-6);
                (i_a / i_rated).abs()
            })
            .sum();
        sum / self.feeders.len() as f64
    }

    /// Evaluate busbar protection.
    ///
    /// Trips if `I_diff_pu > diff_threshold_pu`.
    pub fn evaluate(&self, feeder_currents_a: &[f64]) -> BusProtResult {
        let i_diff = self.differential_current_pu(feeder_currents_a);
        if i_diff > self.diff_threshold_pu {
            BusProtResult::Operate
        } else {
            BusProtResult::Secure
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. Protection Coordination Report
// ═══════════════════════════════════════════════════════════════════════════════

/// Protection coordination report for a complete system study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtectionReport {
    /// Name of the system or substation studied.
    pub system_name: String,
    /// Maximum fault level at the study bus \[kA\].
    pub fault_level_ka: f64,
    /// Relay settings included in the study.
    pub relay_settings: Vec<OcRelaySettings>,
    /// All detected coordination violations.
    pub violations: Vec<CoordinationViolation>,
    /// Textual recommendations.
    pub recommendations: Vec<String>,
}

impl ProtectionReport {
    /// Generate a coordination report from an [`OcCoordinationTool`].
    pub fn generate(tool: &OcCoordinationTool, fault_ka: f64, system_name: &str) -> Self {
        let violations = tool.check_coordination();
        let mut recommendations = Vec::new();

        if violations.is_empty() {
            recommendations
                .push("All relay pairs satisfy the target CTI — no action required.".to_string());
        } else {
            for v in &violations {
                recommendations.push(format!(
                    "Relay {} (backup for relay {}): increase TMS to achieve CTI ≥ {:.2} s \
                     at {:.0} [A] (current CTI = {:.3} s).",
                    v.backup, v.primary, v.required_cti_s, v.fault_current_a, v.actual_cti_s
                ));
            }
        }

        Self {
            system_name: system_name.to_string(),
            fault_level_ka: fault_ka,
            relay_settings: tool.relays.clone(),
            violations,
            recommendations,
        }
    }

    /// Returns `true` if no coordination violations were found.
    pub fn is_coordinated(&self) -> bool {
        self.violations.is_empty()
    }

    /// Human-readable summary string.
    pub fn summary_text(&self) -> String {
        let status = if self.is_coordinated() {
            "PASS — fully coordinated"
        } else {
            "FAIL — coordination violations detected"
        };
        let mut s = format!(
            "Protection Coordination Report — {}\n\
             Fault level: {:.3} [kA]  |  Relays: {}  |  Status: {}\n",
            self.system_name,
            self.fault_level_ka,
            self.relay_settings.len(),
            status,
        );
        if !self.violations.is_empty() {
            s.push_str(&format!("Violations ({})\n", self.violations.len()));
            for v in &self.violations {
                s.push_str(&format!(
                    "  Relay {} → {}: CTI = {:.3} s (required {:.3} s) at {:.0} [A]\n",
                    v.primary, v.backup, v.actual_cti_s, v.required_cti_s, v.fault_current_a
                ));
            }
        }
        s.push_str("Recommendations:\n");
        for r in &self.recommendations {
            s.push_str(&format!("  - {}\n", r));
        }
        s
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex;

    // ── Helpers ─────────────────────────────────────────────────────────────

    fn make_iec_calc() -> Iec60909Calculator {
        Iec60909Calculator {
            c_factor: 1.05,
            base_mva: 100.0,
            base_kv: 110.0,
            system_impedance_pu: Complex::new(0.01, 0.10),
        }
    }

    fn make_si_relay(id: usize, pickup_a: f64, tms: f64) -> OcRelaySettings {
        OcRelaySettings {
            id,
            name: format!("R{id}"),
            location: "feeder".to_string(),
            curve_type: IdmtCurve::Si,
            time_multiplier: tms,
            pickup_current_a: pickup_a,
            instantaneous_a: None,
            ct_ratio: 400.0,
        }
    }

    fn make_87_relay() -> DifferentialProtection87 {
        DifferentialProtection87 {
            device_type: DiffDeviceType::Transformer,
            rated_current_a: 1000.0,
            base_diff_current_pct: 0.20,
            slope1_pct: 0.25,
            slope2_pct: 0.50,
            breakpoint1_pu: 1.0,
            breakpoint2_pu: 2.5,
            harmonic_restraint: true,
            second_harmonic_pct: 0.15,
            fifth_harmonic_pct: 0.35,
        }
    }

    // ── IEC 60909 tests ──────────────────────────────────────────────────────

    #[test]
    fn test_iec60909_three_phase_positive_ka() {
        let calc = make_iec_calc();
        let i_ka = calc.three_phase_fault_current_ka(1.0);
        assert!(
            i_ka > 0.0,
            "3-phase fault current must be positive, got {i_ka}"
        );
    }

    #[test]
    fn test_iec60909_peak_kappa_increases_with_xr() {
        let calc = make_iec_calc();
        let k_low = calc.peak_factor_kappa(1.0);
        let k_high = calc.peak_factor_kappa(10.0);
        assert!(
            k_high > k_low,
            "κ should increase with X/R: κ(10)={k_high:.4} vs κ(1)={k_low:.4}"
        );
    }

    #[test]
    fn test_iec60909_peak_exceeds_rms() {
        let calc = make_iec_calc();
        let i_rms = calc.three_phase_fault_current_ka(1.0);
        let i_peak = calc.peak_fault_current_ka(i_rms, 10.0);
        assert!(
            i_peak > i_rms,
            "Peak current {i_peak:.4} kA must exceed rms {i_rms:.4} kA"
        );
    }

    #[test]
    fn test_iec60909_slg_positive_ka() {
        let calc = make_iec_calc();
        let z_base = (calc.base_kv * calc.base_kv) / calc.base_mva; // Ω
        let z1 = Complex::new(0.01 * z_base, 0.10 * z_base);
        let z2 = z1;
        let z0 = Complex::new(0.03 * z_base, 0.30 * z_base);
        let i_ka = calc.single_line_to_ground_ka(z1, z2, z0);
        assert!(i_ka > 0.0, "SLG fault current must be positive, got {i_ka}");
    }

    #[test]
    fn test_iec60909_break_fault_positive_and_bounded() {
        let calc = make_iec_calc();
        let i_sym = 5.0; // kA
        let tau = 0.05; // s DC time constant
        let i_break = calc.break_fault_current_ka(i_sym, 0.06, tau);
        // μ = 0.84 + 0.26·exp(−0.26·t/τ) ∈ [0.84, 1.10], so Ib ≤ 1.10·Isym
        assert!(
            i_break > 0.0,
            "Breaking current must be positive, got {i_break:.4}"
        );
        assert!(
            i_break <= 1.11 * i_sym,
            "Breaking current {i_break:.4} must be ≤ 1.10·I_sym = {:.4}",
            1.10 * i_sym
        );
        // For long break times, μ → 0.84 and Ib < Isym
        let i_break_long = calc.break_fault_current_ka(i_sym, 10.0, tau);
        assert!(
            i_break_long < i_sym,
            "Long break time: Ib {i_break_long:.4} should be < I_sym {i_sym:.4}"
        );
    }

    // ── OC Coordination tests ────────────────────────────────────────────────

    #[test]
    fn test_oc_si_curve_operating_time() {
        let tool = OcCoordinationTool {
            relays: vec![make_si_relay(0, 100.0, 0.2)],
            ctis: vec![],
            target_cti_s: 0.3,
        };
        let relay = &tool.relays[0];
        // Manual: m = 500/100 = 5.0, t = 0.2 * 0.14 / (5^0.02 - 1)
        let m = 500.0_f64 / 100.0;
        let (k, alpha) = (0.14_f64, 0.02_f64);
        let expected = 0.2 * k / (m.powf(alpha) - 1.0);
        let actual = tool.operating_time(relay, 500.0);
        assert!(
            (actual - expected).abs() < 1e-9,
            "SI curve: expected {expected:.6} s, got {actual:.6} s"
        );
    }

    #[test]
    fn test_oc_check_coordination_violation() {
        let relays = vec![
            make_si_relay(0, 100.0, 0.10), // primary — fast
            make_si_relay(1, 100.0, 0.11), // backup  — barely slower
        ];
        let tool = OcCoordinationTool {
            relays,
            ctis: vec![CoordinationPair {
                primary: 0,
                backup: 1,
                fault_current_a: 500.0,
            }],
            target_cti_s: 0.3,
        };
        let violations = tool.check_coordination();
        assert!(
            !violations.is_empty(),
            "Expect coordination violation when TMS difference is tiny"
        );
        let v = &violations[0];
        assert!(v.actual_cti_s < v.required_cti_s);
    }

    #[test]
    fn test_oc_suggest_tms_achieves_cti() {
        let relays = vec![
            make_si_relay(0, 100.0, 0.10), // primary
            make_si_relay(1, 100.0, 0.10), // backup — start same TMS
        ];
        let mut tool = OcCoordinationTool {
            relays,
            ctis: vec![],
            target_cti_s: 0.3,
        };
        let new_tms = tool.suggest_tms(1, &[(0, 500.0)]);
        assert!(
            new_tms > 0.0,
            "Suggested TMS must be positive, got {new_tms}"
        );

        // Verify post-application the relay pair is coordinated
        let violations = {
            let mut tool2 = tool.clone();
            tool2.ctis = vec![CoordinationPair {
                primary: 0,
                backup: 1,
                fault_current_a: 500.0,
            }];
            tool2.check_coordination()
        };
        assert!(
            violations.is_empty(),
            "After suggest_tms the pair should be coordinated, violations: {violations:?}"
        );
    }

    // ── Differential protection tests ────────────────────────────────────────

    #[test]
    fn test_diff_below_threshold_no_trip() {
        let relay = make_87_relay();
        // Small differential — both currents nearly equal and opposite (normal load)
        let decision = relay.evaluate(1.0, -1.02, 0.0, 0.0);
        assert_eq!(
            decision,
            DiffRelayDecision::NoTrip,
            "Small differential should not trip: {decision:?}"
        );
    }

    #[test]
    fn test_diff_above_threshold_operate() {
        let relay = make_87_relay();
        // Large unbalanced current — simulate internal fault
        let decision = relay.evaluate(5.0, 0.0, 0.0, 0.0);
        assert_eq!(
            decision,
            DiffRelayDecision::Operate,
            "Large differential with no harmonics should trip: {decision:?}"
        );
    }

    #[test]
    fn test_diff_high_second_harmonic_blocks_inrush() {
        let relay = make_87_relay();
        // Sufficient differential to trip but with >15% 2nd harmonic
        let decision = relay.evaluate(3.0, 0.0, 0.20, 0.0); // 20% 2nd harmonic
        assert_eq!(
            decision,
            DiffRelayDecision::BlockedInrush,
            "High 2nd harmonic should block (inrush): {decision:?}"
        );
    }

    #[test]
    fn test_diff_high_fifth_harmonic_blocks_overexcitation() {
        let relay = make_87_relay();
        // Sufficient differential to trip but with >35% 5th harmonic
        let decision = relay.evaluate(3.0, 0.0, 0.05, 0.40); // 40% 5th harmonic
        assert_eq!(
            decision,
            DiffRelayDecision::BlockedOverexcitation,
            "High 5th harmonic should block (overexcitation): {decision:?}"
        );
    }

    // ── Distance protection tests ────────────────────────────────────────────

    #[test]
    fn test_distance_fault_zone1() {
        let line_z = Complex::new(0.01, 0.05);
        let settings = DistanceZoneSettings::new_three_zone(line_z, 100.0);
        // Fault at 70% of line — within zone 1 (80% reach)
        let z_fault = line_z * 0.70;
        let zone = settings.zone_for_fault(z_fault);
        assert_eq!(zone, Some(1), "Fault at 70% should be in zone 1: {zone:?}");
    }

    #[test]
    fn test_distance_fault_beyond_zone3() {
        let line_z = Complex::new(0.01, 0.05);
        let settings = DistanceZoneSettings::new_three_zone(line_z, 100.0);
        // Fault at 300% of line — beyond zone 3 (200% reach)
        let z_fault = line_z * 3.0;
        let zone = settings.zone_for_fault(z_fault);
        assert_eq!(
            zone, None,
            "Fault at 300% should be beyond all zones: {zone:?}"
        );
    }

    #[test]
    fn test_distance_load_encroachment() {
        let line_z = Complex::new(0.01, 0.05);
        let settings = DistanceZoneSettings::new_three_zone(line_z, 100.0);
        // Load impedance at 180% of line magnitude — within zone 3 (200%)
        let z_load = line_z * 1.8;
        assert!(
            settings.check_load_encroachment(z_load),
            "Load at 180% should encroach on zone 3"
        );
        // Load impedance at 250% — outside zone 3
        let z_load_far = line_z * 2.5;
        assert!(
            !settings.check_load_encroachment(z_load_far),
            "Load at 250% should NOT encroach on zone 3"
        );
    }

    // ── Bus protection tests ─────────────────────────────────────────────────

    #[test]
    fn test_bus_balanced_secure() {
        let bus = BusProtection87B {
            bus_id: 1,
            feeders: vec![
                FeederCt {
                    feeder_id: 0,
                    rated_current_a: 1000.0,
                    ct_ratio: 400.0,
                    polarity: 1.0,
                },
                FeederCt {
                    feeder_id: 1,
                    rated_current_a: 1000.0,
                    ct_ratio: 400.0,
                    polarity: -1.0,
                },
            ],
            diff_threshold_pu: 0.10,
            check_zone_assignment: false,
        };
        // Equal and opposite currents — perfect balance
        let result = bus.evaluate(&[800.0, 800.0]);
        assert_eq!(
            result,
            BusProtResult::Secure,
            "Balanced bus should be secure"
        );
    }

    #[test]
    fn test_bus_large_differential_operate() {
        let bus = BusProtection87B {
            bus_id: 2,
            feeders: vec![
                FeederCt {
                    feeder_id: 0,
                    rated_current_a: 1000.0,
                    ct_ratio: 400.0,
                    polarity: 1.0,
                },
                FeederCt {
                    feeder_id: 1,
                    rated_current_a: 1000.0,
                    ct_ratio: 400.0,
                    polarity: 1.0,
                },
            ],
            diff_threshold_pu: 0.10,
            check_zone_assignment: false,
        };
        // Both feeders inject current into bus — large differential (internal fault)
        let result = bus.evaluate(&[600.0, 600.0]);
        assert_eq!(
            result,
            BusProtResult::Operate,
            "Large differential should operate bus protection"
        );
    }

    // ── Report tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_report_is_coordinated_when_no_violations() {
        let relays = vec![
            make_si_relay(0, 100.0, 0.10),
            make_si_relay(1, 100.0, 0.50), // large TMS gap
        ];
        let tool = OcCoordinationTool {
            relays,
            ctis: vec![CoordinationPair {
                primary: 0,
                backup: 1,
                fault_current_a: 500.0,
            }],
            target_cti_s: 0.3,
        };
        let report = ProtectionReport::generate(&tool, 10.0, "TestSystem");
        assert!(
            report.is_coordinated(),
            "Should be coordinated with large TMS gap"
        );
        let summary = report.summary_text();
        assert!(summary.contains("PASS"), "Summary should say PASS");
    }
}
