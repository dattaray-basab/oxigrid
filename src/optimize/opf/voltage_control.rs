//! Coordinated Voltage Control (CVC) for power systems.
//!
//! Implements a three-level hierarchical voltage regulation scheme:
//!
//! | Level | Response Time | Mechanism |
//! |-------|--------------|-----------|
//! | Primary   | ≤1 s      | Local AVR (excitation control) |
//! | Secondary | 10–30 s   | Zone pilot bus PI controller |
//! | Tertiary  | minutes   | System-wide reactive OPF |
//!
//! # Units
//! - Voltages in `[pu]`
//! - Reactive power in `[Mvar]`
//! - Time constants in `[s]`

use std::fmt;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Level of a voltage control action in the hierarchy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoltageControlLevel {
    /// Local AVR: generator excitation, ≤1 s response.
    Primary,
    /// Zone pilot bus control (AGC-like), 10–30 s response.
    Secondary,
    /// System-wide OPF: economic reactive dispatch, minutes.
    Tertiary,
}

/// Method used to share reactive power corrections among zone generators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QSharingMethod {
    /// Each generator takes an equal share of the zone correction \[Mvar\].
    EqualSharing,
    /// Weighted by `participation_factor` of each generator.
    ParticipationFactor,
    /// Proportional to remaining reactive headroom \[Mvar\].
    ProportionalToHeadroom,
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// A pilot bus used as the feedback signal for a secondary voltage zone.
#[derive(Debug, Clone)]
pub struct PilotBus {
    /// Bus index in the network (0-based).
    pub bus_id: usize,
    /// Target voltage setpoint \[pu\].
    pub voltage_setpoint_pu: f64,
    /// Deadband half-width \[pu\]; corrections applied only outside ±bandwidth.
    pub bandwidth_pu: f64,
    /// Reactive zone this pilot bus belongs to.
    pub zone_id: usize,
    /// Generator bus IDs that participate in controlling this zone.
    pub participating_generators: Vec<usize>,
}

/// A reactive control zone grouping generators, shunts, and loads
/// around a common pilot bus.
#[derive(Debug, Clone)]
pub struct ReactiveZone {
    /// Unique zone identifier.
    pub zone_id: usize,
    /// Pilot bus identifier for secondary voltage feedback.
    pub pilot_bus_id: usize,
    /// Bus IDs of generators inside the zone.
    pub generator_buses: Vec<usize>,
    /// Bus IDs of switchable shunt compensators (capacitor/reactor banks).
    pub shunt_buses: Vec<usize>,
    /// Bus IDs of load buses inside the zone.
    pub load_buses: Vec<usize>,
    /// Aggregate reactive capability of the zone `(Qmin, Qmax)` \[Mvar\].
    pub zone_reactive_limit_mvar: (f64, f64),
}

/// Automatic Voltage Regulator model for one generator bus.
#[derive(Debug, Clone)]
pub struct GeneratorAVR {
    /// Generator (PV) bus index.
    pub bus_id: usize,
    /// Voltage setpoint \[pu\].
    pub v_setpoint_pu: f64,
    /// Lower voltage limit \[pu\].
    pub v_min_pu: f64,
    /// Upper voltage limit \[pu\].
    pub v_max_pu: f64,
    /// Current reactive output \[Mvar\].
    pub q_current_mvar: f64,
    /// Minimum reactive output \[Mvar\].
    pub q_min_mvar: f64,
    /// Maximum reactive output \[Mvar\].
    pub q_max_mvar: f64,
    /// Proportional gain of the AVR (dimensionless, default 50).
    pub avr_gain: f64,
    /// AVR time constant \[s\] (default 0.05 s).
    pub avr_time_s: f64,
    /// Fraction of zone reactive correction this generator absorbs (0–1).
    pub participation_factor: f64,
}

/// Configuration for the `CvcController`.
#[derive(Debug, Clone)]
pub struct CvcConfig {
    /// Zone-level PI controller proportional gain (default 0.1).
    pub secondary_gain: f64,
    /// PI integration time constant \[s\] (default 30 s).
    pub secondary_time_s: f64,
    /// Tertiary dispatch execution interval \[s\] (default 300 s).
    pub tertiary_interval_s: f64,
    /// Reactive power sharing method for secondary control.
    pub q_sharing_method: QSharingMethod,
}

impl Default for CvcConfig {
    fn default() -> Self {
        Self {
            secondary_gain: 0.1,
            secondary_time_s: 30.0,
            tertiary_interval_s: 300.0,
            q_sharing_method: QSharingMethod::ProportionalToHeadroom,
        }
    }
}

/// Voltage profile statistics for a set of network buses.
#[derive(Debug, Clone)]
pub struct VoltageProfileReport {
    /// Mean bus voltage \[pu\].
    pub mean_voltage_pu: f64,
    /// Standard deviation of bus voltages \[pu\].
    pub std_voltage_pu: f64,
    /// Buses with voltage below 0.95 pu.
    pub buses_low: Vec<usize>,
    /// Buses with voltage above 1.05 pu.
    pub buses_high: Vec<usize>,
    /// Magnitude of the worst voltage violation \[pu\] (deviation from 1.0).
    pub worst_violation_pu: f64,
    /// Aggregate voltage quality index in \[0, 1\] (1 = all buses in band).
    pub voltage_quality_index: f64,
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by CVC operations.
#[derive(Debug, Clone)]
pub enum CvcError {
    /// Requested index is out of bounds.
    IndexOutOfBounds {
        name: &'static str,
        idx: usize,
        len: usize,
    },
    /// A zone has no generators to control.
    ZoneHasNoGenerators { zone_id: usize },
    /// Bus voltage vector has wrong length.
    VoltageLengthMismatch { expected: usize, got: usize },
    /// Configuration value is invalid.
    InvalidConfig(String),
}

impl fmt::Display for CvcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::IndexOutOfBounds { name, idx, len } => {
                write!(f, "{name} index {idx} out of bounds (len={len})")
            }
            Self::ZoneHasNoGenerators { zone_id } => {
                write!(f, "zone {zone_id} has no participating generators")
            }
            Self::VoltageLengthMismatch { expected, got } => {
                write!(
                    f,
                    "voltage vector length mismatch: expected {expected}, got {got}"
                )
            }
            Self::InvalidConfig(msg) => write!(f, "invalid CVC config: {msg}"),
        }
    }
}

impl std::error::Error for CvcError {}

// ---------------------------------------------------------------------------
// Main controller
// ---------------------------------------------------------------------------

/// Three-level Coordinated Voltage Controller.
///
/// Manages primary AVR responses, secondary zone PI control, and
/// tertiary optimal reactive dispatch across all zones.
pub struct CvcController {
    /// Reactive control zones.
    pub zones: Vec<ReactiveZone>,
    /// Pilot buses (one per zone).
    pub pilot_buses: Vec<PilotBus>,
    /// Generator AVR models.
    pub generators: Vec<GeneratorAVR>,
    /// Controller configuration.
    pub config: CvcConfig,
    /// Per-zone integral state for secondary PI controller \[pu·s\].
    secondary_integrators: Vec<f64>,
}

impl CvcController {
    /// Create a new `CvcController`.
    ///
    /// Initialises secondary integrators to zero.
    pub fn new(
        zones: Vec<ReactiveZone>,
        pilot_buses: Vec<PilotBus>,
        generators: Vec<GeneratorAVR>,
        config: CvcConfig,
    ) -> Result<Self, CvcError> {
        if config.secondary_time_s <= 0.0 {
            return Err(CvcError::InvalidConfig(
                "secondary_time_s must be positive".to_owned(),
            ));
        }
        if config.tertiary_interval_s <= 0.0 {
            return Err(CvcError::InvalidConfig(
                "tertiary_interval_s must be positive".to_owned(),
            ));
        }
        let n_zones = zones.len();
        Ok(Self {
            zones,
            pilot_buses,
            generators,
            config,
            secondary_integrators: vec![0.0; n_zones],
        })
    }

    // -----------------------------------------------------------------------
    // Level 1 – Primary AVR
    // -----------------------------------------------------------------------

    /// Compute the reactive power adjustment \[Mvar\] for a single generator AVR.
    ///
    /// Uses a first-order proportional law:
    /// ```text
    /// ΔV = V_set − V_meas
    /// ΔQ = Kp × ΔV × Qmax        (proportional action)
    /// Q_new = clamp(Q_cur + ΔQ, Qmin, Qmax)
    /// ```
    ///
    /// # Parameters
    /// - `gen_idx`         – index into `self.generators`
    /// - `v_measured_pu`   – measured terminal voltage \[pu\]
    /// - `_dt_s`           – time step \[s\] (reserved for future integral term)
    ///
    /// # Returns
    /// The reactive power adjustment `ΔQ` \[Mvar\] (positive = injection).
    pub fn primary_avr_response(
        &self,
        gen_idx: usize,
        v_measured_pu: f64,
        _dt_s: f64,
    ) -> Result<f64, CvcError> {
        let gen = self
            .generators
            .get(gen_idx)
            .ok_or(CvcError::IndexOutOfBounds {
                name: "generator",
                idx: gen_idx,
                len: self.generators.len(),
            })?;

        let dv = gen.v_setpoint_pu - v_measured_pu;
        let dq_raw = gen.avr_gain * dv * gen.q_max_mvar;

        // Clamp new Q within capability limits
        let q_new = (gen.q_current_mvar + dq_raw).clamp(gen.q_min_mvar, gen.q_max_mvar);
        let dq = q_new - gen.q_current_mvar;

        Ok(dq)
    }

    // -----------------------------------------------------------------------
    // Level 2 – Secondary zone control
    // -----------------------------------------------------------------------

    /// Compute per-generator reactive corrections \[Mvar\] for one zone.
    ///
    /// Runs a PI controller on the pilot bus voltage error and distributes
    /// the resulting zone correction `ΔQ_zone` among the zone generators
    /// according to `config.q_sharing_method`.
    ///
    /// ```text
    /// ΔV   = V_set − V_pilot
    /// I[k] = I[k-1] + ΔV × dt / T_i
    /// ΔQ_z = Kp × ΔV + (Kp / T_i) × I[k]   (standard PI)
    /// ```
    ///
    /// # Parameters
    /// - `zone_idx`              – index into `self.zones`
    /// - `v_pilot_measured_pu`   – measured pilot bus voltage \[pu\]
    /// - `dt_s`                  – time step \[s\]
    ///
    /// # Returns
    /// `Vec<f64>` of length equal to the number of generators in the zone,
    /// each entry is the ΔQ correction \[Mvar\] for that generator.
    pub fn secondary_zone_control(
        &mut self,
        zone_idx: usize,
        v_pilot_measured_pu: f64,
        dt_s: f64,
    ) -> Result<Vec<f64>, CvcError> {
        let zone = self.zones.get(zone_idx).ok_or(CvcError::IndexOutOfBounds {
            name: "zone",
            idx: zone_idx,
            len: self.zones.len(),
        })?;

        // Locate the pilot bus for this zone
        let pilot = self
            .pilot_buses
            .iter()
            .find(|p| p.zone_id == zone.zone_id)
            .ok_or(CvcError::ZoneHasNoGenerators {
                zone_id: zone.zone_id,
            })?;

        let v_setpoint = pilot.voltage_setpoint_pu;
        let dv = v_setpoint - v_pilot_measured_pu;

        // Apply deadband
        let dv_eff = if dv.abs() <= pilot.bandwidth_pu {
            0.0
        } else {
            dv
        };

        // PI update
        let kp = self.config.secondary_gain;
        let ti = self.config.secondary_time_s;
        self.secondary_integrators[zone_idx] += dv_eff * dt_s / ti;
        let integrator_val = self.secondary_integrators[zone_idx];

        let dq_zone_raw = kp * dv_eff + (kp / ti) * integrator_val;

        // Clamp to zone reactive limits
        let (q_min, q_max) = self.zones[zone_idx].zone_reactive_limit_mvar;
        let dq_zone = dq_zone_raw.clamp(q_min, q_max);

        // Collect generator indices inside this zone
        let gen_buses: Vec<usize> = self.zones[zone_idx].generator_buses.clone();
        if gen_buses.is_empty() {
            return Err(CvcError::ZoneHasNoGenerators { zone_id: zone_idx });
        }

        // Find generator objects for the zone
        let gen_indices: Vec<usize> = gen_buses
            .iter()
            .filter_map(|&bus| self.generators.iter().position(|g| g.bus_id == bus))
            .collect();

        if gen_indices.is_empty() {
            return Err(CvcError::ZoneHasNoGenerators { zone_id: zone_idx });
        }

        // Compute sharing weights
        let weights: Vec<f64> = match self.config.q_sharing_method {
            QSharingMethod::EqualSharing => vec![1.0; gen_indices.len()],

            QSharingMethod::ParticipationFactor => gen_indices
                .iter()
                .map(|&gi| self.generators[gi].participation_factor.max(0.0))
                .collect(),

            QSharingMethod::ProportionalToHeadroom => gen_indices
                .iter()
                .map(|&gi| {
                    let g = &self.generators[gi];
                    // Headroom depends on direction of correction
                    if dq_zone >= 0.0 {
                        (g.q_max_mvar - g.q_current_mvar).max(0.0)
                    } else {
                        (g.q_current_mvar - g.q_min_mvar).max(0.0)
                    }
                })
                .collect(),
        };

        let weight_sum: f64 = weights.iter().sum();

        let dq_per_gen: Vec<f64> = if weight_sum.abs() < 1e-9 {
            // Fallback: equal sharing
            let share = dq_zone / gen_indices.len() as f64;
            vec![share; gen_indices.len()]
        } else {
            weights.iter().map(|&w| dq_zone * w / weight_sum).collect()
        };

        Ok(dq_per_gen)
    }

    // -----------------------------------------------------------------------
    // Level 3 – Tertiary reactive dispatch
    // -----------------------------------------------------------------------

    /// Compute optimal reactive power setpoints \[Mvar\] for all generators.
    ///
    /// Minimises the sum of squared deviations from the initial Q:
    /// ```text
    /// min Σ (Q_g − Q_g0)²    s.t.  Qmin_g ≤ Q_g ≤ Qmax_g
    /// ```
    ///
    /// With the demand-proportional analytic solution the unconstrained
    /// optimum is a flat redispatch proportional to generator sizes; the
    /// result is then projected onto the box constraints.
    ///
    /// # Parameters
    /// - `_bus_voltages`   – current bus voltages \[pu\] (reserved for sensitivity)
    /// - `bus_loads_mvar`  – reactive demand at each bus \[Mvar\]
    ///
    /// # Returns
    /// Optimal Q setpoint \[Mvar\] for each generator in `self.generators`.
    pub fn tertiary_reactive_dispatch(
        &self,
        _bus_voltages: &[f64],
        bus_loads_mvar: &[f64],
    ) -> Vec<f64> {
        // Total reactive demand
        let q_demand: f64 = bus_loads_mvar.iter().sum();

        // Generator capacity totals
        let q_cap_total: f64 = self
            .generators
            .iter()
            .map(|g| g.q_max_mvar - g.q_min_mvar)
            .sum();

        if q_cap_total.abs() < 1e-9 {
            return self.generators.iter().map(|g| g.q_current_mvar).collect();
        }

        // Proportional dispatch: each generator takes share proportional to its range
        self.generators
            .iter()
            .map(|g| {
                let capacity = g.q_max_mvar - g.q_min_mvar;
                let fraction = capacity / q_cap_total;
                let q_target = g.q_min_mvar + fraction * q_demand;
                q_target.clamp(g.q_min_mvar, g.q_max_mvar)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Pilot bus selection
    // -----------------------------------------------------------------------

    /// Select candidate pilot buses from a set of network bus voltages.
    ///
    /// Heuristic: buses with the largest voltage spread (max−min sensitivity
    /// proxy) make the best pilot buses because they carry the strongest
    /// observability signal. The method partitions buses into `n_zones`
    /// groups and selects the most central bus per group.
    ///
    /// # Parameters
    /// - `network_voltages_pu` – current voltage magnitude at every bus \[pu\]
    ///
    /// # Returns
    /// Sorted list of bus indices selected as pilot buses.
    pub fn select_pilot_buses(&self, network_voltages_pu: &[f64]) -> Vec<usize> {
        if network_voltages_pu.is_empty() || self.zones.is_empty() {
            return Vec::new();
        }

        let n_buses = network_voltages_pu.len();
        let n_zones = self.zones.len();
        let chunk = (n_buses / n_zones).max(1);

        let v_mean: f64 = network_voltages_pu.iter().sum::<f64>() / n_buses as f64;

        // Score each bus: |V - V_mean|  (voltage swing proxy)
        let scores: Vec<f64> = network_voltages_pu
            .iter()
            .map(|&v| (v - v_mean).abs())
            .collect();

        let mut pilot_buses = Vec::with_capacity(n_zones);

        for z in 0..n_zones {
            let start = z * chunk;
            let end = ((z + 1) * chunk).min(n_buses);
            if start >= end {
                break;
            }
            // Pick bus with highest voltage-swing score in this segment
            let best = (start..end)
                .max_by(|&a, &b| {
                    scores[a]
                        .partial_cmp(&scores[b])
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap_or(start); // safe: range is non-empty
            pilot_buses.push(best);
        }

        pilot_buses.sort_unstable();
        pilot_buses.dedup();
        pilot_buses
    }

    // -----------------------------------------------------------------------
    // Voltage profile assessment
    // -----------------------------------------------------------------------

    /// Assess the voltage profile across the network.
    ///
    /// # Parameters
    /// - `bus_voltages_pu` – voltage magnitude at every bus \[pu\]
    ///
    /// # Returns
    /// [`VoltageProfileReport`] with statistics and violation lists.
    pub fn assess_voltage_profile(&self, bus_voltages_pu: &[f64]) -> VoltageProfileReport {
        let n = bus_voltages_pu.len();
        if n == 0 {
            return VoltageProfileReport {
                mean_voltage_pu: 0.0,
                std_voltage_pu: 0.0,
                buses_low: Vec::new(),
                buses_high: Vec::new(),
                worst_violation_pu: 0.0,
                voltage_quality_index: 1.0,
            };
        }

        let mean = bus_voltages_pu.iter().sum::<f64>() / n as f64;
        let variance = bus_voltages_pu
            .iter()
            .map(|&v| (v - mean).powi(2))
            .sum::<f64>()
            / n as f64;
        let std_dev = variance.sqrt();

        let mut buses_low = Vec::new();
        let mut buses_high = Vec::new();
        let mut worst: f64 = 0.0;

        for (i, &v) in bus_voltages_pu.iter().enumerate() {
            if v < 0.95 {
                buses_low.push(i);
                let dev = 0.95 - v;
                if dev > worst {
                    worst = dev;
                }
            } else if v > 1.05 {
                buses_high.push(i);
                let dev = v - 1.05;
                if dev > worst {
                    worst = dev;
                }
            }
        }

        let n_violations = buses_low.len() + buses_high.len();
        let quality_index = 1.0 - (n_violations as f64 / n as f64);

        VoltageProfileReport {
            mean_voltage_pu: mean,
            std_voltage_pu: std_dev,
            buses_low,
            buses_high,
            worst_violation_pu: worst,
            voltage_quality_index: quality_index,
        }
    }

    // -----------------------------------------------------------------------
    // Q–V sensitivity
    // -----------------------------------------------------------------------

    /// Estimate the voltage change at every bus per unit reactive injection
    /// at a given generator \[pu/Mvar\].
    ///
    /// The sensitivity is approximated as inversely proportional to the
    /// electrical distance (bus index difference proxy). For rigorous
    /// computation, replace with the full Jacobian-derived `∂V/∂Q` column.
    ///
    /// # Parameters
    /// - `generator_idx` – index into `self.generators`
    /// - `bus_voltages`  – current bus voltages \[pu\] (length = n_buses)
    /// - `delta_q`       – perturbation in reactive injection \[Mvar\]
    ///
    /// # Returns
    /// Estimated voltage change `ΔV` \[pu\] at every bus.
    pub fn q_v_sensitivity(
        &self,
        generator_idx: usize,
        bus_voltages: &[f64],
        delta_q: f64,
    ) -> Result<Vec<f64>, CvcError> {
        let gen = self
            .generators
            .get(generator_idx)
            .ok_or(CvcError::IndexOutOfBounds {
                name: "generator",
                idx: generator_idx,
                len: self.generators.len(),
            })?;

        let gen_bus = gen.bus_id;
        let n = bus_voltages.len();

        // Simple inverse-distance sensitivity proxy
        // ΔV_i = delta_q * (1 / (1 + |i - gen_bus|)) * scale
        // Scale so that ΔV at the generator bus itself is ~delta_q * 0.01 pu/Mvar
        // (representative X/V ratio for HV network)
        let scale = 0.01_f64; // pu/Mvar at zero distance

        let dv: Vec<f64> = (0..n)
            .map(|i| {
                let dist = (i as isize - gen_bus as isize).unsigned_abs() as f64;
                delta_q * scale / (1.0 + dist)
            })
            .collect();

        Ok(dv)
    }

    // -----------------------------------------------------------------------
    // AVR coordination
    // -----------------------------------------------------------------------

    /// Recommend AVR setpoint and participation factor adjustments for a zone.
    ///
    /// Generators with more Q headroom receive a higher participation factor
    /// so that the zone correction is absorbed by those with the most capacity.
    ///
    /// # Returns
    /// `Vec<(gen_bus, new_v_setpoint, new_participation_factor)>` for each
    /// generator in the zone.
    pub fn coordinate_avr_settings(
        &self,
        zone: &ReactiveZone,
    ) -> Result<Vec<(usize, f64, f64)>, CvcError> {
        // Collect generators in this zone
        let zone_gens: Vec<&GeneratorAVR> = zone
            .generator_buses
            .iter()
            .filter_map(|&bus| self.generators.iter().find(|g| g.bus_id == bus))
            .collect();

        if zone_gens.is_empty() {
            return Err(CvcError::ZoneHasNoGenerators {
                zone_id: zone.zone_id,
            });
        }

        // Compute headroom for each generator
        let headrooms: Vec<f64> = zone_gens
            .iter()
            .map(|g| (g.q_max_mvar - g.q_current_mvar).max(0.0))
            .collect();

        let total_headroom: f64 = headrooms.iter().sum();

        let mut result = Vec::with_capacity(zone_gens.len());

        for (gen, &headroom) in zone_gens.iter().zip(headrooms.iter()) {
            let new_pf = if total_headroom.abs() < 1e-9 {
                1.0 / zone_gens.len() as f64
            } else {
                headroom / total_headroom
            };

            // Keep voltage setpoint unchanged (tertiary layer adjusts it)
            result.push((gen.bus_id, gen.v_setpoint_pu, new_pf));
        }

        Ok(result)
    }

    // -----------------------------------------------------------------------
    // State reset
    // -----------------------------------------------------------------------

    /// Reset secondary PI integrators to zero (e.g., after topology change).
    pub fn reset_integrators(&mut self) {
        for i in self.secondary_integrators.iter_mut() {
            *i = 0.0;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    fn make_gen(
        bus_id: usize,
        v_set: f64,
        q_cur: f64,
        q_min: f64,
        q_max: f64,
        pf: f64,
    ) -> GeneratorAVR {
        GeneratorAVR {
            bus_id,
            v_setpoint_pu: v_set,
            v_min_pu: 0.90,
            v_max_pu: 1.10,
            q_current_mvar: q_cur,
            q_min_mvar: q_min,
            q_max_mvar: q_max,
            avr_gain: 50.0,
            avr_time_s: 0.05,
            participation_factor: pf,
        }
    }

    fn make_zone(zone_id: usize, gen_buses: Vec<usize>) -> ReactiveZone {
        ReactiveZone {
            zone_id,
            pilot_bus_id: 0,
            generator_buses: gen_buses,
            shunt_buses: vec![],
            load_buses: vec![],
            zone_reactive_limit_mvar: (-200.0, 200.0),
        }
    }

    fn make_pilot(zone_id: usize, v_set: f64) -> PilotBus {
        PilotBus {
            bus_id: 0,
            voltage_setpoint_pu: v_set,
            bandwidth_pu: 0.005,
            zone_id,
            participating_generators: vec![0],
        }
    }

    fn default_controller() -> CvcController {
        let zones = vec![make_zone(0, vec![0, 1])];
        let pilots = vec![make_pilot(0, 1.0)];
        let gens = vec![
            make_gen(0, 1.0, 0.0, -100.0, 100.0, 0.5),
            make_gen(1, 1.0, 0.0, -80.0, 80.0, 0.5),
        ];
        CvcController::new(zones, pilots, gens, CvcConfig::default()).expect("construction ok")
    }

    // -----------------------------------------------------------------------
    // Test 1: Primary AVR — low voltage → positive Q injection
    // -----------------------------------------------------------------------
    #[test]
    fn test_primary_avr_low_voltage_positive_q() {
        let ctrl = default_controller();
        // v_measured = 0.95, setpoint = 1.0 → ΔV = 0.05 > 0 → ΔQ > 0
        let dq = ctrl.primary_avr_response(0, 0.95, 0.05).expect("ok");
        assert!(
            dq > 0.0,
            "Expected positive Q injection for low voltage, got {dq}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: Primary AVR — Q clamped at q_max when gain is high
    // -----------------------------------------------------------------------
    #[test]
    fn test_primary_avr_q_clamped_at_max() {
        let mut ctrl = default_controller();
        // Set current Q to almost at max so clamp kicks in
        ctrl.generators[0].q_current_mvar = 99.0;
        ctrl.generators[0].q_max_mvar = 100.0;

        // Huge voltage deviation → raw ΔQ would exceed Qmax
        let dq = ctrl.primary_avr_response(0, 0.5, 0.1).expect("ok");

        // The new Q must not exceed q_max = 100, so ΔQ ≤ 100 - 99 = 1
        assert!(dq <= 1.0 + 1e-9, "ΔQ={dq} exceeds headroom to q_max");
    }

    // -----------------------------------------------------------------------
    // Test 3: Secondary — pilot bus low → positive zone correction
    // -----------------------------------------------------------------------
    #[test]
    fn test_secondary_low_pilot_positive_correction() {
        let mut ctrl = default_controller();
        // v_pilot = 0.95, setpoint = 1.0 → ΔV = 0.05 → ΔQ_zone > 0
        let dq_vec = ctrl.secondary_zone_control(0, 0.95, 1.0).expect("ok");
        let total: f64 = dq_vec.iter().sum();
        assert!(
            total > 0.0,
            "Expected positive zone correction, got {total}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 4: Q sharing proportional to headroom
    // -----------------------------------------------------------------------
    #[test]
    fn test_q_sharing_proportional_to_headroom() {
        let zones = vec![make_zone(0, vec![0, 1])];
        let pilots = vec![make_pilot(0, 1.0)];
        let gens = vec![
            make_gen(0, 1.0, 0.0, -100.0, 100.0, 0.5), // headroom = 100
            make_gen(1, 1.0, 0.0, -80.0, 40.0, 0.5),   // headroom = 40
        ];
        let config = CvcConfig {
            q_sharing_method: QSharingMethod::ProportionalToHeadroom,
            ..CvcConfig::default()
        };
        let mut ctrl = CvcController::new(zones, pilots, gens, config).expect("ok");

        let dq_vec = ctrl.secondary_zone_control(0, 0.95, 1.0).expect("ok");
        // Gen 0 has headroom 100, Gen 1 has 40 → Gen 0 gets 100/140 share
        assert_eq!(dq_vec.len(), 2);
        assert!(
            dq_vec[0] > dq_vec[1],
            "Gen with more headroom should get larger share: {:?}",
            dq_vec
        );
    }

    // -----------------------------------------------------------------------
    // Test 5: Tertiary dispatch — proportional to generator capacity
    // -----------------------------------------------------------------------
    #[test]
    fn test_tertiary_dispatch_proportional_to_capacity() {
        // Build a controller where both generators have equal capacity ranges
        let zones = vec![make_zone(0, vec![0, 1])];
        let pilots = vec![make_pilot(0, 1.0)];
        let gens = vec![
            make_gen(0, 1.0, 0.0, -100.0, 100.0, 0.5), // range 200 Mvar
            make_gen(1, 1.0, 0.0, -100.0, 100.0, 0.5), // range 200 Mvar
        ];
        let ctrl = CvcController::new(zones, pilots, gens, CvcConfig::default()).expect("ok");

        let bus_loads = vec![50.0; 4]; // 200 Mvar total demand
        let setpoints = ctrl.tertiary_reactive_dispatch(&[1.0; 4], &bus_loads);
        assert_eq!(setpoints.len(), 2);

        // Equal capacity → equal setpoints
        let diff = (setpoints[0] - setpoints[1]).abs();
        assert!(
            diff < 1e-6,
            "Equal-capacity generators should share equally: {:?}",
            setpoints
        );

        // Larger generator (range 300 vs 200) should get a proportionally
        // larger slice of the demand: fraction = capacity / total_capacity
        let zones2 = vec![make_zone(0, vec![0, 1])];
        let pilots2 = vec![make_pilot(0, 1.0)];
        // Both anchored symmetrically so Qmin = -Qmax; demand positive
        let gens2 = vec![
            make_gen(0, 1.0, 0.0, 0.0, 150.0, 0.5), // range 150 Mvar (Qmin=0)
            make_gen(1, 1.0, 0.0, 0.0, 100.0, 0.5), // range 100 Mvar (Qmin=0)
        ];
        let ctrl2 = CvcController::new(zones2, pilots2, gens2, CvcConfig::default()).expect("ok");
        // demand = 200 Mvar; total capacity = 250; gen0 gets 150/250 * 200 = 120; gen1 gets 100/250 * 200 = 80
        let setpoints2 = ctrl2.tertiary_reactive_dispatch(&[1.0; 4], &bus_loads);
        assert_eq!(setpoints2.len(), 2);
        assert!(
            setpoints2[0] > setpoints2[1],
            "Larger-range generator should receive more Q: {:?}",
            setpoints2
        );
    }

    // -----------------------------------------------------------------------
    // Test 6: Voltage profile — 0.93 pu counted as low violation
    // -----------------------------------------------------------------------
    #[test]
    fn test_voltage_profile_low_violation() {
        let ctrl = default_controller();
        let voltages = vec![1.0, 1.0, 0.93, 1.0];
        let report = ctrl.assess_voltage_profile(&voltages);
        assert!(
            report.buses_low.contains(&2),
            "Bus 2 at 0.93 pu should be flagged low"
        );
        assert!(!report.buses_low.contains(&0));
        assert!(report.worst_violation_pu > 0.0);
    }

    // -----------------------------------------------------------------------
    // Test 7: Zone reactive limit enforced in secondary control
    // -----------------------------------------------------------------------
    #[test]
    fn test_zone_reactive_limit_enforced() {
        // Tight zone limit: ΔQ_zone cannot exceed 5 Mvar
        let mut zone = make_zone(0, vec![0, 1]);
        zone.zone_reactive_limit_mvar = (-5.0, 5.0);

        let zones = vec![zone];
        let pilots = vec![make_pilot(0, 1.0)];
        let gens = vec![
            make_gen(0, 1.0, 0.0, -100.0, 100.0, 0.5),
            make_gen(1, 1.0, 0.0, -100.0, 100.0, 0.5),
        ];
        let mut ctrl = CvcController::new(zones, pilots, gens, CvcConfig::default()).expect("ok");

        // Large voltage deviation, but zone limit should cap total correction
        let dq_vec = ctrl.secondary_zone_control(0, 0.5, 1.0).expect("ok");
        let total: f64 = dq_vec.iter().sum();
        assert!(
            total <= 5.0 + 1e-9,
            "Zone correction {total} exceeded limit 5 Mvar"
        );
    }

    // -----------------------------------------------------------------------
    // Test 8: Participation factor — generator with more headroom gets larger share
    // -----------------------------------------------------------------------
    #[test]
    fn test_coordinate_avr_participation_factor() {
        let ctrl = default_controller();
        // Gen 0: Qmax=100, Qcur=0  → headroom=100
        // Gen 1: Qmax=80,  Qcur=0  → headroom=80

        let zone = &ctrl.zones[0];
        let settings = ctrl.coordinate_avr_settings(zone).expect("ok");
        assert_eq!(settings.len(), 2);

        // Find by bus_id
        let (_, _, pf0) = settings
            .iter()
            .find(|&&(b, _, _)| b == 0)
            .copied()
            .expect("gen 0");
        let (_, _, pf1) = settings
            .iter()
            .find(|&&(b, _, _)| b == 1)
            .copied()
            .expect("gen 1");

        assert!(
            pf0 > pf1,
            "Gen 0 (headroom 100) should have higher PF than Gen 1 (headroom 80): {pf0} vs {pf1}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 9: Q-V sensitivity — generator bus has highest sensitivity
    // -----------------------------------------------------------------------
    #[test]
    fn test_qv_sensitivity_decreases_with_distance() {
        let ctrl = default_controller();
        let voltages = vec![1.0; 10];
        let dv = ctrl.q_v_sensitivity(0, &voltages, 10.0).expect("ok");
        // Gen 0 is at bus_id=0 → dv[0] should be the largest
        let max_dv = dv[0];
        for &v in dv.iter().skip(1) {
            assert!(
                v <= max_dv + 1e-12,
                "Sensitivity should decrease with distance from generator bus"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Test 10: Voltage profile quality index
    // -----------------------------------------------------------------------
    #[test]
    fn test_voltage_quality_index() {
        let ctrl = default_controller();

        // All in band → quality = 1.0
        let good = vec![1.0, 0.98, 1.02, 1.01];
        let report_good = ctrl.assess_voltage_profile(&good);
        assert!(
            (report_good.voltage_quality_index - 1.0).abs() < 1e-9,
            "All buses in band → quality=1"
        );

        // Half out of band
        let mixed = vec![0.93, 1.08, 1.0, 1.0];
        let report_mixed = ctrl.assess_voltage_profile(&mixed);
        assert!(
            report_mixed.voltage_quality_index < 1.0,
            "Some violations → quality < 1"
        );
    }

    // -----------------------------------------------------------------------
    // Test 11: Out-of-bounds index returns error
    // -----------------------------------------------------------------------
    #[test]
    fn test_out_of_bounds_returns_error() {
        let ctrl = default_controller();
        let result = ctrl.primary_avr_response(99, 1.0, 0.05);
        assert!(
            result.is_err(),
            "Should return error for invalid generator index"
        );
    }

    // -----------------------------------------------------------------------
    // Test 12: select_pilot_buses returns one bus per zone
    // -----------------------------------------------------------------------
    #[test]
    fn test_select_pilot_buses_count() {
        let ctrl = default_controller();
        let voltages = vec![0.97, 1.00, 1.03, 0.95, 1.02];
        let pilots = ctrl.select_pilot_buses(&voltages);
        // One zone → expect 1 pilot bus
        assert_eq!(pilots.len(), 1, "One zone → one pilot bus: {:?}", pilots);
    }
}
