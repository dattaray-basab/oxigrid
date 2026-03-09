/// Grid integration analysis for renewable energy sources.
///
/// Covers:
/// - Penetration analysis: instantaneous and annual RE fraction
/// - Hosting capacity: maximum RE that can be added without violations
/// - Ramping requirement: net-load ramp rates, flexibility adequacy
/// - Curtailment analysis: energy curtailed vs. penetration level
/// - Capacity factor and utilisation statistics
///
/// # References
/// - EPRI, "Hosting Capacity Analysis for Distributed Energy Resources", 2016
/// - NERC, "Integration of Variable Generation", 2012
/// - IEA, "Status of Power System Transformation", 2019
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Core types
// ────────────────────────────────────────────────────────────────────────────

/// Snapshot of the power system state for integration analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridSnapshot {
    /// Total load demand [MW]
    pub load_mw: f64,
    /// Total renewable generation [MW]
    pub renewable_mw: f64,
    /// Total conventional generation [MW]
    pub conventional_mw: f64,
    /// Curtailed renewable energy [MW]
    pub curtailment_mw: f64,
    /// Timestamp offset [hours from start]
    pub time_h: f64,
}

impl GridSnapshot {
    /// Instantaneous penetration as fraction [0, 1].
    pub fn penetration(&self) -> f64 {
        if self.load_mw < 1e-6 {
            return 0.0;
        }
        (self.renewable_mw / self.load_mw).clamp(0.0, 1.0)
    }

    /// Net load [MW] = load - renewable (without curtailment).
    pub fn net_load_mw(&self) -> f64 {
        (self.load_mw - self.renewable_mw).max(0.0)
    }
}

/// Configuration for penetration analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PenetrationConfig {
    /// Minimum stable load fraction for conventional units (e.g. 0.05 = 5% of peak)
    pub min_conventional_fraction: f64,
    /// Maximum instantaneous penetration allowed [fraction 0–1]
    pub max_penetration: f64,
    /// Minimum load [MW] below which system is considered lightly loaded
    pub min_load_mw: f64,
}

impl Default for PenetrationConfig {
    fn default() -> Self {
        Self {
            min_conventional_fraction: 0.05,
            max_penetration: 0.80,
            min_load_mw: 50.0,
        }
    }
}

/// Results from penetration analysis over a time series.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PenetrationResult {
    /// Annual energy penetration = ΣE_re / ΣE_load [fraction]
    pub annual_energy_penetration: f64,
    /// Instantaneous penetration statistics
    pub max_instantaneous: f64,
    pub mean_instantaneous: f64,
    pub p95_instantaneous: f64,
    /// Number of hours with penetration > max_penetration threshold
    pub hours_above_limit: f64,
    /// Curtailment energy [MWh]
    pub total_curtailment_mwh: f64,
    /// Curtailment ratio = E_curtailed / E_available
    pub curtailment_ratio: f64,
    /// Capacity factor = E_generated / (P_installed * T)
    pub capacity_factor: f64,
}

/// Analyse penetration over a time series of snapshots.
pub fn penetration_analysis(
    snapshots: &[GridSnapshot],
    installed_capacity_mw: f64,
    config: &PenetrationConfig,
) -> PenetrationResult {
    if snapshots.is_empty() {
        return PenetrationResult {
            annual_energy_penetration: 0.0,
            max_instantaneous: 0.0,
            mean_instantaneous: 0.0,
            p95_instantaneous: 0.0,
            hours_above_limit: 0.0,
            total_curtailment_mwh: 0.0,
            curtailment_ratio: 0.0,
            capacity_factor: 0.0,
        };
    }

    let n = snapshots.len() as f64;
    let dt_h = if snapshots.len() > 1 {
        (snapshots.last().unwrap().time_h - snapshots[0].time_h) / (snapshots.len() - 1) as f64
    } else {
        1.0
    };

    let total_load_mwh: f64 = snapshots.iter().map(|s| s.load_mw * dt_h).sum();
    let total_re_mwh: f64 = snapshots.iter().map(|s| s.renewable_mw * dt_h).sum();
    let total_curtail_mwh: f64 = snapshots.iter().map(|s| s.curtailment_mw * dt_h).sum();
    let total_avail_mwh = total_re_mwh + total_curtail_mwh;

    let mut penetrations: Vec<f64> = snapshots.iter().map(|s| s.penetration()).collect();
    let mean_p = penetrations.iter().sum::<f64>() / n;
    let max_p = penetrations.iter().cloned().fold(0.0_f64, f64::max);
    penetrations.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p95_idx = ((0.95 * n) as usize).min(snapshots.len() - 1);
    let p95_p = penetrations[p95_idx];

    let hours_above: f64 = snapshots
        .iter()
        .filter(|s| s.penetration() > config.max_penetration)
        .count() as f64
        * dt_h;

    let total_hours = dt_h * snapshots.len() as f64;
    let capacity_factor = if installed_capacity_mw > 1e-6 {
        total_re_mwh / (installed_capacity_mw * total_hours)
    } else {
        0.0
    };

    PenetrationResult {
        annual_energy_penetration: if total_load_mwh > 1e-6 {
            total_re_mwh / total_load_mwh
        } else {
            0.0
        },
        max_instantaneous: max_p,
        mean_instantaneous: mean_p,
        p95_instantaneous: p95_p,
        hours_above_limit: hours_above,
        total_curtailment_mwh: total_curtail_mwh,
        curtailment_ratio: if total_avail_mwh > 1e-6 {
            total_curtail_mwh / total_avail_mwh
        } else {
            0.0
        },
        capacity_factor: capacity_factor.clamp(0.0, 1.0),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Hosting capacity
// ────────────────────────────────────────────────────────────────────────────

/// Constraints that limit hosting capacity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HostingCapacityConstraints {
    /// Thermal limit of the feeder/branch [MW]
    pub thermal_limit_mw: f64,
    /// Voltage rise limit (ΔV/V_ref) allowed due to RE injection [p.u.]
    pub voltage_rise_limit_pu: f64,
    /// Minimum load that must always be served by convention [MW]
    pub min_load_mw: f64,
    /// Feeder impedance (simplified R+jX magnitude) [Ω or p.u.]
    pub feeder_impedance_pu: f64,
    /// Short-circuit ratio at PCC (grid stiffness)
    pub short_circuit_ratio: f64,
}

impl Default for HostingCapacityConstraints {
    fn default() -> Self {
        Self {
            thermal_limit_mw: 20.0,
            voltage_rise_limit_pu: 0.05,
            min_load_mw: 2.0,
            feeder_impedance_pu: 0.05,
            short_circuit_ratio: 10.0,
        }
    }
}

/// Result of hosting capacity analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HostingCapacityResult {
    /// Maximum DER capacity before thermal violation [MW]
    pub thermal_limit_mw: f64,
    /// Maximum DER capacity before voltage violation [MW]
    pub voltage_limit_mw: f64,
    /// Maximum DER capacity before stability margin is reduced [MW]
    pub stability_limit_mw: f64,
    /// Overall hosting capacity = min of all limits [MW]
    pub hosting_capacity_mw: f64,
    /// Binding constraint name
    pub binding_constraint: String,
    /// Utilisation of hosting capacity with current DER [fraction]
    pub current_utilisation: f64,
}

/// Estimate hosting capacity using simplified analytical methods.
///
/// Thermal limit: I_RE ≤ I_thermal → P_RE ≤ S_thermal (thermal rating)
/// Voltage rise:  ΔV ≈ P·R/(V²) → P_RE ≤ ΔV_max · V² / R
/// Stability: SCR-based limit → P_RE ≤ P_sc / k_scr
pub fn estimate_hosting_capacity(
    constraints: &HostingCapacityConstraints,
    current_der_mw: f64,
    base_voltage_kv: f64,
) -> HostingCapacityResult {
    // Thermal limit (direct)
    let thermal = constraints.thermal_limit_mw;

    // Voltage rise limit: ΔV ≈ P·X/(V_base²) for resistive feeder
    // P_max = ΔV_limit * V_base² / Z_feeder
    let v_base_sq = base_voltage_kv * base_voltage_kv; // kV² (cancel with impedance units)
    let voltage = if constraints.feeder_impedance_pu > 1e-6 {
        constraints.voltage_rise_limit_pu * v_base_sq / constraints.feeder_impedance_pu
    } else {
        thermal
    };

    // Stability limit: P_RE ≤ P_load / SCR_min (simplified)
    // High SCR → stiff grid → more hosting capacity
    let stability = constraints.short_circuit_ratio * constraints.min_load_mw * 2.0;

    let hosting_capacity_mw = thermal.min(voltage).min(stability);
    let binding_constraint = if thermal <= voltage && thermal <= stability {
        "thermal".to_string()
    } else if voltage <= stability {
        "voltage_rise".to_string()
    } else {
        "stability".to_string()
    };

    HostingCapacityResult {
        thermal_limit_mw: thermal,
        voltage_limit_mw: voltage,
        stability_limit_mw: stability,
        hosting_capacity_mw,
        binding_constraint,
        current_utilisation: if hosting_capacity_mw > 1e-6 {
            (current_der_mw / hosting_capacity_mw).clamp(0.0, 1.0)
        } else {
            1.0
        },
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Ramping requirement analysis
// ────────────────────────────────────────────────────────────────────────────

/// Net-load ramp statistics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RampingRequirement {
    /// Maximum upward ramp rate observed [MW/h]
    pub max_up_ramp_mwh: f64,
    /// Maximum downward ramp rate observed [MW/h]
    pub max_down_ramp_mwh: f64,
    /// 95th percentile upward ramp [MW/h]
    pub p95_up_ramp_mwh: f64,
    /// 95th percentile downward ramp [MW/h]
    pub p95_down_ramp_mwh: f64,
    /// Mean absolute ramp rate [MW/h]
    pub mean_abs_ramp_mwh: f64,
    /// Required flexible capacity for 3-hour ramp event [MW]
    pub three_hour_ramp_mw: f64,
    /// Fraction of periods where ramp exceeds conventional flexibility
    pub flexibility_shortage_fraction: f64,
}

/// Compute net-load ramping requirements from a series of snapshots.
///
/// Net load = load - renewable (uncontrolled).
/// Ramp at time t = net_load(t) - net_load(t-1) per hour.
pub fn compute_ramping_requirements(
    snapshots: &[GridSnapshot],
    conventional_ramp_mw_per_h: f64,
) -> RampingRequirement {
    if snapshots.len() < 2 {
        return RampingRequirement {
            max_up_ramp_mwh: 0.0,
            max_down_ramp_mwh: 0.0,
            p95_up_ramp_mwh: 0.0,
            p95_down_ramp_mwh: 0.0,
            mean_abs_ramp_mwh: 0.0,
            three_hour_ramp_mw: 0.0,
            flexibility_shortage_fraction: 0.0,
        };
    }

    let net_loads: Vec<f64> = snapshots.iter().map(|s| s.net_load_mw()).collect();
    let dt_h = if snapshots.len() > 1 {
        let dt = snapshots[1].time_h - snapshots[0].time_h;
        if dt.abs() < 1e-9 {
            1.0
        } else {
            dt
        }
    } else {
        1.0
    };

    let ramps: Vec<f64> = net_loads.windows(2).map(|w| (w[1] - w[0]) / dt_h).collect();

    let up_ramps: Vec<f64> = ramps.iter().cloned().filter(|&r| r > 0.0).collect();
    let down_ramps: Vec<f64> = ramps
        .iter()
        .cloned()
        .filter(|&r| r < 0.0)
        .map(|r| -r)
        .collect();

    let max_up = up_ramps.iter().cloned().fold(0.0_f64, f64::max);
    let max_down = down_ramps.iter().cloned().fold(0.0_f64, f64::max);

    let p95_up = sorted_percentile(&mut up_ramps.clone(), 0.95);
    let p95_down = sorted_percentile(&mut down_ramps.clone(), 0.95);

    let mean_abs = ramps.iter().map(|r| r.abs()).sum::<f64>() / ramps.len() as f64;

    // 3-hour ramp: max net-load change over any 3-hour window
    let steps_3h = (3.0 / dt_h).round() as usize;
    let three_hour = if steps_3h > 0 && net_loads.len() > steps_3h {
        net_loads
            .windows(steps_3h + 1)
            .map(|w| (w[w.len() - 1] - w[0]).abs())
            .fold(0.0_f64, f64::max)
    } else {
        max_up.max(max_down) * 3.0
    };

    let shortage: usize = ramps
        .iter()
        .filter(|&&r| r.abs() > conventional_ramp_mw_per_h)
        .count();
    let shortage_frac = shortage as f64 / ramps.len() as f64;

    RampingRequirement {
        max_up_ramp_mwh: max_up,
        max_down_ramp_mwh: max_down,
        p95_up_ramp_mwh: p95_up,
        p95_down_ramp_mwh: p95_down,
        mean_abs_ramp_mwh: mean_abs,
        three_hour_ramp_mw: three_hour,
        flexibility_shortage_fraction: shortage_frac,
    }
}

fn sorted_percentile(data: &mut [f64], p: f64) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let idx = ((p * data.len() as f64) as usize).min(data.len() - 1);
    data[idx]
}

// ────────────────────────────────────────────────────────────────────────────
// Curtailment vs. penetration curve
// ────────────────────────────────────────────────────────────────────────────

/// A point on the curtailment-penetration curve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CurtailmentPoint {
    /// Installed RE capacity as fraction of peak load
    pub re_fraction: f64,
    /// Energy curtailment rate [fraction of available RE]
    pub curtailment_rate: f64,
    /// Capacity factor at this penetration
    pub capacity_factor: f64,
    /// Penetration (annual energy fraction)
    pub energy_penetration: f64,
}

/// Generate curtailment vs. penetration curve.
///
/// Scales a base renewable profile by different capacity factors and
/// computes curtailment for each level given a minimum load constraint.
pub fn curtailment_curve(
    load_profile_mw: &[f64],
    re_profile_pu: &[f64],
    re_fractions: &[f64],
    min_load_constraint_fraction: f64,
) -> Vec<CurtailmentPoint> {
    let n = load_profile_mw.len().min(re_profile_pu.len());
    if n == 0 {
        return vec![];
    }

    let peak_load = load_profile_mw.iter().cloned().fold(0.0_f64, f64::max);
    let total_load_mwh: f64 = load_profile_mw.iter().sum();

    re_fractions
        .iter()
        .map(|&frac| {
            let installed_mw = frac * peak_load;
            let mut total_re = 0.0_f64;
            let mut total_curtail = 0.0_f64;

            for i in 0..n {
                let load = load_profile_mw[i];
                let re_avail = re_profile_pu[i] * installed_mw;
                // Cannot have net load < min_load_constraint
                let min_conv = min_load_constraint_fraction * peak_load;
                let max_re_abs = (load - min_conv).max(0.0);
                let re_actual = re_avail.min(max_re_abs);
                total_re += re_actual;
                total_curtail += (re_avail - re_actual).max(0.0);
            }

            let total_avail = total_re + total_curtail;
            let curtailment_rate = if total_avail > 1e-6 {
                total_curtail / total_avail
            } else {
                0.0
            };
            let capacity_factor = if installed_mw > 1e-6 {
                total_re / (installed_mw * n as f64)
            } else {
                0.0
            };
            let energy_penetration = if total_load_mwh > 1e-6 {
                total_re / total_load_mwh
            } else {
                0.0
            };

            CurtailmentPoint {
                re_fraction: frac,
                curtailment_rate,
                capacity_factor,
                energy_penetration,
            }
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Flexibility adequacy
// ────────────────────────────────────────────────────────────────────────────

/// Flexible resource inventory for adequacy assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityResource {
    pub name: String,
    /// Maximum upward ramp [MW/h]
    pub ramp_up_mwh: f64,
    /// Maximum downward ramp [MW/h]
    pub ramp_down_mwh: f64,
    /// Available flexible capacity [MW]
    pub capacity_mw: f64,
}

impl FlexibilityResource {
    pub fn pumped_hydro(capacity_mw: f64) -> Self {
        Self {
            name: "pumped_hydro".into(),
            ramp_up_mwh: capacity_mw * 4.0,
            ramp_down_mwh: capacity_mw * 4.0,
            capacity_mw,
        }
    }
    pub fn gas_peaker(capacity_mw: f64) -> Self {
        Self {
            name: "gas_peaker".into(),
            ramp_up_mwh: capacity_mw * 10.0,
            ramp_down_mwh: capacity_mw * 6.0,
            capacity_mw,
        }
    }
    pub fn battery_storage(capacity_mw: f64) -> Self {
        Self {
            name: "battery".into(),
            ramp_up_mwh: capacity_mw * 60.0,
            ramp_down_mwh: capacity_mw * 60.0,
            capacity_mw,
        }
    }
    pub fn demand_response(capacity_mw: f64) -> Self {
        Self {
            name: "demand_response".into(),
            ramp_up_mwh: capacity_mw * 2.0,
            ramp_down_mwh: capacity_mw * 1.0,
            capacity_mw,
        }
    }
}

/// Flexibility adequacy assessment result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityAdequacy {
    /// Total upward flexibility [MW/h]
    pub total_up_ramp_mwh: f64,
    /// Total downward flexibility [MW/h]
    pub total_down_ramp_mwh: f64,
    /// Required upward ramp (from ramping analysis)
    pub required_up_ramp_mwh: f64,
    /// Required downward ramp
    pub required_down_ramp_mwh: f64,
    /// Upward adequacy margin [MW/h] (positive = surplus)
    pub up_adequacy_margin_mwh: f64,
    /// Downward adequacy margin [MW/h]
    pub down_adequacy_margin_mwh: f64,
    /// Is the system adequate for upward ramping?
    pub upward_adequate: bool,
    /// Is the system adequate for downward ramping?
    pub downward_adequate: bool,
}

/// Assess flexibility adequacy given available resources and ramping requirements.
pub fn assess_flexibility_adequacy(
    resources: &[FlexibilityResource],
    ramp_req: &RampingRequirement,
) -> FlexibilityAdequacy {
    let total_up: f64 = resources.iter().map(|r| r.ramp_up_mwh).sum();
    let total_down: f64 = resources.iter().map(|r| r.ramp_down_mwh).sum();

    let req_up = ramp_req.p95_up_ramp_mwh;
    let req_down = ramp_req.p95_down_ramp_mwh;

    FlexibilityAdequacy {
        total_up_ramp_mwh: total_up,
        total_down_ramp_mwh: total_down,
        required_up_ramp_mwh: req_up,
        required_down_ramp_mwh: req_down,
        up_adequacy_margin_mwh: total_up - req_up,
        down_adequacy_margin_mwh: total_down - req_down,
        upward_adequate: total_up >= req_up,
        downward_adequate: total_down >= req_down,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_snapshots(n: usize, load_mw: f64, re_fraction: f64) -> Vec<GridSnapshot> {
        (0..n)
            .map(|i| {
                let t = i as f64;
                let re =
                    load_mw * re_fraction * (0.5 + 0.5 * (t * std::f64::consts::TAU / 24.0).sin());
                GridSnapshot {
                    load_mw,
                    renewable_mw: re.min(load_mw),
                    conventional_mw: (load_mw - re).max(0.0),
                    curtailment_mw: 0.0,
                    time_h: t,
                }
            })
            .collect()
    }

    #[test]
    fn test_grid_snapshot_penetration() {
        let s = GridSnapshot {
            load_mw: 100.0,
            renewable_mw: 30.0,
            conventional_mw: 70.0,
            curtailment_mw: 0.0,
            time_h: 0.0,
        };
        assert!((s.penetration() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn test_snapshot_net_load() {
        let s = GridSnapshot {
            load_mw: 100.0,
            renewable_mw: 40.0,
            conventional_mw: 60.0,
            curtailment_mw: 0.0,
            time_h: 0.0,
        };
        assert!((s.net_load_mw() - 60.0).abs() < 1e-9);
    }

    #[test]
    fn test_penetration_zero_load() {
        let s = GridSnapshot {
            load_mw: 0.0,
            renewable_mw: 0.0,
            conventional_mw: 0.0,
            curtailment_mw: 0.0,
            time_h: 0.0,
        };
        assert_eq!(s.penetration(), 0.0);
    }

    #[test]
    fn test_penetration_analysis_basic() {
        let snapshots = make_snapshots(24, 100.0, 0.3);
        let cfg = PenetrationConfig::default();
        let result = penetration_analysis(&snapshots, 30.0, &cfg);
        assert!(result.annual_energy_penetration > 0.0);
        assert!(result.annual_energy_penetration <= 1.0);
        assert!(result.max_instantaneous <= 1.0);
        assert!(result.capacity_factor <= 1.0);
    }

    #[test]
    fn test_penetration_analysis_empty() {
        let result = penetration_analysis(&[], 100.0, &PenetrationConfig::default());
        assert_eq!(result.annual_energy_penetration, 0.0);
    }

    #[test]
    fn test_penetration_analysis_high_re() {
        let mut snapshots = make_snapshots(48, 100.0, 0.9);
        // Add curtailment at peak
        snapshots[12].curtailment_mw = 20.0;
        let cfg = PenetrationConfig::default();
        let result = penetration_analysis(&snapshots, 90.0, &cfg);
        // make_snapshots uses sinusoidal profile: avg RE = load * fraction * 0.5 ≈ 45%
        assert!(
            result.annual_energy_penetration > 0.3,
            "High RE should give high penetration"
        );
    }

    #[test]
    fn test_hosting_capacity_thermal_binding() {
        let constraints = HostingCapacityConstraints {
            thermal_limit_mw: 5.0,
            voltage_rise_limit_pu: 0.10,
            min_load_mw: 2.0,
            feeder_impedance_pu: 0.01, // low impedance → high voltage limit
            short_circuit_ratio: 20.0,
        };
        let result = estimate_hosting_capacity(&constraints, 2.0, 11.0);
        assert_eq!(result.binding_constraint, "thermal");
        assert!((result.hosting_capacity_mw - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_hosting_capacity_current_utilisation() {
        let constraints = HostingCapacityConstraints::default();
        let result = estimate_hosting_capacity(&constraints, 10.0, 11.0);
        assert!(result.current_utilisation > 0.0);
        assert!(result.current_utilisation <= 1.0);
    }

    #[test]
    fn test_ramping_requirements_basic() {
        let snapshots = make_snapshots(48, 100.0, 0.4);
        let ramp = compute_ramping_requirements(&snapshots, 50.0);
        assert!(ramp.max_up_ramp_mwh >= 0.0);
        assert!(ramp.max_down_ramp_mwh >= 0.0);
        assert!(ramp.mean_abs_ramp_mwh >= 0.0);
        assert!(
            ramp.flexibility_shortage_fraction >= 0.0 && ramp.flexibility_shortage_fraction <= 1.0
        );
    }

    #[test]
    fn test_ramping_requirements_single_snapshot() {
        let snapshots = make_snapshots(1, 100.0, 0.3);
        let ramp = compute_ramping_requirements(&snapshots, 50.0);
        assert_eq!(ramp.max_up_ramp_mwh, 0.0);
    }

    #[test]
    fn test_ramping_three_hour_event() {
        // Create a sharp 3-hour ramp up (duck curve style)
        let mut snapshots: Vec<GridSnapshot> = (0..12)
            .map(|i| GridSnapshot {
                load_mw: 100.0,
                renewable_mw: if i < 6 { 60.0 } else { 5.0 },
                conventional_mw: if i < 6 { 40.0 } else { 95.0 },
                curtailment_mw: 0.0,
                time_h: i as f64 * 0.5,
            })
            .collect();
        // Set last snapshot time for proper dt computation
        for (i, s) in snapshots.iter_mut().enumerate() {
            s.time_h = i as f64 * 0.5;
        }
        let ramp = compute_ramping_requirements(&snapshots, 100.0);
        assert!(ramp.max_up_ramp_mwh > 0.0, "Should detect ramp-up event");
    }

    #[test]
    fn test_curtailment_curve_low_penetration() {
        let load = vec![100.0_f64; 24];
        let re_pu = vec![0.5_f64; 24];
        let fractions = vec![0.1, 0.3, 0.5, 0.8, 1.0];
        // Use high min_load constraint (60%) so that RE above 40 MW gets curtailed
        let curve = curtailment_curve(&load, &re_pu, &fractions, 0.60);
        assert_eq!(curve.len(), 5);
        // At low penetration (frac=0.1: 5 MW), no curtailment; at high (frac=1.0: 50 MW > 40 MW limit), curtailment occurs
        assert!(
            curve[0].curtailment_rate < curve[4].curtailment_rate + 1e-9,
            "Higher RE should cause more curtailment: {:.4} vs {:.4}",
            curve[0].curtailment_rate,
            curve[4].curtailment_rate
        );
    }

    #[test]
    fn test_curtailment_curve_energy_penetration_increases() {
        let load = vec![100.0_f64; 24];
        let re_pu = vec![0.7_f64; 24];
        let fractions = vec![0.2, 0.5, 1.0];
        let curve = curtailment_curve(&load, &re_pu, &fractions, 0.05);
        assert!(
            curve[0].energy_penetration <= curve[2].energy_penetration,
            "Energy penetration should not decrease with more RE"
        );
    }

    #[test]
    fn test_flexibility_resource_presets() {
        let ph = FlexibilityResource::pumped_hydro(100.0);
        assert!(ph.ramp_up_mwh > 0.0);
        let bat = FlexibilityResource::battery_storage(50.0);
        assert!(bat.ramp_up_mwh > ph.ramp_up_mwh / 2.0); // battery ramps faster
        let dr = FlexibilityResource::demand_response(30.0);
        assert!(dr.capacity_mw == 30.0);
    }

    #[test]
    fn test_flexibility_adequacy_sufficient() {
        let resources = vec![
            FlexibilityResource::gas_peaker(200.0),
            FlexibilityResource::battery_storage(50.0),
        ];
        let ramp_req = RampingRequirement {
            max_up_ramp_mwh: 300.0,
            max_down_ramp_mwh: 200.0,
            p95_up_ramp_mwh: 150.0,
            p95_down_ramp_mwh: 100.0,
            mean_abs_ramp_mwh: 80.0,
            three_hour_ramp_mw: 200.0,
            flexibility_shortage_fraction: 0.05,
        };
        let adeq = assess_flexibility_adequacy(&resources, &ramp_req);
        assert!(
            adeq.upward_adequate,
            "Should be adequate with gas peaker + battery"
        );
        assert!(adeq.downward_adequate);
    }

    #[test]
    fn test_flexibility_adequacy_insufficient() {
        let resources = vec![FlexibilityResource::demand_response(10.0)];
        let ramp_req = RampingRequirement {
            max_up_ramp_mwh: 500.0,
            max_down_ramp_mwh: 400.0,
            p95_up_ramp_mwh: 300.0,
            p95_down_ramp_mwh: 250.0,
            mean_abs_ramp_mwh: 100.0,
            three_hour_ramp_mw: 400.0,
            flexibility_shortage_fraction: 0.3,
        };
        let adeq = assess_flexibility_adequacy(&resources, &ramp_req);
        assert!(!adeq.upward_adequate, "DR alone insufficient for 300 MW/h");
    }
}
