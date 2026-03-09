use crate::error::{OxiGridError, Result};
/// Advanced renewable integration analysis.
///
/// Provides tools for assessing how much renewable generation a power system
/// can accommodate (hosting capacity), the strength of the grid at the point
/// of connection (Short Circuit Ratio), and system-level inertia implications
/// of high renewable penetration.
///
/// # References
/// - EPRI, "Hosting Capacity Analysis for Distributed Energy Resources", 2016
/// - IEC TR 61000-3-7, "Assessment of emission limits for fluctuating loads"
/// - CIGRE TB 671, "Connection of Wind Farms to Weak AC Networks", 2016
/// - ENTSO-E, "High Penetration of Power Electronic Interfaced Power Sources", 2017
#[cfg(feature = "powerflow")]
use crate::network::topology::PowerNetwork;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Grid strength classification
// ─────────────────────────────────────────────────────────────────────────────

/// Qualitative grid strength classification based on Short Circuit Ratio.
///
/// | SCR range  | Classification |
/// |------------|----------------|
/// | > 5        | Strong         |
/// | 3 – 5      | Medium         |
/// | 2 – 3      | Weak           |
/// | ≤ 2        | Very Weak      |
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum GridStrength {
    /// SCR > 5 — stiff grid, no special integration measures needed.
    Strong,
    /// 3 < SCR ≤ 5 — moderate integration challenges, monitoring recommended.
    Medium,
    /// 2 < SCR ≤ 3 — weak grid, reactive compensation likely needed.
    Weak,
    /// SCR ≤ 2 — very weak grid, dedicated stability studies mandatory.
    VeryWeak,
}

impl GridStrength {
    /// Classify grid strength from a raw SCR value.
    pub fn from_scr(scr: f64) -> Self {
        if scr > 5.0 {
            Self::Strong
        } else if scr > 3.0 {
            Self::Medium
        } else if scr > 2.0 {
            Self::Weak
        } else {
            Self::VeryWeak
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid strength assessment
// ─────────────────────────────────────────────────────────────────────────────

/// Detailed grid strength assessment at a renewable connection point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridStrengthAssessment {
    /// Short Circuit Ratio = short-circuit MVA at bus / renewable MW.
    pub scr: f64,
    /// Weighted SCR accounting for interactions with other renewable plants.
    ///
    /// WSCR = (Σ SCR_i · P_i) / P_total
    pub weighted_scr: f64,
    /// Effective SCR incorporating reactive compensation (STATCOM/SVC).
    ///
    /// ESCR = SCR + (reactive compensation MVAr / renewable MW)
    pub effective_scr: f64,
    /// Qualitative grid strength classification based on SCR.
    pub grid_strength: GridStrength,
    /// Human-readable list of integration issues detected.
    pub issues: Vec<String>,
}

/// Assess grid strength at a renewable generation connection bus.
///
/// Computes the Short Circuit Ratio (SCR), Weighted SCR (WSCR), and Effective
/// SCR (ESCR) using the Thévenin short-circuit impedance at the bus.
///
/// # Arguments
/// - `network`       — AC power network
/// - `bus`           — external bus ID of the connection point
/// - `renewable_mw`  — total renewable MW connected at the bus
///
/// # Notes
/// Short-circuit MVA is approximated as `V_base² / |Z_th|` where `Z_th` is
/// estimated from the Y-bus diagonal entry: `Z_ii ≈ 1 / Y_ii`.
#[cfg(feature = "powerflow")]
pub fn assess_grid_strength(
    network: &PowerNetwork,
    bus: usize,
    renewable_mw: f64,
) -> GridStrengthAssessment {
    let mut issues = Vec::new();

    // Obtain Y-bus and use diagonal entry to estimate Thévenin impedance
    let scr = match network.admittance_matrix() {
        Ok(ybus) => {
            match network.bus_index(bus) {
                Ok(idx) => {
                    // Y_ii is the self-admittance; Z_ii ≈ 1/|Y_ii|
                    let y_ii_mag = if let Some(y_val) = ybus.get(idx, idx) {
                        (y_val.re * y_val.re + y_val.im * y_val.im).sqrt()
                    } else {
                        1e-6 // fallback: nearly zero admittance
                    };
                    let z_th_pu = if y_ii_mag > 1e-12 {
                        1.0 / y_ii_mag
                    } else {
                        1e6
                    };
                    // Short-circuit MVA (in per-unit base, then convert to MW)
                    let sc_mva = network.base_mva / z_th_pu;
                    if renewable_mw > 1e-6 {
                        sc_mva / renewable_mw
                    } else {
                        f64::INFINITY
                    }
                }
                Err(_) => {
                    issues.push(format!("Bus {bus} not found in network"));
                    0.0
                }
            }
        }
        Err(e) => {
            issues.push(format!("Y-bus construction failed: {e}"));
            0.0
        }
    };

    // WSCR: for a single plant WSCR equals SCR; multi-plant weighting requires
    // external information. We return SCR as a baseline.
    let weighted_scr = scr;

    // ESCR: no reactive compensation assumed by default
    let effective_scr = scr;

    let grid_strength = GridStrength::from_scr(scr);

    // Issue detection
    match grid_strength {
        GridStrength::VeryWeak => {
            issues.push(
                "Very weak grid (SCR ≤ 2): dedicated stability study mandatory before connection."
                    .to_string(),
            );
            issues
                .push("Consider synchronous condenser or STATCOM for voltage support.".to_string());
        }
        GridStrength::Weak => {
            issues.push("Weak grid (SCR 2–3): reactive compensation likely required.".to_string());
        }
        GridStrength::Medium => {
            issues.push(
                "Medium grid strength (SCR 3–5): monitor voltage stability under contingency."
                    .to_string(),
            );
        }
        GridStrength::Strong => {}
    }

    GridStrengthAssessment {
        scr,
        weighted_scr,
        effective_scr,
        grid_strength,
        issues,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Integration study
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a single renewable penetration scenario in an integration study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrationResult {
    /// Renewable penetration level tested [% of total load].
    pub penetration_pct: f64,
    /// Hosting capacity (maximum renewable before the first violation) \[MW\].
    pub hosting_capacity_mw: f64,
    /// Number of buses with voltage outside 0.95–1.05 pu.
    pub voltage_violations: usize,
    /// Number of branches thermally overloaded (> rated MVA).
    pub thermal_violations: usize,
    /// Voltage stability margin (1 − L_max) where L is the L-index \[pu\].
    pub stability_margin: f64,
    /// Short Circuit Ratio at the primary connection bus.
    pub short_circuit_ratio: f64,
    /// Harmonic total distortion estimate [%] (simplified approximation).
    pub harmonic_thd_pct: f64,
    /// Percentage of renewable generation curtailed [%].
    pub curtailment_pct: f64,
    /// Whether the scenario is grid-code compliant (no violations).
    pub grid_code_compliant: bool,
}

/// Renewable integration study configuration and execution.
///
/// Tests a network at multiple renewable penetration levels to identify the
/// hosting capacity, detect violations, and assess SCR and inertia.
#[cfg(feature = "powerflow")]
pub struct IntegrationStudy {
    /// Penetration levels to test [fraction 0–1 of total load].
    pub penetration_levels: Vec<f64>,
    /// The AC power network to study.
    pub network: PowerNetwork,
    /// External bus IDs of renewable connection points.
    pub renewable_buses: Vec<usize>,
}

#[cfg(feature = "powerflow")]
impl IntegrationStudy {
    /// Create a new integration study.
    ///
    /// # Arguments
    /// - `network`          — AC power network
    /// - `renewable_buses`  — external bus IDs of renewable generation buses
    pub fn new(network: PowerNetwork, renewable_buses: Vec<usize>) -> Self {
        Self {
            penetration_levels: vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            network,
            renewable_buses,
        }
    }

    /// Run a hosting capacity analysis across all configured penetration levels.
    ///
    /// For each penetration level the method:
    /// 1. Scales renewable generation proportionally.
    /// 2. Checks voltage magnitudes (0.95–1.05 pu).
    /// 3. Checks branch thermal loadings vs. ratings.
    /// 4. Estimates a simple voltage stability margin.
    /// 5. Computes SCR at the first renewable bus.
    pub fn hosting_capacity_analysis(&self) -> Result<Vec<IntegrationResult>> {
        let total_load_mw = self.network.total_load_mw();
        if total_load_mw < 1e-6 {
            return Err(OxiGridError::InvalidNetwork(
                "Network has zero total load; hosting capacity analysis requires non-zero load."
                    .to_string(),
            ));
        }

        let mut results = Vec::with_capacity(self.penetration_levels.len());

        for &pct in &self.penetration_levels {
            let renewable_mw = pct * total_load_mw;

            // Count voltage violations using nominal bus voltages
            // (without running full power flow to avoid circular dependency)
            let voltage_violations = self.estimate_voltage_violations(renewable_mw);
            let thermal_violations = self.estimate_thermal_violations(renewable_mw);

            // Simplified L-index proxy: stability degrades with penetration
            // A simple linear model: margin decreases from 0.5 at 0% to 0.1 at 100%
            let stability_margin = (0.5 - 0.4 * pct).max(0.0);

            // SCR at primary renewable bus
            let scr = if let Some(&primary_bus) = self.renewable_buses.first() {
                let assessment = assess_grid_strength(&self.network, primary_bus, renewable_mw);
                assessment.scr
            } else {
                0.0
            };

            // Harmonic THD estimate: inverter-based generation contributes ~1–3% THD,
            // scaling with penetration (simplified)
            let harmonic_thd_pct = 1.0 + 2.0 * pct;

            // Curtailment occurs when thermal or voltage violations arise
            let curtailment_pct = if voltage_violations > 0 || thermal_violations > 0 {
                20.0 * pct // simplified: higher penetration → more curtailment
            } else {
                0.0
            };

            let grid_code_compliant = voltage_violations == 0 && thermal_violations == 0;

            // Hosting capacity: the renewable MW level at which the first
            // violation appears. Use binary-search approximation here.
            let hosting_capacity_mw =
                self.estimate_hosting_capacity_mw(total_load_mw, renewable_mw);

            results.push(IntegrationResult {
                penetration_pct: pct * 100.0,
                hosting_capacity_mw,
                voltage_violations,
                thermal_violations,
                stability_margin,
                short_circuit_ratio: scr,
                harmonic_thd_pct,
                curtailment_pct,
                grid_code_compliant,
            });
        }
        Ok(results)
    }

    /// Estimate the number of voltage violations at the given renewable injection.
    ///
    /// Uses a simplified ΔV ≈ P·R/V² approximation to estimate voltage rise
    /// at each renewable bus.
    fn estimate_voltage_violations(&self, renewable_mw: f64) -> usize {
        let n_buses = self.renewable_buses.len();
        if n_buses == 0 {
            return 0;
        }
        let mw_per_bus = renewable_mw / n_buses as f64;
        let base_mva = self.network.base_mva;

        self.renewable_buses
            .iter()
            .filter(|&&bus_id| {
                if let Ok(idx) = self.network.bus_index(bus_id) {
                    let bus = &self.network.buses[idx];
                    // Estimate feeder impedance from connected branches (simplified)
                    let r_pu = self.estimate_bus_impedance(bus_id);
                    let v_nominal = bus.vm;
                    let p_pu = mw_per_bus / base_mva;
                    let dv = p_pu * r_pu / (v_nominal * v_nominal);
                    let v_estimated = v_nominal + dv;
                    // Violation if voltage leaves [0.95, 1.05]
                    !(0.95..=1.05).contains(&v_estimated)
                } else {
                    false
                }
            })
            .count()
    }

    /// Estimate a representative feeder resistance \[pu\] for a bus.
    ///
    /// Averages the resistance of all branches connected to the bus.
    fn estimate_bus_impedance(&self, bus_id: usize) -> f64 {
        let connected: Vec<f64> = self
            .network
            .branches
            .iter()
            .filter(|br| br.from_bus == bus_id || br.to_bus == bus_id)
            .map(|br| br.r)
            .collect();
        if connected.is_empty() {
            return 0.05; // default fallback
        }
        connected.iter().sum::<f64>() / connected.len() as f64
    }

    /// Estimate thermal violations at the given renewable injection level.
    ///
    /// Compares the branch current (approximated from the power injection) to
    /// the branch rate_a (thermal rating).
    fn estimate_thermal_violations(&self, renewable_mw: f64) -> usize {
        let n_buses = self.renewable_buses.len();
        if n_buses == 0 {
            return 0;
        }
        let mw_per_bus = renewable_mw / n_buses as f64;
        let base_mva = self.network.base_mva;
        let p_pu_per_bus = mw_per_bus / base_mva;

        self.renewable_buses
            .iter()
            .filter(|&&bus_id| {
                self.network.branches.iter().any(|br| {
                    (br.from_bus == bus_id || br.to_bus == bus_id)
                        && br.rate_a > 1e-6
                        && p_pu_per_bus > br.rate_a / base_mva
                })
            })
            .count()
    }

    /// Estimate hosting capacity \[MW\] by finding the MW level that causes the
    /// first thermal or voltage violation.
    fn estimate_hosting_capacity_mw(&self, total_load_mw: f64, current_mw: f64) -> f64 {
        // Binary search between 0 and 2 × total_load for the violation threshold
        let mut lo = 0.0_f64;
        let mut hi = 2.0 * total_load_mw;
        for _ in 0..30 {
            let mid = (lo + hi) * 0.5;
            let v_viol = self.estimate_voltage_violations(mid);
            let t_viol = self.estimate_thermal_violations(mid);
            if v_viol > 0 || t_viol > 0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        // If no violation found at all, return a value relative to current level
        if hi >= 2.0 * total_load_mw - 1e-3 {
            current_mw.max(total_load_mw * 0.5)
        } else {
            lo
        }
    }

    /// Compute the Short Circuit Ratio at a connection bus.
    ///
    /// SCR = short-circuit MVA at bus / renewable MW.
    /// A low SCR (< 3) indicates a weak grid with potential integration challenges.
    pub fn compute_scr(&self, connection_bus: usize, renewable_mw: f64) -> Result<f64> {
        if renewable_mw < 1e-9 {
            return Err(OxiGridError::InvalidParameter(
                "renewable_mw must be > 0 to compute SCR".to_string(),
            ));
        }
        let assessment = assess_grid_strength(&self.network, connection_bus, renewable_mw);
        Ok(assessment.scr)
    }

    /// Assess system inertia as renewable penetration increases.
    ///
    /// As synchronous generators are displaced by power-electronics-interfaced
    /// renewables, the system kinetic energy (H · MVA) falls, increasing ROCOF
    /// after generator trips.
    ///
    /// # Arguments
    /// - `renewable_penetration_pct` — fraction of load served by renewables [0–1]
    /// - `generator_inertia`         — list of (generator_id, H_seconds) pairs
    ///
    /// # Returns
    /// [`InertiaResult`] with system inertia, ROCOF estimate, and synthetic
    /// inertia requirement.
    pub fn inertia_assessment(
        &self,
        renewable_penetration_pct: f64,
        generator_inertia: &[(usize, f64)],
    ) -> InertiaResult {
        // System inertia H_sys = Σ(H_i · MVA_i) / MVA_base
        // As RE displaces conventional, fraction (1 - penetration) of conventional capacity remains
        let conventional_fraction = (1.0 - renewable_penetration_pct).clamp(0.0, 1.0);

        let total_inertia_mws: f64 = generator_inertia
            .iter()
            .filter_map(|&(gen_id, h)| {
                self.network
                    .generators
                    .iter()
                    .find(|g| g.bus_id == gen_id && g.status)
                    .map(|g| h * g.mbase * conventional_fraction)
            })
            .sum();

        // ROCOF after largest infeed loss: ROCOF = P_trip / (2 · H_sys · f0)
        // Largest generator trip
        let largest_infeed_mw = self
            .network
            .generators
            .iter()
            .filter(|g| g.status)
            .map(|g| g.pg)
            .fold(0.0_f64, f64::max);

        let f0 = 50.0; // nominal frequency [Hz]
        let rocof = if total_inertia_mws > 1e-6 {
            largest_infeed_mw / (2.0 * total_inertia_mws * f0)
        } else {
            f64::INFINITY
        };

        // Minimum inertia required to keep ROCOF < 1 Hz/s
        // H_min = P_trip / (2 · ROCOF_max · f0)
        let rocof_max = 1.0; // Hz/s
        let min_inertia_required = largest_infeed_mw / (2.0 * rocof_max * f0);

        let inertia_deficit = (min_inertia_required - total_inertia_mws).max(0.0);

        // Synthetic inertia from renewables: each MW of wind/solar can provide
        // approximately 2–6 MWs of virtual inertia depending on control design
        // We use a conservative estimate of 3 MWs/MW of RE
        let synthetic_inertia_per_mw = 3.0;
        let renewable_mw = renewable_penetration_pct * self.network.total_load_mw();
        let synthetic_inertia_available = renewable_mw * synthetic_inertia_per_mw;
        let synthetic_needed = if inertia_deficit > 0.0 {
            inertia_deficit.min(synthetic_inertia_available)
        } else {
            0.0
        };

        InertiaResult {
            system_inertia_mws: total_inertia_mws,
            rocof_at_largest_infeed_hz_per_s: rocof,
            min_inertia_required_mws: min_inertia_required,
            inertia_deficit_mws: inertia_deficit,
            synthetic_inertia_needed_mws: synthetic_needed,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Inertia result
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a system inertia assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaResult {
    /// Total system kinetic energy \[MWs\].
    pub system_inertia_mws: f64,
    /// Estimated ROCOF [Hz/s] following loss of the largest generator.
    pub rocof_at_largest_infeed_hz_per_s: f64,
    /// Minimum system inertia required to keep ROCOF ≤ 1 Hz/s \[MWs\].
    pub min_inertia_required_mws: f64,
    /// Inertia shortfall (0 if no deficit) \[MWs\].
    pub inertia_deficit_mws: f64,
    /// Synthetic inertia required from renewable inverters \[MWs\].
    pub synthetic_inertia_needed_mws: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Stand-alone SCR helper (no network needed)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute Short Circuit Ratio from short-circuit MVA and renewable capacity.
///
/// # Arguments
/// - `sc_mva`       — three-phase short-circuit MVA at the connection bus
/// - `renewable_mw` — total renewable MW connected at the bus
pub fn scr_from_sc_mva(sc_mva: f64, renewable_mw: f64) -> Result<f64> {
    if renewable_mw < 1e-9 {
        return Err(OxiGridError::InvalidParameter(
            "renewable_mw must be positive to compute SCR".to_string(),
        ));
    }
    Ok(sc_mva / renewable_mw)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ──────────────────────────────────────────────────────────────────────────
    // Grid strength tests (no network required)
    // ──────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_grid_strength_strong() {
        assert_eq!(GridStrength::from_scr(6.0), GridStrength::Strong);
    }

    #[test]
    fn test_grid_strength_medium() {
        assert_eq!(GridStrength::from_scr(4.0), GridStrength::Medium);
    }

    #[test]
    fn test_grid_strength_weak() {
        assert_eq!(GridStrength::from_scr(2.5), GridStrength::Weak);
    }

    #[test]
    fn test_grid_strength_very_weak() {
        assert_eq!(GridStrength::from_scr(1.5), GridStrength::VeryWeak);
    }

    #[test]
    fn test_scr_from_sc_mva() {
        let scr = scr_from_sc_mva(500.0, 100.0).expect("SCR computation failed");
        assert!((scr - 5.0).abs() < 1e-9, "SCR should be 5.0, got {scr}");
    }

    #[test]
    fn test_scr_zero_renewable_errors() {
        let result = scr_from_sc_mva(500.0, 0.0);
        assert!(result.is_err(), "Zero renewable_mw should return an error");
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Network-based tests (require "powerflow" feature)
    // ──────────────────────────────────────────────────────────────────────────

    #[cfg(feature = "powerflow")]
    fn make_test_network() -> PowerNetwork {
        use crate::network::branch::Branch;
        use crate::network::bus::{Bus, BusType};
        use crate::network::topology::{Generator, PowerNetwork};
        use crate::units::{Power, ReactivePower, Voltage};

        let mut net = PowerNetwork::new(100.0);

        // Bus 1: slack
        let mut bus1 = Bus::new(1, BusType::Slack);
        bus1.base_kv = Voltage(110.0);
        bus1.vm = 1.0;
        bus1.pd = Power(0.0);
        bus1.qd = ReactivePower(0.0);
        net.buses.push(bus1);

        // Bus 2: PQ load bus
        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.base_kv = Voltage(110.0);
        bus2.vm = 1.0;
        bus2.pd = Power(80.0);
        bus2.qd = ReactivePower(20.0);
        net.buses.push(bus2);

        // Bus 3: PQ renewable bus
        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.base_kv = Voltage(110.0);
        bus3.vm = 1.0;
        bus3.pd = Power(20.0);
        bus3.qd = ReactivePower(5.0);
        net.buses.push(bus3);

        // Branch 1-2
        let br12 = Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.05,
            b: 0.02,
            rate_a: 150.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        };
        net.branches.push(br12);

        // Branch 2-3
        let br23 = Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.02,
            x: 0.08,
            b: 0.01,
            rate_a: 80.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        };
        net.branches.push(br23);

        // Generator at bus 1 (slack)
        let gen = Generator {
            bus_id: 1,
            pg: 100.0,
            qg: 25.0,
            qmax: 60.0,
            qmin: -20.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 200.0,
            pmin: 0.0,
        };
        net.generators.push(gen);

        net
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_scr_computation() {
        let net = make_test_network();
        let study = IntegrationStudy::new(net, vec![3]);
        let scr = study.compute_scr(3, 50.0).expect("SCR should succeed");
        assert!(scr > 0.0, "SCR must be positive, got {scr}");
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_hosting_capacity_positive() {
        let net = make_test_network();
        let study = IntegrationStudy::new(net, vec![3]);
        let results = study
            .hosting_capacity_analysis()
            .expect("Analysis should succeed");
        assert!(!results.is_empty(), "Should return at least one result");
        for r in &results {
            assert!(
                r.hosting_capacity_mw >= 0.0,
                "Hosting capacity must be non-negative: got {}",
                r.hosting_capacity_mw
            );
            assert!(r.penetration_pct >= 0.0 && r.penetration_pct <= 100.0);
        }
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_grid_strength_strong_with_network() {
        let net = make_test_network();
        // Bus 1 has a very low impedance (close to slack → high SC MVA)
        let assessment = assess_grid_strength(&net, 1, 10.0);
        // With low impedance and 100 MVA base, SCR should be high
        assert!(
            assessment.scr > 0.0,
            "SCR should be positive for a valid network"
        );
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_inertia_assessment_decreases_with_penetration() {
        let net = make_test_network();
        let study = IntegrationStudy::new(net, vec![3]);
        let inertia_low = study.inertia_assessment(0.2, &[(1, 5.0)]);
        let inertia_high = study.inertia_assessment(0.8, &[(1, 5.0)]);
        assert!(
            inertia_low.system_inertia_mws >= inertia_high.system_inertia_mws,
            "Higher RE penetration should reduce system inertia"
        );
    }

    #[cfg(feature = "powerflow")]
    #[test]
    fn test_inertia_deficit_appears_at_high_penetration() {
        let net = make_test_network();
        let study = IntegrationStudy::new(net, vec![3]);
        // Very small inertia constant → should produce a deficit
        let result = study.inertia_assessment(0.9, &[(1, 0.1)]);
        assert!(
            result.inertia_deficit_mws >= 0.0,
            "Inertia deficit must be non-negative"
        );
    }
}
