//! Cyber-Physical Security Simulation for Power Grids.
//!
//! This module provides a simulation framework for modelling attacker-defender
//! interactions in grid cyber-physical systems, including:
//!
//! - **Attack scenario modelling** — FDI, DoS, replay, command injection,
//!   sensor tampering, load-altering attacks
//! - **Defense layer evaluation** — firewalls, IDS, encryption, redundancy,
//!   anomaly detection
//! - **Monte Carlo risk assessment** — probabilistic impact quantification
//! - **Resilience metrics** — absorptive, adaptive, restorative capacity
//!
//! # References
//! - NERC CIP-013, IEC 62351, NIST SP 800-82 Rev 3
//! - Anderson & Boulanger, "SCADA Cybersecurity", 2012

// ─── LCG Parameters ──────────────────────────────────────────────────────────

const LCG_MULT: u64 = 6364136223846793005u64;
const LCG_ADD: u64 = 1442695040888963407u64;

/// Advance the LCG state and return a uniform sample in \[0, 1).
#[inline]
fn lcg_next(state: &mut u64) -> f64 {
    *state = state.wrapping_mul(LCG_MULT).wrapping_add(LCG_ADD);
    (*state >> 11) as f64 / (1u64 << 53) as f64
}

// ─── Attack Vector ────────────────────────────────────────────────────────────

/// The type and parameters of a cyber attack against the grid.
#[derive(Debug, Clone)]
pub enum AttackVector {
    /// False Data Injection — corrupt state estimator measurements on target buses.
    ///
    /// `magnitude_pu` is the per-unit bias injected into voltage/power measurements.
    FalseDataInjection {
        /// Bus indices whose measurements are targeted.
        target_buses: Vec<usize>,
        /// Injection magnitude \[pu\].
        magnitude_pu: f64,
    },
    /// Denial-of-Service — disable target SCADA/communication components.
    ///
    /// `duration_s` is the duration of the outage \[s\].
    DenialOfService {
        /// Component identifiers (SCADA RTU names, EMS node IDs, etc.).
        target_components: Vec<String>,
        /// Attack duration \[s\].
        duration_s: f64,
    },
    /// Replay Attack — replay stale control messages to a target controller.
    ReplayAttack {
        /// Identifier of the targeted communications channel or device.
        target: String,
        /// Replay delay \[s\].
        delay_s: f64,
    },
    /// Command Injection — inject a false set-point to a generating unit or EMS.
    CommandInjection {
        /// Name of the targeted controller (e.g. "AGC", "GenSet-3").
        target_controller: String,
        /// False MW set-point injected \[MW\].
        false_setpoint_mw: f64,
    },
    /// Sensor Tampering — physically or digitally bias sensor readings.
    SensorTampering {
        /// Identifiers of sensors that are compromised.
        sensor_ids: Vec<String>,
        /// Measurement bias \[pu\].
        bias_pu: f64,
    },
    /// Load-Altering Attack — directly alter loads on target buses via smart meters.
    LoadAlteringAttack {
        /// Bus indices whose loads are altered.
        target_buses: Vec<usize>,
        /// Load change \[MW\] (positive = added load, negative = load shedding).
        delta_mw: f64,
    },
}

// ─── Defense Layers ───────────────────────────────────────────────────────────

/// A single layer in a multi-layer defense strategy.
#[derive(Debug, Clone)]
pub enum DefenseLayer {
    /// Network-level packet filtering.
    ///
    /// `effectiveness` is the probability of blocking a penetration attempt \[0–1\].
    Firewall {
        /// Blocking probability \[0–1\].
        effectiveness: f64,
    },
    /// Host-based or network intrusion detection system.
    IntrusionDetection {
        /// True-positive detection rate \[0–1\].
        detection_rate: f64,
        /// False-positive alert rate \[0–1\].
        false_positive_rate: f64,
    },
    /// Data-in-transit and data-at-rest encryption.
    Encryption {
        /// Symmetric key strength \[bits\] (e.g. 128, 256).
        key_strength_bits: u32,
    },
    /// Physical access control around substations and control rooms.
    PhysicalSecurity {
        /// Protection level 1 (minimal) – 5 (military-grade).
        protection_level: u8,
    },
    /// Redundant backup systems that take over when primary components fail.
    Redundancy {
        /// Number of backup systems available.
        backup_systems: usize,
    },
    /// Statistical anomaly detection on measurement streams.
    AnomalyDetection {
        /// Detection threshold (e.g. 3σ chi-squared threshold).
        threshold: f64,
    },
}

impl DefenseLayer {
    /// Marginal effectiveness of this layer in reducing penetration probability.
    ///
    /// Returns a value in \[0, 1\]; multiplied into the surviving-attack probability.
    pub fn layer_effectiveness(&self) -> f64 {
        match self {
            DefenseLayer::Firewall { effectiveness } => effectiveness.clamp(0.0, 1.0),
            DefenseLayer::IntrusionDetection { detection_rate, .. } => {
                detection_rate.clamp(0.0, 1.0)
            }
            DefenseLayer::Encryption { key_strength_bits } => {
                // Normalise: 128 bits → 0.5, 256 bits → 0.9, 512 bits → 0.99
                let bits = *key_strength_bits as f64;
                (bits / (bits + 128.0)).clamp(0.0, 1.0)
            }
            DefenseLayer::PhysicalSecurity { protection_level } => {
                ((*protection_level as f64).clamp(1.0, 5.0) - 1.0) / 4.0
            }
            DefenseLayer::Redundancy { backup_systems } => {
                // Effectiveness = 1 − (1 / (1 + backup_systems))
                let n = *backup_systems as f64;
                (n / (n + 1.0)).clamp(0.0, 1.0)
            }
            DefenseLayer::AnomalyDetection { threshold } => {
                // Higher threshold = more sensitive detection
                (1.0 - (-*threshold / 10.0).exp()).clamp(0.0, 1.0)
            }
        }
    }
}

// ─── Attack Timing / Objective ────────────────────────────────────────────────

/// When the attack is launched relative to system state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttackTiming {
    /// During peak demand period.
    PeakLoad,
    /// When renewable penetration is highest (minimum system inertia).
    MinimumInertia,
    /// When the system is already stressed by a recent fault.
    PostFault,
    /// Multiple simultaneous sub-attacks.
    Coordinated {
        /// Number of simultaneous attack vectors launched.
        simultaneous_attacks: usize,
    },
}

/// Attacker's strategic objective.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttackObjective {
    /// Maximise the amount of customer load disconnected \[MW\].
    MaximizeLoadShed,
    /// Trigger a cascading failure propagating across the network.
    TriggerCascade,
    /// Corrupt historical or real-time data without physical disruption.
    StealthDataCorruption,
    /// Stage infrastructure access for a later ransomware deployment.
    RansomwarePreparation,
}

// ─── Attack Scenario ─────────────────────────────────────────────────────────

/// A complete cyber attack scenario specification.
#[derive(Debug, Clone)]
pub struct CpAttackScenario {
    /// Unique identifier for this scenario.
    pub scenario_id: String,
    /// Attacker capability level: 1 (script-kiddie) – 5 (nation-state).
    pub attacker_capability: u8,
    /// Attack vector type and parameters.
    pub attack_vector: AttackVector,
    /// Timing strategy.
    pub timing: AttackTiming,
    /// Strategic objective.
    pub objective: AttackObjective,
}

// ─── Simulation Configuration ─────────────────────────────────────────────────

/// Configuration for the cyber-physical security simulation.
#[derive(Debug, Clone)]
pub struct CyberPhysicalSimConfig {
    /// Number of buses in the power network.
    pub num_buses: usize,
    /// Number of Monte Carlo iterations per scenario (default 200).
    pub monte_carlo_runs: usize,
    /// Simulation time horizon \[s\] (default 3600).
    pub time_horizon_s: f64,
    /// Expected recovery time after a successful attack \[s\] (default 600).
    pub recovery_time_s: f64,
}

impl Default for CyberPhysicalSimConfig {
    fn default() -> Self {
        Self {
            num_buses: 14,
            monte_carlo_runs: 200,
            time_horizon_s: 3600.0,
            recovery_time_s: 600.0,
        }
    }
}

// ─── Output Types ─────────────────────────────────────────────────────────────

/// Severity classification of an attack impact.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum ImpactSeverity {
    /// No measurable effect on grid operation.
    Negligible,
    /// Small, localised disturbance; no load shed.
    Minor,
    /// Noticeable but manageable disturbance.
    Moderate,
    /// Significant load shed or voltage violations; operator intervention required.
    Severe,
    /// Widespread blackout or cascading failure.
    Catastrophic,
}

impl ImpactSeverity {
    fn from_load_shed_fraction(frac: f64, cascade: bool) -> Self {
        if cascade || frac >= 0.50 {
            ImpactSeverity::Catastrophic
        } else if frac >= 0.30 {
            ImpactSeverity::Severe
        } else if frac >= 0.10 {
            ImpactSeverity::Moderate
        } else if frac > 0.0 {
            ImpactSeverity::Minor
        } else {
            ImpactSeverity::Negligible
        }
    }
}

/// Result of simulating one attack scenario instance.
#[derive(Debug, Clone)]
pub struct AttackImpactResult {
    /// Whether the attacker successfully penetrated the defense perimeter.
    pub attack_success: bool,
    /// Penetration probability computed from capability vs defenses \[0–1\].
    pub penetration_probability: f64,
    /// Load shed caused by the attack \[MW\].
    pub load_shed_mw: f64,
    /// Number of buses with voltage outside acceptable limits.
    pub voltage_violations: usize,
    /// Frequency deviation caused by load-generation imbalance \[Hz\].
    pub frequency_deviation_hz: f64,
    /// Whether the attack triggered a cascading failure.
    pub cascade_triggered: bool,
    /// Estimated time to restore normal operation \[s\].
    pub recovery_time_s: f64,
    /// Overall impact severity classification.
    pub impact_severity: ImpactSeverity,
}

/// Physical power system impact of a cyber attack.
#[derive(Debug, Clone)]
pub struct PhysicalImpact {
    /// Load shed caused by voltage collapse or islanding \[MW\].
    pub load_shed_mw: f64,
    /// Probability of a cascading failure \[0–1\].
    pub cascade_probability: f64,
    /// Estimated frequency deviation \[Hz\].
    pub frequency_deviation_hz: f64,
    /// Voltage stability margin (1.0 = fully stable, 0.0 = collapse boundary).
    pub voltage_stability_margin: f64,
}

/// Probabilistic risk matrix from Monte Carlo analysis.
#[derive(Debug, Clone)]
pub struct RiskMatrix {
    /// Expected annual energy loss across scenarios \[MWh/year\].
    pub expected_annual_loss_mwh: f64,
    /// Probability of at least one cascading failure \[0–1\].
    pub p_cascade: f64,
    /// Probability of a complete blackout \[0–1\].
    pub p_blackout: f64,
    /// Composite risk score \[0–100\].
    pub risk_score: f64,
    /// Worst-case energy loss across all Monte Carlo samples \[MWh\].
    pub worst_case_loss_mwh: f64,
}

/// Return-on-investment analysis for a candidate defense investment.
#[derive(Debug, Clone)]
pub struct DefenseRoi {
    /// Percentage reduction in risk score from the new defense layer \[%\].
    pub risk_reduction_pct: f64,
    /// Annual energy-loss reduction from adding the defense \[MWh/year\].
    pub risk_reduction_mwh_per_year: f64,
    /// Benefit-to-cost ratio (dimensionless).
    pub benefit_to_cost_ratio: f64,
    /// Estimated payback period \[years\].
    pub payback_years: f64,
}

/// Report of anomaly detection on a measurement snapshot.
#[derive(Debug, Clone)]
pub struct AnomalyReport {
    /// Chi-squared statistic computed on measurement residuals.
    pub chi_squared: f64,
    /// Detection threshold applied.
    pub threshold: f64,
    /// Whether an anomaly (possible FDI or sensor fault) was detected.
    pub anomaly_detected: bool,
    /// Indices of measurements identified as suspicious.
    pub suspicious_measurements: Vec<usize>,
}

/// Composite resilience metrics summarising historical attack performance.
#[derive(Debug, Clone)]
pub struct ResilienceMetrics {
    /// Fraction of worst-case load that was NOT shed (1 = full absorption).
    pub absorptive_capacity: f64,
    /// Fraction of attacks that were successfully mitigated before physical impact.
    pub adaptive_capacity: f64,
    /// Reciprocal of mean recovery time (higher = faster restoration) \[1/s\].
    pub restorative_capacity: f64,
    /// Composite resilience index (geometric mean of the three capacities).
    pub resilience_index: f64,
}

// ─── Simulator ────────────────────────────────────────────────────────────────

/// Cyber-physical security simulator for power grid attack-defense evaluation.
pub struct CyberPhysicalSim {
    /// Simulation configuration.
    pub config: CyberPhysicalSimConfig,
    /// Ordered stack of defense layers in place.
    pub defense_layers: Vec<DefenseLayer>,
    /// Internal LCG state for reproducible randomness.
    pub lcg_state: u64,
}

impl CyberPhysicalSim {
    /// Create a new simulator with the given configuration and no defense layers.
    pub fn new(config: CyberPhysicalSimConfig) -> Self {
        Self {
            lcg_state: 0xDEAD_C0DE_CAFE_BABEu64,
            config,
            defense_layers: Vec::new(),
        }
    }

    /// Add a defense layer to the simulator.
    pub fn add_defense(&mut self, layer: DefenseLayer) {
        self.defense_layers.push(layer);
    }

    // ─── Public API ──────────────────────────────────────────────────────────

    /// Simulate a single attack scenario and return the impact result.
    ///
    /// # Parameters
    /// - `scenario` — the attack scenario specification
    /// - `bus_voltages_pu` — pre-attack bus voltage magnitudes \[pu\]
    /// - `load_mw` — pre-attack bus active load profile \[MW\]
    pub fn simulate_attack(
        &mut self,
        scenario: &CpAttackScenario,
        bus_voltages_pu: &[f64],
        load_mw: &[f64],
    ) -> AttackImpactResult {
        let penetration_probability = self.calculate_penetration_probability(scenario);

        // Determine attack success via LCG draw
        let u = lcg_next(&mut self.lcg_state);
        let attack_success = u < penetration_probability;

        if !attack_success {
            return AttackImpactResult {
                attack_success: false,
                penetration_probability,
                load_shed_mw: 0.0,
                voltage_violations: 0,
                frequency_deviation_hz: 0.0,
                cascade_triggered: false,
                recovery_time_s: 0.0,
                impact_severity: ImpactSeverity::Negligible,
            };
        }

        // Compute impact based on attack vector
        let (load_shed_mw, voltage_violations, frequency_deviation_hz, cascade_triggered) =
            self.compute_vector_impact(scenario, bus_voltages_pu, load_mw);

        let total_load: f64 = load_mw.iter().sum::<f64>().max(1.0);
        let shed_fraction = (load_shed_mw / total_load).clamp(0.0, 1.0);
        let impact_severity =
            ImpactSeverity::from_load_shed_fraction(shed_fraction, cascade_triggered);

        // Recovery time: base + proportional to shed + cascade penalty
        let recovery_time_s = self.config.recovery_time_s
            * (1.0 + 2.0 * shed_fraction)
            * if cascade_triggered { 3.0 } else { 1.0 };

        AttackImpactResult {
            attack_success,
            penetration_probability,
            load_shed_mw,
            voltage_violations,
            frequency_deviation_hz,
            cascade_triggered,
            recovery_time_s,
            impact_severity,
        }
    }

    /// Calculate the probability that an attack penetrates all defense layers.
    ///
    /// Base probability is derived from attacker capability; each defense layer
    /// multiplicatively reduces the remaining probability.
    pub fn calculate_penetration_probability(&self, scenario: &CpAttackScenario) -> f64 {
        // Base probability: capability² / 25 × 0.1  → [0.004 … 0.1]
        let cap = scenario.attacker_capability.clamp(1, 5) as f64;
        let base = 0.1 * (cap * cap) / 25.0;

        // Timing amplifier
        let timing_mult = match &scenario.timing {
            AttackTiming::PeakLoad => 1.3,
            AttackTiming::MinimumInertia => 1.5,
            AttackTiming::PostFault => 1.8,
            AttackTiming::Coordinated {
                simultaneous_attacks,
            } => 1.0 + 0.3 * (*simultaneous_attacks as f64).min(5.0),
        };

        let raw = (base * timing_mult).clamp(0.0, 1.0);

        // Each defense layer multiplies the surviving probability by (1 − eff_i)
        let survival = self.defense_layers.iter().fold(1.0_f64, |acc, layer| {
            acc * (1.0 - layer.layer_effectiveness())
        });

        (raw * survival).clamp(0.0, 1.0)
    }

    /// Assess the physical power system impact from an `AttackImpactResult`.
    ///
    /// # Parameters
    /// - `attack_result` — result of a previous `simulate_attack` call
    /// - `bus_voltages_pu` — pre-attack bus voltages \[pu\]
    pub fn assess_physical_impact(
        &self,
        attack_result: &AttackImpactResult,
        bus_voltages_pu: &[f64],
    ) -> PhysicalImpact {
        if !attack_result.attack_success {
            return PhysicalImpact {
                load_shed_mw: 0.0,
                cascade_probability: 0.0,
                frequency_deviation_hz: 0.0,
                voltage_stability_margin: 1.0,
            };
        }

        // Voltage-induced load shed: buses below 0.9 pu shed load proportionally
        let n = bus_voltages_pu.len().max(1);
        let voltage_violations = bus_voltages_pu.iter().filter(|&&v| v < 0.9).count();
        let voltage_stability_margin = 1.0 - (voltage_violations as f64 / n as f64).clamp(0.0, 1.0);

        let load_shed_mw = attack_result.load_shed_mw;

        // Cascade: if shed > 20% → cascade probability = shed fraction
        let total_load_estimate = load_shed_mw / 0.20_f64.max(1e-9); // invert 20 % threshold
        let shed_fraction = if total_load_estimate > 0.0 {
            (load_shed_mw / total_load_estimate).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let cascade_probability = if shed_fraction > 0.20 {
            shed_fraction.clamp(0.0, 1.0)
        } else {
            0.0
        };

        PhysicalImpact {
            load_shed_mw,
            cascade_probability,
            frequency_deviation_hz: attack_result.frequency_deviation_hz,
            voltage_stability_margin,
        }
    }

    /// Monte Carlo risk assessment over a set of attack scenarios.
    ///
    /// For each Monte Carlo run and each scenario, samples attack success and
    /// accumulates loss statistics.
    ///
    /// # Parameters
    /// - `scenarios` — list of `(scenario, bus_voltages_pu, load_mw)` tuples
    pub fn monte_carlo_risk_assessment(
        &mut self,
        scenarios: &[(CpAttackScenario, Vec<f64>, Vec<f64>)],
    ) -> RiskMatrix {
        if scenarios.is_empty() {
            return RiskMatrix {
                expected_annual_loss_mwh: 0.0,
                p_cascade: 0.0,
                p_blackout: 0.0,
                risk_score: 0.0,
                worst_case_loss_mwh: 0.0,
            };
        }

        let n_runs = self.config.monte_carlo_runs;
        let time_horizon_h = self.config.time_horizon_s / 3600.0;

        let mut total_loss_mwh = 0.0_f64;
        let mut cascade_count = 0u64;
        let mut blackout_count = 0u64;
        let mut worst_loss_mwh = 0.0_f64;

        for _ in 0..n_runs {
            let mut run_loss_mwh = 0.0_f64;
            let mut run_cascade = false;
            let mut run_blackout = false;

            for (scenario, voltages, loads) in scenarios {
                let result = self.simulate_attack(scenario, voltages, loads);
                if result.attack_success {
                    let loss_mwh = result.load_shed_mw * time_horizon_h;
                    run_loss_mwh += loss_mwh;
                    if result.cascade_triggered {
                        run_cascade = true;
                    }
                    if result.impact_severity == ImpactSeverity::Catastrophic {
                        run_blackout = true;
                    }
                }
            }

            total_loss_mwh += run_loss_mwh;
            if run_loss_mwh > worst_loss_mwh {
                worst_loss_mwh = run_loss_mwh;
            }
            if run_cascade {
                cascade_count += 1;
            }
            if run_blackout {
                blackout_count += 1;
            }
        }

        let n = n_runs as f64;
        let expected_annual_loss_mwh = (total_loss_mwh / n) * (8760.0 / time_horizon_h);
        let p_cascade = (cascade_count as f64 / n).clamp(0.0, 1.0);
        let p_blackout = (blackout_count as f64 / n).clamp(0.0, 1.0);

        // Risk score [0–100]: normalise expected loss by an arbitrary reference (10 GWh/year)
        let risk_score = (expected_annual_loss_mwh / 10_000.0 * 100.0).clamp(0.0, 100.0);

        RiskMatrix {
            expected_annual_loss_mwh,
            p_cascade,
            p_blackout,
            risk_score,
            worst_case_loss_mwh: worst_loss_mwh,
        }
    }

    /// Evaluate the return on investment of adding a new defense layer.
    ///
    /// Runs Monte Carlo risk assessment with and without the candidate layer and
    /// returns cost-benefit metrics.
    ///
    /// # Parameters
    /// - `new_defense` — candidate defense layer to evaluate
    /// - `attack_scenarios` — scenarios used for risk quantification
    /// - `defense_cost_usd` — estimated annual cost of the new defense \[USD\]
    /// - `voll_usd_per_mwh` — value of lost load \[USD/MWh\]
    pub fn evaluate_defense_investment(
        &mut self,
        new_defense: DefenseLayer,
        attack_scenarios: &[(CpAttackScenario, Vec<f64>, Vec<f64>)],
        defense_cost_usd: f64,
        voll_usd_per_mwh: f64,
    ) -> DefenseRoi {
        // Baseline risk (without new defense)
        let baseline = self.monte_carlo_risk_assessment(attack_scenarios);

        // Add defense temporarily
        self.defense_layers.push(new_defense);
        let with_defense = self.monte_carlo_risk_assessment(attack_scenarios);
        self.defense_layers.pop();

        let baseline_score = baseline.risk_score.max(1e-9);
        let new_score = with_defense.risk_score;

        let risk_reduction_pct =
            ((baseline_score - new_score) / baseline_score * 100.0).clamp(0.0, 100.0);

        let risk_reduction_mwh_per_year =
            (baseline.expected_annual_loss_mwh - with_defense.expected_annual_loss_mwh).max(0.0);

        let annual_benefit_usd = risk_reduction_mwh_per_year * voll_usd_per_mwh;
        let benefit_to_cost_ratio = if defense_cost_usd > 0.0 {
            annual_benefit_usd / defense_cost_usd
        } else {
            f64::INFINITY
        };

        let payback_years = if annual_benefit_usd > 0.0 {
            defense_cost_usd / annual_benefit_usd
        } else {
            f64::INFINITY
        };

        DefenseRoi {
            risk_reduction_pct,
            risk_reduction_mwh_per_year,
            benefit_to_cost_ratio,
            payback_years,
        }
    }

    /// Detect anomalies in a measurement snapshot using a chi-squared test.
    ///
    /// Computes normalised residuals between `measurements_normal` (baseline)
    /// and `measurements_current`, then applies a chi-squared test.
    ///
    /// # Parameters
    /// - `measurements_normal` — reference (nominal) measurements
    /// - `measurements_current` — current measurements to test
    pub fn detect_anomaly(
        &self,
        measurements_normal: &[f64],
        measurements_current: &[f64],
    ) -> AnomalyReport {
        let n = measurements_normal.len().min(measurements_current.len());
        if n == 0 {
            return AnomalyReport {
                chi_squared: 0.0,
                threshold: 9.0,
                anomaly_detected: false,
                suspicious_measurements: vec![],
            };
        }

        // Compute mean and std of normal measurements
        let mean: f64 = measurements_normal.iter().sum::<f64>() / n as f64;
        let variance: f64 = measurements_normal
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / n as f64;
        let std_dev = variance.sqrt().max(1e-9);

        // Normalised residuals and chi-squared statistic
        let residuals: Vec<f64> = (0..n)
            .map(|i| (measurements_current[i] - measurements_normal[i]) / std_dev)
            .collect();

        let chi_squared: f64 = residuals.iter().map(|&r| r * r).sum::<f64>() / n as f64;

        // 3σ threshold in chi-squared sense: (3σ)² / n ≈ 9 (per measurement)
        let threshold = 9.0_f64;
        let anomaly_detected = chi_squared > threshold;

        // Flag individual measurements with |residual| > 3σ
        let suspicious_measurements: Vec<usize> = residuals
            .iter()
            .enumerate()
            .filter(|(_, &r)| r.abs() > 3.0)
            .map(|(i, _)| i)
            .collect();

        AnomalyReport {
            chi_squared,
            threshold,
            anomaly_detected,
            suspicious_measurements,
        }
    }

    /// Compute resilience metrics from a history of attack impact results.
    ///
    /// # Parameters
    /// - `impact_history` — `(result, total_load_mw)` pairs from past attack simulations
    pub fn resilience_metrics(
        &self,
        impact_history: &[(AttackImpactResult, f64)],
    ) -> ResilienceMetrics {
        if impact_history.is_empty() {
            // No attacks → perfect resilience
            return ResilienceMetrics {
                absorptive_capacity: 1.0,
                adaptive_capacity: 1.0,
                restorative_capacity: 1.0,
                resilience_index: 1.0,
            };
        }

        // Absorptive capacity: 1 − max_load_shed_fraction
        let max_shed_fraction = impact_history
            .iter()
            .map(|(r, total)| {
                if *total > 0.0 {
                    r.load_shed_mw / total
                } else {
                    0.0
                }
            })
            .fold(0.0_f64, f64::max)
            .clamp(0.0, 1.0);

        let absorptive_capacity = (1.0 - max_shed_fraction).clamp(0.0, 1.0);

        // Adaptive capacity: fraction of attacks that were successfully mitigated
        // (attack either failed or caused only Negligible impact)
        let mitigated = impact_history
            .iter()
            .filter(|(r, _)| !r.attack_success || r.impact_severity == ImpactSeverity::Negligible)
            .count();
        let adaptive_capacity = (mitigated as f64 / impact_history.len() as f64).clamp(0.0, 1.0);

        // Restorative capacity: 1 / mean_recovery_time (1/s)
        // For successful attacks, use the reported recovery time; unsuccessful → 0 s
        let total_recovery: f64 = impact_history
            .iter()
            .map(|(r, _)| {
                if r.attack_success {
                    r.recovery_time_s
                } else {
                    0.0
                }
            })
            .sum();
        let n_successful = impact_history
            .iter()
            .filter(|(r, _)| r.attack_success)
            .count();

        let restorative_capacity = if n_successful == 0 {
            1.0 // no successful attacks → perfect restoration
        } else {
            let mean_recovery = total_recovery / n_successful as f64;
            (1.0 / mean_recovery.max(1.0)).clamp(0.0, 1.0)
        };

        // Resilience index: geometric mean of the three capacities
        let product = absorptive_capacity * adaptive_capacity * restorative_capacity;
        let resilience_index = product.powf(1.0 / 3.0).clamp(0.0, 1.0);

        ResilienceMetrics {
            absorptive_capacity,
            adaptive_capacity,
            restorative_capacity,
            resilience_index,
        }
    }

    // ─── Internal helpers ────────────────────────────────────────────────────

    /// Compute attack impact parameters for each `AttackVector` type.
    fn compute_vector_impact(
        &mut self,
        scenario: &CpAttackScenario,
        bus_voltages_pu: &[f64],
        load_mw: &[f64],
    ) -> (f64, usize, f64, bool) {
        let total_load: f64 = load_mw.iter().sum::<f64>().max(1.0);
        let n_buses = bus_voltages_pu.len().max(1);

        match &scenario.attack_vector {
            AttackVector::FalseDataInjection {
                target_buses,
                magnitude_pu,
            } => {
                // Perturbed measurements cause wrong control actions.
                // Load shed ≈ magnitude_pu × fraction of buses targeted × total load
                let fraction = (target_buses.len() as f64 / n_buses as f64).clamp(0.0, 1.0);
                let load_shed_mw = magnitude_pu * fraction * total_load;

                // Voltage violations: targeted buses likely to violate
                let voltage_violations = target_buses
                    .iter()
                    .filter(|&&b| {
                        bus_voltages_pu
                            .get(b)
                            .map(|&v| !(0.9..=1.1).contains(&v))
                            .unwrap_or(false)
                    })
                    .count()
                    + (fraction * 2.0) as usize;

                let freq_dev = 0.1 * magnitude_pu * fraction;
                let cascade = load_shed_mw / total_load > 0.30;
                (load_shed_mw, voltage_violations, freq_dev, cascade)
            }

            AttackVector::DenialOfService {
                target_components,
                duration_s,
            } => {
                // DoS on N components forces manual re-dispatch; during gap, load may be shed.
                let component_fraction = (target_components.len() as f64 / n_buses as f64).min(1.0);
                // Load shed proportional to disabled capacity during DoS window
                let redispatch_fraction = component_fraction * (duration_s / 3600.0).min(1.0);
                let load_shed_mw = redispatch_fraction * total_load * 0.5;

                let voltage_violations = (component_fraction * n_buses as f64 * 0.2) as usize;
                let freq_dev = 0.05 * component_fraction;
                let cascade = load_shed_mw / total_load > 0.30;
                (load_shed_mw, voltage_violations, freq_dev, cascade)
            }

            AttackVector::ReplayAttack { target: _, delay_s } => {
                // Stale commands lead to transient over/under-voltage
                let delay_fraction = (delay_s / self.config.time_horizon_s).clamp(0.0, 1.0);
                let load_shed_mw = delay_fraction * total_load * 0.10;
                let voltage_violations = (delay_fraction * n_buses as f64 * 0.1) as usize;
                let freq_dev = 0.02 * delay_fraction;
                (load_shed_mw, voltage_violations, freq_dev, false)
            }

            AttackVector::CommandInjection {
                target_controller: _,
                false_setpoint_mw,
            } => {
                // Injected set-point causes generation–load imbalance
                let imbalance = false_setpoint_mw.abs();
                let load_shed_mw = (imbalance - total_load * 0.05).max(0.0).min(total_load);
                let freq_dev = imbalance / (total_load * 10.0);
                let voltage_violations = if freq_dev > 0.5 {
                    (n_buses / 4).max(1)
                } else {
                    0
                };
                let cascade = load_shed_mw / total_load > 0.25;
                (load_shed_mw, voltage_violations, freq_dev, cascade)
            }

            AttackVector::SensorTampering {
                sensor_ids,
                bias_pu,
            } => {
                // Biased sensors propagate through state estimator
                let sensor_fraction = (sensor_ids.len() as f64 / n_buses as f64).min(1.0);
                let load_shed_mw = bias_pu * sensor_fraction * total_load * 0.5;
                let voltage_violations = sensor_ids.len().min(n_buses / 2);
                let freq_dev = 0.05 * bias_pu * sensor_fraction;
                (load_shed_mw, voltage_violations, freq_dev, false)
            }

            AttackVector::LoadAlteringAttack {
                target_buses,
                delta_mw,
            } => {
                // Direct load alteration via compromised smart meters
                let n_target = target_buses.len() as f64;
                let total_delta = (delta_mw.abs() * n_target).min(total_load);
                let load_shed_mw = if *delta_mw > 0.0 {
                    // Added load → overload → shed
                    (total_delta - total_load * 0.10).max(0.0)
                } else {
                    // Sudden load drop creates frequency overshoot; minimal shed
                    0.0
                };

                // Voltage violations on target buses
                let voltage_violations = target_buses
                    .iter()
                    .filter(|&&b| b < n_buses)
                    .count()
                    .min(n_buses);

                let freq_dev = total_delta / (total_load * 10.0);
                let cascade = load_shed_mw / total_load > 0.30;
                (load_shed_mw, voltage_violations, freq_dev, cascade)
            }
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_sim() -> CyberPhysicalSim {
        CyberPhysicalSim::new(CyberPhysicalSimConfig {
            num_buses: 10,
            monte_carlo_runs: 200,
            time_horizon_s: 3600.0,
            recovery_time_s: 600.0,
        })
    }

    fn flat_voltages(n: usize, v: f64) -> Vec<f64> {
        vec![v; n]
    }

    fn uniform_loads(n: usize, p: f64) -> Vec<f64> {
        vec![p; n]
    }

    // ── Test 1: High capability + no defense → high penetration probability ──

    #[test]
    fn test_high_capability_weak_defense_high_penetration() {
        let sim = default_sim(); // no defense layers
        let scenario = CpAttackScenario {
            scenario_id: "T1".into(),
            attacker_capability: 5, // nation-state
            attack_vector: AttackVector::FalseDataInjection {
                target_buses: vec![0, 1, 2],
                magnitude_pu: 0.3,
            },
            timing: AttackTiming::PeakLoad,
            objective: AttackObjective::MaximizeLoadShed,
        };
        let p = sim.calculate_penetration_probability(&scenario);
        assert!(
            p > 0.05,
            "Nation-state attacker with no defense should have high penetration: {p:.4}"
        );
    }

    // ── Test 2: Multiple defense layers reduce probability multiplicatively ───

    #[test]
    fn test_multiple_defense_layers_reduce_probability() {
        let scenario = CpAttackScenario {
            scenario_id: "T2".into(),
            attacker_capability: 5,
            attack_vector: AttackVector::FalseDataInjection {
                target_buses: vec![0, 1, 2, 3, 4],
                magnitude_pu: 0.5,
            },
            timing: AttackTiming::MinimumInertia,
            objective: AttackObjective::TriggerCascade,
        };

        let sim_no_defense = default_sim();
        let p_no_defense = sim_no_defense.calculate_penetration_probability(&scenario);

        let mut sim_layered = default_sim();
        sim_layered.add_defense(DefenseLayer::Firewall { effectiveness: 0.8 });
        sim_layered.add_defense(DefenseLayer::IntrusionDetection {
            detection_rate: 0.7,
            false_positive_rate: 0.05,
        });
        sim_layered.add_defense(DefenseLayer::Encryption {
            key_strength_bits: 256,
        });
        let p_layered = sim_layered.calculate_penetration_probability(&scenario);

        assert!(
            p_layered < p_no_defense,
            "Layered defense must reduce penetration probability: {p_layered:.6} vs {p_no_defense:.6}"
        );
    }

    // ── Test 3: FDI attack — load shed proportional to magnitude ─────────────

    #[test]
    fn test_fdi_load_shed_proportional_to_magnitude() {
        let voltages = flat_voltages(10, 1.0);
        let loads = uniform_loads(10, 100.0); // 1000 MW total

        // Low magnitude FDI
        let mut sim_low = CyberPhysicalSim {
            config: CyberPhysicalSimConfig {
                num_buses: 10,
                monte_carlo_runs: 1,
                ..Default::default()
            },
            defense_layers: vec![],
            lcg_state: 0, // LCG with state=0 → first draw close to 0 → success likely
        };
        // Force attack success by seeding lcg such that draw < penetration_prob
        // nation-state, no defense, peak load → high probability
        let scenario_low = CpAttackScenario {
            scenario_id: "low".into(),
            attacker_capability: 5,
            attack_vector: AttackVector::FalseDataInjection {
                target_buses: vec![0, 1, 2, 3, 4],
                magnitude_pu: 0.1,
            },
            timing: AttackTiming::PeakLoad,
            objective: AttackObjective::MaximizeLoadShed,
        };
        let scenario_high = CpAttackScenario {
            scenario_id: "high".into(),
            attacker_capability: 5,
            attack_vector: AttackVector::FalseDataInjection {
                target_buses: vec![0, 1, 2, 3, 4],
                magnitude_pu: 0.5,
            },
            timing: AttackTiming::PeakLoad,
            objective: AttackObjective::MaximizeLoadShed,
        };

        // Use deterministic impact calculation directly
        let (shed_low, _, _, _) = sim_low.compute_vector_impact(&scenario_low, &voltages, &loads);
        let (shed_high, _, _, _) = sim_low.compute_vector_impact(&scenario_high, &voltages, &loads);

        assert!(
            shed_high > shed_low,
            "Higher FDI magnitude should cause more load shed: {shed_high:.2} vs {shed_low:.2}"
        );
    }

    // ── Test 4: DoS on critical component → severe impact ────────────────────

    #[test]
    fn test_dos_attack_severe_impact() {
        let voltages = flat_voltages(10, 1.0);
        let loads = uniform_loads(10, 100.0); // 1000 MW total

        let mut sim = CyberPhysicalSim {
            config: CyberPhysicalSimConfig {
                num_buses: 10,
                monte_carlo_runs: 1,
                time_horizon_s: 3600.0,
                recovery_time_s: 600.0,
            },
            defense_layers: vec![],
            lcg_state: 0,
        };

        let scenario = CpAttackScenario {
            scenario_id: "dos".into(),
            attacker_capability: 5,
            attack_vector: AttackVector::DenialOfService {
                target_components: (0..8).map(|i| format!("RTU-{i}")).collect(),
                duration_s: 3600.0,
            },
            timing: AttackTiming::PostFault,
            objective: AttackObjective::MaximizeLoadShed,
        };

        // Compute impact directly
        let (shed, violations, _freq, _cascade) =
            sim.compute_vector_impact(&scenario, &voltages, &loads);

        assert!(
            shed > 0.0,
            "DoS on 8/10 components should cause load shed: {shed:.2} MW"
        );
        assert!(
            violations > 0,
            "DoS on critical components should cause voltage violations"
        );
    }

    // ── Test 5: Monte Carlo — risk score non-negative, probabilities in [0,1] ─

    #[test]
    fn test_monte_carlo_valid_output_ranges() {
        let mut sim = CyberPhysicalSim::new(CyberPhysicalSimConfig {
            num_buses: 5,
            monte_carlo_runs: 50,
            time_horizon_s: 3600.0,
            recovery_time_s: 300.0,
        });

        let voltages = flat_voltages(5, 1.0);
        let loads = uniform_loads(5, 50.0);

        let scenarios = vec![
            (
                CpAttackScenario {
                    scenario_id: "mc1".into(),
                    attacker_capability: 3,
                    attack_vector: AttackVector::LoadAlteringAttack {
                        target_buses: vec![0, 1],
                        delta_mw: 30.0,
                    },
                    timing: AttackTiming::PeakLoad,
                    objective: AttackObjective::MaximizeLoadShed,
                },
                voltages.clone(),
                loads.clone(),
            ),
            (
                CpAttackScenario {
                    scenario_id: "mc2".into(),
                    attacker_capability: 4,
                    attack_vector: AttackVector::CommandInjection {
                        target_controller: "AGC".into(),
                        false_setpoint_mw: 200.0,
                    },
                    timing: AttackTiming::MinimumInertia,
                    objective: AttackObjective::TriggerCascade,
                },
                voltages.clone(),
                loads.clone(),
            ),
        ];

        let risk = sim.monte_carlo_risk_assessment(&scenarios);

        assert!(
            risk.risk_score >= 0.0 && risk.risk_score <= 100.0,
            "Risk score must be in [0,100]: {:.2}",
            risk.risk_score
        );
        assert!(
            (0.0..=1.0).contains(&risk.p_cascade),
            "P(cascade) must be in [0,1]: {:.4}",
            risk.p_cascade
        );
        assert!(
            (0.0..=1.0).contains(&risk.p_blackout),
            "P(blackout) must be in [0,1]: {:.4}",
            risk.p_blackout
        );
        assert!(
            risk.expected_annual_loss_mwh >= 0.0,
            "Expected annual loss must be non-negative"
        );
    }

    // ── Test 6: Anomaly detection — corrupted measurement detected at 3σ ─────

    #[test]
    fn test_anomaly_detection_at_3_sigma() {
        let sim = default_sim();

        // Normal measurements: all 1.0 pu
        let normal: Vec<f64> = vec![1.0; 20];

        // Current measurements: one large outlier at index 5 (> 3σ above mean)
        let mut current = normal.clone();
        current[5] = 1.0 + 20.0; // +20σ deviation

        let report = sim.detect_anomaly(&normal, &current);

        assert!(
            report.anomaly_detected,
            "Large measurement corruption (20σ) must be detected; chi²={:.2}, threshold={:.2}",
            report.chi_squared, report.threshold
        );
        assert!(
            report.suspicious_measurements.contains(&5),
            "Corrupted index 5 must be flagged; suspicious: {:?}",
            report.suspicious_measurements
        );
        assert!(
            report.chi_squared > report.threshold,
            "chi² must exceed threshold: {:.2} vs {:.2}",
            report.chi_squared,
            report.threshold
        );
    }

    // ── Test 7: Defense ROI — stronger defense gives better ROI when attacks are frequent ──

    #[test]
    fn test_defense_roi_stronger_is_better_roi() {
        let voltages = flat_voltages(10, 1.0);
        let loads = uniform_loads(10, 100.0);

        let scenarios = vec![(
            CpAttackScenario {
                scenario_id: "roi".into(),
                attacker_capability: 4,
                attack_vector: AttackVector::FalseDataInjection {
                    target_buses: vec![0, 1, 2, 3],
                    magnitude_pu: 0.4,
                },
                timing: AttackTiming::PeakLoad,
                objective: AttackObjective::MaximizeLoadShed,
            },
            voltages.clone(),
            loads.clone(),
        )];

        let mut sim = CyberPhysicalSim::new(CyberPhysicalSimConfig {
            num_buses: 10,
            monte_carlo_runs: 50,
            time_horizon_s: 3600.0,
            recovery_time_s: 600.0,
        });

        let roi_weak = sim.evaluate_defense_investment(
            DefenseLayer::Firewall { effectiveness: 0.3 },
            &scenarios,
            10_000.0,
            3_000.0,
        );

        let roi_strong = sim.evaluate_defense_investment(
            DefenseLayer::Firewall { effectiveness: 0.9 },
            &scenarios,
            10_000.0,
            3_000.0,
        );

        assert!(
            roi_strong.risk_reduction_pct >= roi_weak.risk_reduction_pct,
            "Stronger defense should reduce risk more: strong={:.2}% vs weak={:.2}%",
            roi_strong.risk_reduction_pct,
            roi_weak.risk_reduction_pct
        );
    }

    // ── Test 8: Resilience — no attacks → resilience_index = 1.0 ─────────────

    #[test]
    fn test_resilience_no_attacks_perfect_index() {
        let sim = default_sim();
        let metrics = sim.resilience_metrics(&[]);

        assert!(
            (metrics.resilience_index - 1.0).abs() < 1e-9,
            "No attacks → resilience_index must be 1.0; got {:.6}",
            metrics.resilience_index
        );
        assert!(
            (metrics.absorptive_capacity - 1.0).abs() < 1e-9,
            "absorptive_capacity must be 1.0 with no attacks"
        );
        assert!(
            (metrics.adaptive_capacity - 1.0).abs() < 1e-9,
            "adaptive_capacity must be 1.0 with no attacks"
        );
        assert!(
            (metrics.restorative_capacity - 1.0).abs() < 1e-9,
            "restorative_capacity must be 1.0 with no attacks"
        );
    }

    // ── Test 9: Physical security defense layer effectiveness ─────────────────

    #[test]
    fn test_physical_security_layer_effectiveness() {
        let low = DefenseLayer::PhysicalSecurity {
            protection_level: 1,
        };
        let high = DefenseLayer::PhysicalSecurity {
            protection_level: 5,
        };

        assert!(
            high.layer_effectiveness() > low.layer_effectiveness(),
            "Higher protection level must have higher effectiveness: {:.3} vs {:.3}",
            high.layer_effectiveness(),
            low.layer_effectiveness()
        );
        assert!(
            (low.layer_effectiveness() - 0.0).abs() < 1e-9,
            "Level 1 protection should have 0 effectiveness"
        );
        assert!(
            (high.layer_effectiveness() - 1.0).abs() < 1e-9,
            "Level 5 protection should have 1.0 effectiveness"
        );
    }

    // ── Test 10: Resilience — failed attacks are counted as mitigated ─────────

    #[test]
    fn test_resilience_all_failed_attacks_max_adaptive() {
        let sim = default_sim();

        // All attacks failed (attack_success = false)
        let history: Vec<(AttackImpactResult, f64)> = (0..5)
            .map(|_| {
                (
                    AttackImpactResult {
                        attack_success: false,
                        penetration_probability: 0.1,
                        load_shed_mw: 0.0,
                        voltage_violations: 0,
                        frequency_deviation_hz: 0.0,
                        cascade_triggered: false,
                        recovery_time_s: 0.0,
                        impact_severity: ImpactSeverity::Negligible,
                    },
                    1000.0,
                )
            })
            .collect();

        let metrics = sim.resilience_metrics(&history);

        assert!(
            (metrics.adaptive_capacity - 1.0).abs() < 1e-9,
            "All failed attacks → adaptive_capacity must be 1.0; got {:.6}",
            metrics.adaptive_capacity
        );
        assert!(
            (metrics.absorptive_capacity - 1.0).abs() < 1e-9,
            "No load shed → absorptive_capacity must be 1.0; got {:.6}",
            metrics.absorptive_capacity
        );
    }
}
