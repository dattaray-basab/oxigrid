//! Renewable Integration Study Platform.
//!
//! Provides a structured workflow for evaluating the technical and economic
//! impacts of large-scale renewable energy integration into a power system.
//!
//! # Key Metrics Computed
//!
//! | Metric | Description |
//! |--------|-------------|
//! | `hosting_capacity_mw` | Maximum renewable \[MW\] the grid can absorb |
//! | `annual_curtailment_gwh` | Energy curtailed due to excess generation \[GWh/year\] |
//! | `system_inertia_mjs` | Total kinetic energy of synchronous machines \[MJ\] |
//! | `min_scr` | Short Circuit Ratio at the weakest renewable bus |
//! | `frequency_nadir_hz` | Worst-case frequency dip after largest contingency \[Hz\] |
//! | `rocof_hz_per_s` | Rate-of-change-of-frequency at event inception \[Hz/s\] |
//! | `merit_score` | Composite 0–100 integration viability score |

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the integration study platform.
#[derive(Debug, Error)]
pub enum StudyError {
    /// The scenario contains invalid or self-contradictory data.
    #[error("invalid scenario: {0}")]
    InvalidScenario(String),

    /// A numerical computation failed.
    #[error("computation error: {0}")]
    ComputationError(String),

    /// No buses have been configured in the base network.
    #[error("base network is empty — add buses before running a study")]
    EmptyNetwork,
}

// ── Public types ───────────────────────────────────────────────────────────────

/// Which study metrics to compute.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StudyMetric {
    /// Maximum hosting capacity of the feeder/substation.
    HostingCapacity,
    /// Annual energy curtailed.
    Curtailment,
    /// Frequency response (nadir, ROCOF).
    FrequencyResponse,
    /// Voltage stability margin.
    VoltageStability,
    /// Impact on existing protection devices.
    ProtectionImpact,
    /// Wholesale market price impact.
    MarketImpact,
    /// Transmission corridor congestion.
    TransmissionCongestion,
}

/// Configuration for an integration study campaign.
#[derive(Debug, Clone)]
pub struct IntegrationStudyConfig {
    /// Descriptive name for the study.
    pub study_name: String,
    /// Target renewable penetration percentage (0–100).
    pub target_renewable_pct: f64,
    /// Calendar years included in the study.
    pub study_years: Vec<usize>,
    /// Metrics to evaluate.
    pub metrics: Vec<StudyMetric>,
}

/// One renewable deployment scenario.
#[derive(Debug, Clone)]
pub struct RenewableScenario {
    /// Reference year.
    pub year: usize,
    /// Solar capacity additions: `(bus_index, capacity_mw)`.
    pub solar_mw: Vec<(usize, f64)>,
    /// Wind capacity additions: `(bus_index, capacity_mw)`.
    pub wind_mw: Vec<(usize, f64)>,
    /// Battery storage additions: `(bus_index, energy_mwh)`.
    pub battery_mwh: Vec<(usize, f64)>,
    /// Conventional generation retired from service \[MW\].
    pub retired_conventional_mw: f64,
    /// Load growth relative to base year \[%\].
    pub load_growth_pct: f64,
}

/// Results of one scenario study.
#[derive(Debug, Clone)]
pub struct StudyResult {
    /// The scenario that was evaluated.
    pub scenario: RenewableScenario,
    /// Maximum renewable capacity the grid can accommodate \[MW\].
    pub hosting_capacity_mw: f64,
    /// Annual energy curtailed due to over-generation \[GWh\].
    pub annual_curtailment_gwh: f64,
    /// Curtailment as a percentage of total renewable generation.
    pub curtailment_pct: f64,
    /// Total synchronous machine kinetic energy \[MJ\].
    pub system_inertia_mjs: f64,
    /// Short Circuit Ratio at the weakest renewable bus.
    pub min_scr: f64,
    /// Estimated worst-case frequency nadir \[Hz\].
    pub frequency_nadir_hz: f64,
    /// Estimated worst-case rate-of-change-of-frequency \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// Hours per year with voltage violations.
    pub voltage_violations: usize,
    /// Hours per year with thermal overloads.
    pub thermal_violations: usize,
    /// Annual transmission congestion cost \[M$\].
    pub congestion_cost_m_usd: f64,
    /// Estimated grid upgrade cost \[M$\].
    pub integration_cost_m_usd: f64,
    /// Composite integration viability score (0–100; higher is better).
    pub merit_score: f64,
}

/// Central orchestrator for renewable integration studies.
pub struct IntegrationStudyPlatform {
    #[allow(dead_code)]
    config: IntegrationStudyConfig,
    base_network_buses: usize,
    base_network_load_mw: Vec<f64>,
    /// `(bus_fraction, rated_mw, inertia_constant_s)` for each synchronous machine.
    conventional_generators: Vec<(f64, f64, f64)>,
}

impl IntegrationStudyPlatform {
    /// Create a new platform with the given configuration and base network.
    ///
    /// `base_load_mw` should have one entry per bus (load at that bus \[MW\]).
    pub fn new(config: IntegrationStudyConfig, n_buses: usize, base_load_mw: Vec<f64>) -> Self {
        Self {
            config,
            base_network_buses: n_buses,
            base_network_load_mw: base_load_mw,
            conventional_generators: Vec::new(),
        }
    }

    /// Register a conventional synchronous generator.
    ///
    /// - `bus_pct` — bus position as fraction of total buses (0.0–1.0).
    /// - `p_mw` — rated active power \[MW\].
    /// - `h_s` — inertia constant \[s\].
    pub fn add_conventional_generator(&mut self, bus_pct: f64, p_mw: f64, h_s: f64) {
        self.conventional_generators.push((bus_pct, p_mw, h_s));
    }

    /// Run a full integration study for one scenario.
    pub fn study_scenario(&self, scenario: &RenewableScenario) -> Result<StudyResult, StudyError> {
        if self.base_network_buses == 0 {
            return Err(StudyError::EmptyNetwork);
        }

        let total_base_load: f64 = self.base_network_load_mw.iter().sum();
        let load_growth_factor = 1.0 + scenario.load_growth_pct / 100.0;
        let total_load = total_base_load * load_growth_factor;

        // ── Total renewable capacity ──────────────────────────────────────
        let total_solar: f64 = scenario.solar_mw.iter().map(|(_, mw)| mw).sum();
        let total_wind: f64 = scenario.wind_mw.iter().map(|(_, mw)| mw).sum();
        let total_renewable_mw = total_solar + total_wind;

        // ── Hosting capacity ──────────────────────────────────────────────
        // Conservative: grid can absorb up to 120 % of load before requiring
        // major infrastructure upgrades.
        let hosting_capacity_mw = total_renewable_mw.min(total_load * 1.2);

        // ── Curtailment ───────────────────────────────────────────────────
        let renewable_fraction = total_renewable_mw / total_load.max(1.0);
        let curtailment_pct = if renewable_fraction > 0.8 {
            ((renewable_fraction - 0.8) * 50.0).min(30.0)
        } else {
            0.0
        };
        let annual_curtailment_gwh =
            hosting_capacity_mw * (curtailment_pct / 100.0) * 8760.0 / 1000.0;

        // ── System inertia ────────────────────────────────────────────────
        // Reduce inertia proportionally to retired capacity.
        let total_conventional_mw: f64 =
            self.conventional_generators.iter().map(|(_, p, _)| p).sum();
        let retired_fraction = if total_conventional_mw > 1e-9 {
            (scenario.retired_conventional_mw / total_conventional_mw).min(1.0)
        } else {
            0.0
        };
        let system_inertia_mjs: f64 = self
            .conventional_generators
            .iter()
            .map(|(_, p, h)| p * h * (1.0 - retired_fraction))
            .sum();

        // ── Short Circuit Ratio ───────────────────────────────────────────
        let remaining_conventional = total_conventional_mw * (1.0 - retired_fraction);
        let min_scr = if total_renewable_mw > 1e-9 {
            (remaining_conventional / total_renewable_mw).clamp(0.0, 10.0)
        } else {
            10.0 // no renewable → no SCR issue
        };

        // ── Frequency metrics (swing equation) ───────────────────────────
        // ROCOF = ΔP / (2H) where H = system inertia / total MW.
        let total_mw = total_load.max(1.0);
        let h_pu = system_inertia_mjs / total_mw; // inertia constant [s]
        let h_pu = h_pu.max(0.1); // avoid division by zero
                                  // Largest contingency: 10 % of remaining conventional.
        let delta_p_mw = remaining_conventional * 0.10;
        let rocof_hz_per_s = delta_p_mw / (2.0 * h_pu * total_mw / 100.0).max(1.0);
        // Simplified nadir model: 0.5 s primary response.
        let frequency_nadir_hz = 60.0 - rocof_hz_per_s * 0.5;

        // ── Voltage / thermal violations ──────────────────────────────────
        let voltage_violations: usize = if renewable_fraction > 0.6 {
            ((renewable_fraction - 0.6) * 1000.0) as usize
        } else {
            0
        };
        let thermal_violations: usize = if renewable_fraction > 0.7 {
            ((renewable_fraction - 0.7) * 500.0) as usize
        } else {
            0
        };

        // ── Economic ─────────────────────────────────────────────────────
        let congestion_cost_m_usd = thermal_violations as f64 * 0.01;
        let integration_cost_m_usd = total_renewable_mw * 0.05;

        // ── Merit score ───────────────────────────────────────────────────
        let mut score = 100.0_f64;
        score -= curtailment_pct * 1.5;
        if system_inertia_mjs < 3000.0 {
            score -= (3000.0 - system_inertia_mjs) * 0.005;
        }
        if min_scr < 3.0 {
            score -= (3.0 - min_scr) * 5.0;
        }
        if voltage_violations > 100 {
            score -= (voltage_violations as f64 - 100.0) * 0.01;
        }
        let merit_score = score.clamp(0.0, 100.0);

        Ok(StudyResult {
            scenario: scenario.clone(),
            hosting_capacity_mw,
            annual_curtailment_gwh,
            curtailment_pct,
            system_inertia_mjs,
            min_scr,
            frequency_nadir_hz,
            rocof_hz_per_s,
            voltage_violations,
            thermal_violations,
            congestion_cost_m_usd,
            integration_cost_m_usd,
            merit_score,
        })
    }

    /// Run the study for every scenario and return results in order.
    pub fn run_all_scenarios(
        &self,
        scenarios: Vec<RenewableScenario>,
    ) -> Result<Vec<StudyResult>, StudyError> {
        scenarios.iter().map(|s| self.study_scenario(s)).collect()
    }

    /// Recommend grid upgrades based on study results.
    pub fn recommend_upgrades(&self, results: &[StudyResult]) -> Vec<String> {
        let mut recs: Vec<String> = Vec::new();

        if results.iter().any(|r| r.curtailment_pct > 10.0) {
            recs.push("Install battery storage to absorb excess renewable generation".to_string());
        }
        if results.iter().any(|r| r.system_inertia_mjs < 2000.0) {
            recs.push(
                "Add synchronous condensers or virtual inertia for frequency support".to_string(),
            );
        }
        if results.iter().any(|r| r.min_scr < 2.0) {
            recs.push(
                "Strengthen transmission network near renewable interconnection points".to_string(),
            );
        }
        if results.iter().any(|r| r.voltage_violations > 500) {
            recs.push("Deploy dynamic VAR compensation (SVCs or STATCOMs)".to_string());
        }
        if results.iter().any(|r| r.thermal_violations > 200) {
            recs.push("Upgrade transmission lines to handle increased renewable flows".to_string());
        }
        if results.iter().any(|r| r.congestion_cost_m_usd > 1.0) {
            recs.push("Build new transmission capacity to reduce congestion".to_string());
        }

        recs
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn base_platform() -> IntegrationStudyPlatform {
        let cfg = IntegrationStudyConfig {
            study_name: "Test Study".to_string(),
            target_renewable_pct: 50.0,
            study_years: vec![2025, 2030],
            metrics: vec![StudyMetric::HostingCapacity, StudyMetric::Curtailment],
        };
        let load = vec![100.0_f64; 5]; // 5 buses × 100 MW = 500 MW
        let mut platform = IntegrationStudyPlatform::new(cfg, 5, load);
        // Two conventional generators: 200 MW each, H=5 s.
        platform.add_conventional_generator(0.2, 200.0, 5.0);
        platform.add_conventional_generator(0.8, 200.0, 5.0);
        platform
    }

    fn low_pen_scenario() -> RenewableScenario {
        RenewableScenario {
            year: 2025,
            solar_mw: vec![(0, 50.0)],
            wind_mw: vec![],
            battery_mwh: vec![],
            retired_conventional_mw: 0.0,
            load_growth_pct: 0.0,
        }
    }

    fn high_pen_scenario() -> RenewableScenario {
        RenewableScenario {
            year: 2030,
            solar_mw: vec![(0, 300.0)],
            wind_mw: vec![(1, 350.0)],
            battery_mwh: vec![],
            retired_conventional_mw: 150.0,
            load_growth_pct: 5.0,
        }
    }

    #[test]
    fn test_low_penetration_high_inertia() {
        let p = base_platform();
        let r = p
            .study_scenario(&low_pen_scenario())
            .expect("study_scenario");
        assert!(
            r.system_inertia_mjs > 0.0,
            "inertia should be positive, got {}",
            r.system_inertia_mjs
        );
        assert_eq!(
            r.curtailment_pct, 0.0,
            "low penetration should have zero curtailment"
        );
    }

    #[test]
    fn test_high_penetration_curtailment_rises() {
        let p = base_platform();
        let r = p
            .study_scenario(&high_pen_scenario())
            .expect("study_scenario");
        assert!(
            r.curtailment_pct > 0.0,
            "high penetration should have positive curtailment, got {}",
            r.curtailment_pct
        );
    }

    #[test]
    fn test_frequency_nadir_below_nominal() {
        let p = base_platform();
        let r = p
            .study_scenario(&high_pen_scenario())
            .expect("study_scenario");
        assert!(
            r.frequency_nadir_hz < 60.0,
            "nadir must be below 60 Hz, got {}",
            r.frequency_nadir_hz
        );
    }

    #[test]
    fn test_merit_score_decreases_high_penetration() {
        let p = base_platform();
        let low = p.study_scenario(&low_pen_scenario()).expect("low");
        let high = p.study_scenario(&high_pen_scenario()).expect("high");
        assert!(
            high.merit_score <= low.merit_score,
            "high penetration merit ({}) should be ≤ low penetration merit ({})",
            high.merit_score,
            low.merit_score
        );
    }

    #[test]
    fn test_recommendations_generated_for_high_penetration() {
        let p = base_platform();
        let results = vec![p.study_scenario(&high_pen_scenario()).expect("high")];
        let recs = p.recommend_upgrades(&results);
        assert!(
            !recs.is_empty(),
            "high-penetration scenario should generate at least one recommendation"
        );
    }

    // ── 7 new tests ────────────────────────────────────────────────────────────

    /// Test 1: An empty network (n_buses = 0) must return `StudyError::EmptyNetwork`.
    #[test]
    fn test_empty_network_returns_error() {
        let cfg = IntegrationStudyConfig {
            study_name: "Empty Network Test".to_string(),
            target_renewable_pct: 50.0,
            study_years: vec![2025],
            metrics: vec![StudyMetric::HostingCapacity],
        };
        let platform = IntegrationStudyPlatform::new(cfg, 0, vec![]);
        let result = platform.study_scenario(&low_pen_scenario());
        match result {
            Err(StudyError::EmptyNetwork) => {}
            other => panic!("expected Err(StudyError::EmptyNetwork), got {:?}", other),
        }
    }

    /// Test 2: With zero renewable capacity the SCR is clamped to 10.0,
    /// curtailment is exactly 0 %, and the frequency nadir is close to 60 Hz.
    #[test]
    fn test_zero_renewable_gives_clamped_scr_and_no_curtailment() {
        let p = base_platform();
        let scenario = RenewableScenario {
            year: 2025,
            solar_mw: vec![],
            wind_mw: vec![],
            battery_mwh: vec![],
            retired_conventional_mw: 0.0,
            load_growth_pct: 0.0,
        };
        let r = p
            .study_scenario(&scenario)
            .expect("study_scenario for zero-renewable case");
        assert_eq!(
            r.curtailment_pct, 0.0,
            "zero renewable must have zero curtailment, got {}",
            r.curtailment_pct
        );
        assert!(
            (r.min_scr - 10.0).abs() < 1e-9,
            "zero renewable SCR must be clamped to 10.0, got {}",
            r.min_scr
        );
        // With no renewable penetration the nadir should be within 1 Hz of 60 Hz.
        assert!(
            r.frequency_nadir_hz > 59.0,
            "frequency nadir with zero renewables should be close to 60 Hz, got {}",
            r.frequency_nadir_hz
        );
    }

    /// Test 3: Retiring all conventional capacity drives system inertia to zero.
    #[test]
    fn test_full_conventional_retirement_zeroes_inertia() {
        let p = base_platform();
        // Both generators are 200 MW each → retire 400 MW total.
        let scenario = RenewableScenario {
            year: 2030,
            solar_mw: vec![(0, 100.0)],
            wind_mw: vec![],
            battery_mwh: vec![],
            retired_conventional_mw: 400.0,
            load_growth_pct: 0.0,
        };
        let r = p
            .study_scenario(&scenario)
            .expect("study_scenario for full retirement");
        assert!(
            r.system_inertia_mjs.abs() < 1e-9,
            "full conventional retirement must reduce inertia to zero, got {}",
            r.system_inertia_mjs
        );
    }

    /// Test 4: `run_all_scenarios` returns exactly one `StudyResult` per input scenario.
    #[test]
    fn test_run_all_scenarios_result_count_matches_input() {
        let p = base_platform();
        let scenarios = vec![low_pen_scenario(), high_pen_scenario()];
        let results = p
            .run_all_scenarios(scenarios)
            .expect("run_all_scenarios should succeed");
        assert_eq!(
            results.len(),
            2,
            "expected 2 results for 2 scenarios, got {}",
            results.len()
        );
    }

    /// Test 5: `recommend_upgrades` returns an empty vec when all metrics are healthy
    /// (low-penetration scenario, no thresholds breached).
    #[test]
    fn test_recommend_upgrades_empty_for_healthy_scenario() {
        let p = base_platform();
        let r = p
            .study_scenario(&low_pen_scenario())
            .expect("low pen study");
        let recs = p.recommend_upgrades(&[r]);
        assert!(
            recs.is_empty(),
            "no recommendations expected for a healthy low-penetration result, got {:?}",
            recs
        );
    }

    /// Test 6: When solar + wind >> load the hosting capacity is capped at 120 % of load.
    #[test]
    fn test_hosting_capacity_capped_at_120_pct_of_load() {
        let p = base_platform(); // base load = 500 MW → cap = 600 MW
        let scenario = RenewableScenario {
            year: 2025,
            solar_mw: vec![(0, 1000.0)], // 1000 MW solar …
            wind_mw: vec![(1, 500.0)],   // … + 500 MW wind = 1500 MW total, >> 600 MW cap
            battery_mwh: vec![],
            retired_conventional_mw: 0.0,
            load_growth_pct: 0.0,
        };
        let r = p
            .study_scenario(&scenario)
            .expect("study_scenario for oversized renewables");
        let expected_cap = 500.0_f64 * 1.2; // 600 MW
        assert!(
            (r.hosting_capacity_mw - expected_cap).abs() < 1e-9,
            "hosting capacity should be capped at {:.1} MW, got {:.6}",
            expected_cap,
            r.hosting_capacity_mw
        );
    }

    /// Test 7: A higher load-growth percentage lowers the renewable fraction for the same
    /// renewable MW, which must eliminate curtailment that would otherwise occur.
    #[test]
    fn test_high_load_growth_reduces_curtailment() {
        let p = base_platform();
        // 500 MW of renewables against 500 MW base load → fraction = 1.0 → curtailment.
        let scenario_no_growth = RenewableScenario {
            year: 2025,
            solar_mw: vec![(0, 300.0)],
            wind_mw: vec![(1, 200.0)],
            battery_mwh: vec![],
            retired_conventional_mw: 0.0,
            load_growth_pct: 0.0,
        };
        // Same renewable MW but load grows 200 % → effective load = 1500 MW → fraction ≈ 0.33.
        let scenario_high_growth = RenewableScenario {
            year: 2025,
            solar_mw: vec![(0, 300.0)],
            wind_mw: vec![(1, 200.0)],
            battery_mwh: vec![],
            retired_conventional_mw: 0.0,
            load_growth_pct: 200.0,
        };
        let r_no_growth = p
            .study_scenario(&scenario_no_growth)
            .expect("study_scenario no-growth");
        let r_high_growth = p
            .study_scenario(&scenario_high_growth)
            .expect("study_scenario high-growth");

        assert!(
            r_no_growth.curtailment_pct > 0.0,
            "zero load growth with 500 MW renewables / 500 MW load should curtail, got {}",
            r_no_growth.curtailment_pct
        );
        assert_eq!(
            r_high_growth.curtailment_pct, 0.0,
            "200 %% load growth should reduce renewable fraction enough to eliminate curtailment, got {}",
            r_high_growth.curtailment_pct
        );
    }
}
