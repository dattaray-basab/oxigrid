//! Auto-generated test module (consolidated from inline `#[cfg(test)] mod` blocks)

use super::*;

#[cfg(test)]
mod tests_2 {
    use super::*;
    fn make_config() -> IrpConfig {
        IrpConfig {
            planning_horizon_years: 5,
            base_year: 2025,
            discount_rate: 0.07,
            reserve_margin_pct: 15.0,
            co2_reduction_target_pct: 30.0,
            reliability_lole_h_per_yr: 3.0,
            budget_constraint_billion_eur: 500.0,
        }
    }
    fn make_forecasts(n: usize) -> Vec<PlanningLoadForecast> {
        (0..n)
            .map(|i| PlanningLoadForecast {
                year: 2025 + i,
                peak_load_mw: 5000.0 + i as f64 * 100.0,
                annual_energy_twh: 40.0 + i as f64 * 0.5,
                peak_demand_growth_pct: 2.0,
                der_penetration_pct: 5.0,
                ev_load_mw: 100.0,
                heat_pump_load_mw: 50.0,
            })
            .collect()
    }
    fn solar_option() -> ResourceOption {
        ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 200.0,
            capital_cost_million_eur: 140.0,
            opex_million_eur_per_yr: 2.0,
            capacity_factor: 0.22,
            variability_factor: 0.8,
            lifetime_years: 25,
        }
    }
    fn gas_baseload_option() -> ResourceOption {
        ResourceOption::BaseloadPlant {
            technology: "CCGT".to_string(),
            capacity_mw: 400.0,
            capital_cost_million_eur: 320.0,
            opex_million_eur_per_yr: 8.0,
            capacity_factor: 0.55,
            co2_kg_per_mwh: 400.0,
            lifetime_years: 30,
            build_time_years: 3,
        }
    }
    fn battery_option() -> ResourceOption {
        ResourceOption::EnergyStorage {
            technology: "Li-ion".to_string(),
            power_mw: 100.0,
            energy_mwh: 400.0,
            capital_cost_million_eur: 80.0,
            opex_million_eur_per_yr: 1.0,
            roundtrip_efficiency: 0.88,
            lifetime_years: 15,
        }
    }
    fn transmission_option() -> ResourceOption {
        ResourceOption::TransmissionUpgrade {
            from_bus: 1,
            to_bus: 5,
            capacity_increase_mw: 300.0,
            capital_cost_million_eur: 50.0,
            lifetime_years: 40,
        }
    }
    fn make_planner() -> IntegratedResourcePlanner {
        let opts = vec![
            solar_option(),
            gas_baseload_option(),
            battery_option(),
            transmission_option(),
        ];
        let forecasts = make_forecasts(5);
        let config = make_config();
        IntegratedResourcePlanner::new(opts, forecasts, config, 4500.0, 450.0)
    }
    #[test]
    fn test_irp_config_creation() {
        let cfg = make_config();
        assert_eq!(cfg.planning_horizon_years, 5);
        assert_eq!(cfg.base_year, 2025);
        assert!((cfg.discount_rate - 0.07).abs() < 1e-9);
        assert!((cfg.reserve_margin_pct - 15.0).abs() < 1e-9);
        assert!((cfg.co2_reduction_target_pct - 30.0).abs() < 1e-9);
    }
    #[test]
    fn test_load_forecast_creation() {
        let forecasts = make_forecasts(3);
        assert_eq!(forecasts.len(), 3);
        assert_eq!(forecasts[0].year, 2025);
        assert!((forecasts[0].peak_load_mw - 5000.0).abs() < 1e-9);
        assert!(forecasts[1].annual_energy_twh > forecasts[0].annual_energy_twh);
    }
    #[test]
    fn test_baseload_plant_option() {
        let opt = gas_baseload_option();
        assert!((opt.capacity_mw() - 400.0).abs() < 1e-9);
        assert!((opt.capacity_factor() - 0.55).abs() < 1e-9);
        assert!((opt.co2_kg_per_mwh() - 400.0).abs() < 1e-9);
        assert!(!opt.is_renewable());
        assert!(opt.is_dispatchable());
        assert_eq!(opt.lifetime_years(), 30);
    }
    #[test]
    fn test_renewable_option() {
        let opt = solar_option();
        assert!((opt.capacity_mw() - 200.0).abs() < 1e-9);
        assert!(opt.is_renewable());
        assert!(!opt.is_dispatchable());
        assert!((opt.co2_kg_per_mwh() - 0.0).abs() < 1e-9);
        assert_eq!(opt.lifetime_years(), 25);
    }
    #[test]
    fn test_storage_option() {
        let opt = battery_option();
        assert!((opt.capacity_mw() - 100.0).abs() < 1e-9);
        assert!(!opt.is_renewable());
        assert!(opt.is_dispatchable());
        let elcc = IntegratedResourcePlanner::compute_elcc(&opt);
        assert!((elcc - 95.0).abs() < 1e-9);
    }
    #[test]
    fn test_transmission_upgrade_option() {
        let opt = transmission_option();
        assert!((opt.capacity_mw() - 300.0).abs() < 1e-9);
        assert_eq!(opt.lifetime_years(), 40);
        assert!(!opt.is_renewable());
        assert!(!opt.is_dispatchable());
    }
    #[test]
    fn test_compute_lcoe_baseload() {
        let planner = make_planner();
        let opt = gas_baseload_option();
        let lcoe = planner.compute_lcoe(&opt, 2025);
        assert!(lcoe > 5.0, "LCOE too low: {lcoe}");
        assert!(lcoe < 500.0, "LCOE too high: {lcoe}");
    }
    #[test]
    fn test_compute_lcoe_renewable() {
        let planner = make_planner();
        let opt = solar_option();
        let lcoe = planner.compute_lcoe(&opt, 2025);
        assert!(lcoe > 10.0, "LCOE too low: {lcoe}");
        assert!(lcoe < 500.0, "LCOE too high: {lcoe}");
    }
    #[test]
    fn test_compute_cba_positive_bcr() {
        let planner = make_planner();
        let cba = planner.compute_cba(0, 2025);
        assert!(cba.bcr > 0.0, "BCR should be positive, got {}", cba.bcr);
        assert!(cba.npv_benefit_million_eur >= 0.0);
        assert!(cba.npv_cost_million_eur >= 0.0);
    }
    #[test]
    fn test_compute_cba_negative_bcr() {
        let expensive = ResourceOption::BaseloadPlant {
            technology: "Exotic".to_string(),
            capacity_mw: 1.0,
            capital_cost_million_eur: 50_000.0,
            opex_million_eur_per_yr: 1_000.0,
            capacity_factor: 0.001,
            co2_kg_per_mwh: 0.0,
            lifetime_years: 5,
            build_time_years: 1,
        };
        let opts = vec![expensive];
        let forecasts = make_forecasts(5);
        let config = make_config();
        let planner = IntegratedResourcePlanner::new(opts, forecasts, config, 1000.0, 100.0);
        let cba = planner.compute_cba(0, 2025);
        assert!(
            cba.bcr < 1.0,
            "Expected BCR < 1.0 for prohibitively expensive option, got {}",
            cba.bcr
        );
    }
    #[test]
    fn test_greedy_optimization_basic() {
        let mut planner = make_planner();
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize should succeed");
        assert!(!result.annual_snapshots.is_empty());
        assert_eq!(result.annual_snapshots.len(), 5);
        for w in result.annual_snapshots.windows(2) {
            assert!(w[1].year > w[0].year);
        }
    }
    #[test]
    fn test_greedy_meets_reserve_margin() {
        let opts = vec![gas_baseload_option(), solar_option()];
        let forecasts = make_forecasts(3);
        let config = IrpConfig {
            planning_horizon_years: 3,
            reserve_margin_pct: 10.0,
            budget_constraint_billion_eur: 1000.0,
            ..make_config()
        };
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 5000.0, 500.0);
        let result = planner.optimize_greedy().expect("should succeed");
        let last = result.annual_snapshots.last().expect("has snapshots");
        assert!(
            last.installed_capacity_mw >= last.peak_demand_mw,
            "capacity {} < peak {}",
            last.installed_capacity_mw,
            last.peak_demand_mw
        );
    }
    #[test]
    fn test_greedy_co2_reduction() {
        let opts = vec![solar_option()];
        let forecasts = make_forecasts(5);
        let config = make_config();
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 2000.0, 600.0);
        let result = planner.optimize_greedy().expect("should succeed");
        let final_snap = result.annual_snapshots.last().expect("has snapshots");
        assert!(
            final_snap.co2_intensity_kg_per_mwh <= 600.0 + 1e-6,
            "CO₂ intensity should not increase, got {}",
            final_snap.co2_intensity_kg_per_mwh
        );
    }
    #[test]
    fn test_generate_alternatives_3_portfolios() {
        let mut planner = make_planner();
        let alts = planner.generate_alternatives();
        assert_eq!(
            alts.len(),
            3,
            "Should generate exactly 3 alternative portfolios"
        );
        for p in &alts {
            assert!(p.total_capacity_mw >= 0.0);
            assert!(p.total_renewable_pct >= 0.0 && p.total_renewable_pct <= 100.0);
        }
    }
    #[test]
    fn test_lole_estimate_adequate() {
        let planner = make_planner();
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 7000.0,
            total_renewable_pct: 20.0,
            total_npv_cost_million_eur: 500.0,
            co2_reduction_pct: 20.0,
            reserve_margin_pct: 40.0,
            lole_estimate_h_per_yr: 0.0,
            meets_reliability: true,
            meets_co2_target: false,
            meets_budget: true,
        };
        let lole = planner.estimate_lole(&portfolio, 2025);
        assert!(
            lole < 100.0,
            "LOLE should be low for adequate capacity: {lole}"
        );
    }
    #[test]
    fn test_lole_estimate_inadequate() {
        let planner = make_planner();
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 3000.0,
            total_renewable_pct: 0.0,
            total_npv_cost_million_eur: 100.0,
            co2_reduction_pct: 0.0,
            reserve_margin_pct: -40.0,
            lole_estimate_h_per_yr: 0.0,
            meets_reliability: false,
            meets_co2_target: false,
            meets_budget: true,
        };
        let lole = planner.estimate_lole(&portfolio, 2025);
        assert!(
            lole > 1000.0,
            "LOLE should be very high for inadequate capacity: {lole}"
        );
    }
    #[test]
    fn test_npv_calculation() {
        let cashflows = vec![100.0, 100.0, 100.0];
        let npv = IntegratedResourcePlanner::npv(&cashflows, 0.10);
        assert!((npv - 248.685).abs() < 0.1, "NPV calculation wrong: {npv}");
        let npv_zero = IntegratedResourcePlanner::npv(&cashflows, 0.0);
        assert!(
            (npv_zero - 300.0).abs() < 1e-6,
            "NPV at 0% should be 300: {npv_zero}"
        );
    }
    #[test]
    fn test_esia_solar_assessment() {
        let opt = ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 100.0,
            capital_cost_million_eur: 70.0,
            opex_million_eur_per_yr: 1.0,
            capacity_factor: 0.22,
            variability_factor: 0.8,
            lifetime_years: 25,
        };
        let esia = EsiaAssessment::assess(&opt, 0, false);
        assert!((esia.land_use_km2 - 1.0).abs() < 1e-9);
        assert!((esia.water_consumption_m3_per_mwh - 0.001).abs() < 1e-9);
        assert!((esia.noise_level_db - 35.0).abs() < 1e-9);
        assert_eq!(esia.visual_impact, VisualImpact::Low);
        assert!((esia.biodiversity_impact - 2.0).abs() < 1e-9);
        assert!((esia.jobs_permanent - 10.0).abs() < 1e-9);
    }
    #[test]
    fn test_esia_wind_assessment() {
        let opt = ResourceOption::RenewableResource {
            technology: "wind_onshore".to_string(),
            capacity_mw: 200.0,
            capital_cost_million_eur: 260.0,
            opex_million_eur_per_yr: 6.0,
            capacity_factor: 0.35,
            variability_factor: 0.6,
            lifetime_years: 25,
        };
        let esia = EsiaAssessment::assess(&opt, 1, false);
        assert!((esia.land_use_km2 - 10.0).abs() < 1e-9);
        assert!((esia.noise_level_db - 45.0).abs() < 1e-9);
        assert_eq!(esia.visual_impact, VisualImpact::Medium);
        assert!((esia.biodiversity_impact - 4.0).abs() < 1e-9);
        let esia_urban = EsiaAssessment::assess(&opt, 1, true);
        assert!((esia_urban.noise_level_db - 50.0).abs() < 1e-9);
        assert_eq!(esia_urban.visual_impact, VisualImpact::High);
    }
    #[test]
    fn test_mcda_balanced_scoring() {
        let mcda = McdaAnalysis::new(McdaWeights::balanced());
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 6000.0,
            total_renewable_pct: 40.0,
            total_npv_cost_million_eur: 500.0,
            co2_reduction_pct: 35.0,
            reserve_margin_pct: 20.0,
            lole_estimate_h_per_yr: 2.0,
            meets_reliability: true,
            meets_co2_target: true,
            meets_budget: true,
        };
        let esia = vec![EsiaAssessment::assess(&solar_option(), 0, false)];
        let score = mcda.score_portfolio(&portfolio, &esia);
        assert!((0.0..=1.0).contains(&score), "Score out of range: {score}");
        assert!(
            score > 0.3,
            "Score should be reasonable for a good portfolio: {score}"
        );
    }
    #[test]
    fn test_mcda_rank_portfolios() {
        let mcda = McdaAnalysis::new(McdaWeights::balanced());
        let good_portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 6000.0,
            total_renewable_pct: 60.0,
            total_npv_cost_million_eur: 200.0,
            co2_reduction_pct: 50.0,
            reserve_margin_pct: 20.0,
            lole_estimate_h_per_yr: 1.0,
            meets_reliability: true,
            meets_co2_target: true,
            meets_budget: true,
        };
        let bad_portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 4000.0,
            total_renewable_pct: 5.0,
            total_npv_cost_million_eur: 10_000.0,
            co2_reduction_pct: 2.0,
            reserve_margin_pct: -10.0,
            lole_estimate_h_per_yr: 50.0,
            meets_reliability: false,
            meets_co2_target: false,
            meets_budget: false,
        };
        let portfolios = vec![bad_portfolio, good_portfolio];
        let esia_data: Vec<Vec<EsiaAssessment>> = vec![
            vec![EsiaAssessment::assess(&gas_baseload_option(), 0, false)],
            vec![EsiaAssessment::assess(&solar_option(), 0, false)],
        ];
        let ranked = mcda.rank_portfolios(&portfolios, &esia_data);
        assert_eq!(ranked.len(), 2);
        assert_eq!(ranked[0].0, 1, "Good portfolio should be ranked first");
        assert!(
            ranked[0].1 > ranked[1].1,
            "First-ranked score should be higher"
        );
    }
    /// Verify that the greedy optimizer's final portfolio meets or exceeds the
    /// renewable penetration level implied by choosing only renewable options.
    #[test]
    fn test_resource_mix_renewable_fraction_nonzero() {
        let opts = vec![solar_option(), gas_baseload_option()];
        let forecasts = make_forecasts(3);
        let config = make_config();
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 4000.0, 600.0);
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize must succeed");
        assert!(
            result.portfolio.total_renewable_pct >= 0.0,
            "Renewable fraction must be non-negative, got {}",
            result.portfolio.total_renewable_pct
        );
        assert!(
            result.portfolio.total_capacity_mw >= 4000.0,
            "Total capacity should not shrink, got {}",
            result.portfolio.total_capacity_mw
        );
    }
    /// Total portfolio NPV cost must be non-negative (costs cannot be negative).
    #[test]
    fn test_total_cost_nonnegative() {
        let mut planner = make_planner();
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize must succeed");
        assert!(
            result.portfolio.total_npv_cost_million_eur >= 0.0,
            "Total NPV cost must be non-negative, got {}",
            result.portfolio.total_npv_cost_million_eur
        );
    }
    /// When only zero-CO₂ renewables are available and existing CO₂ intensity
    /// is non-zero, the final CO₂ reduction percentage must be > 0.
    #[test]
    fn test_renewable_penetration_reduces_co2() {
        let opts = vec![solar_option()];
        let forecasts = make_forecasts(5);
        let config = make_config();
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 3000.0, 500.0);
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize must succeed");
        assert!(
            result.portfolio.co2_reduction_pct >= 0.0,
            "CO₂ reduction must be ≥ 0, got {}",
            result.portfolio.co2_reduction_pct
        );
    }
    /// Each yearly snapshot's installed capacity must be ≥ peak demand when
    /// `capacity_adequacy` is true.
    #[test]
    fn test_peak_demand_coverage_consistency() {
        let mut planner = make_planner();
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize must succeed");
        for snap in &result.annual_snapshots {
            if snap.capacity_adequacy {
                assert!(
                    snap.installed_capacity_mw >= snap.peak_demand_mw,
                    "Year {}: capacity {} < peak {} but capacity_adequacy=true",
                    snap.year,
                    snap.installed_capacity_mw,
                    snap.peak_demand_mw
                );
            }
        }
    }
    /// When the existing capacity far exceeds peak demand and budget is large,
    /// the portfolio's reliability flag should be true.
    #[test]
    fn test_reliability_requirement_satisfied_with_excess_capacity() {
        let opts = vec![gas_baseload_option()];
        let forecasts = make_forecasts(3);
        let config = IrpConfig {
            planning_horizon_years: 3,
            reserve_margin_pct: 5.0,
            reliability_lole_h_per_yr: 100.0,
            budget_constraint_billion_eur: 1000.0,
            ..make_config()
        };
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 8000.0, 200.0);
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize must succeed");
        assert!(
            result.portfolio.meets_reliability,
            "Should meet reliability with excess capacity; LOLE={}",
            result.portfolio.lole_estimate_h_per_yr
        );
    }
    /// Sensitivity analysis must return exactly 4 results (2 parameters × 2 variations).
    #[test]
    fn test_sensitivity_analysis_output_count() {
        let planner = make_planner();
        let portfolio = ResourcePortfolio {
            selected_options: vec![(0, 2025)],
            total_capacity_mw: 5500.0,
            total_renewable_pct: 10.0,
            total_npv_cost_million_eur: 300.0,
            co2_reduction_pct: 15.0,
            reserve_margin_pct: 10.0,
            lole_estimate_h_per_yr: 2.0,
            meets_reliability: true,
            meets_co2_target: false,
            meets_budget: true,
        };
        let sensitivity = planner.run_sensitivity(&portfolio);
        assert_eq!(
            sensitivity.len(),
            4,
            "Expected 4 sensitivity results (2 params × 2 variations), got {}",
            sensitivity.len()
        );
        for s in &sensitivity {
            assert!(
                !s.parameter.is_empty(),
                "Sensitivity result must have a non-empty parameter name"
            );
            assert!(
                s.variation_pct.abs() > 0.0,
                "Variation percentage must be non-zero"
            );
        }
    }
    /// When the CO₂ reduction achieved equals the target, `meets_co2_target`
    /// must be true; when well below, it must be false.
    #[test]
    fn test_co2_target_flag_correctness() {
        let meets = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 5500.0,
            total_renewable_pct: 30.0,
            total_npv_cost_million_eur: 400.0,
            co2_reduction_pct: 30.0,
            reserve_margin_pct: 10.0,
            lole_estimate_h_per_yr: 1.0,
            meets_reliability: true,
            meets_co2_target: true,
            meets_budget: true,
        };
        assert!(
            meets.meets_co2_target,
            "Portfolio with 30 % reduction should meet the 30 % target"
        );
        let misses = ResourcePortfolio {
            co2_reduction_pct: 5.0,
            meets_co2_target: false,
            ..meets.clone()
        };
        assert!(
            !misses.meets_co2_target,
            "Portfolio with only 5 % reduction should not meet the 30 % target"
        );
    }
    /// Verify the CBA cost curve shape: building the same technology sooner
    /// (lower delay) should have equal or higher NPV cost than later (delay
    /// factor < 1 discounts future cash flows more).
    #[test]
    fn test_cost_curve_earlier_build_higher_npv_cost() {
        let planner = make_planner();
        let cba_early = planner.compute_cba(1, 2025);
        let cba_late = planner.compute_cba(1, 2030);
        assert!(
            cba_early.npv_cost_million_eur >= cba_late.npv_cost_million_eur,
            "NPV cost for earlier build ({}) should be >= later build ({})",
            cba_early.npv_cost_million_eur,
            cba_late.npv_cost_million_eur
        );
    }
    /// A DemandResponse option has zero CO₂ and zero capital cost, so its LCOE
    /// must still be non-negative (no negative LCOE).
    #[test]
    fn test_demand_response_lcoe_nonnegative() {
        let dr_option = ResourceOption::DemandResponse {
            peak_reduction_mw: 150.0,
            annual_cost_million_eur: 5.0,
            response_time_min: 10.0,
        };
        let opts = vec![dr_option];
        let forecasts = make_forecasts(3);
        let config = make_config();
        let planner = IntegratedResourcePlanner::new(opts, forecasts, config, 5000.0, 300.0);
        let lcoe = planner.compute_lcoe(&planner.options[0], 2025);
        assert!(
            lcoe >= 0.0,
            "LCOE for DemandResponse must be non-negative, got {lcoe}"
        );
    }
    /// The MCDA green-focused weights must sum to approximately 1.0.
    #[test]
    fn test_mcda_green_focused_weights_sum_to_one() {
        let w = McdaWeights::green_focused();
        let total = w.cost + w.reliability + w.environment + w.social + w.flexibility;
        assert!(
            (total - 1.0).abs() < 1e-9,
            "Green-focused weights must sum to 1.0, got {total}"
        );
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;

    // Helper: build a minimal load-forecast vec for the planner constructor.
    fn single_year_forecast(year: usize, peak_mw: f64) -> Vec<PlanningLoadForecast> {
        vec![PlanningLoadForecast {
            year,
            peak_load_mw: peak_mw,
            annual_energy_twh: 5.0,
            peak_demand_growth_pct: 2.0,
            der_penetration_pct: 0.0,
            ev_load_mw: 0.0,
            heat_pump_load_mw: 0.0,
        }]
    }

    #[test]
    fn renewable_resource_is_renewable_true() {
        let opt = ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 100.0,
            capital_cost_million_eur: 80.0,
            opex_million_eur_per_yr: 1.0,
            capacity_factor: 0.20,
            variability_factor: 0.5,
            lifetime_years: 25,
        };
        assert!(opt.is_renewable());
    }

    #[test]
    fn baseload_plant_is_not_renewable() {
        let opt = ResourceOption::BaseloadPlant {
            technology: "ccgt".to_string(),
            capacity_mw: 400.0,
            capital_cost_million_eur: 500.0,
            opex_million_eur_per_yr: 10.0,
            capacity_factor: 0.85,
            co2_kg_per_mwh: 400.0,
            lifetime_years: 30,
            build_time_years: 3,
        };
        assert!(!opt.is_renewable());
    }

    #[test]
    fn baseload_and_storage_are_dispatchable() {
        let baseload = ResourceOption::BaseloadPlant {
            technology: "nuclear".to_string(),
            capacity_mw: 1000.0,
            capital_cost_million_eur: 5000.0,
            opex_million_eur_per_yr: 50.0,
            capacity_factor: 0.90,
            co2_kg_per_mwh: 12.0,
            lifetime_years: 60,
            build_time_years: 8,
        };
        let storage = ResourceOption::EnergyStorage {
            technology: "li-ion".to_string(),
            power_mw: 100.0,
            energy_mwh: 400.0,
            capital_cost_million_eur: 150.0,
            opex_million_eur_per_yr: 2.0,
            roundtrip_efficiency: 0.90,
            lifetime_years: 15,
        };
        assert!(baseload.is_dispatchable());
        assert!(storage.is_dispatchable());
    }

    #[test]
    fn renewable_resource_is_not_dispatchable() {
        let opt = ResourceOption::RenewableResource {
            technology: "wind".to_string(),
            capacity_mw: 200.0,
            capital_cost_million_eur: 300.0,
            opex_million_eur_per_yr: 5.0,
            capacity_factor: 0.35,
            variability_factor: 0.7,
            lifetime_years: 25,
        };
        assert!(!opt.is_dispatchable());
    }

    #[test]
    fn co2_stored_for_baseload_zero_for_renewable() {
        let baseload = ResourceOption::BaseloadPlant {
            technology: "coal".to_string(),
            capacity_mw: 500.0,
            capital_cost_million_eur: 800.0,
            opex_million_eur_per_yr: 20.0,
            capacity_factor: 0.80,
            co2_kg_per_mwh: 820.0,
            lifetime_years: 40,
            build_time_years: 4,
        };
        let renewable = ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 100.0,
            capital_cost_million_eur: 80.0,
            opex_million_eur_per_yr: 1.0,
            capacity_factor: 0.20,
            variability_factor: 0.5,
            lifetime_years: 25,
        };
        assert!((baseload.co2_kg_per_mwh() - 820.0).abs() < 1e-9);
        assert!(renewable.co2_kg_per_mwh().abs() < 1e-9);
    }

    #[test]
    fn lifetime_years_defaults_demand_response_20_distribution_30() {
        let dr = ResourceOption::DemandResponse {
            peak_reduction_mw: 50.0,
            annual_cost_million_eur: 1.0,
            response_time_min: 5.0,
        };
        let dist = ResourceOption::DistributionUpgrade {
            feeder_id: 1,
            capacity_increase_mw: 20.0,
            capital_cost_million_eur: 10.0,
            smart_grid: false,
        };
        assert_eq!(dr.lifetime_years(), 20);
        assert_eq!(dist.lifetime_years(), 30);
    }

    #[test]
    fn irp_config_default_horizon_and_discount() {
        let cfg = IrpConfig::default();
        assert_eq!(cfg.planning_horizon_years, 20);
        assert!((cfg.discount_rate - 0.07).abs() < 1e-9);
    }

    #[test]
    fn capacity_mw_dispatches_energy_storage_and_demand_response() {
        let storage = ResourceOption::EnergyStorage {
            technology: "bess".to_string(),
            power_mw: 75.0,
            energy_mwh: 300.0,
            capital_cost_million_eur: 100.0,
            opex_million_eur_per_yr: 1.5,
            roundtrip_efficiency: 0.88,
            lifetime_years: 15,
        };
        let dr = ResourceOption::DemandResponse {
            peak_reduction_mw: 40.0,
            annual_cost_million_eur: 0.8,
            response_time_min: 10.0,
        };
        assert!((storage.capacity_mw() - 75.0).abs() < 1e-9);
        assert!((dr.capacity_mw() - 40.0).abs() < 1e-9);
    }

    /// compute_lcoe with r=0 must equal capex*CRF / (cap*cf*8760) * 1e6
    /// where CRF = 1/n for r=0.
    #[test]
    fn compute_lcoe_zero_opex_zero_discount() {
        let baseload = ResourceOption::BaseloadPlant {
            technology: "gas".to_string(),
            capacity_mw: 100.0,
            capital_cost_million_eur: 200.0,
            opex_million_eur_per_yr: 0.0,
            capacity_factor: 0.80,
            co2_kg_per_mwh: 400.0,
            lifetime_years: 20,
            build_time_years: 2,
        };
        let config = IrpConfig {
            discount_rate: 0.0,
            ..IrpConfig::default()
        };
        let planner = IntegratedResourcePlanner::new(
            vec![baseload.clone()],
            single_year_forecast(2025, 500.0),
            config,
            0.0,
            500.0,
        );
        let lcoe = planner.compute_lcoe(&baseload, 2025);
        // CRF = 1/20, annual_energy = 100*0.8*8760 = 700800 MWh
        // annualised = 200e6 * (1/20) = 10e6 EUR
        // LCOE = 10e6 / 700800 ≈ 14.268 EUR/MWh
        let expected = 200.0 * (1.0 / 20.0) * 1_000_000.0 / (100.0 * 0.80 * 8760.0);
        assert!(
            (lcoe - expected).abs() < 0.01,
            "LCOE={lcoe:.4} expected={expected:.4}"
        );
    }
}
