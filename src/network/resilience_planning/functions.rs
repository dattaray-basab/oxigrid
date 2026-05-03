//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::ExtremeEventType;
use super::types_2::{FireSeverity, HazardSeverity, HazardType};

/// Compute the binomial coefficient C(n, k) with saturating arithmetic.
pub fn combinations(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result.saturating_mul((n - i) as u64);
        result /= (i + 1) as u64;
    }
    result
}
/// Value of lost load used in benefit calculations \[$/MWh\].
pub const VOLL_DOLLARS_PER_MWH: f64 = 10_000.0;
/// Map `HazardSeverity` to a dimensionless multiplicative factor.
pub fn severity_factor(severity: &HazardSeverity) -> f64 {
    match severity {
        HazardSeverity::Low => 0.1,
        HazardSeverity::Medium => 0.3,
        HazardSeverity::High => 0.6,
        HazardSeverity::Extreme => 1.0,
    }
}
/// Stable string key for deduplicating hazard types.
pub fn hazard_type_key(ht: &HazardType) -> &'static str {
    match ht {
        HazardType::Hurricane => "hurricane",
        HazardType::IceStorm => "ice_storm",
        HazardType::Wildfire => "wildfire",
        HazardType::Flood => "flood",
        HazardType::ExtremeHeat => "extreme_heat",
        HazardType::Earthquake => "earthquake",
    }
}
/// Advance one LCG step (Knuth multiplicative congruential generator).
#[inline]
pub fn ep_lcg_next(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64)
}
/// Convert a raw LCG state to a uniform sample in [0, 1).
#[inline]
pub fn ep_lcg_f64(state: u64) -> f64 {
    (state >> 11) as f64 / (1u64 << 53) as f64
}
/// Convert an [`ExtremeEventType`] to a canonical string key used in
/// failure probability maps.
///
/// # Examples
/// ```
/// use oxigrid::network::resilience_planning::{ExtremeEventType, extreme_event_key};
/// assert_eq!(extreme_event_key(&ExtremeEventType::Hurricane { category: 3 }), "Hurricane_3");
/// ```
pub fn extreme_event_key(event: &ExtremeEventType) -> String {
    match event {
        ExtremeEventType::Hurricane { category } => format!("Hurricane_{}", category),
        ExtremeEventType::IceStorm { ice_thickness_mm } => {
            if *ice_thickness_mm <= 5.0 {
                "IceStorm_5mm".to_string()
            } else if *ice_thickness_mm <= 25.0 {
                "IceStorm_25mm".to_string()
            } else {
                "IceStorm_50mm".to_string()
            }
        }
        ExtremeEventType::Wildfire { severity } => match severity {
            FireSeverity::Low => "Wildfire_Low".to_string(),
            FireSeverity::Moderate => "Wildfire_Moderate".to_string(),
            FireSeverity::High => "Wildfire_High".to_string(),
            FireSeverity::Extreme => "Wildfire_Extreme".to_string(),
        },
        ExtremeEventType::Earthquake { magnitude } => {
            if *magnitude < 5.0 {
                "Earthquake_Minor".to_string()
            } else if *magnitude < 7.0 {
                "Earthquake_Moderate".to_string()
            } else {
                "Earthquake_Major".to_string()
            }
        }
        ExtremeEventType::Flood { depth_m } => {
            if *depth_m < 1.0 {
                "Flood_Minor".to_string()
            } else if *depth_m < 3.0 {
                "Flood_Moderate".to_string()
            } else {
                "Flood_Major".to_string()
            }
        }
        ExtremeEventType::HeatWave { .. } => "HeatWave".to_string(),
        ExtremeEventType::CyberAttack { .. } => "CyberAttack".to_string(),
    }
}
/// Return a default failure probability for a component type given an event
/// key, based on empirical fragility data.
pub fn ep_default_failure_prob(component_type: &str, key: &str) -> f64 {
    match component_type {
        "overhead_line" => match key {
            "Hurricane_1" => 0.05,
            "Hurricane_2" => 0.10,
            "Hurricane_3" => 0.20,
            "Hurricane_4" => 0.40,
            "Hurricane_5" => 0.60,
            "IceStorm_5mm" => 0.05,
            "IceStorm_25mm" => 0.15,
            "IceStorm_50mm" => 0.35,
            "Wildfire_Low" => 0.05,
            "Wildfire_Moderate" => 0.15,
            "Wildfire_High" => 0.30,
            "Wildfire_Extreme" => 0.55,
            "Earthquake_Minor" => 0.02,
            "Earthquake_Moderate" => 0.10,
            "Earthquake_Major" => 0.25,
            "Flood_Minor" => 0.02,
            "Flood_Moderate" => 0.05,
            "Flood_Major" => 0.10,
            "HeatWave" => 0.02,
            "CyberAttack" => 0.01,
            _ => 0.05,
        },
        "substation" => match key {
            "Hurricane_1" => 0.01,
            "Hurricane_2" => 0.02,
            "Hurricane_3" => 0.05,
            "Hurricane_4" => 0.12,
            "Hurricane_5" => 0.25,
            "IceStorm_5mm" => 0.01,
            "IceStorm_25mm" => 0.03,
            "IceStorm_50mm" => 0.08,
            "Wildfire_Low" => 0.01,
            "Wildfire_Moderate" => 0.03,
            "Wildfire_High" => 0.05,
            "Wildfire_Extreme" => 0.10,
            "Earthquake_Minor" => 0.01,
            "Earthquake_Moderate" => 0.06,
            "Earthquake_Major" => 0.15,
            "Flood_Minor" => 0.03,
            "Flood_Moderate" => 0.10,
            "Flood_Major" => 0.20,
            "HeatWave" => 0.01,
            "CyberAttack" => 0.05,
            _ => 0.05,
        },
        "transformer" => (ep_default_failure_prob("substation", key) * 1.2).min(1.0),
        _ => 0.05,
    }
}
#[cfg(test)]
mod tests {
    use super::super::*;
    fn sample_trapezoid() -> ResiliencePlanningTrapezoid {
        ResiliencePlanningTrapezoid::new(0.0, 1.0, 3.0, 8.0, 1.0, 0.4, 0.9)
    }
    #[test]
    fn trapezoid_resilience_index_in_range() {
        let t = sample_trapezoid();
        let idx = t.resilience_index();
        assert!(idx >= 0.0, "index must be >= 0, got {idx}");
        assert!(idx <= 1.0, "index must be <= 1, got {idx}");
    }
    #[test]
    fn trapezoid_recovery_rate_positive() {
        let t = sample_trapezoid();
        assert!(t.recovery_rate() > 0.0, "recovery_rate should be positive");
    }
    #[test]
    fn trapezoid_absorptive_capacity_le_1() {
        let t = sample_trapezoid();
        let ac = t.absorptive_capacity();
        assert!(ac <= 1.0, "absorptive_capacity must be <= 1, got {ac}");
        assert!(ac >= 0.0);
    }
    #[test]
    fn trapezoid_rapidity_positive() {
        let t = sample_trapezoid();
        assert!(t.rapidity() > 0.0);
    }
    #[test]
    fn trapezoid_triangle_area_positive() {
        let t = sample_trapezoid();
        assert!(t.resilience_triangle_area() > 0.0);
    }
    #[test]
    fn trapezoid_restorative_capacity_correct() {
        let t = sample_trapezoid();
        let rc = t.restorative_capacity();
        assert!((rc - 5.0).abs() < 1e-10, "expected 5.0, got {rc}");
    }
    #[test]
    fn nk_combination_count_n10_k2_is_45() {
        let nk = NkContingencyAnalysis::new(10, 2);
        assert_eq!(nk.combination_count(), 45);
    }
    #[test]
    fn nk_combination_count_n5_k0_is_1() {
        let nk = NkContingencyAnalysis::new(5, 0);
        assert_eq!(nk.combination_count(), 1);
    }
    #[test]
    fn nk_most_critical_by_load_lost() {
        let nk = NkContingencyAnalysis::new(10, 1);
        let r1 = nk.analyze_contingency(&[1], 0.5);
        let r2 = nk.analyze_contingency(&[2], 0.9);
        let r3 = nk.analyze_contingency(&[3], 0.3);
        let critical = nk.most_critical_elements(&[r1, r2, r3], 2);
        assert_eq!(critical[0], 3);
        assert_eq!(critical[1], 1);
    }
    #[test]
    fn nk_system_adequacy_all_ok() {
        let nk = NkContingencyAnalysis::new(5, 1);
        let r1 = nk.analyze_contingency(&[0], 0.95);
        let r2 = nk.analyze_contingency(&[1], 0.90);
        let adequacy = nk.system_adequacy(&[r1, r2]);
        assert!(
            (adequacy - 1.0).abs() < 1e-10,
            "all non-critical → adequacy=1"
        );
    }
    #[test]
    fn nk_system_adequacy_partial() {
        let nk = NkContingencyAnalysis::new(5, 1);
        let r1 = nk.analyze_contingency(&[0], 0.95);
        let r2 = nk.analyze_contingency(&[1], 0.50);
        let adequacy = nk.system_adequacy(&[r1, r2]);
        assert!((adequacy - 0.5).abs() < 1e-10);
    }
    fn sample_element() -> GridElement {
        GridElement {
            id: 0,
            name: "Feeder-A".to_string(),
            element_type: ElementType::Distribution,
            failure_rate_per_year: 2.0,
            mttr_h: 4.0,
            load_served_mw: 5.0,
            hardening_options: vec![HardeningOption {
                name: "Underground cable".to_string(),
                cost: 50_000.0,
                failure_rate_reduction: 0.8,
                mttr_reduction: 0.5,
                residual_value_pct: 0.1,
            }],
        }
    }
    fn sample_optimizer() -> HardeningOptimizer {
        HardeningOptimizer {
            elements: vec![sample_element()],
            budget: 100_000.0,
            planning_horizon_years: 20.0,
            discount_rate: 0.05,
        }
    }
    #[test]
    fn hardening_ens_formula() {
        let opt = sample_optimizer();
        let el = &opt.elements[0];
        let ens = opt.expected_energy_not_served_mwh_per_year(el);
        assert!((ens - 40.0).abs() < 1e-10, "expected 40, got {ens}");
    }
    #[test]
    fn hardening_greedy_within_budget() {
        let opt = sample_optimizer();
        let plan = opt.optimize_greedy();
        assert!(plan.total_cost <= opt.budget, "plan cost exceeds budget");
        assert!(!plan.selected_options.is_empty());
    }
    #[test]
    fn hardening_bcr_positive() {
        let opt = sample_optimizer();
        let el = &opt.elements[0];
        let option = &el.hardening_options[0];
        let bcr = opt.benefit_cost_ratio(el, option);
        assert!(bcr > 0.0, "BCR should be positive, got {bcr}");
    }
    fn adequate_microgrid() -> ResilienceMicrogrid {
        ResilienceMicrogrid {
            id: 1,
            location: "Hospital District".to_string(),
            critical_load_mw: 2.0,
            pv_mw: 1.0,
            battery_mwh: 10.0,
            battery_mw: 1.5,
            diesel_mw: 1.0,
            autonomy_days: 3.0,
        }
    }
    #[test]
    fn microgrid_can_island_with_resources() {
        let mg = adequate_microgrid();
        assert!(mg.can_island());
    }
    #[test]
    fn microgrid_sizing_adequate() {
        let mg = adequate_microgrid();
        let mg2 = ResilienceMicrogrid {
            battery_mwh: 50.0,
            ..mg
        };
        assert_eq!(mg2.sizing_check(), SizingStatus::Adequate);
    }
    #[test]
    fn microgrid_sizing_battery_undersized() {
        let mg = ResilienceMicrogrid {
            id: 2,
            location: "Test".to_string(),
            critical_load_mw: 10.0,
            pv_mw: 2.0,
            battery_mwh: 1.0,
            battery_mw: 5.0,
            diesel_mw: 6.0,
            autonomy_days: 2.0,
        };
        match mg.sizing_check() {
            SizingStatus::BatteryUndersized { deficit_mwh } => assert!(deficit_mwh > 0.0),
            other => panic!("expected BatteryUndersized, got {other:?}"),
        }
    }
    fn sample_estimator() -> RestorationEstimator {
        RestorationEstimator {
            crew_count: 4,
            crew_travel_time_h: 1.0,
            repair_time_distribution: RepairDistribution {
                min_h: 2.0,
                mode_h: 4.0,
                max_h: 9.0,
            },
            damage_assessment_time_h: 0.5,
        }
    }
    #[test]
    fn restoration_expected_repair_time_triangular_mean() {
        let est = sample_estimator();
        let mean = est.expected_repair_time_h();
        assert!((mean - 5.0).abs() < 1e-10, "expected 5.0, got {mean}");
    }
    #[test]
    fn restoration_percentile_above_min() {
        let est = sample_estimator();
        let p50 = est.percentile_repair_time_h(0.5);
        assert!(p50 > 3.5, "p50={p50} should exceed 3.5");
    }
    #[test]
    fn restoration_sequence_total_time_positive() {
        let est = sample_estimator();
        let plan = est.restoration_sequence(5, 3.0, 10.0);
        assert!(plan.total_time_h > 0.0);
        assert_eq!(plan.phases.len(), 4);
    }
    fn sample_assessor() -> ClimateRiskAssessor {
        ClimateRiskAssessor {
            region: "Gulf Coast".to_string(),
            hazards: vec![
                ClimateHazard {
                    hazard_type: HazardType::Hurricane,
                    annual_probability: 0.05,
                    severity: HazardSeverity::High,
                    affected_fraction: 0.4,
                    damage_fraction: 0.3,
                },
                ClimateHazard {
                    hazard_type: HazardType::Flood,
                    annual_probability: 0.1,
                    severity: HazardSeverity::Medium,
                    affected_fraction: 0.2,
                    damage_fraction: 0.15,
                },
            ],
        }
    }
    #[test]
    fn climate_risk_damage_positive() {
        let ca = sample_assessor();
        let dmg = ca.annual_expected_damage_pct();
        assert!(dmg > 0.0, "expected damage should be positive, got {dmg}");
    }
    #[test]
    fn climate_risk_resilience_score_in_range() {
        let ca = sample_assessor();
        let score = ca.resilience_score();
        assert!(
            (0.0..=100.0).contains(&score),
            "score {score} out of [0,100]"
        );
    }
    #[test]
    fn climate_risk_most_critical_hazard_is_hurricane() {
        let ca = sample_assessor();
        let critical = ca.most_critical_hazard().expect("should have a hazard");
        assert_eq!(critical.hazard_type, HazardType::Hurricane);
    }
    #[test]
    fn climate_risk_recommendations_nonempty() {
        let ca = sample_assessor();
        let recs = ca.recommended_hardening();
        assert!(!recs.is_empty());
        assert_eq!(recs.len(), 2);
    }
    fn ep_line(id: usize, crit: f64, repair_h: f64) -> ComponentVulnerability {
        ComponentVulnerability {
            component_id: id,
            component_type: "overhead_line".to_string(),
            failure_probability: std::collections::HashMap::new(),
            repair_time_h: repair_h,
            replacement_cost_usd: 50_000.0,
            criticality_score: crit,
        }
    }
    fn ep_sub(id: usize, crit: f64) -> ComponentVulnerability {
        ComponentVulnerability {
            component_id: id,
            component_type: "substation".to_string(),
            failure_probability: std::collections::HashMap::new(),
            repair_time_h: 48.0,
            replacement_cost_usd: 500_000.0,
            criticality_score: crit,
        }
    }
    fn ep_planner() -> ResiliencePlanner {
        let comps = vec![ep_line(0, 0.5, 8.0), ep_line(1, 0.3, 12.0), ep_sub(2, 0.2)];
        let mut p = ResiliencePlanner::new(comps, 5000, 100.0);
        p.annual_event_probability = 0.10;
        p
    }
    #[test]
    fn test_vulnerability_assessment_hurricane() {
        let planner = ep_planner();
        let v1 = planner.assess_vulnerability(&ExtremeEventType::Hurricane { category: 1 });
        let v5 = planner.assess_vulnerability(&ExtremeEventType::Hurricane { category: 5 });
        let p1 = v1
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        let p5 = v5
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        assert!(p5 > p1, "cat 5 must exceed cat 1: {} vs {}", p5, p1);
    }
    #[test]
    fn test_vulnerability_assessment_ice_storm() {
        let planner = ep_planner();
        let v = planner.assess_vulnerability(&ExtremeEventType::IceStorm {
            ice_thickness_mm: 25.0,
        });
        let prob = v
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        assert!(prob > 0.0 && prob <= 1.0, "IceStorm_25mm prob={}", prob);
    }
    #[test]
    fn test_baseline_metrics_no_events() {
        let mut comp = ep_line(0, 1.0, 4.0);
        comp.failure_probability
            .insert("Hurricane_3".to_string(), 0.0);
        let planner = ResiliencePlanner::new(vec![comp], 1000, 50.0);
        let m = planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 3 });
        assert!(
            (m.robustness - 1.0).abs() < 1e-6,
            "robustness={}",
            m.robustness
        );
    }
    #[test]
    fn test_monte_carlo_convergence() {
        let planner = ep_planner();
        let event = ExtremeEventType::Hurricane { category: 3 };
        let m100 = planner.simulate_event(&event, 100);
        let m2000 = planner.simulate_event(&event, 2000);
        let diff = (m100.robustness - m2000.robustness).abs();
        assert!(diff < 0.5, "convergence diff={:.4}", diff);
    }
    #[test]
    fn test_resilience_trapezoid_formula() {
        let robustness = 0.6_f64;
        let t_event = 24.0_f64;
        let t_restore = 12.0_f64;
        let area = 0.5 * (1.0 + robustness) * t_event + 0.5 * (robustness + 1.0) * t_restore;
        assert!((area - 28.8).abs() < 1e-6, "area={}", area);
    }
    #[test]
    fn test_hardening_bcr_ranking() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let ranked = planner.rank_hardening_options(&measures);
        assert!(!ranked.is_empty());
        for i in 1..ranked.len() {
            assert!(
                ranked[i - 1].1 >= ranked[i].1 - 1e-12,
                "not descending at {}",
                i
            );
        }
    }
    #[test]
    fn test_greedy_budget_optimization() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 150_000.0);
        assert!(
            plan.total_cost_usd <= 150_000.0 + 1e-6,
            "cost={}",
            plan.total_cost_usd
        );
    }
    #[test]
    fn test_greedy_selects_best_bcr() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let ranked = planner.rank_hardening_options(&measures);
        let plan = planner.optimize_hardening_budget(&measures, 1e9);
        if !plan.measures.is_empty() && !ranked.is_empty() {
            let best_opt = &measures[ranked[0].0].option;
            assert_eq!(&plan.measures[0].option, best_opt);
        }
    }
    #[test]
    fn test_compute_measure_benefit() {
        let planner = ep_planner();
        let m = ExtremHardeningMeasure {
            option: ExtremeHardeningOption::StormHardenedPole,
            target_component_ids: vec![0],
            capital_cost_usd: 50_000.0,
            annual_opex_usd: 1_000.0,
            failure_prob_reduction: 0.5,
            repair_time_reduction_h: 4.0,
            implementation_years: 2,
        };
        let b = planner.compute_measure_benefit(&m, &ExtremeEventType::Hurricane { category: 3 });
        assert!(b > 0.0, "benefit={}", b);
    }
    #[test]
    fn test_resilience_index_bounds() {
        let planner = ep_planner();
        let metrics =
            planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 3 });
        let idx = planner.compute_resilience_index(&metrics);
        assert!((0.0..=1.0).contains(&idx), "index={}", idx);
    }
    #[test]
    fn test_generate_hardening_measures() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        assert!(!measures.is_empty());
        assert!(measures.len() >= planner.components.len());
    }
    #[test]
    fn test_plan_total_cost() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 1e9);
        let manual: f64 = plan
            .measures
            .iter()
            .map(|m| planner.ep_measure_cost(m))
            .sum();
        assert!((plan.total_cost_usd - manual).abs() < 1.0, "cost mismatch");
    }
    #[test]
    fn test_plan_bcr_positive() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 500_000.0);
        assert!(
            plan.benefit_cost_ratio >= 0.0,
            "bcr={}",
            plan.benefit_cost_ratio
        );
    }
    #[test]
    fn test_redundancy_metric() {
        let mut comp = ep_line(0, 1.0, 2.0);
        comp.failure_probability
            .insert("Hurricane_1".to_string(), 0.01);
        let planner = ResiliencePlanner::new(vec![comp], 100, 10.0);
        let m = planner.simulate_event(&ExtremeEventType::Hurricane { category: 1 }, 500);
        assert!(m.redundancy > 0.5, "redundancy={}", m.redundancy);
    }
    #[test]
    fn test_rapidity_positive() {
        let planner = ep_planner();
        let m = planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 2 });
        assert!(m.rapidity >= 0.0, "rapidity={}", m.rapidity);
    }
    #[test]
    fn test_hurricane_categories() {
        let p1 = ep_default_failure_prob("overhead_line", "Hurricane_1");
        let p5 = ep_default_failure_prob("overhead_line", "Hurricane_5");
        assert!(p5 > p1, "cat5={} must exceed cat1={}", p5, p1);
    }
    #[test]
    fn test_empty_components() {
        let planner = ResiliencePlanner::new(vec![], 0, 0.0);
        let event = ExtremeEventType::Hurricane { category: 3 };
        assert!(planner.assess_vulnerability(&event).is_empty());
        let m = planner.compute_baseline_metrics(&event);
        assert_eq!(m.robustness, 1.0);
        let mc = planner.simulate_event(&event, 100);
        assert_eq!(mc.robustness, 1.0);
    }
    #[test]
    fn test_zero_budget() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 0.0);
        assert!(
            plan.measures.is_empty(),
            "expected empty plan, got {}",
            plan.measures.len()
        );
        assert_eq!(plan.total_cost_usd, 0.0);
    }
    #[test]
    fn test_full_budget() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 1e12);
        let ranked = planner.rank_hardening_options(&measures);
        let n_pos = ranked.iter().filter(|(_, bcr)| *bcr > 0.0).count();
        assert_eq!(plan.measures.len(), n_pos, "expected {} measures", n_pos);
    }
    #[test]
    fn test_voll_impact() {
        let event = ExtremeEventType::Hurricane { category: 3 };
        let m = ExtremHardeningMeasure {
            option: ExtremeHardeningOption::StormHardenedPole,
            target_component_ids: vec![0],
            capital_cost_usd: 50_000.0,
            annual_opex_usd: 1_000.0,
            failure_prob_reduction: 0.5,
            repair_time_reduction_h: 4.0,
            implementation_years: 2,
        };
        let mut low = ep_planner();
        low.voll_usd_per_mwh = 1_000.0;
        let mut high = ep_planner();
        high.voll_usd_per_mwh = 50_000.0;
        let b_low = low.compute_measure_benefit(&m, &event);
        let b_high = high.compute_measure_benefit(&m, &event);
        assert!(b_high > b_low, "high={} low={}", b_high, b_low);
    }
    #[test]
    fn test_extreme_event_key_hurricane() {
        assert_eq!(
            extreme_event_key(&ExtremeEventType::Hurricane { category: 1 }),
            "Hurricane_1"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::Hurricane { category: 5 }),
            "Hurricane_5"
        );
    }
    #[test]
    fn test_extreme_event_key_ice_storm() {
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 5.0
            }),
            "IceStorm_5mm"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 25.0
            }),
            "IceStorm_25mm"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 50.0
            }),
            "IceStorm_50mm"
        );
    }
    #[test]
    fn test_transformer_vs_substation_prob() {
        let p_sub = ep_default_failure_prob("substation", "Hurricane_3");
        let p_tf = ep_default_failure_prob("transformer", "Hurricane_3");
        assert!(p_tf >= p_sub, "transformer={} sub={}", p_tf, p_sub);
    }
    #[test]
    fn test_resilience_index_perfect() {
        let planner = ResiliencePlanner::new(vec![], 0, 100.0);
        let m = ExtremeResilienceMetrics {
            expected_loss_mwh: 0.0,
            expected_repair_time_h: 0.0,
            resilience_trapezoid_area: 48.0,
            rapidity: 1.0,
            robustness: 1.0,
            redundancy: 1.0,
            resourcefulness: 1.0,
        };
        let idx = planner.compute_resilience_index(&m);
        assert!((idx - 1.0).abs() < 1e-9, "idx={}", idx);
    }
}
