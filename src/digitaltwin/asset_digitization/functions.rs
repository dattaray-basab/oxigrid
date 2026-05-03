//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

/// Seconds per Julian year (365.25 days).
pub const SECS_PER_YEAR: f64 = 365.25 * 24.0 * 3600.0;
/// Current reference epoch used when no external clock is supplied \[s\].
/// Corresponds to 2026-03-09 00:00:00 UTC.
pub const NOW_UNIX: f64 = 1_741_478_400.0 + 365.25 * 24.0 * 3600.0;
/// Value of Lost Load \[USD/MWh\].
pub const VOLL: f64 = 10_000.0;
/// Default energy not served per asset failure \[MWh\].
pub const DEFAULT_ENS_MWH: f64 = 100.0;
/// Parse an ISO 8601 date string `"YYYY-MM-DD"` into a Unix timestamp `s`.
///
/// Returns `None` on parse failure.
pub(crate) fn parse_iso_date(date: &str) -> Option<f64> {
    let parts: Vec<&str> = date.split('-').collect();
    if parts.len() != 3 {
        return None;
    }
    let year: i64 = parts[0].parse().ok()?;
    let month: i64 = parts[1].parse().ok()?;
    let day: i64 = parts[2].parse().ok()?;
    let y = if month <= 2 { year - 1 } else { year };
    let m = if month <= 2 { month + 12 } else { month };
    let jdn: i64 = 365 * y + y / 4 - y / 100 + y / 400 + (153 * m + 8) / 5 + day - 678_882;
    const JDN_1970: i64 = 40_680;
    let days_since_epoch = jdn - JDN_1970;
    Some(days_since_epoch as f64 * 86_400.0)
}
/// Compute asset age in years from a commissioning date ISO string.
///
/// Uses `NOW_UNIX` as the reference instant.
pub(crate) fn asset_age_years(commissioning_date: &str) -> f64 {
    match parse_iso_date(commissioning_date) {
        Some(unix) => ((NOW_UNIX - unix) / SECS_PER_YEAR).max(0.0),
        None => 0.0,
    }
}
#[cfg(test)]
mod tests {
    use super::super::*;
    use std::collections::HashMap;
    /// Fixed "current" time used in all tests so results are deterministic.
    /// Matches NOW_UNIX defined at module level (2026-03-09).
    const T_NOW: f64 = NOW_UNIX;
    fn make_transformer(age_years: f64, criticality: f64) -> DigitalAssetRecord {
        DigitalAssetRecord {
            asset_id: "TRF-001".to_string(),
            asset_class: AssetClass::PowerTransformer {
                mva_rating: 100.0,
                voltage_kv: 220.0,
                vector_group: "YNd11".to_string(),
            },
            installation_date_unix: T_NOW - age_years * SECS_PER_YEAR,
            manufacturer: "ABB".to_string(),
            model: "TRAFO-220".to_string(),
            serial_number: "SN-000001".to_string(),
            substation_id: "SS-01".to_string(),
            bus_id: 0,
            inspection_history: Vec::new(),
            maintenance_history: Vec::new(),
            operating_hours: age_years * 8760.0,
            load_factor_avg: 0.7,
            criticality_score: criticality,
        }
    }
    fn make_circuit_breaker(age_years: f64, criticality: f64) -> DigitalAssetRecord {
        DigitalAssetRecord {
            asset_id: "CB-001".to_string(),
            asset_class: AssetClass::CircuitBreaker {
                rated_kv: 145.0,
                rated_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            installation_date_unix: T_NOW - age_years * SECS_PER_YEAR,
            manufacturer: "Siemens".to_string(),
            model: "3AP1FI".to_string(),
            serial_number: "SN-CB-001".to_string(),
            substation_id: "SS-01".to_string(),
            bus_id: 1,
            inspection_history: Vec::new(),
            maintenance_history: Vec::new(),
            operating_hours: age_years * 8760.0,
            load_factor_avg: 0.5,
            criticality_score: criticality,
        }
    }
    fn add_inspection(
        asset: &mut DigitalAssetRecord,
        days_ago: f64,
        score: f64,
        severity: DefectSeverity,
    ) {
        asset.inspection_history.push(InspectionRecord {
            inspection_id: format!("INS-{}", asset.inspection_history.len()),
            date_unix: T_NOW - days_ago * 86400.0,
            inspection_type: InspectionType::Visual,
            inspector_id: "ENG-01".to_string(),
            findings: vec!["Routine visual check".to_string()],
            condition_score: score,
            recommended_actions: Vec::new(),
            defect_severity: severity,
        });
    }
    /// Condition score from a recent (30-day-old) inspection with score 4.0
    /// should remain very close to 4.0 after degradation.
    #[test]
    fn test_condition_score_recent_inspection() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let mut asset = make_transformer(5.0, 0.8);
        add_inspection(&mut asset, 30.0, 4.0, DefectSeverity::None);
        let score = scheduler.current_condition_score(&asset);
        assert!((score - 4.0).abs() < 0.1, "Expected ~4.0, got {score:.4}");
    }
    /// A 40-year-old transformer should have a high failure probability (>0.5).
    #[test]
    fn test_failure_probability_old_transformer() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(40.0, 0.9);
        let p = scheduler.predict_failure_probability(&asset, 1.0);
        assert!(
            p > 0.5,
            "Expected P > 0.5 for 40-year transformer, got {p:.4}"
        );
    }
    /// High failure probability × high criticality should yield a positive risk score.
    #[test]
    fn test_risk_score_high_probability_and_criticality() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(38.0, 1.0);
        let risk = scheduler.calculate_risk_score(&asset);
        assert!(risk > 0.0, "Risk score should be positive, got {risk:.6}");
    }
    /// DGA with high C2H2 and C2H2/C2H4 ratio should classify as electrical arcing.
    #[test]
    fn test_dga_arcing_diagnosis() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(10.0, 0.8);
        let mut gas: HashMap<String, f64> = HashMap::new();
        gas.insert("H2".to_string(), 100.0);
        gas.insert("CH4".to_string(), 80.0);
        gas.insert("C2H2".to_string(), 50.0);
        gas.insert("C2H4".to_string(), 30.0);
        gas.insert("C2H6".to_string(), 10.0);
        let result = scheduler
            .dga_analysis(&gas, &asset)
            .expect("DGA should succeed for transformer");
        assert_eq!(
            result.fault_type, "Electrical Discharge (Arcing)",
            "Expected arcing, got: {}",
            result.fault_type
        );
        assert_eq!(result.severity, DefectSeverity::Critical);
    }
    /// A brand-new asset (age ≈ 0, condition = 5) should yield AHI close to 100.
    #[test]
    fn test_ahi_new_asset() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let mut asset = make_transformer(0.01, 0.5);
        add_inspection(&mut asset, 1.0, 5.0, DefectSeverity::None);
        asset.maintenance_history.push(MaintenanceRecord {
            date_unix: T_NOW - 30.0 * 86400.0,
            maintenance_type: MaintenanceType::Preventive,
            cost_usd: 5_000.0,
            downtime_h: 4.0,
            performed_by: "OEM".to_string(),
            parts_replaced: Vec::new(),
        });
        let report = scheduler.asset_health_index(&asset);
        assert!(
            report.health_index > 90.0,
            "Expected AHI > 90 for new asset, got {:.2}",
            report.health_index
        );
    }
    /// With a zero budget the plan should schedule no actions.
    #[test]
    fn test_maintenance_plan_zero_budget() {
        let mut asset = make_transformer(30.0, 0.9);
        add_inspection(&mut asset, 365.0, 2.5, DefectSeverity::Moderate);
        let scheduler = MaintenanceScheduler::new(vec![asset], 0.0, 5);
        let plan = scheduler.generate_maintenance_plan(0.0);
        assert!(
            plan.scheduled_actions.is_empty(),
            "Zero budget should schedule no actions, got {}",
            plan.scheduled_actions.len()
        );
    }
    /// An asset with a condition score below 2.0 should appear in the
    /// replacement prioritisation list.
    #[test]
    fn test_replacement_priority_poor_condition() {
        let mut asset = make_circuit_breaker(28.0, 0.8);
        add_inspection(&mut asset, 10.0, 1.5, DefectSeverity::Significant);
        let scheduler = MaintenanceScheduler::new(vec![asset], 200_000.0, 5);
        let recs = scheduler.replacement_prioritization(1_000_000.0);
        assert!(
            !recs.is_empty(),
            "Expected at least one replacement recommendation for poor-condition asset"
        );
        assert_eq!(recs[0].asset_id, "CB-001");
    }
    /// Fleet mean age should equal the arithmetic mean of individual ages.
    #[test]
    fn test_fleet_mean_age_calculation() {
        let a1 = make_transformer(10.0, 0.7);
        let a2 = make_circuit_breaker(20.0, 0.5);
        let scheduler = MaintenanceScheduler::new(vec![a1, a2], 50_000.0, 3);
        let report = scheduler.fleet_statistics();
        let expected_mean = 15.0;
        assert!(
            (report.mean_age_years - expected_mean).abs() < 0.5,
            "Expected mean age ~15 yr, got {:.4}",
            report.mean_age_years
        );
    }
    /// DGA on a non-transformer asset should return an error.
    #[test]
    fn test_dga_non_transformer_returns_error() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 50_000.0, 3);
        let asset = make_circuit_breaker(5.0, 0.5);
        let gas: HashMap<String, f64> = HashMap::new();
        let result = scheduler.dga_analysis(&gas, &asset);
        assert!(
            result.is_err(),
            "DGA on a circuit breaker should return Err"
        );
    }
    /// Budget exactly covering one maintenance action should schedule exactly one.
    #[test]
    fn test_maintenance_plan_budget_covers_one() {
        let mut xfmr = make_transformer(30.0, 0.9);
        add_inspection(&mut xfmr, 180.0, 2.0, DefectSeverity::Moderate);
        let mut cb = make_circuit_breaker(25.0, 0.8);
        add_inspection(&mut cb, 180.0, 2.5, DefectSeverity::Minor);
        let scheduler = MaintenanceScheduler::new(vec![xfmr, cb], 22_000.0, 1);
        let plan = scheduler.generate_maintenance_plan(0.0);
        assert_eq!(
            plan.scheduled_actions.len(),
            1,
            "Budget of $22k over 1yr should schedule exactly one transformer maintenance"
        );
    }
    fn make_twin(
        id: &str,
        category: AssetCategory,
        health: f64,
        risk: RiskLevel,
    ) -> AssetDigitalTwin {
        AssetDigitalTwin {
            asset_id: id.to_string(),
            asset_name: format!("Asset {}", id),
            category,
            substation_id: "SS-01".to_string(),
            commissioning_date: "2016-01-01".to_string(),
            manufacturer: "TestCo".to_string(),
            model_number: "MDL-100".to_string(),
            serial_number: "SN-TEST-001".to_string(),
            location: AssetLocation {
                latitude: 59.44,
                longitude: 24.75,
                bay: "A1".to_string(),
                panel: "P1".to_string(),
                rack: None,
            },
            condition: AssetCondition {
                overall_health_index: health,
                mechanical_condition: health,
                electrical_condition: health,
                insulation_condition: health,
                cooling_condition: health,
                last_inspection_date: "2025-06-01".to_string(),
                next_maintenance_due: "2027-06-01".to_string(),
                defect_codes: Vec::new(),
                risk_level: risk,
            },
            nameplate: AssetNameplate {
                rated_voltage_kv: 110.0,
                rated_current_ka: 1.0,
                rated_power_mva: 50.0,
                frequency_hz: 50.0,
                temperature_class: "F".to_string(),
                protection_class: "IP54".to_string(),
                weight_kg: 5000.0,
            },
            telemetry: AssetTelemetry::default(),
            maintenance_history: Vec::new(),
            failure_history: Vec::new(),
            digital_twin_accuracy: 0.9,
        }
    }
    #[test]
    fn test_asset_digital_twin_creation() {
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        assert_eq!(twin.asset_id, "T-001");
        assert_eq!(twin.condition.overall_health_index, 90.0);
        assert_eq!(twin.digital_twin_accuracy, 0.9);
    }
    #[test]
    fn test_asset_registry_add_get() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            85.0,
            RiskLevel::Low,
        );
        registry.add_asset(twin);
        assert!(registry.get_asset("T-001").is_some());
        assert!(registry.get_asset("T-999").is_none());
    }
    #[test]
    fn test_assets_by_category_transformer() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            80.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "B-001",
            AssetCategory::Breaker {
                current_rating_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            75.0,
            RiskLevel::Low,
        ));
        let transformers = registry.assets_by_category("Transformer");
        assert_eq!(transformers.len(), 1);
        assert_eq!(transformers[0].asset_id, "T-001");
    }
    #[test]
    fn test_high_risk_assets_filter() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "T-002",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            40.0,
            RiskLevel::High,
        ));
        registry.add_asset(make_twin(
            "T-003",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            20.0,
            RiskLevel::Critical,
        ));
        let high_risk = registry.high_risk_assets();
        assert_eq!(high_risk.len(), 2);
    }
    #[test]
    fn test_fleet_health_score() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "G-001",
            AssetCategory::Generator {
                rated_mw: 100.0,
                technology: "CCGT".to_string(),
            },
            80.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            60.0,
            RiskLevel::Medium,
        ));
        let score = registry.fleet_health_score();
        assert!(
            (score - 73.33).abs() < 1.0,
            "Fleet health score ~73.3, got {:.2}",
            score
        );
    }
    #[test]
    fn test_fleet_health_score_perfect() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 100_000.0,
                primary_kv: 220.0,
                secondary_kv: 110.0,
            },
            100.0,
            RiskLevel::Low,
        ));
        let score = registry.fleet_health_score();
        assert!(
            (score - 100.0).abs() < 1e-6,
            "Score should be 100.0, got {:.6}",
            score
        );
    }
    #[test]
    fn test_total_replacement_value() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 100_000.0,
                primary_kv: 220.0,
                secondary_kv: 110.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "B-001",
            AssetCategory::Breaker {
                current_rating_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        let total = registry.total_replacement_value_million_eur();
        assert!(
            (total - 10.1).abs() < 0.01,
            "Expected 10.1 M€, got {:.4}",
            total
        );
    }
    #[test]
    fn test_aging_report_young_fleet() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            95.0,
            RiskLevel::Low,
        );
        twin.commissioning_date = "2024-01-01".to_string();
        registry.add_asset(twin);
        let report = registry.aging_analysis();
        assert_eq!(report.total_assets, 1);
        assert_eq!(
            report.beyond_half_life, 0,
            "Young asset should not be beyond half-life"
        );
        assert_eq!(report.beyond_design_life, 0);
    }
    #[test]
    fn test_aging_report_old_fleet() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-OLD",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            30.0,
            RiskLevel::High,
        );
        twin.commissioning_date = "1981-01-01".to_string();
        registry.add_asset(twin);
        let report = registry.aging_analysis();
        assert_eq!(
            report.beyond_design_life, 1,
            "45-year transformer should be beyond design life"
        );
        assert!(report.beyond_half_life >= 1);
        assert!(report.capital_replacement_5yr_million_eur > 0.0);
    }
    #[test]
    fn test_twin_synchronizer_creation() {
        let sync = TwinSynchronizer::new(300.0, 0.8);
        assert_eq!(sync.stale_threshold_s, 300.0);
        assert_eq!(sync.min_accuracy_threshold, 0.8);
    }
    #[test]
    fn test_sync_status_fresh() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry.timestamp = 1_000_000.0;
        registry.add_asset(twin);
        let sync = TwinSynchronizer::new(300.0, 0.8);
        let statuses = sync.check_sync_status(&registry, 1_000_010.0);
        assert_eq!(statuses.len(), 1);
        assert!(!statuses[0].is_stale, "10 s gap should not be stale");
        assert!((statuses[0].sync_gap_s - 10.0).abs() < 1.0);
    }
    #[test]
    fn test_sync_status_stale() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry.timestamp = 1_000_000.0;
        registry.add_asset(twin);
        let sync = TwinSynchronizer::new(300.0, 0.8);
        let statuses = sync.check_sync_status(&registry, 1_000_600.0);
        assert!(statuses[0].is_stale, "600 s gap should be stale");
    }
    #[test]
    fn test_update_telemetry_success() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        let new_tel = AssetTelemetry {
            timestamp: 9_999_999.0,
            current_ka: 1.2,
            voltage_kv: 110.0,
            power_mw: 45.0,
            temperature_c: 75.0,
            vibration_mm_per_s: 1.5,
            partial_discharge_pc: 50.0,
            oil_temperature_c: 65.0,
            dissolved_gas_h2_ppm: 10.0,
            sf6_pressure_bar: 0.0,
        };
        let result = TwinSynchronizer::update_telemetry(&mut registry, "T-001", new_tel);
        assert!(result.is_ok());
        let asset = registry.get_asset("T-001").expect("asset should exist");
        assert_eq!(asset.telemetry.timestamp, 9_999_999.0);
        assert_eq!(asset.telemetry.current_ka, 1.2);
    }
    #[test]
    fn test_update_telemetry_not_found() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let tel = AssetTelemetry::default();
        let result = TwinSynchronizer::update_telemetry(&mut registry, "NONEXISTENT", tel);
        assert!(result.is_err(), "Should return error for missing asset");
    }
    #[test]
    fn test_compute_accuracy_full_data() {
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry = AssetTelemetry {
            timestamp: NOW_UNIX - 100.0,
            current_ka: 1.0,
            voltage_kv: 110.0,
            power_mw: 50.0,
            temperature_c: 75.0,
            vibration_mm_per_s: 1.0,
            partial_discharge_pc: 100.0,
            oil_temperature_c: 65.0,
            dissolved_gas_h2_ppm: 5.0,
            sf6_pressure_bar: 6.0,
        };
        twin.maintenance_history.push(AssetMaintenanceRecord {
            date: "2026-01-15".to_string(),
            maintenance_type: AssetMaintenanceType::Preventive,
            work_order: "WO-001".to_string(),
            performed_by: "TechTeam".to_string(),
            duration_hours: 4.0,
            cost_eur: 5000.0,
            findings: "All OK".to_string(),
            corrective_actions: Vec::new(),
        });
        let accuracy = TwinSynchronizer::compute_accuracy(&twin);
        assert!(
            accuracy >= 0.9,
            "Full-data asset accuracy should be ≥ 0.9, got {:.4}",
            accuracy
        );
    }
    #[test]
    fn test_cbm_assess_normal() {
        let tel = AssetTelemetry {
            timestamp: 1_000_000.0,
            current_ka: 0.8,
            voltage_kv: 110.0,
            power_mw: 40.0,
            temperature_c: 70.0,
            vibration_mm_per_s: 1.0,
            partial_discharge_pc: 50.0,
            oil_temperature_c: 60.0,
            dissolved_gas_h2_ppm: 5.0,
            sf6_pressure_bar: 6.0,
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::Low);
    }
    #[test]
    fn test_cbm_assess_high_temperature() {
        let tel = AssetTelemetry {
            temperature_c: 125.0,
            ..AssetTelemetry::default()
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::High);
    }
    #[test]
    fn test_cbm_assess_high_pd() {
        let tel = AssetTelemetry {
            temperature_c: 60.0,
            partial_discharge_pc: 1500.0,
            ..AssetTelemetry::default()
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::High);
    }
    #[test]
    fn test_estimate_rul_new_asset() {
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            95.0,
            RiskLevel::Low,
        );
        let rul = CbmAssessor::estimate_rul(&twin, 20.0, 40.0);
        assert!(
            (rul - 37.5).abs() < 0.1,
            "Expected RUL ~37.5, got {:.4}",
            rul
        );
    }
    #[test]
    fn test_estimate_rul_aged_asset() {
        let twin = make_twin(
            "T-OLD",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            15.0,
            RiskLevel::Critical,
        );
        let rul = CbmAssessor::estimate_rul(&twin, 20.0, 40.0);
        assert_eq!(rul, 0.0, "Health below threshold should give RUL=0");
    }
    #[test]
    fn test_recommend_maintenance_interval() {
        let mut twin_new = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin_new.condition.overall_health_index = 90.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_new),
            5.0,
            "Health ≥80 → 5yr"
        );
        let mut twin_mid = twin_new.clone();
        twin_mid.condition.overall_health_index = 65.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_mid),
            3.0,
            "Health 60-79 → 3yr"
        );
        let mut twin_low = twin_new.clone();
        twin_low.condition.overall_health_index = 45.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_low),
            1.0,
            "Health 40-59 → 1yr"
        );
        let mut twin_crit = twin_new.clone();
        twin_crit.condition.overall_health_index = 30.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_crit),
            0.5,
            "Health <40 → 0.5yr"
        );
    }
}
