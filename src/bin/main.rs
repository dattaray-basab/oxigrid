use oxigrid::analytics::operational_analytics::{OperationalKpi, OperationalKpiCategory};
use oxigrid::battery::ecm::{ParameterSet, TwoRcModel};
use oxigrid::optimize::microgrid::ems::{DieselGen, EmsBattery, EmsDispatcher};
use oxigrid::powerquality::indices::{check_en50160_compliance, En50160Limits};
use oxigrid::powerquality::waveform::HarmonicComponent;
use oxigrid::prelude::OcvSocCurve;
use oxigrid::prelude::Result;
use oxigrid::prelude::*;
use oxigrid::protection::{OvercurrentRelay, ProtectionCoordinationOptimizer, RelayCharacteristic};
use oxigrid::renewable::forecast::persistence::{
    skill_score, DiurnalPersistence, PersistenceForecast,
};
use oxigrid::renewable::solar::irradiance::SolarPosition;

fn main() -> Result<()> {
    println!("OxiGrid Cross-Feature Demo");
    println!("==========================\n");

    run_powerflow_demo()?;
    run_battery_demo();
    run_renewable_demo();
    run_ems_demo();
    run_analytics_demo();
    run_protection_demo();
    run_powerquality_demo();

    println!("\nDemo complete.");
    Ok(())
}

fn run_powerflow_demo() -> Result<()> {
    println!("--- Power Flow Demo ---");
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let network = PowerNetwork::from_matpower(path)?;

    println!("Loaded IEEE 14 bus case ({})", path);
    println!(
        "Buses: {}, Branches: {}",
        network.bus_count(),
        network.branch_count()
    );

    let config = PowerFlowConfig::default();
    let result = network.solve_powerflow(&config)?;

    println!("Newton-Raphson converged: {}", result.converged);
    println!("Bus 1 voltage = {:.4} pu", result.voltage_magnitude[0]);
    println!("Total real loss = {:.3} MW\n", result.total_p_loss_mw);

    let dc_config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        ..PowerFlowConfig::default()
    };
    let dc_result = network.solve_powerflow(&dc_config)?;
    println!(
        "DC approximation angle at bus 14 = {:.3} deg\n",
        dc_result.voltage_angle[13].to_degrees()
    );
    Ok(())
}

fn run_battery_demo() {
    println!("--- Battery ECM Demo ---");
    let parameters = ParameterSet::kokam_75ah_lfp();
    let ocv_curve = OcvSocCurve::lfp_default();
    let mut model = TwoRcModel::new(
        ocv_curve.clone(),
        parameters.r0,
        parameters.r1,
        parameters.c1,
        parameters.r2,
        parameters.c2,
        parameters.capacity_ah,
    )
    .with_soc(0.9);

    let profile = [10.0, -20.0, 0.0, 15.0, -5.0];
    let dt = 60.0;
    let ambient = Temperature(298.15);

    for (step, &current_a) in profile.iter().enumerate() {
        let state = model.step(Current(current_a), dt, ambient);
        println!(
            "Step {:>2}: I = {:+5.1} A, SoC = {:.3}, V = {:.3} V, T = {:.1} °C",
            step + 1,
            current_a,
            state.soc.0,
            state.voltage.0,
            state.temperature.0 - 273.15,
        );
    }
    println!();
}

fn run_renewable_demo() {
    println!("--- Renewable Forecast Demo ---");
    let lat = 35.0;
    let day = 172_u32;
    let horizon = 24;

    let mut observations = Vec::with_capacity(horizon);
    for hour in 0..horizon {
        let position = SolarPosition::compute(lat, day, hour as f64 + 0.5);
        let ghi = if position.is_daytime() {
            800.0 * position.elevation.sin().max(0.0)
        } else {
            0.0
        };
        observations.push(ghi);
    }

    let mut persistence = PersistenceForecast::new(observations[0]);
    let mut diurnal = DiurnalPersistence::new(24);
    for obs in observations.iter().take(24) {
        diurnal.update(*obs);
    }

    let mut persistence_errors = Vec::new();
    let mut diurnal_errors = Vec::new();

    for &obs in observations.iter().skip(1) {
        let persistence_fc = persistence.last_value;
        let diurnal_fc = diurnal.update(obs);
        persistence_errors.push((obs - persistence_fc).powi(2));
        diurnal_errors.push((obs - diurnal_fc).powi(2));
        persistence.update(obs);
    }

    let rmse = |errs: &[f64]| -> f64 { (errs.iter().sum::<f64>() / errs.len() as f64).sqrt() };
    let persistence_rmse = rmse(&persistence_errors);
    let diurnal_rmse = rmse(&diurnal_errors);
    println!("Persistence RMSE = {:.3}", persistence_rmse);
    println!("Diurnal persistence RMSE = {:.3}", diurnal_rmse);
    println!(
        "Skill score = {:.3}\n",
        skill_score(diurnal_rmse, persistence_rmse)
    );
}

fn run_ems_demo() {
    println!("--- EMS Microgrid Dispatch Demo ---");
    let pv_kw: Vec<f64> = (0..24)
        .map(|h| {
            let position = SolarPosition::compute(35.0, 172, h as f64 + 0.5);
            if position.is_daytime() {
                400.0 * position.elevation.sin().max(0.0)
            } else {
                0.0
            }
        })
        .collect();

    let load_kw: Vec<f64> = (0..24)
        .map(|h| {
            let base = 40.0;
            let morning = 25.0 * (-((h as f64 - 9.0) / 3.0).powi(2)).exp();
            let evening = 45.0 * (-((h as f64 - 18.0) / 3.0).powi(2)).exp();
            base + morning + evening
        })
        .collect();

    let wind_kw = vec![0.0; 24];
    let battery = EmsBattery::lifepo4_100kwh();
    let diesel = DieselGen::diesel_100kw();
    let mut dispatcher = EmsDispatcher::new(battery, diesel);
    let plan = dispatcher.dispatch(&load_kw, &pv_kw, &wind_kw, 1.0);

    println!("Dispatch intervals: {}", plan.intervals.len());
    println!(
        "Total cost = ${:.2}, renewable fraction = {:.1}%\n",
        plan.total_cost_usd,
        plan.renewable_fraction * 100.0
    );
}

fn run_analytics_demo() {
    println!("--- Operational Analytics Demo ---");
    let mut analytics = oxigrid::analytics::operational_analytics::OperationalAnalytics::new();

    let kpis = vec![
        OperationalKpi {
            id: 1,
            name: "SAIFI".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 1.2,
            unit: "events/customer".to_string(),
            target: Some(1.0),
            threshold_warning: Some(1.5),
            threshold_critical: Some(2.0),
            higher_is_better: false,
            timestamp: 1.0,
        },
        OperationalKpi {
            id: 2,
            name: "SAIFI".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 1.8,
            unit: "events/customer".to_string(),
            target: Some(1.0),
            threshold_warning: Some(1.5),
            threshold_critical: Some(2.0),
            higher_is_better: false,
            timestamp: 2.0,
        },
        OperationalKpi {
            id: 3,
            name: "SAIFI".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 0.9,
            unit: "events/customer".to_string(),
            target: Some(1.0),
            threshold_warning: Some(1.5),
            threshold_critical: Some(2.0),
            higher_is_better: false,
            timestamp: 3.0,
        },
    ];

    for kpi in kpis.iter().cloned() {
        analytics.add_kpi_reading(kpi);
    }

    if let Some(stats) = analytics.compute_time_series_stats("SAIFI") {
        println!(
            "SAIFI mean = {:.3}, trend = {:?}",
            stats.mean, stats.trend_direction
        );
    }

    let anomaly = analytics.detect_anomalies("SAIFI", 2.4);
    println!(
        "Anomaly score = {:.3}, is_anomaly = {}\n",
        anomaly.combined_score, anomaly.is_anomaly
    );
}

fn run_protection_demo() {
    println!("--- Protection Coordination Demo ---");
    let relays = vec![
        OvercurrentRelay::new(
            1,
            "Line 1 Primary",
            1,
            (1, 2),
            RelayCharacteristic::StandardInverseIec,
            120.0,
            0.5,
            20.0,
            6000.0,
            200.0,
        ),
        OvercurrentRelay::new(
            2,
            "Line 1 Backup",
            2,
            (2, 3),
            RelayCharacteristic::StandardInverseIec,
            180.0,
            1.0,
            20.0,
            6000.0,
            200.0,
        ),
    ];

    let fault_points = vec![oxigrid::protection::coordination_optimizer::FaultPoint {
        location_bus: 2,
        fault_current_a: 3000.0,
        distance_from_relay_km: 5.0,
    }];

    let mut optimizer = ProtectionCoordinationOptimizer::new(relays, fault_points);
    optimizer.add_coordination_pair(1, 2);
    let solution = optimizer.optimize();

    println!("Coordinated pairs = {}", solution.n_pairs_coordinated);
    println!("Selectivity index = {:.3}\n", solution.selectivity_index);
}

fn run_powerquality_demo() {
    println!("--- Power Quality Demo ---");
    let limits = En50160Limits::standard();
    let v_rms_timeline = vec![1.0; 96];
    let frequency_timeline = vec![50.0; 360];
    let harmonics = vec![
        HarmonicComponent {
            order: 1,
            magnitude_pu: 1.0,
            phase_rad: 0.0,
            power: 0.0,
        },
        HarmonicComponent {
            order: 3,
            magnitude_pu: 0.02,
            phase_rad: 0.0,
            power: 0.0,
        },
        HarmonicComponent {
            order: 5,
            magnitude_pu: 0.01,
            phase_rad: 0.0,
            power: 0.0,
        },
    ];
    let report = check_en50160_compliance(
        &v_rms_timeline,
        &frequency_timeline,
        &harmonics,
        0.8,
        1.2,
        &limits,
    );
    println!("EN 50160 overall compliance = {}", report.overall_compliant);
}
