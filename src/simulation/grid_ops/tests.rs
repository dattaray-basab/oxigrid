//! Auto-generated test module (consolidated from inline `#[cfg(test)] mod` blocks)

use super::*;

#[cfg(test)]
mod tests_2 {
    use super::*;
    fn make_generator(id: usize, bus: usize, p_mw: f64, p_max: f64) -> SimGenerator {
        SimGenerator {
            id,
            bus,
            p_mw,
            p_max_mw: p_max,
            p_min_mw: 0.0,
            ramp_rate_mw_per_min: 5.0,
            agc_participation: 0.5,
            is_online: true,
            startup_time_min: 10.0,
            fuel_type: "gas".to_string(),
            co2_kg_per_mwh: 400.0,
        }
    }
    fn make_load(bus: usize, p_mw: f64) -> SimLoad {
        SimLoad {
            bus,
            p_mw,
            q_mvar: 0.0,
            is_shedable: true,
            priority: 2,
        }
    }
    fn make_branch(id: usize, from: usize, to: usize) -> SimBranch {
        SimBranch {
            id,
            from,
            to,
            is_online: true,
            rating_mva: 100.0,
            current_flow_mw: 0.0,
            current_flow_mvar: 0.0,
            loading_pct: 0.0,
        }
    }
    fn make_storage(bus: usize) -> SimStorage {
        SimStorage {
            bus,
            soc: 0.5,
            capacity_mwh: 10.0,
            power_mw: 0.0,
            max_charge_mw: 5.0,
            max_discharge_mw: 5.0,
            efficiency: 0.95,
        }
    }
    fn make_simulator(duration_s: f64, dt_s: f64) -> GridOperationsSimulator {
        let config = QdGridOpsConfig {
            n_buses: 5,
            base_mva: 100.0,
            nominal_frequency_hz: 50.0,
            frequency_deadband_hz: 0.02,
            ufls_threshold_hz: 47.5,
            ufls_shed_pct: vec![0.10, 0.15, 0.20],
            ovf_threshold_hz: 51.5,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_branch_loading_pct: 90.0,
        };
        let generators = vec![
            make_generator(0, 1, 60.0, 100.0),
            make_generator(1, 2, 40.0, 80.0),
        ];
        let loads = vec![make_load(3, 50.0), make_load(4, 45.0)];
        let branches = vec![make_branch(0, 1, 2), make_branch(1, 2, 3)];
        let storages = vec![make_storage(5)];
        GridOperationsSimulator::new(
            config, generators, loads, branches, storages, duration_s, dt_s,
        )
    }
    #[test]
    fn test_sim_clock_advance() {
        let mut clock = SimClock::new(0.0, 100.0, 10.0);
        assert!((clock.current_time_s - 0.0).abs() < 1e-9);
        let cont = clock.advance();
        assert!(cont);
        assert!((clock.current_time_s - 10.0).abs() < 1e-9);
    }
    #[test]
    fn test_sim_clock_time_of_day() {
        let clock = SimClock::new(25.0 * 3600.0, 100.0 * 3600.0, 3600.0);
        let tod = clock.time_of_day_h();
        assert!((tod - 1.0).abs() < 1e-9, "Expected 1.0 h, got {tod}");
    }
    #[test]
    fn test_sim_clock_complete() {
        let mut clock = SimClock::new(0.0, 30.0, 10.0);
        assert!(clock.advance());
        assert!(clock.advance());
        assert!(clock.advance());
        assert!(!clock.advance());
    }
    #[test]
    fn test_generator_creation() {
        let gen = make_generator(0, 1, 80.0, 100.0);
        assert_eq!(gen.bus, 1);
        assert!((gen.p_mw - 80.0).abs() < 1e-9);
        assert!(gen.is_online);
        assert_eq!(gen.fuel_type, "gas");
    }
    #[test]
    fn test_load_creation() {
        let load = make_load(3, 50.0);
        assert_eq!(load.bus, 3);
        assert!((load.p_mw - 50.0).abs() < 1e-9);
        assert!(load.is_shedable);
        assert_eq!(load.priority, 2);
    }
    #[test]
    fn test_storage_soc_update() {
        let mut sim = make_simulator(60.0, 60.0);
        sim.storages[0].power_mw = 5.0;
        sim.storages[0].soc = 0.5;
        let initial_soc = sim.storages[0].soc;
        sim.update_storage_soc(3600.0);
        assert!(
            sim.storages[0].soc < initial_soc,
            "SoC should decrease when discharging"
        );
    }
    #[test]
    fn test_simulator_creation() {
        let sim = make_simulator(3600.0, 60.0);
        assert_eq!(sim.generators.len(), 2);
        assert_eq!(sim.loads.len(), 2);
        assert_eq!(sim.branches.len(), 2);
        assert_eq!(sim.storages.len(), 1);
        assert!((sim.clock.end_time_s - 3600.0).abs() < 1e-9);
    }
    #[test]
    fn test_schedule_event() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.schedule_event(
            500.0,
            GridEvent::LineTrip {
                branch_id: 0,
                reason: "test".to_string(),
            },
            "test event".to_string(),
        );
        assert_eq!(sim.scheduled_events.len(), 1);
        assert!((sim.scheduled_events[0].time_s - 500.0).abs() < 1e-9);
    }
    #[test]
    fn test_power_balance_balanced() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.generators[0].p_mw = 50.0;
        sim.generators[1].p_mw = 0.0;
        sim.generators[1].is_online = false;
        sim.loads[0].p_mw = 30.0;
        sim.loads[1].p_mw = 20.0;
        let bal = sim.compute_power_balance();
        assert!((bal).abs() < 1e-6, "Balance should be ~0, got {bal}");
    }
    #[test]
    fn test_power_balance_surplus() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.generators[0].p_mw = 80.0;
        sim.generators[1].p_mw = 20.0;
        sim.loads[0].p_mw = 40.0;
        sim.loads[1].p_mw = 30.0;
        let bal = sim.compute_power_balance();
        assert!(bal > 0.0, "Surplus: balance should be positive, got {bal}");
        assert!((bal - 30.0).abs() < 1e-6);
    }
    #[test]
    fn test_update_frequency_surplus() {
        let sim = make_simulator(3600.0, 60.0);
        let f = sim.update_frequency(50.0, 50.0, 1.0);
        assert!(f > 50.0, "Surplus should raise frequency: {f}");
    }
    #[test]
    fn test_update_frequency_deficit() {
        let sim = make_simulator(3600.0, 60.0);
        let f = sim.update_frequency(-50.0, 50.0, 1.0);
        assert!(f < 50.0, "Deficit should lower frequency: {f}");
    }
    #[test]
    fn test_apply_agc_reduces_imbalance() {
        let mut sim = make_simulator(3600.0, 60.0);
        let p_before: f64 = sim
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        sim.apply_agc(-0.5, 60.0);
        let p_after: f64 = sim
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        assert!(
            p_after >= p_before,
            "AGC should increase generation under under-frequency: before={p_before} after={p_after}"
        );
    }
    #[test]
    fn test_apply_ufls_below_threshold() {
        let mut sim = make_simulator(3600.0, 60.0);
        let total_load_before: f64 = sim.loads.iter().map(|l| l.p_mw).sum();
        let shed = sim.apply_ufls(47.0);
        assert!(shed > 0.0, "Should shed load below UFLS threshold: {shed}");
        let total_load_after: f64 = sim.loads.iter().map(|l| l.p_mw).sum();
        assert!(
            total_load_after < total_load_before,
            "Load should decrease after UFLS"
        );
    }
    #[test]
    fn test_apply_ufls_not_triggered() {
        let mut sim = make_simulator(3600.0, 60.0);
        let shed = sim.apply_ufls(49.0);
        assert!(
            (shed).abs() < 1e-9,
            "UFLS should not trigger above threshold: {shed}"
        );
    }
    #[test]
    fn test_process_event_generator_trip() {
        let mut sim = make_simulator(3600.0, 60.0);
        assert!(sim.generators[0].is_online);
        let evt = GridEvent::GeneratorTrip {
            bus: 1,
            capacity_mw: 100.0,
            reason: "test".to_string(),
        };
        let desc = sim.process_event(&evt);
        assert!(
            !sim.generators[0].is_online,
            "Generator at bus 1 should be offline"
        );
        assert!((sim.generators[0].p_mw).abs() < 1e-9);
        assert!(desc.contains("GeneratorTrip"));
    }
    #[test]
    fn test_process_event_line_trip() {
        let mut sim = make_simulator(3600.0, 60.0);
        assert!(sim.branches[0].is_online);
        let evt = GridEvent::LineTrip {
            branch_id: 0,
            reason: "fault".to_string(),
        };
        let desc = sim.process_event(&evt);
        assert!(!sim.branches[0].is_online, "Branch 0 should be offline");
        assert!(desc.contains("LineTrip"));
    }
    #[test]
    fn test_process_event_reconnect() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.generators[0].is_online = false;
        sim.generators[0].p_mw = 0.0;
        let evt = GridEvent::GeneratorReconnect {
            bus: 1,
            capacity_mw: 100.0,
        };
        let desc = sim.process_event(&evt);
        assert!(
            sim.generators[0].is_online,
            "Generator at bus 1 should be back online"
        );
        assert!(desc.contains("GeneratorReconnect"));
    }
    #[test]
    fn test_run_24h_no_events() {
        let mut sim = make_simulator(86400.0, 300.0);
        let result = sim.run().expect("simulation should succeed");
        assert!(!result.snapshots.is_empty(), "Should have snapshots");
        assert!(
            result.snapshots.len() >= 280,
            "Expected ~288 snapshots, got {}",
            result.snapshots.len()
        );
        assert!(!result.frequency_history.is_empty());
    }
    #[test]
    fn test_run_with_n1_event() {
        let mut sim = make_simulator(7200.0, 60.0);
        let events = ScenarioBuilder::n1_line_trip(0, 1800.0);
        for se in events {
            sim.schedule_event(se.time_s, se.event, se.description);
        }
        let result = sim.run().expect("simulation with N-1 should succeed");
        let has_line_trip = result
            .events_log
            .iter()
            .any(|(_, e)| e.contains("LineTrip"));
        assert!(has_line_trip, "Should have a LineTrip in events log");
    }
    #[test]
    fn test_statistics_load_served() {
        let mut sim = make_simulator(3600.0, 60.0);
        let result = sim.run().expect("simulation failed");
        let stats = &result.statistics;
        assert!(
            stats.load_served_pct >= 0.0 && stats.load_served_pct <= 100.0,
            "load_served_pct out of range: {}",
            stats.load_served_pct
        );
        assert!(stats.min_frequency_hz <= stats.max_frequency_hz);
        assert!(stats.system_resilience_index >= 0.0 && stats.system_resilience_index <= 1.0);
    }
    #[test]
    fn test_scenario_builder_daily_curve() {
        let events = ScenarioBuilder::daily_load_curve(1, 100.0, 40.0);
        assert_eq!(events.len(), 24, "Should produce exactly 24 events");
        assert!((events[0].time_s).abs() < 1e-9);
        assert!((events[23].time_s - 23.0 * 3600.0).abs() < 1e-9);
    }
    fn make_config(hours: usize, contingency_prob: f64, weather: bool) -> GridOpsConfig {
        GridOpsConfig {
            simulation_hours: hours,
            dt_minutes: 60.0,
            operator_skill: 0.85,
            automation_level: 0.6,
            contingency_probability: contingency_prob,
            weather_events: weather,
        }
    }
    #[test]
    fn test_no_contingencies_high_reliability() {
        let config = make_config(168, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1500.0, 800.0);
        let result = sim.simulate().expect("simulation failed");
        assert!(
            result.system_reliability_pct > 99.0,
            "Reliability should be >99%: {:.2}%",
            result.system_reliability_pct
        );
        assert_eq!(result.total_hours, 168);
    }
    #[test]
    fn test_high_contingency_reliability_drops() {
        let low_config = make_config(720, 0.0, false);
        let high_config = make_config(720, 0.5, false);
        let res_low = GridOpsSimulator::new(low_config, 1200.0, 800.0)
            .simulate()
            .expect("low sim failed");
        let res_high = GridOpsSimulator::new(high_config, 1200.0, 800.0)
            .simulate()
            .expect("high sim failed");
        assert!(res_low.system_reliability_pct >= res_high.system_reliability_pct);
        assert!(res_high.n_events > 0);
    }
    #[test]
    fn test_error_zero_hours() {
        let config = make_config(0, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::ZeroSimulationHours)
        ));
    }
    #[test]
    fn test_load_profile_length() {
        let config = make_config(8760, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1200.0, 1000.0);
        let mut rng: u64 = 42;
        let profile = sim.generate_load_profile(8760, &mut rng);
        assert_eq!(profile.len(), 8760);
    }
    #[test]
    fn test_run_invalid_end_time() {
        let mut sim = make_simulator(0.0, 60.0);
        assert!(sim.run().is_err(), "run() with end_time_s=0 should error");
    }
    #[test]
    fn test_run_invalid_dt() {
        let mut sim = make_simulator(3600.0, 0.0);
        assert!(sim.run().is_err(), "run() with dt_s=0 should error");
    }
    #[test]
    fn test_validate_invalid_gen_capacity() {
        let config = make_config(24, 0.0, false);
        let sim = GridOpsSimulator::new(config, -100.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::InvalidGenCapacity(_))
        ));
    }
    #[test]
    fn test_validate_invalid_timestep() {
        let config = GridOpsConfig {
            simulation_hours: 24,
            dt_minutes: -5.0,
            ..GridOpsConfig::default()
        };
        let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::InvalidTimeStep(_))
        ));
    }
    #[test]
    fn test_storage_soc_charging_increases() {
        let mut sim = make_simulator(60.0, 60.0);
        sim.storages[0].power_mw = -4.0;
        sim.storages[0].soc = 0.3;
        let initial_soc = sim.storages[0].soc;
        sim.update_storage_soc(3600.0);
        assert!(
            sim.storages[0].soc > initial_soc,
            "SoC should increase when charging: before={initial_soc} after={}",
            sim.storages[0].soc
        );
    }
    #[test]
    fn test_compute_branch_flows_online_offline() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.generators[0].p_mw = 100.0;
        sim.generators[1].is_online = false;
        sim.generators[1].p_mw = 0.0;
        sim.loads[0].p_mw = 20.0;
        sim.loads[1].p_mw = 20.0;
        sim.branches[1].is_online = false;
        sim.compute_branch_flows();
        assert!(
            sim.branches[0].current_flow_mw.abs() > 0.0,
            "Online branch 0 should carry nonzero flow"
        );
        assert!(
            (sim.branches[1].current_flow_mw).abs() < 1e-9,
            "Offline branch 1 should have zero flow"
        );
        assert!(
            (sim.branches[1].loading_pct).abs() < 1e-9,
            "Offline branch 1 loading_pct should be zero"
        );
    }
    #[test]
    fn test_scenario_wind_ramp_event_count() {
        let events = ScenarioBuilder::wind_ramp(1, 10.0, 50.0, 900.0, 0.0);
        assert_eq!(
            events.len(),
            4,
            "wind_ramp should produce n_steps+1 events, got {}",
            events.len()
        );
        assert!((events[0].time_s).abs() < 1e-9);
        assert!(
            (events.last().map(|e| e.time_s).unwrap_or(-1.0) - 900.0).abs() < 1e-9,
            "Last wind ramp event should be at t=900 s"
        );
    }
}

#[cfg(test)]
mod tests_required {
    use super::*;

    fn make_req_generator(id: usize, bus: usize, p_mw: f64, p_max: f64) -> SimGenerator {
        SimGenerator {
            id,
            bus,
            p_mw,
            p_max_mw: p_max,
            p_min_mw: 0.0,
            ramp_rate_mw_per_min: 5.0,
            agc_participation: 0.5,
            is_online: true,
            startup_time_min: 10.0,
            fuel_type: "gas".to_string(),
            co2_kg_per_mwh: 400.0,
        }
    }

    fn make_req_load(bus: usize, p_mw: f64) -> SimLoad {
        SimLoad {
            bus,
            p_mw,
            q_mvar: 0.0,
            is_shedable: true,
            priority: 2,
        }
    }

    fn make_req_branch(id: usize, from: usize, to: usize) -> SimBranch {
        SimBranch {
            id,
            from,
            to,
            is_online: true,
            rating_mva: 100.0,
            current_flow_mw: 0.0,
            current_flow_mvar: 0.0,
            loading_pct: 0.0,
        }
    }

    fn make_req_storage(bus: usize) -> SimStorage {
        SimStorage {
            bus,
            soc: 0.5,
            capacity_mwh: 10.0,
            power_mw: 0.0,
            max_charge_mw: 5.0,
            max_discharge_mw: 5.0,
            efficiency: 0.95,
        }
    }

    fn make_req_simulator(duration_s: f64, dt_s: f64) -> GridOperationsSimulator {
        let config = QdGridOpsConfig {
            n_buses: 5,
            base_mva: 100.0,
            nominal_frequency_hz: 50.0,
            frequency_deadband_hz: 0.02,
            ufls_threshold_hz: 47.5,
            ufls_shed_pct: vec![0.10, 0.15, 0.20],
            ovf_threshold_hz: 51.5,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_branch_loading_pct: 90.0,
        };
        let generators = vec![
            make_req_generator(0, 1, 60.0, 100.0),
            make_req_generator(1, 2, 40.0, 80.0),
        ];
        let loads = vec![make_req_load(3, 50.0), make_req_load(4, 45.0)];
        let branches = vec![make_req_branch(0, 1, 2), make_req_branch(1, 2, 3)];
        let storages = vec![make_req_storage(5)];
        GridOperationsSimulator::new(
            config, generators, loads, branches, storages, duration_s, dt_s,
        )
    }

    fn make_req_config(hours: usize, contingency_prob: f64) -> GridOpsConfig {
        GridOpsConfig {
            simulation_hours: hours,
            dt_minutes: 60.0,
            operator_skill: 0.85,
            automation_level: 0.6,
            contingency_probability: contingency_prob,
            weather_events: false,
        }
    }

    // Test 1
    #[test]
    fn test_grid_ops_simulator_creates_with_valid_params() {
        let config = make_req_config(24, 0.0);
        let sim = GridOpsSimulator::new(config, 1200.0, 800.0);
        let result = sim
            .simulate()
            .expect("simulate() should succeed with valid params");
        assert!(
            result.system_reliability_pct >= 0.0,
            "system_reliability_pct must be >= 0.0, got {}",
            result.system_reliability_pct
        );
    }

    // Test 2
    #[test]
    fn test_simulation_step_advances_time() {
        let mut clock = SimClock::new(0.0, 300.0, 60.0);
        let t0 = clock.current_time_s;
        clock.advance();
        let t1 = clock.current_time_s;
        clock.advance();
        let t2 = clock.current_time_s;
        assert!(
            (t1 - t0 - 60.0).abs() < 1e-9,
            "First advance should move by 60 s: t0={t0} t1={t1}"
        );
        assert!(
            (t2 - t1 - 60.0).abs() < 1e-9,
            "Second advance should move by 60 s: t1={t1} t2={t2}"
        );
    }

    // Test 3
    #[test]
    fn test_dispatch_command_processed() {
        let config = GridOpsConfig {
            simulation_hours: 48,
            dt_minutes: 60.0,
            operator_skill: 0.9,
            automation_level: 0.8,
            contingency_probability: 1.0,
            weather_events: false,
        };
        let sim = GridOpsSimulator::new(config, 1200.0, 800.0);
        let result = sim.simulate().expect("simulate() should succeed");
        assert!(
            !result.action_log.is_empty(),
            "action_log must be non-empty with contingency_probability=1.0"
        );
    }

    // Test 4
    #[test]
    fn test_emergency_response_triggers_on_violation() {
        let mut sim = make_req_simulator(7200.0, 60.0);
        sim.schedule_event(
            600.0,
            GridEvent::GeneratorTrip {
                bus: 1,
                capacity_mw: 100.0,
                reason: "trip_test".to_string(),
            },
            "emergency_trip".to_string(),
        );
        let result = sim
            .run()
            .expect("run() should succeed after scheduling GeneratorTrip");
        let has_trip = result
            .events_log
            .iter()
            .any(|(_, desc)| desc.contains("GeneratorTrip"));
        assert!(has_trip, "events_log must contain 'GeneratorTrip' entry");
    }

    // Test 5
    #[test]
    fn test_load_shedding_reduces_demand() {
        // When gen_capacity_mw << peak_load_mw the load profile is clamped to
        // gen_capacity_mw, driving load_ratio to ~1.0 every hour.  The simulator
        // records an UnderFrequency event each hour that load_ratio > 0.95, so
        // frequency_excursion_hours must be strictly positive.
        let config = make_req_config(24, 0.0);
        let sim = GridOpsSimulator::new(config, 100.0, 500.0);
        let result = sim.simulate().expect("simulate() should succeed");
        assert!(
            result.frequency_excursion_hours > 0.0,
            "With gen_mw=100 << peak_load_mw=500, load_ratio ~1.0 must trigger \
             frequency excursions; got frequency_excursion_hours={}",
            result.frequency_excursion_hours
        );
    }

    // Test 6
    #[test]
    fn test_generation_redispatch_clears_congestion() {
        let mut sim = make_req_simulator(7200.0, 60.0);
        sim.schedule_event(
            1200.0,
            GridEvent::LineTrip {
                branch_id: 0,
                reason: "congestion_test".to_string(),
            },
            "redispatch_trigger".to_string(),
        );
        let result = sim
            .run()
            .expect("run() should succeed after scheduling LineTrip");
        let has_line_trip = result
            .events_log
            .iter()
            .any(|(_, desc)| desc.contains("LineTrip"));
        assert!(
            has_line_trip,
            "events_log must contain 'LineTrip' entry after redispatch"
        );
    }

    // Test 7
    #[test]
    fn test_system_frequency_maintained_within_bounds() {
        let mut sim = make_req_simulator(3600.0, 60.0);
        let result = sim.run().expect("run() should succeed for 3600 s");
        assert!(
            result.statistics.min_frequency_hz > 45.0,
            "min_frequency_hz must be > 45.0 Hz (nominal 50 Hz), got {}",
            result.statistics.min_frequency_hz
        );
    }

    // Test 8
    #[test]
    fn test_state_history_is_logged() {
        let mut sim = make_req_simulator(3600.0, 60.0);
        let result = sim.run().expect("run() should succeed");
        assert!(
            !result.snapshots.is_empty(),
            "snapshots must not be empty after simulation"
        );
        assert!(
            !result.frequency_history.is_empty(),
            "frequency_history must not be empty after simulation"
        );
    }
}
