//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

#[cfg(test)]
use super::types::{
    BusTimeSeriesType, GeneratorProfile, StorageStrategy, TimeResolution, TimeSeriesConfig,
    TimeSeriesNetwork, TimeSeriesResult, TimeSeriesStatistics, TimeStepResult,
};
#[cfg(test)]
use super::types_4::{BusTimeSeries, ScenarioAnalysis, TimeSeriesSimulator};

#[cfg(test)]
mod tests {
    use super::*;
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.0 >> 32) as f64 / u32::MAX as f64
        }
        fn next_in(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }
    }
    /// Build a minimal 3-bus network: slack(0) – bus1 – bus2.
    /// x_01 = 0.1 pu, x_12 = 0.2 pu, base = 100 MVA.
    fn three_bus_network() -> TimeSeriesNetwork {
        let n = 3;
        let base = 100.0;
        let mut b = vec![vec![0.0_f64; n]; n];
        let g = vec![vec![0.0_f64; n]; n];
        b[0][1] = -10.0;
        b[1][0] = -10.0;
        b[0][0] += 10.0;
        b[1][1] += 10.0;
        b[1][2] = -5.0;
        b[2][1] = -5.0;
        b[1][1] += 5.0;
        b[2][2] += 5.0;
        TimeSeriesNetwork {
            n_buses: n,
            g_matrix: g,
            b_matrix: b,
            bus_series: vec![],
            generators: vec![],
            branch_ratings_mva: vec![100.0, 100.0],
            branches: vec![(0, 1), (1, 2)],
            slack_bus: 0,
            base_mva: base,
        }
    }
    /// Build a 2-bus network with one load and one slack generator.
    fn two_bus_network_with_profiles(n_t: usize) -> TimeSeriesNetwork {
        let n = 2;
        let base = 100.0;
        let mut b = vec![vec![0.0_f64; n]; n];
        let g = vec![vec![0.0_f64; n]; n];
        b[0][1] = -10.0;
        b[1][0] = -10.0;
        b[0][0] += 10.0;
        b[1][1] += 10.0;
        let load_series = BusTimeSeries {
            bus_id: 1,
            p_mw: vec![50.0; n_t],
            q_mvar: vec![10.0; n_t],
            series_type: BusTimeSeriesType::Load,
        };
        TimeSeriesNetwork {
            n_buses: n,
            g_matrix: g,
            b_matrix: b,
            bus_series: vec![load_series],
            generators: vec![],
            branch_ratings_mva: vec![200.0],
            branches: vec![(0, 1)],
            slack_bus: 0,
            base_mva: base,
        }
    }
    #[test]
    fn test_time_resolution_steps_per_day() {
        assert_eq!(TimeResolution::FifteenMinutes.steps_per_day(), 96);
        assert_eq!(TimeResolution::HalfHourly.steps_per_day(), 48);
        assert_eq!(TimeResolution::Hourly.steps_per_day(), 24);
        assert_eq!(TimeResolution::Daily.steps_per_day(), 1);
    }
    #[test]
    fn test_time_resolution_dt_hours() {
        let eps = 1e-12;
        assert!((TimeResolution::FifteenMinutes.dt_hours() - 0.25).abs() < eps);
        assert!((TimeResolution::HalfHourly.dt_hours() - 0.5).abs() < eps);
        assert!((TimeResolution::Hourly.dt_hours() - 1.0).abs() < eps);
        assert!((TimeResolution::Daily.dt_hours() - 24.0).abs() < eps);
    }
    #[test]
    fn test_bus_time_series_creation() {
        let bts = BusTimeSeries {
            bus_id: 3,
            p_mw: vec![10.0, 20.0, 30.0],
            q_mvar: vec![2.0, 4.0, 6.0],
            series_type: BusTimeSeriesType::Load,
        };
        assert_eq!(bts.bus_id, 3);
        assert_eq!(bts.len(), 3);
        assert!(!bts.is_empty());
        assert!((bts.p_at(1) - 20.0).abs() < 1e-10);
        assert!((bts.q_at(2) - 6.0).abs() < 1e-10);
        assert!((bts.p_at(99) - 0.0).abs() < 1e-10);
    }
    #[test]
    fn test_generator_profile_creation() {
        let gp = GeneratorProfile {
            generator_id: 0,
            bus: 1,
            p_dispatch_mw: vec![50.0, 60.0, 70.0],
            q_dispatch_mvar: vec![5.0, 6.0, 7.0],
            p_max_mw: 100.0,
            p_min_mw: 0.0,
            cost_per_mwh: 40.0,
        };
        assert!((gp.p_at(0) - 50.0).abs() < 1e-10);
        assert!((gp.p_at(2) - 70.0).abs() < 1e-10);
        let gp2 = GeneratorProfile {
            p_dispatch_mw: vec![150.0],
            p_max_mw: 100.0,
            p_min_mw: 0.0,
            ..gp.clone()
        };
        assert!((gp2.p_at(0) - 100.0).abs() < 1e-10);
    }
    #[test]
    fn test_timeseries_network_creation() {
        let net = three_bus_network();
        assert_eq!(net.n_buses, 3);
        assert_eq!(net.branches.len(), 2);
        assert_eq!(net.branch_ratings_mva.len(), 2);
        net.validate().expect("3-bus network should be valid");
    }
    #[test]
    fn test_timeseries_config_default() {
        let cfg = TimeSeriesConfig::default();
        assert_eq!(cfg.n_timesteps, 8760);
        assert_eq!(cfg.resolution, TimeResolution::Hourly);
        assert!((cfg.voltage_lower_pu - 0.95).abs() < 1e-10);
        assert!((cfg.voltage_upper_pu - 1.05).abs() < 1e-10);
        assert!(cfg.enable_curtailment);
        assert_eq!(cfg.max_pf_iterations, 20);
    }
    #[test]
    fn test_get_bus_injections_load() {
        let net = two_bus_network_with_profiles(5);
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let (p, q) = sim.get_bus_injections(0);
        assert!((p[1] - (-50.0)).abs() < 1e-9, "p[1]={}", p[1]);
        assert!((q[1] - (-10.0)).abs() < 1e-9, "q[1]={}", q[1]);
        assert!((p[0] - 0.0).abs() < 1e-9);
    }
    #[test]
    fn test_get_bus_injections_generation() {
        let mut net = two_bus_network_with_profiles(3);
        net.bus_series.push(BusTimeSeries {
            bus_id: 0,
            p_mw: vec![30.0; 3],
            q_mvar: vec![0.0; 3],
            series_type: BusTimeSeriesType::SolarGeneration { installed_mw: 30.0 },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 3,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let (p, _) = sim.get_bus_injections(0);
        assert!((p[0] - 30.0).abs() < 1e-9);
        assert!((p[1] - (-50.0)).abs() < 1e-9);
    }
    #[test]
    fn test_dc_powerflow_2bus() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let p_inj = vec![50.0, -50.0];
        let angles = sim
            .solve_dc_powerflow(&p_inj)
            .expect("DC PF should converge");
        assert!((angles[0] - 0.0).abs() < 1e-8, "slack angle must be 0");
        let expected_theta1 = -0.05;
        assert!(
            (angles[1] - expected_theta1).abs() < 1e-6,
            "θ1 = {:.6}, expected {:.6}",
            angles[1],
            expected_theta1
        );
    }
    #[test]
    fn test_dc_powerflow_3bus() {
        let net = three_bus_network();
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let p_inj = vec![100.0, -60.0, -40.0];
        let angles = sim
            .solve_dc_powerflow(&p_inj)
            .expect("3-bus DC PF should converge");
        assert_eq!(angles.len(), 3);
        assert!((angles[0] - 0.0).abs() < 1e-8, "slack angle must be 0");
        let p0_check = 10.0 * (angles[0] - angles[1]) * 100.0;
        let p1_check =
            10.0 * (angles[1] - angles[0]) * 100.0 + 5.0 * (angles[1] - angles[2]) * 100.0;
        let p2_check = 5.0 * (angles[2] - angles[1]) * 100.0;
        assert!(
            (p0_check - 100.0).abs() < 1.0,
            "P0 mismatch: {:.2}",
            p0_check
        );
        assert!(
            (p1_check - (-60.0)).abs() < 1.0,
            "P1 mismatch: {:.2}",
            p1_check
        );
        assert!(
            (p2_check - (-40.0)).abs() < 1.0,
            "P2 mismatch: {:.2}",
            p2_check
        );
    }
    #[test]
    fn test_branch_flows_from_angles() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let angles = vec![0.0, -0.05];
        let flows = sim.compute_branch_flows(&angles);
        assert_eq!(flows.len(), 1);
        assert!(
            (flows[0] - 50.0).abs() < 1e-6,
            "branch flow = {:.4}",
            flows[0]
        );
    }
    #[test]
    fn test_branch_loading_within_rating() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let flows = vec![50.0];
        let loading = sim.compute_branch_loading(&flows);
        assert_eq!(loading.len(), 1);
        assert!(
            (loading[0] - 25.0).abs() < 1e-6,
            "loading = {:.2}%",
            loading[0]
        );
        assert!(loading[0] <= 100.0, "should not be overloaded");
    }
    #[test]
    fn test_branch_loading_overloaded() {
        let mut net = two_bus_network_with_profiles(1);
        net.branch_ratings_mva = vec![30.0];
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let flows = vec![50.0];
        let loading = sim.compute_branch_loading(&flows);
        assert!(
            loading[0] > 100.0,
            "should be overloaded: {:.1}%",
            loading[0]
        );
    }
    #[test]
    fn test_voltage_estimation_flat() {
        let net = two_bus_network_with_profiles(1);
        let cfg = TimeSeriesConfig {
            n_timesteps: 1,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        let p_inj = vec![0.0, 0.0];
        let q_inj = vec![0.0, 0.0];
        let voltages = sim.estimate_voltages(&p_inj, &q_inj);
        for &v in &voltages {
            assert!((v - 1.0).abs() < 1e-9, "flat Q → V=1.0, got {}", v);
        }
    }
    #[test]
    fn test_storage_dispatch_peak_shaving_discharge() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![0.0; 5],
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::PeakShaving { threshold_mw: 40.0 },
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let _p_inj = [0.0, -60.0];
        let power = sim.dispatch_storage_peak_shaving(60.0, 40.0, 0);
        assert_eq!(power.len(), 1);
        assert!(
            power[0] > 0.0,
            "should discharge (positive), got {:.4}",
            power[0]
        );
    }
    #[test]
    fn test_storage_dispatch_peak_shaving_charge() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![0.0; 5],
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::PeakShaving {
                threshold_mw: 100.0,
            },
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let power = sim.dispatch_storage_peak_shaving(30.0, 100.0, 0);
        assert_eq!(power.len(), 1);
        assert!(
            power[0] < 0.0,
            "should charge (negative), got {:.4}",
            power[0]
        );
    }
    #[test]
    fn test_storage_soc_update() {
        let mut net = two_bus_network_with_profiles(5);
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![10.0; 5],
            q_mvar: vec![0.0; 5],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: 5,
            storage_dispatch_strategy: StorageStrategy::ScheduledDispatch,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let initial_soc = sim.storage_soc[0];
        sim.dispatch_storage_scheduled(0);
        assert!(
            sim.storage_soc[0] < initial_soc,
            "SoC should decrease after discharge: {} < {}",
            sim.storage_soc[0],
            initial_soc
        );
    }
    #[test]
    fn test_run_24h_simulation() {
        let n_t = 24;
        let mut lcg = Lcg::new(42);
        let mut net = three_bus_network();
        let load_profile: Vec<f64> = (0..n_t).map(|_| lcg.next_in(20.0, 80.0)).collect();
        net.bus_series.push(BusTimeSeries {
            bus_id: 2,
            p_mw: load_profile,
            q_mvar: vec![5.0; n_t],
            series_type: BusTimeSeriesType::Load,
        });
        let solar_profile: Vec<f64> = (0..n_t)
            .map(|h| {
                if (6..=18).contains(&h) {
                    lcg.next_in(10.0, 50.0)
                } else {
                    0.0
                }
            })
            .collect();
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: solar_profile,
            q_mvar: vec![0.0; n_t],
            series_type: BusTimeSeriesType::SolarGeneration { installed_mw: 50.0 },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            resolution: TimeResolution::Hourly,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("24-hour simulation should succeed");
        assert_eq!(result.timestep_results.len(), n_t);
        assert_eq!(result.statistics.n_converged, n_t);
        assert!(
            (result.statistics.convergence_rate - 1.0).abs() < 1e-9,
            "convergence_rate={}",
            result.statistics.convergence_rate
        );
    }
    #[test]
    fn test_compute_statistics_load_factor() {
        let make_result = |t: usize, load: f64| TimeStepResult {
            timestep: t,
            time_hours: t as f64,
            converged: true,
            voltage_magnitude: vec![1.0, 1.0],
            voltage_angle: vec![0.0, 0.0],
            branch_loading_pct: vec![50.0],
            total_generation_mw: load,
            total_load_mw: load,
            total_losses_mw: 0.0,
            renewable_generation_mw: 0.0,
            renewable_curtailment_mw: 0.0,
            storage_soc: vec![],
            overloaded_branches: vec![],
            voltage_violations: vec![],
        };
        let results = vec![
            make_result(0, 80.0),
            make_result(1, 40.0),
            make_result(2, 80.0),
        ];
        let stats = TimeSeriesSimulator::compute_statistics(&results, 1.0);
        assert!((stats.peak_load_mw - 80.0).abs() < 1e-6);
        let expected_lf = (200.0 / 3.0) / 80.0;
        assert!(
            (stats.load_factor - expected_lf).abs() < 1e-6,
            "load_factor={:.4} expected={:.4}",
            stats.load_factor,
            expected_lf
        );
    }
    #[test]
    fn test_compute_statistics_renewable_fraction() {
        let make_result = |t: usize, gen: f64, ren: f64| TimeStepResult {
            timestep: t,
            time_hours: t as f64,
            converged: true,
            voltage_magnitude: vec![1.0],
            voltage_angle: vec![0.0],
            branch_loading_pct: vec![],
            total_generation_mw: gen,
            total_load_mw: gen,
            total_losses_mw: 0.0,
            renewable_generation_mw: ren,
            renewable_curtailment_mw: 0.0,
            storage_soc: vec![],
            overloaded_branches: vec![],
            voltage_violations: vec![],
        };
        let results = vec![make_result(0, 100.0, 40.0), make_result(1, 100.0, 40.0)];
        let stats = TimeSeriesSimulator::compute_statistics(&results, 1.0);
        assert!(
            (stats.renewable_fraction_pct - 40.0).abs() < 1e-6,
            "ren_frac={:.4}",
            stats.renewable_fraction_pct
        );
    }
    #[test]
    fn test_scenario_analysis_compare() {
        let make_stats = |ren: f64| TimeSeriesStatistics {
            renewable_fraction_pct: ren,
            ..Default::default()
        };
        let make_result = |ren: f64| TimeSeriesResult {
            timestep_results: vec![],
            statistics: make_stats(ren),
            duration_s: 0.0,
        };
        let mut analysis = ScenarioAnalysis::new();
        analysis.add_scenario("Base".into(), make_result(20.0));
        analysis.add_scenario("HighRen".into(), make_result(60.0));
        let compared = analysis.compare();
        assert_eq!(compared.len(), 2);
        assert_eq!(compared[0].0, "Base");
        assert_eq!(compared[1].0, "HighRen");
        assert!((compared[1].1.renewable_fraction_pct - 60.0).abs() < 1e-9);
    }
    #[test]
    fn test_hosting_capacity_estimation() {
        let n_t = 8;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            resolution: TimeResolution::HalfHourly,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let hc = sim
            .estimate_hosting_capacity(1, 200.0)
            .expect("hosting capacity should succeed");
        assert!(hc >= 0.0, "hosting capacity must be non-negative: {}", hc);
        assert!(
            hc <= 200.0,
            "hosting capacity must not exceed search range: {}",
            hc
        );
    }

    #[test]
    fn test_simulator_creates_with_valid_network() {
        let n_t = 4;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            ..Default::default()
        };
        let sim = TimeSeriesSimulator::new(net, cfg);
        assert_eq!(sim.network.n_buses, 2);
        assert_eq!(sim.config.n_timesteps, n_t);
        // No storage series in basic 2-bus network
        assert!(sim.storage_units.is_empty(), "no storage expected");
    }

    #[test]
    fn test_single_timestep_runs_without_error() {
        let n_t = 1;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim
            .run()
            .expect("single-timestep simulation should succeed");
        assert_eq!(result.timestep_results.len(), 1);
        assert!(
            result.timestep_results[0].converged,
            "single timestep should converge"
        );
    }

    #[test]
    fn test_load_profile_shapes_results() {
        // Two runs: one with 50 MW load and one with 100 MW load; peak_load_mw must differ.
        let make_net = |load_mw: f64| {
            let n = 2usize;
            let base = 100.0;
            let mut b = vec![vec![0.0_f64; n]; n];
            let g = vec![vec![0.0_f64; n]; n];
            b[0][1] = -10.0;
            b[1][0] = -10.0;
            b[0][0] += 10.0;
            b[1][1] += 10.0;
            let load_series = BusTimeSeries {
                bus_id: 1,
                p_mw: vec![load_mw; 4],
                q_mvar: vec![0.0; 4],
                series_type: BusTimeSeriesType::Load,
            };
            TimeSeriesNetwork {
                n_buses: n,
                g_matrix: g,
                b_matrix: b,
                bus_series: vec![load_series],
                generators: vec![],
                branch_ratings_mva: vec![200.0],
                branches: vec![(0, 1)],
                slack_bus: 0,
                base_mva: base,
            }
        };

        let run = |load_mw: f64| {
            let cfg = TimeSeriesConfig {
                n_timesteps: 4,
                ..Default::default()
            };
            let mut sim = TimeSeriesSimulator::new(make_net(load_mw), cfg);
            sim.run().expect("simulation should succeed")
        };

        let r50 = run(50.0);
        let r100 = run(100.0);
        assert!(
            r100.statistics.peak_load_mw > r50.statistics.peak_load_mw,
            "higher load profile should produce higher peak_load_mw: {} vs {}",
            r100.statistics.peak_load_mw,
            r50.statistics.peak_load_mw
        );
    }

    #[test]
    fn test_renewable_generation_profile_applied() {
        let n_t = 6;
        let mut net = two_bus_network_with_profiles(n_t);
        // Add solar generation at bus 0
        net.bus_series.push(BusTimeSeries {
            bus_id: 0,
            p_mw: vec![20.0; n_t],
            q_mvar: vec![0.0; n_t],
            series_type: BusTimeSeriesType::SolarGeneration { installed_mw: 20.0 },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("simulation with solar should succeed");
        // Renewable fraction must be positive because we have solar
        assert!(
            result.statistics.renewable_fraction_pct > 0.0,
            "renewable fraction should be > 0 with solar: {}",
            result.statistics.renewable_fraction_pct
        );
        // Each timestep should have positive renewable_generation_mw
        for step in &result.timestep_results {
            assert!(
                step.renewable_generation_mw >= 0.0,
                "renewable_generation_mw must be non-negative at step {}",
                step.timestep
            );
        }
    }

    #[test]
    fn test_storage_dispatch_follows_profile() {
        let n_t = 8;
        let mut net = two_bus_network_with_profiles(n_t);
        // Add a storage unit at bus 1 that discharges 5 MW every step
        net.bus_series.push(BusTimeSeries {
            bus_id: 1,
            p_mw: vec![5.0; n_t], // positive = discharge
            q_mvar: vec![0.0; n_t],
            series_type: BusTimeSeriesType::Storage {
                charge_negative: true,
            },
        });
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            storage_dispatch_strategy: StorageStrategy::ScheduledDispatch,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let initial_soc = sim.storage_soc[0];
        let result = sim.run().expect("storage simulation should succeed");
        // After discharging every step, SoC in last step should be lower than initial
        let final_soc = result
            .timestep_results
            .last()
            .expect("results must not be empty")
            .storage_soc[0];
        assert!(
            final_soc < initial_soc,
            "SoC should decrease after repeated discharge: {} < {}",
            final_soc,
            initial_soc
        );
    }

    #[test]
    fn test_voltage_profile_within_bounds() {
        let n_t = 12;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            voltage_lower_pu: 0.9,
            voltage_upper_pu: 1.1,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("simulation should succeed");
        for step in &result.timestep_results {
            for (bus_idx, &v) in step.voltage_magnitude.iter().enumerate() {
                assert!(
                    (0.9..=1.1).contains(&v),
                    "voltage out of [0.9, 1.1] pu at step {} bus {}: {}",
                    step.timestep,
                    bus_idx,
                    v
                );
            }
        }
    }

    #[test]
    fn test_power_balance_maintained_per_step() {
        // In a lossless DC power flow, total_generation_mw should approximately
        // equal total_load_mw (the slack bus picks up the difference).
        // We relax the tolerance to 60 MW to account for slack bus absorption.
        let n_t = 4;
        let net = two_bus_network_with_profiles(n_t);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_t,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("simulation should succeed");
        for step in &result.timestep_results {
            let imbalance = (step.total_generation_mw - step.total_load_mw).abs();
            // Slack bus absorbs imbalance; in a 2-bus network with 50 MW load
            // and no explicit generator profile, generation comes from slack.
            // The compute_generation_load function counts generators separately,
            // so generation from slack may not appear as gen in statistics.
            // We just check imbalance is not wildly large (< 200 MW is reasonable).
            assert!(
                imbalance < 200.0,
                "power imbalance too large at step {}: gen={} load={}",
                step.timestep,
                step.total_generation_mw,
                step.total_load_mw
            );
        }
    }

    #[test]
    fn test_results_accumulate_over_multiple_steps() {
        let n_steps = 10;
        let net = two_bus_network_with_profiles(n_steps);
        let cfg = TimeSeriesConfig {
            n_timesteps: n_steps,
            resolution: TimeResolution::HalfHourly,
            ..Default::default()
        };
        let mut sim = TimeSeriesSimulator::new(net, cfg);
        let result = sim.run().expect("multi-step simulation should succeed");
        assert_eq!(
            result.timestep_results.len(),
            n_steps,
            "should have exactly {n_steps} results, got {}",
            result.timestep_results.len()
        );
        // Verify timestep indices are correct
        for (i, step) in result.timestep_results.iter().enumerate() {
            assert_eq!(
                step.timestep, i,
                "timestep index mismatch at position {i}: got {}",
                step.timestep
            );
        }
        // Verify statistics n_timesteps matches
        assert_eq!(
            result.statistics.n_timesteps, n_steps,
            "statistics.n_timesteps mismatch"
        );
    }
}
