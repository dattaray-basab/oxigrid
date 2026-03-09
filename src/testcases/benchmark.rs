//! Benchmark scenarios for power flow validation and comparison.
//!
//! Provides standard benchmark suites with known (reference) solutions for
//! validating power flow solver implementations.  A `BenchmarkReport` records
//! whether the solver's numerical results fall within the tolerance of the
//! published reference values.

use crate::error::Result;
use crate::network::topology::PowerNetwork;
use crate::testcases::ieee::{ieee14, ieee30, ieee57};

#[cfg(feature = "powerflow")]
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Known reference solution for a power flow benchmark.
#[derive(Debug, Clone)]
pub struct ExpectedPowerFlowResult {
    /// Whether the power flow is expected to converge.
    pub converged: bool,
    /// Typical iteration count for Newton-Raphson (informational).
    pub n_iterations: usize,
    /// Maximum voltage magnitude across all buses \[p.u.\].
    pub max_voltage_pu: f64,
    /// Minimum voltage magnitude across all buses \[p.u.\].
    pub min_voltage_pu: f64,
    /// Total active power losses \[MW\].
    pub total_losses_mw: f64,
    /// Slack bus active power generation \[MW\].
    pub slack_generation_mw: f64,
    /// Absolute comparison tolerance \[p.u.\] (applies to voltages).
    pub tolerance: f64,
}

/// A benchmark scenario combining a network with a reference solution.
pub struct BenchmarkScenario {
    /// Descriptive scenario name.
    pub name: String,
    /// The power network to solve.
    pub network: PowerNetwork,
    /// Reference solution for comparison.
    pub expected_result: ExpectedPowerFlowResult,
}

/// Report produced by running a single benchmark.
#[derive(Debug, Clone)]
pub struct BenchmarkReport {
    /// Name of the scenario that was run.
    pub scenario_name: String,
    /// Whether all checks passed within tolerance.
    pub passed: bool,
    /// Whether the solver converged.
    pub actual_converged: bool,
    /// Number of iterations the solver took.
    pub actual_iterations: usize,
    /// Maximum absolute voltage error vs. reference \[p.u.\].
    pub voltage_error_pu: f64,
    /// Absolute losses error vs. reference \[MW\].
    pub losses_error_mw: f64,
    /// Any diagnostic messages (warnings, notes).
    pub notes: Vec<String>,
}

impl BenchmarkReport {
    /// Return `true` iff this report indicates a passing result.
    pub fn is_pass(&self) -> bool {
        self.passed
    }
}

// ---------------------------------------------------------------------------
// Standard benchmark suite
// ---------------------------------------------------------------------------

/// Build the standard power flow benchmark suite.
///
/// Includes IEEE 14, 30, and 57-bus systems with published reference solutions.
/// The reference values are taken from the MATPOWER documentation and the
/// Power Systems Test Case Archive (University of Washington).
pub fn power_flow_benchmarks() -> Vec<BenchmarkScenario> {
    let mut benchmarks = Vec::new();

    // IEEE 14-bus
    if let Ok(net) = ieee14() {
        benchmarks.push(BenchmarkScenario {
            name: "IEEE 14-Bus".to_string(),
            network: net,
            expected_result: ExpectedPowerFlowResult {
                converged: true,
                n_iterations: 4,
                max_voltage_pu: 1.060,
                min_voltage_pu: 1.020,
                total_losses_mw: 13.4,
                slack_generation_mw: 232.4,
                tolerance: 0.02,
            },
        });
    }

    // IEEE 30-bus
    if let Ok(net) = ieee30() {
        benchmarks.push(BenchmarkScenario {
            name: "IEEE 30-Bus".to_string(),
            network: net,
            expected_result: ExpectedPowerFlowResult {
                converged: true,
                n_iterations: 5,
                max_voltage_pu: 1.060,
                min_voltage_pu: 0.995,
                total_losses_mw: 17.6,
                slack_generation_mw: 260.2,
                tolerance: 0.02,
            },
        });
    }

    // IEEE 57-bus
    if let Ok(net) = ieee57() {
        benchmarks.push(BenchmarkScenario {
            name: "IEEE 57-Bus".to_string(),
            network: net,
            expected_result: ExpectedPowerFlowResult {
                converged: true,
                n_iterations: 5,
                max_voltage_pu: 1.040,
                min_voltage_pu: 0.930,
                total_losses_mw: 27.9,
                slack_generation_mw: 478.9,
                tolerance: 0.03,
            },
        });
    }

    benchmarks
}

// ---------------------------------------------------------------------------
// IEEE 9-bus stability benchmark
// ---------------------------------------------------------------------------

/// IEEE 9-bus Single-Machine-Infinite-Bus (SMIB) stability benchmark.
///
/// Returns the power network and transient simulation configuration for
/// the classic Anderson & Fouad 9-bus stability test case.
///
/// The system consists of 9 buses, 9 branches, and 3 generators.
/// It is widely used for transient stability validation.
#[cfg(feature = "stability")]
pub fn ieee9_stability() -> Result<(PowerNetwork, crate::stability::transient::TransientConfig)> {
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::Generator;
    use crate::stability::transient::{TransientConfig, TransientEvent};
    use crate::units::{Power, ReactivePower, Voltage};

    let mut net = PowerNetwork::new(100.0);

    // Bus data (Anderson & Fouad, Power Systems Control & Stability)
    let bus_info: &[(usize, BusType, f64, f64, f64)] = &[
        (1, BusType::Slack, 0.0, 0.0, 1.040),
        (2, BusType::PV, 0.0, 0.0, 1.025),
        (3, BusType::PV, 0.0, 0.0, 1.025),
        (4, BusType::PQ, 0.0, 0.0, 1.026),
        (5, BusType::PQ, 125.0, 50.0, 0.996),
        (6, BusType::PQ, 90.0, 30.0, 1.013),
        (7, BusType::PQ, 0.0, 0.0, 1.026),
        (8, BusType::PQ, 100.0, 35.0, 1.016),
        (9, BusType::PQ, 0.0, 0.0, 1.032),
    ];

    for &(id, bus_type, pd, qd, vm) in bus_info {
        net.buses.push(Bus {
            id,
            name: format!("Bus {id}"),
            bus_type,
            base_kv: Voltage(230.0),
            vm,
            va: 0.0,
            pd: Power(pd),
            qd: ReactivePower(qd),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    // Branches
    let branch_data: &[(usize, usize, f64, f64, f64)] = &[
        (1, 4, 0.0, 0.0576, 0.0),
        (4, 5, 0.0100, 0.0850, 0.1760),
        (5, 6, 0.0170, 0.0920, 0.1580),
        (3, 6, 0.0, 0.0586, 0.0),
        (6, 7, 0.0390, 0.1700, 0.3580),
        (7, 8, 0.0085, 0.0720, 0.1490),
        (8, 2, 0.0, 0.0625, 0.0),
        (8, 9, 0.0320, 0.1610, 0.3060),
        (9, 4, 0.0100, 0.0850, 0.1760),
    ];

    for &(from, to, r, x, b) in branch_data {
        net.branches.push(Branch {
            from_bus: from,
            to_bus: to,
            r,
            x,
            b,
            rate_a: 250.0,
            rate_b: 250.0,
            rate_c: 250.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
    }

    // Generators
    let gen_data: &[(usize, f64, f64, f64, f64)] = &[
        (1, 71.6, 27.0, 1.040, 247.5),
        (2, 163.0, 6.7, 1.025, 192.0),
        (3, 85.0, -10.9, 1.025, 128.0),
    ];
    for &(bus_id, pg, qg, vg, pmax) in gen_data {
        net.generators.push(Generator {
            bus_id,
            pg,
            qg,
            qmax: pmax * 0.5,
            qmin: -pmax * 0.3,
            vg,
            mbase: 100.0,
            status: true,
            pmax,
            pmin: 0.0,
        });
    }

    // Transient config: 3-phase fault at bus 7 at t=0.1s, cleared at t=0.25s
    let cfg = TransientConfig {
        t_end: 3.0,
        events: vec![
            TransientEvent::FaultOn {
                time: 0.1,
                bus: 6,
                fault_impedance: 0.0,
            },
            TransientEvent::FaultOff { time: 0.25, bus: 6 },
        ],
        ..TransientConfig::default()
    };

    Ok((net, cfg))
}

// ---------------------------------------------------------------------------
// Benchmark execution
// ---------------------------------------------------------------------------

/// Run a single benchmark scenario and return a `BenchmarkReport`.
///
/// If the `powerflow` feature is disabled, the report will always show
/// `passed = false` with an explanatory note.
#[cfg(feature = "powerflow")]
pub fn run_benchmark(
    scenario: &BenchmarkScenario,
    solver: &crate::powerflow::newton_raphson::NewtonRaphsonSolver,
) -> Result<BenchmarkReport> {
    use crate::powerflow::PowerFlowSolver;

    let cfg = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };

    let mut notes = Vec::new();
    let result = solver.solve(&scenario.network, &cfg);

    match result {
        Ok(pf) => {
            let max_v = pf
                .voltage_magnitude
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let min_v = pf
                .voltage_magnitude
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);
            let losses = pf.total_p_loss();

            let exp = &scenario.expected_result;
            let tol = exp.tolerance;

            let v_err = (max_v - exp.max_voltage_pu)
                .abs()
                .max((min_v - exp.min_voltage_pu).abs());
            let l_err = (losses - exp.total_losses_mw).abs();

            let passed = pf.converged == exp.converged
                && v_err <= tol
                && l_err <= exp.total_losses_mw * 0.15 + 1.0;

            if !pf.converged && exp.converged {
                notes.push("Solver did not converge (expected convergence)".to_string());
            }
            if v_err > tol {
                notes.push(format!(
                    "Voltage error {v_err:.4} p.u. exceeds tolerance {tol:.4}"
                ));
            }

            Ok(BenchmarkReport {
                scenario_name: scenario.name.clone(),
                passed,
                actual_converged: pf.converged,
                actual_iterations: pf.iterations,
                voltage_error_pu: v_err,
                losses_error_mw: l_err,
                notes,
            })
        }
        Err(e) => {
            notes.push(format!("Solver returned error: {e}"));
            Ok(BenchmarkReport {
                scenario_name: scenario.name.clone(),
                passed: false,
                actual_converged: false,
                actual_iterations: 0,
                voltage_error_pu: f64::NAN,
                losses_error_mw: f64::NAN,
                notes,
            })
        }
    }
}

/// Run all standard benchmarks and return reports.
///
/// Uses Newton-Raphson with default settings.
#[cfg(feature = "powerflow")]
pub fn validate_all_benchmarks() -> Vec<BenchmarkReport> {
    use crate::powerflow::newton_raphson::NewtonRaphsonSolver;

    let scenarios = power_flow_benchmarks();
    let solver = NewtonRaphsonSolver;
    let mut reports = Vec::new();

    for scenario in &scenarios {
        match run_benchmark(scenario, &solver) {
            Ok(report) => reports.push(report),
            Err(e) => reports.push(BenchmarkReport {
                scenario_name: scenario.name.clone(),
                passed: false,
                actual_converged: false,
                actual_iterations: 0,
                voltage_error_pu: f64::NAN,
                losses_error_mw: f64::NAN,
                notes: vec![format!("Error: {e}")],
            }),
        }
    }

    reports
}
