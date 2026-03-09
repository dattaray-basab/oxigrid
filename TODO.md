# OxiGrid TODO

## Legend
- [x] Done
- [ ] Not started
- [~] Partial / needs improvement

---

## Phase 1: Foundation (Power Flow)

### 1.1 Project Scaffold
- [x] `cargo init --lib`, edition 2021, MSRV 1.75
- [x] `Cargo.toml` with deps: petgraph 0.8.3, serde 1.0, serde_json, thiserror 2.0.18, log 0.4, tracing 0.1, nalgebra 0.34.1, sprs 0.11.4, num-complex 0.4.6
- [x] dev-deps: criterion 0.8.2, proptest 1.10.0, approx 0.5
- [x] Feature flags: std, no_std_compat, powerflow, stability, battery, battery-p2d, renewable, optimize, harmonics, protection, forecast-ml, io-matpower, io-csv, io-oxirs, simd, parallel
- [x] Feature-gate modules behind their respective feature flags (lib.rs, Cargo.toml, integration tests)
- [ ] `rayon` dependency behind `parallel` feature flag

### 1.2 Error Types (`src/error.rs`)
- [x] `OxiGridError` enum with thiserror: Convergence, InvalidNetwork, ParseError, LinearAlgebra, InvalidParameter
- [x] `Result<T>` type alias

### 1.3 Units Module (`src/units/`)
- [x] `electrical.rs`: Voltage, Current, Power, ReactivePower, Impedance, Frequency, PerUnit (newtype pattern)
- [x] `thermal.rs`: Temperature (Kelvin internal, Celsius conversion), ThermalConductivity, HeatCapacity
- [x] `energy.rs`: Energy (Wh), Capacity (Ah), StateOfCharge (0.0..=1.0 clamped)
- [x] `conversion.rs`: base_impedance(), base_current() helpers
- [x] Derive: Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default
- [x] Arithmetic ops: Add, Sub, Mul<f64>, Div<f64>, Neg (via macro)
- [x] Display impl with units
- [x] PerUnit conversion methods on Voltage, Power, ReactivePower, Current
- [ ] `no_std` support for units module (blueprint requires `no_std` for units/)
- [x] proptest roundtrip tests for PerUnit conversion (blueprint section 7)
- [x] `From` trait implementations between compatible types (`Voltage * Current -> Power`, `Power::to_energy_wh`, `Energy::to_power_w`)

### 1.4 Network Module (`src/network/`)
- [x] `bus.rs`: BusType enum (Slack, PV, PQ), Bus struct with id, name, bus_type, base_kv, vm, va, pd, qd, gs, bs, zone
- [x] `branch.rs`: Branch struct with from_bus, to_bus, r, x, b, rates, tap, shift, status; effective_tap(), tap_complex()
- [x] `topology.rs`: PowerNetwork struct (buses Vec, branches Vec, generators Vec, base_mva); Generator struct
- [x] `topology.rs`: bus_count(), branch_count(), bus_index(), slack_bus_index(), net_injection(), validate(), admittance_matrix(), from_matpower()
- [x] `admittance.rs`: build_y_bus() — sparse Y-bus via sprs::TriMat -> CsMat<Complex64>, pi-model with tap, shunt elements
- [x] `formats/matpower.rs`: MATPOWER .m file parser (baseMVA, bus, branch, gen sections)
- [x] `formats/ieee_cdf.rs`: IEEE Common Data Format parser (blueprint section 3)
- [x] `formats/pandapower.rs`: pandapower JSON parser (blueprint section 3)
- [x] `from_ieee_cdf()` method on PowerNetwork
- [x] `incidence_matrix()` method on PowerNetwork (blueprint section 4.2)
- [ ] Use petgraph::Graph<Bus, Branch> internally (currently uses flat Vec; blueprint specifies petgraph wrapping)

### 1.5 Power Flow Module (`src/powerflow/`)
- [x] `mod.rs`: PowerFlowMethod enum, PowerFlowConfig (default: NR, 50 iter, 1e-8 tol), PowerFlowSolver trait, solve_powerflow() dispatcher
- [x] `jacobian.rs`: Full Jacobian builder (H, N, M, L sub-matrices) using dense nalgebra::DMatrix
- [x] `newton_raphson.rs`: Newton-Raphson AC power flow — bus classification, mismatch vectors, Jacobian solve via LU, voltage update (polar form [dtheta; dV/V])
- [x] `dc_powerflow.rs`: DC approximation — B' matrix, linear solve, theta calculation
- [x] `result.rs`: PowerFlowResult with voltage_magnitude, voltage_angle, p/q_injected, converged, iterations, max_mismatch; Display impl
- [x] `fast_decoupled.rs`: Fast Decoupled Load Flow (FDLF) — B' and B'' matrices (Stott & Alsac 1974)
- [x] `continuation.rs`: Continuation power flow for voltage stability (blueprint section 3)
- [ ] Branch power flow calculation in results (P/Q flow per branch, not just bus injections)
- [ ] Total system losses calculation (currently only sum of injections)
- [x] Q-limit enforcement for PV buses (switch PV→PQ when Q exceeds limits, re-solve with fixed Q)
- [x] Sparse Jacobian: iterate Y-bus non-zeros directly (avoids O(n²) dense conversion); parallel rayon Jacobian behind `parallel` feature flag
- [x] Step-size limiting for numerical stability (±0.5 rad angle, ±0.2 p.u. voltage per iteration)

### 1.6 Prelude & lib.rs
- [x] `prelude.rs`: Re-exports of OxiGridError, Result, Bus, Branch, BusType, Generator, PowerNetwork, PowerFlowConfig, PowerFlowMethod, PowerFlowResult, PowerFlowSolver, units::*
- [x] `lib.rs`: Module declarations for error, network, powerflow, prelude, units
- [x] Feature gates in lib.rs (conditionally compile modules based on feature flags)
- [x] Top-level doc comment with crate overview and examples

### 1.7 Test Data & Integration Tests
- [x] `tests/data/ieee14.m`: IEEE 14-bus MATPOWER format
- [x] `tests/data/ieee30.m`: IEEE 30-bus MATPOWER format
- [x] `tests/ieee14_test.rs`: NR convergence, voltage validation (1e-3 p.u. tolerance), DC power flow, bus count
- [x] `tests/ieee30_test.rs`: Parse validation, NR convergence, DC power flow, slack voltage
- [x] `tests/data/ieee57.m` + tests (blueprint section 7)
- [x] `tests/data/ieee118.m` + tests (5 tests: parse, NR, DC, slack voltage, incidence)
- [x] Tighten IEEE 14-bus voltage tolerance from 1e-3 to 1e-4 p.u.
- [x] Branch power flow validation tests (ieee30_test.rs: 5 branch flow tests; ieee14 branch flows via existing NR tests)
- [x] proptest: random network power conservation
- [x] proptest: PerUnit roundtrip conversion

### 1.8 Benchmarks
- [x] `benches/powerflow_bench.rs`: criterion benchmarks for IEEE 14-bus NR, IEEE 30-bus NR, IEEE 14-bus DC
- [x] IEEE 118-bus benchmark (ieee118_nr, ieee118_dc in powerflow_bench.rs)
- [ ] IEEE 300-bus benchmark (target: < 50ms) — needs ieee300.m data

### 1.9 Documentation
- [x] `///` doc comments on key `pub` items: `topology.rs` (Generator, PowerNetwork, all pub fn), `electrical.rs` (all types + methods), `energy.rs`, `thermal.rs`
- [x] Module-level `//!` doc comments in all 21 mod.rs files
- [ ] Mathematical background sections (LaTeX notation)
- [x] `examples/ieee14_powerflow.rs` runnable example (blueprint section 8)

### 1.10 CI/CD
- [x] `.github/workflows/ci.yml`: fmt, clippy, test (3 platforms), MSRV, bench dry-run, docs, coverage

---

## Phase 2: Battery Core

### 2.1 Battery ECM (`src/battery/ecm/`)
- [x] `BatteryModel` trait: voltage(), step() -> BatteryState
- [x] `BatteryState` struct: voltage, soc, temperature, internal_resistance, capacity_remaining
- [x] `rint.rs`: Internal resistance model (Rint) — simplest ECM, V = OCV(SoC) - I*R0
- [x] `rc.rs`: 1RC Thevenin model — R0 + (R1||C1), exponential voltage relaxation
- [x] `rc.rs`: 2RC Thevenin model — R0 + (R1||C1) + (R2||C2), two time constants
- [x] `OcvSocCurve`: OCV-SoC lookup table with interpolation
- [x] `parameter.rs`: Parameter identification (optirs integration placeholder)
- [x] Temperature-dependent parameters (R0(T), capacity(T))

### 2.2 SoC Estimation (`src/battery/soc.rs`)
- [x] Coulomb counting: SoC integration with efficiency factor
- [x] Extended Kalman Filter (EKF): State estimation with ECM as process model
- [x] Unscented Kalman Filter (UKF): Alternative to EKF for nonlinear systems

### 2.3 Thermal Model (`src/battery/thermal.rs`)
- [x] Lumped thermal model: dT/dt = (Q_gen - Q_dissipated) / (m * Cp)
- [x] Heat generation: I^2*R (Joule) + entropic heating
- [x] Convective cooling: h*A*(T - T_ambient)
- [ ] 1D thermal model (optional)

### 2.4 Pack Configuration (`src/battery/pack.rs`)
- [x] Series/parallel cell arrangement
- [x] Cell balancing (passive)
- [x] Pack-level voltage, current, SoC aggregation
- [x] BMS interface trait

### 2.5 Battery Tests
- [x] `tests/battery_validation/kokam_75ah.rs`: 1C discharge curve validation (RMSE < 50mV)
- [x] `tests/battery_validation/lfp_cell.rs`: LFP chemistry validation
- [x] EKF SoC estimation accuracy test (< 2% error)
- [x] proptest: charge/discharge energy conservation
- [x] `benches/battery_bench.rs`: 1000 cycle ECM benchmark (target: < 100ms)

---

## Phase 3: Renewables & Optimization

### 3.1 Solar PV (`src/renewable/solar/`)
- [x] `irradiance.rs`: Solar position (Spencer 1971), Liu & Jordan POA, Erbs decomposition
- [x] `pv_cell.rs`: Single-diode 5-parameter model, NR I-V, golden-section MPP
- [x] `inverter.rs`: CEC/Sandia inverter model, European/CEC efficiency ratings
- [x] `mppt.rs`: Perturb & Observe, Incremental Conductance

### 3.2 Wind (`src/renewable/wind/`)
- [x] `turbine.rs`: Power curve, Betz limit, Weibull AEP, log wind profile
- [x] `wake.rs`: Jensen + Frandsen wake, square-sum superposition, met wind convention
- [x] `farm.rs`: Regular grid layout, wake-corrected farm output

### 3.3 Forecasting (`src/renewable/forecast/`)
- [x] `persistence.rs`: Naive persistence, diurnal persistence, skill score
- [x] `arima.rs`: AR(p) Yule-Walker, ARIMA(p,d,0), AIC model selection
- [x] `nn_bridge.rs`: ForecastModel trait + Persistence/ARIMA/Ensemble/ExternalNn bridges

### 3.4 Optimal Power Flow (`src/optimize/opf/`)
- [x] `dc_opf.rs`: DC-OPF, lambda-iteration economic dispatch, merit-order
- [x] Validate against MATPOWER DC-OPF results (< 0.1% error) — `tests/dc_opf_validation_test.rs`

### 3.5 Economic Dispatch (`src/optimize/dispatch/`)
- [x] `economic.rs`: Multi-period economic dispatch
- [x] `unit_commit.rs`: Priority-list unit commitment with min on/off time

### 3.6 Microgrid (`src/optimize/microgrid/`)
- [x] `ems.rs`: Greedy rule-based EMS (renewables → battery → diesel → load shed)
- [x] `islanding.rs`: ROCOF, vector surge, U/O frequency/voltage detection
- [x] `peer_energy.rs`: Double-auction P2P energy market clearing

### 3.7 Storage Optimization (`src/optimize/storage/`)
- [x] `arbitrage.rs`: Price-based greedy battery arbitrage
- [x] `sizing.rs`: Peak shaving, solar shifting, backup, self-consumption sizing

### 3.8 Phase 3 Tests & Benchmarks
- [x] DC-OPF IEEE 14-bus validation test (power balance, gen limits, positive cost/lambda)
- [x] Microgrid EMS 24-hour plan test (target: < 1s)
- [x] `benches/opf_bench.rs`: DC-OPF IEEE 14/30/118-bus benchmark

---

## Phase 4: Advanced

### 4.1 Stability Analysis (`src/stability/`)
- [x] `transient.rs`: Transient stability — swing equation, RK4, SMIB eigenvalues
- [x] `small_signal.rs`: Small-signal stability — A-matrix, nalgebra Schur eigenvalues, oscillation modes
- [x] `voltage.rs`: Voltage stability — PV/QV curves, voltage stability index
- [x] `generator/classical.rs`: Classical generator model (constant E' behind X'd, RK4, SMIB fault sim)
- [x] `generator/detailed.rs`: Detailed generator model (d-q axis, subtransient)
- [x] `generator/governor.rs`: TGOV1 steam governor, droop speed governor

### 4.2 Electrochemical Battery Model (`src/battery/p2d/`)
- [x] `electrode.rs`: Electrode model (cathode/anode), solid-phase diffusion (Fick's law)
- [x] `electrolyte.rs`: Electrolyte transport (concentration, potential)
- [x] `separator.rs`: Separator model
- [x] `solver.rs`: Single Particle Model (SPM) coupled solver

### 4.3 Battery Aging (`src/battery/aging.rs`)
- [x] SEI growth model (calendar + cycling)
- [x] Lithium plating model
- [x] Capacity fade and resistance growth

### 4.4 Harmonics (`src/harmonics/`)
- [x] `analysis.rs`: THD, Goertzel, IEEE 519 voltage compliance
- [x] `filter.rs`: Single-tuned and high-pass passive filter design
- [x] `standards.rs`: IEC 61000-3-2 / IEEE 519-2022 compliance limits

### 4.5 Protection (`src/protection/`)
- [x] `fault.rs`: Z-bus fault current, 3-phase fault, DC offset factor
- [x] `relay.rs`: IEC 60255 / IEEE C37.112 overcurrent, Mho distance relay
- [x] `coordination.rs`: Protection coordination, TCC curve, CTI checking

### 4.6 AC-OPF (`src/optimize/opf/ac_opf.rs`)
- [x] AC Optimal Power Flow via SQP/penalty method (basic gradient descent + NR inner loop)
- [x] Security-Constrained OPF (SCOPF) — `security.rs`

### 4.7 I/O Module (`src/io/`)
- [x] `serialize.rs`: serde-based serialization for all core types
- [x] `csv.rs`: CSV import/export for time-series data
- [x] `oxirs_bridge.rs`: oxirs knowledge graph integration (digital twin, JSON-LD export)

### 4.8 ML Integration
- [x] `renewable/forecast/nn_bridge.rs`: ForecastModel trait + Persistence/ARIMA/Ensemble bridges + ExternalNnBridge placeholder

---

## Cross-Cutting Concerns

### Quality & Testing
- [ ] All `pub fn` have at least one unit test (blueprint section 7)
- [x] proptest property-based tests for numerical invariants (`tests/powerflow_proptest.rs`: 8 proptest props + 2 regular tests)
- [ ] `cargo tarpaulin` coverage target: 80%+

### Performance
- [x] Sparse Jacobian: Y-bus non-zero iteration (avoids O(n²) ybus_to_dense), O(1) index maps
- [x] rayon parallelization for Jacobian construction behind `parallel` feature flag
- [ ] Sparse LU solver (replace nalgebra dense LU with sparse factorization)
- [ ] SIMD optimizations behind `simd` feature flag

### Architecture
- [ ] Trait abstraction layer for linear algebra backend (swap nalgebra/sprs for oxiblas/numrs when ready)
- [ ] `no_std` support for `units/` and `battery/ecm/` modules
- [ ] Feature gates actually controlling module compilation
- [ ] petgraph-based network topology (blueprint specifies petgraph::Graph wrapping)

### Documentation & Examples
- [ ] `examples/ieee14_powerflow.rs`
- [ ] `examples/battery_cycling.rs`
- [ ] `examples/microgrid_optimization.rs`
- [ ] `examples/renewable_forecast.rs`
- [ ] Module-level rustdoc with mathematical background

---

## Current Stats

| Metric | Value |
|--------|-------|
| Rust source files | ~93 |
| Total tests passing | 587/587 |
| Clippy warnings | 0 |
| IEEE 14-bus NR bench | ~29 us |
| IEEE 30-bus NR bench | ~160 us |
| IEEE 14-bus DC bench | ~1.6 us |
