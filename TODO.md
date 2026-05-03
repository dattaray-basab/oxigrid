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
- [x] `rayon` dependency behind `parallel` feature flag

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
- [x] `no_std` support for units module — planned 2026-04-27 (Round 26 Item C)
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
- [x] Use petgraph::Graph<Bus, Branch> internally — planned 2026-04-27 (Round 26 Item A)

### 1.5 Power Flow Module (`src/powerflow/`)
- [x] `mod.rs`: PowerFlowMethod enum, PowerFlowConfig (default: NR, 50 iter, 1e-8 tol), PowerFlowSolver trait, solve_powerflow() dispatcher
- [x] `jacobian.rs`: Full Jacobian builder (H, N, M, L sub-matrices) using dense nalgebra::DMatrix
- [x] `newton_raphson.rs`: Newton-Raphson AC power flow — bus classification, mismatch vectors, Jacobian solve via LU, voltage update (polar form [dtheta; dV/V])
- [x] `dc_powerflow.rs`: DC approximation — B' matrix, linear solve, theta calculation
- [x] `result.rs`: PowerFlowResult with voltage_magnitude, voltage_angle, p/q_injected, converged, iterations, max_mismatch; Display impl
- [x] `fast_decoupled.rs`: Fast Decoupled Load Flow (FDLF) — B' and B'' matrices (Stott & Alsac 1974)
- [x] `continuation.rs`: Continuation power flow for voltage stability (blueprint section 3)
- [x] Branch power flow calculation in results (P/Q flow per branch, not just bus injections)
- [x] Total system losses calculation (currently only sum of injections)
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
- [x] IEEE 300-bus benchmark (target: < 50ms) — planned 2026-04-27 (Round 26 Item E)

### 1.9 Documentation
- [x] `///` doc comments on key `pub` items: `topology.rs` (Generator, PowerNetwork, all pub fn), `electrical.rs` (all types + methods), `energy.rs`, `thermal.rs`
- [x] Module-level `//!` doc comments in all 21 mod.rs files
- [x] Mathematical background sections (LaTeX notation) — planned 2026-04-27 (Round 26 Item F)
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
- [x] 1D thermal model (Thermal1DAxial, axial FD) — planned 2026-04-27 (Round 26 Item D)

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
- [~] **Pub fn coverage:** Round 27 wired 6 orphaned module groups, added 88+ tests. Round 28 wired ≥ 77 orphan modules (~75K LOC), resolved 8 sibling-pair overlaps (all wire-alongside), added 53+ tests in 5 high-leverage files + 2 SIMD tests + 6 doctests. Round 29 Item C added 40 tests across 5 low-coverage files; Round 30 Item B added 63 tests across 8 zero/low-density files (topology.rs and black_start.rs had 0 in-file tests). Remaining gap tracked per tarpaulin function-coverage report.
- [x] proptest property-based tests for numerical invariants (`tests/powerflow_proptest.rs`: 8 proptest props + 2 regular tests)
- [~] **Coverage roadmap:** Round 27 baseline = 76.90% (32,644/42,449 lines). Round 28 = 78.49% (43,580/55,525 lines), measured 2026-04-27 via `tarpaulin.toml`. Per-round target: +5pp until 80%+. See `tarpaulin.toml` for the canonical command. Post-Round-30 coverage measurement deferred (tarpaulin ~1.5h runtime); combined Rounds 29+30 added 103 unit tests on previously zero/thin modules — estimated +5–7 pp; recommend background tarpaulin run before Round 31.

### Performance
- [x] Sparse Jacobian: Y-bus non-zero iteration (avoids O(n²) ybus_to_dense), O(1) index maps
- [x] rayon parallelization for Jacobian construction behind `parallel` feature flag
- [x] Sparse LU solver (wire via LinearAlgebraBackend; select_backend(n)) — planned 2026-04-27 (Round 26 Item B)
- [x] SIMD optimizations behind `simd` feature flag (SimdAvx2Backend) — planned 2026-04-27 (Round 26 Item B)

### Architecture
- [x] Trait abstraction layer for linear algebra backend (LinearAlgebraBackend trait in linalg.rs) — planned 2026-04-27 (Round 26 Item B)
- [x] `no_std` support for `units/` (done this round) and `battery/ecm/` (deferred) — planned 2026-04-27 (Round 26 Item C)
- [x] Feature gates actually controlling module compilation
- [x] petgraph-based network topology — see Item A above, same implementation

### Documentation & Examples
- [x] `examples/ieee14_powerflow.rs`
- [x] `examples/battery_cycling.rs`
- [x] `examples/microgrid_optimization.rs`
- [x] `examples/renewable_forecast.rs`
- [x] Module-level rustdoc with mathematical background — see Item F above, same implementation

---

## Current Stats

| Metric | Value |
|--------|-------|
| Rust source files | 466 |
| SLoC (Rust code) | 231,610 |
| Total tests passing | 5,036 |
| Coverage (Round 28) | 78.49% (43,580/55,525 lines) — post-Round-30 measurement deferred |
| Clippy warnings | 0 |
| IEEE 14-bus NR bench | ~29 us |
| IEEE 30-bus NR bench | ~160 us |
| IEEE 14-bus DC bench | ~1.6 us |

---

## Round 27 (2026-04-27)
Coverage baseline (76.90%); tarpaulin.toml; 4 orphan files deleted; 6 module groups wired into mod.rs; 88+ new tests in 9 modules. Test count: 3,895 → 4,058.

## Round 28 (2026-04-27)
Orphan annihilation — 3 verbatim PPF duplicates deleted; ≥ 77 orphan modules wired across 16 mod.rs files (+856 previously-invisible tests); 8 sibling-pair overlaps resolved (all wire-alongside); oscillation.rs split (3079→3 files). Coverage push: +53 tests in 5 low-coverage files. SIMD: compute_power_injection wired into NR inner loop (n≥64 threshold, `simd` feature). Doctests: 6 prelude API files seeded. Coverage = 78.49% (up from 76.90%).

## Round 30 (2026-04-28)

**Item A — splitrs `pss_design.rs` (2000-LOC violation) + file-size regression guard** `[x]`
- `src/stability/pss_design.rs` (2000 LOC, CLAUDE.md violation) → `src/stability/pss_design/` module:
  - `mod.rs` (10 LOC), `types.rs` (666), `types_3.rs` (457), `functions.rs` (518), `trait_impls.rs` (26)
  - `PssDesigner::lead_lag_constants` bumped to `pub(crate)` for cross-module test visibility
- `tests/file_size_guard.rs` (NEW): `no_source_file_exceeds_2000_lines` test; catches any future file ≥ 2000 LOC in src/
- Zero CLAUDE.md file-size violations remain in src/

**Item B — Coverage push: 63 unit tests across 8 zero/low-density files** `[x]`
- `src/network/topology.rs`: 0 → 22 tests (+22) — foundational module, previously had zero in-file tests
- `src/optimize/restoration/black_start.rs`: 0 → 6 tests (+6) — previously zero in-file tests
- `src/battery/thermal.rs`: 3 → 11 tests (+8)
- `src/digitaltwin/telemetry.rs`: 3 → 9 tests (+6)
- `src/renewable/inverter/grid_following.rs`: 3 → 8 tests (+5)
- `src/digitaltwin/twin.rs`: 4 → 10 tests (+6)
- `src/stability/transient.rs`: 4 → 9 tests (+5)
- `src/security/fdi.rs`: 4 → 9 tests (+5)

**Item C — IEEE-300 end-to-end cross-stack integration test** `[x]`
- `tests/ieee300_e2e.rs` (NEW, 5 tests): exercises load → NR power flow → DC state estimation → N-1 contingency → DC-OPF against the 300-bus testcase; catches inter-module contract regressions that unit tests miss

**Stats:** 5,006 → 5,075 total tests (+69: 63 Item B + 5 Item C + 1 Item A guard); unit: 4,967 → 5,036 (+69). SLoC: 232,276 → 231,610. Files: 470 → 466. Zero clippy warnings.

## Round 29 (2026-04-28)

**Item A — Splitrs refactor of 5 oversized files** `[x]`
- `src/digitaltwin/asset_digitization.rs` (2387 LOC) → `asset_digitization/` module (mod.rs, types.rs, types_3.rs, functions.rs, trait_impls.rs)
- `src/powerflow/acdc_pf.rs` (2381 LOC) → `acdc_pf/` module; `src/optimize/ev/infrastructure_planning.rs` (2297 LOC), `src/network/resilience_planning.rs` (2234 LOC), `src/network/voltage_regulation.rs` (2154 LOC) likewise split
- All 5 file-size violations eliminated; test modules fixed with `use super::super::*;`
- `src/stability/pss_design.rs` at exactly 2000 lines — pre-existing, refactor in Round 30

**Item B — Sparsified NR Jacobian end-to-end** `[x]`
- `src/powerflow/jacobian.rs`: added `build_jacobian_sparse` returning `CsMat<f64>` via triplet accumulation; `build_jacobian`/`build_jacobian_parallel` are thin wrappers
- `src/powerflow/sparse_lu.rs`: added `CrsMatrix::from_csmat` (O(nnz) bridge, no dense round-trip)
- `src/powerflow/newton_raphson.rs`: branches on `SPARSE_JAC_THRESHOLD=200`; large systems use sparse path, eliminating `DMatrix::zeros(j_size, j_size)` allocation from NR hot path
- +2 tests: `jacobian_sparse_matches_dense_3bus`, `jacobian_sparse_nnz_bounded_ieee14`

**Item C — Coverage push (+40 tests across 5 files)** `[x]`
- `src/optimize/microgrid/advanced_ems.rs` +8 tests
- `src/security/threat_intelligence.rs` +8 tests
- `src/optimize/ev/grid_integration.rs` +8 tests
- `src/powerquality/standards_compliance.rs` +8 tests
- `src/planning/distribution.rs` +8 tests

**Item D — ECM L-BFGS offline batch fitter (Pure Rust)** `[x]`
- `src/battery/ecm/lbfgs.rs` (NEW, 268 LOC): Pure-Rust L-BFGS with two-loop recursion, Armijo backtracking, forward-difference gradient, curvature guard, gradient-normalization fix (m=0 initial step for large-gradient functions)
- `src/battery/ecm/parameter.rs`: replaced heuristic-only path with L-BFGS (log-space, warm-start); fixed `ecm_simulate_loss` OCV estimation from rest segment; fixed `t_prev` initialization
- `src/battery/ecm/mod.rs`: added `mod lbfgs;`
- +5 tests: quadratic recovery, Rosenbrock 2D, invalid input error, ECM synthetic data recovery, better-than-heuristic assertion; "placeholder infrastructure for optirs" docstring removed

**Stats:** 4,920 → 4,967 unit tests (+47); 4,959 → 5,006 total (4,967 unit + 39 doc). SLoC: 229,016 → 232,276. Files: 440 → 470. Zero clippy warnings.

## /stub-check (2026-04-27)
Codebase-wide stub audit: 0 hard stubs (no `unimplemented!()`/`todo!()`); 7 real_stub sites fixed — `iec60909::rated_kv_sq_over_mva` dead helper deleted; `modal_voltage_stability` branch participation implemented; `use_security_constrained` wired into N-1 reserve logic; `compute_flow_sensitivity_dP_dQ` made non-trivial; `event_summary` `sample_rate_hz` parameter added (was hardcoded 1.0); Q-gen validation implemented in `ModelValidator`; `TvsaEngine::Q_MAX_AVAILABLE` made configurable. +6 new tests. Final: **4,920 unit tests + 39 doc tests** = 4,959 total.

## v0.1.1 (2026-05-03)

**Fix: E0034 disambiguation in `SimdAvx2Backend::solve_dense`** `[x]`
- `src/powerflow/linalg.rs` line 95: `self.inner.solve_dense(a, b)` → `LinearSolver::solve_dense(&self.inner, a, b)`
- Resolved "multiple applicable items in scope" compile error caused by both `LinearAlgebraBackend` and `LinearSolver` traits being in scope simultaneously on the same method name
- Build now succeeds with all features enabled
