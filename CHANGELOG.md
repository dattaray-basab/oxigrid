# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-03-09

### Added

#### Power Flow
- Newton-Raphson power flow solver with warm-start support
- Fast decoupled load flow (FDLF)
- DC power flow
- Holomorphic embedding load flow method (HELM) with Padé acceleration
- Continuation power flow with nose-point detection
- Probabilistic power flow (Monte Carlo, Point Estimate Method)
- AC branch flows (π-model) and DC branch flows
- SIMD-accelerated NR kernels (AVX2, behind `simd` feature)
- Three-phase unbalanced Newton-Raphson power flow
- AC/DC Weighted Least-Squares state estimation
- EKF dynamic state estimator

#### Network
- Y-bus admittance matrix construction
- Network topology analysis (petgraph-based)
- N-1/N-k contingency analysis
- HVDC links (LCC/VSC) and multi-terminal DC grids
- FACTS models (STATCOM, SVC, TCSC, UPFC)
- Distribution network reconfiguration
- Transformer models (two-winding, three-winding, OLTC, IEC 60076-7 thermal)
- Voltage regulation (regulators, capacitor banks, SVC, coordinated VVC)
- Grid resilience planning (N-k analysis, hardening optimizer)

#### Stability
- Transient stability (RK45 adaptive, event queue, CCT finder)
- Small signal stability analysis (Householder QR + Francis double-shift)
- Voltage stability (L-index, modal analysis, FVSI, VSA)
- Multi-machine transient stability
- AGC + frequency regulation
- Grid-forming inverter dynamics (VSM swing equation)
- Load modeling (ZIP, Exponential, Motor, Composite WECC CLOD, Restorative)

#### Battery
- Equivalent Circuit Model (ECM) with 2-RC network
- P2D Doyle-Fuller-Newman (DFN) electrochemical model
- Thermal model (1D finite-difference + lumped)
- Battery aging and SoH estimation (EKF)
- Battery Management System (BMS) with fault detection
- Pack modeling (cell/module/pack with thermal derating)

#### Renewable Energy
- Solar PV (irradiance, MPPT, inverter, shading)
- Wind (turbine, wake models, offshore 15 MW, farm layout optimization)
- Renewable grid codes (LVRT/HVRT, FCR/FRR, PQ diagram, ramp limits)
- Forecasting: ARIMA/SARIMA, ensemble, weather-coupled NWP, conformal prediction intervals
- Integration analysis (hosting capacity, SCR/WSCR, inertia/ROCOF)

#### Optimization
- DC-OPF (lambda-iteration + LP via OxiZ)
- AC-OPF
- SCOPF (N-1 security-constrained)
- ORPD (optimal reactive power dispatch)
- Multi-period OPF with ramp constraints
- MILP unit commitment (Branch-and-Bound)
- MPC energy management system
- Microgrid EMS with advanced multi-objective optimizer
- Demand response (flexibility portfolio, market clearing, program management)
- Energy storage: price arbitrage, multi-market, degradation-aware scheduling, BESS fleet
- EV charging (smart charging, V2G, fleet coordination, grid integration)
- Hydrogen/P2G (electrolyzer, storage, fuel cell, seasonal storage)
- Market clearing (DAM, RTM, ancillary services, LMP, DSO flexibility, carbon budget)
- Distribution expansion planning, DER integration planning

#### Protection
- Fault analysis (3-phase, SLG, LL, DLG — IEC 60909)
- Protection relay coordination (IDMT curves, CTI grading)
- Differential protection 87T/87B
- Distance protection with zone coordination
- Motor protection (NEMA/IEC thermal overload)
- Fault current limiters (SFCL, resistive, bridge-type)
- IBR protection (distance reach correction, ROCOF, LOM)
- Relay testing (IEC IDMT, pickup, CTI acceptance reports)

#### Harmonics
- THD/spectrum analysis (OxiFFT-based)
- Harmonic standards compliance (IEEE 519, EN 50160, IEC 61000-3-2)
- Passive/active filter design
- Flicker analysis (Pst/Plt)

#### Power Quality
- Event classification (IEEE 1159) with ML classifier (k-NN + CART decision tree)
- PQ indices (THD, K-factor, crest factor)
- Sag/swell detection (ITIC/SEMI F47)
- Standards compliance checker (EN 50160, IEEE 519, NERC TPL)

#### Digital Twin
- Grid digital twin (WLS SE + NR PF, SCADA/PMU telemetry)
- Asset digitization and lifecycle management
- Alert engine and grid replay

#### Security
- Intrusion detection system (IDS)
- Vulnerability assessment
- Cyber-physical attack simulation (FDI, DoS, Monte Carlo)
- Threat intelligence (MITRE ATT&CK for ICS, anomaly detection, NERC CIP)

#### Analytics
- Carbon intensity forecast and LME
- Grid operations KPIs
- Energy equity analysis

#### Monitoring
- Frequency monitoring (ROCOF relay, UFLS, nadir estimator, inertia estimation)

#### IO
- CSV import/export
- MATPOWER format export
- PMU synchrophasor processing
- Time-series data management

#### Test Cases
- IEEE 14/30/57/118/300-bus standard test systems
- RTS-96, PEGASE-89
- Synthetic topologies (Ring/Radial/Meshed/Geographic/SmallWorld/ScaleFree)
- Benchmark suite with reference solutions

[0.1.0]: https://github.com/cool-japan/oxigrid/releases/tag/v0.1.0
