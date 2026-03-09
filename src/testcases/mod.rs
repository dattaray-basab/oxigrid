//! Synthetic grid test case generator for OxiGrid.
//!
//! Provides:
//!
//! - **`ieee`** — Classic IEEE standard test cases (14, 30, 57, 118, 300-bus, RTS-96, PEGASE 89)
//! - **`synthetic`** — Procedural synthetic network generation with configurable topology
//!   (Ring, Radial, Meshed, Geographic, SmallWorld, ScaleFree)
//! - **`distribution`** — Distribution network test cases (IEEE 33, IEEE 69, LV/MV feeders)
//! - **`benchmark`** — Benchmark scenarios with reference solutions and validation utilities
//!
//! # Quick Start
//!
//! ```rust,ignore
//! use oxigrid::testcases::ieee::ieee14;
//! use oxigrid::testcases::synthetic::{SyntheticNetworkConfig, NetworkTopology, generate_synthetic_network};
//!
//! // Load the canonical IEEE 14-bus system
//! let net14 = ieee14().unwrap();
//! assert_eq!(net14.buses.len(), 14);
//!
//! // Generate a 50-bus small-world transmission network
//! let config = SyntheticNetworkConfig {
//!     n_buses: 50,
//!     topology: NetworkTopology::SmallWorld,
//!     ..Default::default()
//! };
//! let net = generate_synthetic_network(&config).unwrap();
//! assert_eq!(net.buses.len(), 50);
//! ```

pub mod benchmark;
pub mod distribution;
pub mod ieee;
pub mod synthetic;
pub mod synthetic_grid;
pub mod validation;

pub use synthetic_grid::{
    GridGenError, SyntheticBranch, SyntheticBus, SyntheticGenerator, SyntheticGrid,
    SyntheticGridConfig, SyntheticGridGenerator, SyntheticLoad, SyntheticRenewable,
    SyntheticTopology,
};

pub use benchmark::{
    power_flow_benchmarks, BenchmarkReport, BenchmarkScenario, ExpectedPowerFlowResult,
};
pub use distribution::{ieee33, ieee69, lv_european_residential, mv_urban_feeder};
pub use ieee::{ieee118, ieee14, ieee30, ieee300, ieee57, pegase89, rts96};
pub use synthetic::{generate_synthetic_network, Lcg64, NetworkTopology, SyntheticNetworkConfig};

pub use benchmark::{run_benchmark, validate_all_benchmarks};
pub use validation::{
    ModelValidator, ValidationCase, ValidationConfig, ValidationResult, ValidationSummary,
};

#[cfg(feature = "stability")]
pub use benchmark::ieee9_stability;
