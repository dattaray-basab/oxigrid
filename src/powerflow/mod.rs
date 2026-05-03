//! AC and DC power flow solvers.
//!
//! # Usage
//!
//! ```rust,ignore
//! use oxigrid::prelude::*;
//! let net = PowerNetwork::from_matpower("ieee14.m")?;
//! let cfg = PowerFlowConfig::default();          // Newton-Raphson, 50 iter, tol 1e-8
//! let res = net.solve_powerflow(&cfg)?;
//! assert!(res.converged);
//! ```
//!
//! # Algorithms
//!
//! | Method | Struct | Notes |
//! |--------|--------|-------|
//! | Newton-Raphson (AC) | `NewtonRaphsonSolver` | Sparse Jacobian, step-size limiting |
//! | Fast Decoupled (AC) | `FastDecoupledSolver` | Stott & Alsac 1974, B'/B'' matrices |
//! | DC Approximation   | `DcPowerFlowSolver`   | Linear B'·θ = P, 1 iteration |
//! | Continuation PF    | `ContinuationSolver`  | P-V curve, voltage stability |
//!
//! The `parallel` feature flag activates rayon-based parallel Jacobian construction.
//!
//! ## Mathematical background
//!
//! **Newton-Raphson AC power flow** solves the nonlinear system F(x) = 0
//! where x = [θ; V] (angles and magnitudes):
//!
//! ```text
//! J · Δx = −f(x)
//! [H  N] [Δθ    ]   [ΔP]
//! [M  L] [ΔV/V  ] = [ΔQ]
//! ```
//!
//! Sub-matrices:
//! - H = ∂P/∂θ, N = ∂P/∂V · V, M = ∂Q/∂θ, L = ∂Q/∂V · V
//!
//! **Fast Decoupled (Stott & Alsac 1974)** decouples P-θ and Q-V subsystems:
//!
//! ```text
//! B' · Δθ = ΔP / V    (P-θ decoupled system)
//! B'' · ΔV = ΔQ / V   (Q-V decoupled system)
//! ```
//!
//! **DC approximation** linearises AC power flow:
//!
//! ```text
//! P = B' · θ    (lossless, unity voltage, no shunts)
//! ```
//!
//! # Examples
//!
//! ```rust
//! use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};
//!
//! // Default config uses Newton-Raphson
//! let config = PowerFlowConfig::default();
//! assert_eq!(config.method, PowerFlowMethod::NewtonRaphson);
//! assert_eq!(config.max_iter, 50);
//! assert!((config.tolerance - 1e-8).abs() < 1e-15);
//!
//! // Override to DC approximation
//! let dc_config = PowerFlowConfig {
//!     method: PowerFlowMethod::DcApproximation,
//!     ..PowerFlowConfig::default()
//! };
//! assert_eq!(dc_config.method, PowerFlowMethod::DcApproximation);
//! ```
pub mod acdc_pf;
pub mod bad_data;
pub mod branch_flows;
pub mod contingency_analysis;
pub use contingency_analysis::*;
pub mod continuation;
pub mod dc_powerflow;
pub mod dsse;
pub mod dynamic_se;
pub mod fast_decoupled;
pub mod flow_decomposition;
pub mod harmonic_pf;
pub use harmonic_pf::*;
pub mod harmonic_pf_problem;
pub use harmonic_pf_problem::{
    solve_complex_linear, HarmonicBranchData, HarmonicBusData, HarmonicCurrentSource,
    HarmonicLoadModel, HarmonicOrderResult, HarmonicPfConfig, HarmonicPfProblem, HarmonicPfResult,
};
pub mod hem;
pub mod jacobian;
pub mod linalg;
pub use linalg::{select_backend, LinearAlgebraBackend};
pub mod newton_raphson;
pub mod probabilistic;
pub mod result;
pub mod sensitivity;
pub mod simd_kernels;
pub mod sparse_lu;
pub use dsse::*;
pub mod state_estimation;
pub mod stochastic_lf;
pub mod timeseries_sim;
pub mod unbalanced_continuation;
pub use acdc_pf::*;
pub use timeseries_sim::{
    BusTimeSeries, BusTimeSeriesType, GeneratorProfile, ScenarioAnalysis, StorageStrategy,
    TimeResolution, TimeSeriesConfig, TimeSeriesNetwork, TimeSeriesResult, TimeSeriesSimulator,
    TimeSeriesStatistics, TimeStepResult,
};
pub use unbalanced_continuation::{
    CollapsePoint, CpfBusType, LoadScalingModel, PvPoint, ThreePhaseBranch, ThreePhaseBus,
    UnbalancedCpf, UnbalancedCpfConfig, UnbalancedCpfResult,
};

use crate::error::Result;
use crate::network::PowerNetwork;
pub use result::{BranchFlow, PowerFlowResult};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PowerFlowMethod {
    NewtonRaphson,
    FastDecoupled,
    DcApproximation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerFlowConfig {
    pub method: PowerFlowMethod,
    pub max_iter: usize,
    pub tolerance: f64,
    pub enforce_q_limits: bool,
}

impl Default for PowerFlowConfig {
    fn default() -> Self {
        Self {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        }
    }
}

pub trait PowerFlowSolver {
    fn solve(&self, network: &PowerNetwork, config: &PowerFlowConfig) -> Result<PowerFlowResult>;
}

impl PowerNetwork {
    pub fn solve_powerflow(&self, config: &PowerFlowConfig) -> Result<PowerFlowResult> {
        match config.method {
            PowerFlowMethod::NewtonRaphson => {
                let solver = newton_raphson::NewtonRaphsonSolver;
                solver.solve(self, config)
            }
            PowerFlowMethod::DcApproximation => {
                let solver = dc_powerflow::DcPowerFlowSolver;
                solver.solve(self, config)
            }
            PowerFlowMethod::FastDecoupled => {
                let solver = fast_decoupled::FastDecoupledSolver;
                solver.solve(self, config)
            }
        }
    }
}
