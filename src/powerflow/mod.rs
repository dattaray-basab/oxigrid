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
pub mod acdc_pf;
pub mod contingency_analysis;
pub use contingency_analysis::*;
pub mod continuation;
pub mod harmonic_pf;
pub use harmonic_pf::*;
pub mod harmonic_pf_problem;
pub use harmonic_pf_problem::{
    solve_complex_linear, HarmonicBranchData, HarmonicBusData, HarmonicCurrentSource,
    HarmonicLoadModel, HarmonicOrderResult, HarmonicPfConfig, HarmonicPfProblem, HarmonicPfResult,
};
pub mod dc_powerflow;
pub mod dsse;
pub mod fast_decoupled;
pub mod hem;
pub mod jacobian;
pub mod newton_raphson;
pub mod probabilistic;
pub mod result;
pub mod sensitivity;
pub mod sparse_lu;
pub use dsse::*;
pub mod state_estimation;
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
