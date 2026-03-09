//! Solar PV modelling: irradiance, cell model, MPPT, and inverter.
//!
//! - [`irradiance`] — Solar position (Spencer 1971), extraterrestrial irradiance,
//!   Erbs beam/diffuse decomposition, Liu & Jordan plane-of-array (POA) model
//! - [`pv_cell`]    — Single-diode 5-parameter model, Newton-Raphson I-V solve,
//!   golden-section MPP tracking
//! - [`mppt`]       — Perturb & Observe (P&O), Incremental Conductance (InCond) MPPT
//! - [`inverter`]   — CEC/Sandia inverter efficiency model, European/CEC efficiency ratings
pub mod inverter;
pub mod irradiance;
pub mod mppt;
pub mod pv_cell;
pub mod pv_system;
pub mod shading;
pub use pv_system::*;

pub mod bifacial;
pub use bifacial::{
    AlbedoConfig,
    // Extended BifacialPvModel API
    AlbedoType,
    AnnualYieldInput,
    BifacialAnnualResult,
    BifacialEnergyResult,
    BifacialGeometry,
    BifacialIrradiance,
    BifacialIrradianceExt,
    BifacialModuleConfig,
    BifacialModuleParams,
    BifacialPvModel,
    BifacialPvSystem,
    BifacialYieldResult,
    GroundAlbedo,
    IrradianceComponents,
    IrradianceInputs,
    RackConfig,
    ShadingAnalysis,
    SurfaceType,
    ViewFactors,
};
