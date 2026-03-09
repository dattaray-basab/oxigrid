//! Multi-Energy System (MES) optimisation module.
//!
//! Optimises the combined operation of electricity, natural gas, heat, cooling,
//! and hydrogen energy carriers within one or more energy hubs.
//!
//! # Sub-modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | \[`hub`\] | Energy hub model: converters, storages, demands, and optimiser |
//!
//! # References
//! - Geidl, M. & Andersson, G., "Optimal Power Flow of Multiple Energy Carriers",
//!   IEEE Trans. Power Syst. 22(1), 2007.
//! - Mancarella, P., "MES (multi-energy systems): an overview of concepts and
//!   evaluation models", Energy 65, 2014.

pub mod hub;

pub use hub::{
    ConverterType, EnergyCarrier, EnergyConverter, EnergyDemand, EnergyHub, EnergyStorage,
    HubHourlyDispatch, MesError, MesOptConfig, MesOptResult, MesOptimizer,
};
