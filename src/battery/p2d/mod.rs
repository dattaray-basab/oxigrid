//! Pseudo-2D (P2D) / Doyle-Fuller-Newman (DFN) electrochemical battery model.
//!
//! Implements the Single Particle Model (SPM) as a simplified P2D model.
//! The SPM couples solid-phase diffusion in cathode and anode with
//! electrolyte transport and Butler-Volmer kinetics.
//!
//! # Modules
//! - [`electrode`]   — solid-phase diffusion via Fick's law (finite difference)
//! - [`electrolyte`] — electrolyte concentration and potential transport
//! - [`separator`]   — separator physical properties and ionic resistance
//! - [`solver`]      — coupled SPM solver integrating all components
pub mod dfn;
pub mod electrode;
pub mod electrolyte;
pub mod separator;
pub mod solver;
