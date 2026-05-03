//! Microgrid energy management and market modules.
//!
//! - [`ems`]         — Rule-based EMS: renewables → battery → diesel → load shed
//! - [`islanding`]   — Anti-islanding protection: ROCOF, vector surge, U/O-F/V detection
//! - [`peer_energy`] — Peer-to-peer energy market double-auction clearing
pub mod advanced_ems;
pub mod control_arch;
pub mod ems;
pub mod energy_community;
pub mod island_resync;
pub mod islanding;
pub mod peer_energy;
pub mod sizing;
