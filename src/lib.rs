#![cfg_attr(not(feature = "std"), no_std)]
//! OxiGrid — Pure Rust Energy Systems Simulation & Optimization Library.
//!
//! # Feature Flags
//!
//! | Feature | Description | Enables |
//! |---------|-------------|---------|
//! | `std` (default) | Standard library support | — |
//! | `powerflow` (default) | AC/DC power flow solvers | `network`, `powerflow` modules |
//! | `stability` | Transient and small-signal stability | requires `powerflow` |
//! | `battery` (default) | Battery ECM, SoC, thermal, aging | `battery` module |
//! | `battery-p2d` | Pseudo-2D / DFN electrochemical model | requires `battery` |
//! | `renewable` | Solar PV, wind, forecasting | `renewable` module |
//! | `optimize` | OPF, economic dispatch, microgrid EMS | requires `powerflow` |
//! | `harmonics` | Harmonic analysis, IEEE 519, passive filters | `harmonics` module |
//! | `protection` | Fault analysis, relay coordination | requires `powerflow` |
//! | `io-matpower` | MATPOWER `.m` file parser | included in `powerflow` |
//! | `parallel` | rayon-based parallelisation (future) | requires `std` |

#[cfg(feature = "std")]
pub mod error;

#[cfg(feature = "std")]
pub mod io;

pub mod units;

#[cfg(feature = "powerflow")]
pub mod network;

#[cfg(feature = "powerflow")]
pub mod powerflow;

#[cfg(feature = "stability")]
pub mod stability;

#[cfg(any(feature = "battery", feature = "battery-p2d"))]
pub mod battery;

#[cfg(feature = "renewable")]
pub mod renewable;

#[cfg(feature = "optimize")]
pub mod optimize;

#[cfg(feature = "optimize")]
pub mod planning;

#[cfg(feature = "harmonics")]
pub mod harmonics;

#[cfg(feature = "protection")]
pub mod protection;

#[cfg(feature = "powerelectronics")]
pub mod powerelectronics;

#[cfg(feature = "powerflow")]
pub mod digitaltwin;

#[cfg(feature = "std")]
pub mod analytics;

#[cfg(feature = "std")]
pub mod monitoring;

#[cfg(feature = "std")]
pub mod powerquality;

#[cfg(feature = "std")]
pub mod security;

#[cfg(feature = "std")]
pub mod simulation;

#[cfg(feature = "powerflow")]
pub mod testcases;

#[cfg(feature = "powerflow")]
pub mod prelude;
