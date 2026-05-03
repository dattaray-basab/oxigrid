//! Renewable energy source models: solar PV, wind, and forecasting.
//!
//! # Modules
//! - [`solar`]    — Solar position (Spencer 1971), Liu & Jordan POA irradiance,
//!   single-diode 5-parameter PV cell model, MPPT (P&O, InCond), CEC/Sandia inverter
//! - [`wind`]     — Wind turbine power curves, Betz limit, Weibull AEP,
//!   Jensen & Frandsen wake models, regular-grid wind farm
//! - [`forecast`] — Persistence (naive + diurnal), AR/ARIMA, neural network bridge trait
pub mod curtailment;
pub mod forecast;
pub mod hosting_capacity;
pub mod integration;
pub mod integration_analysis;
pub mod integration_study;
pub mod inverter;
pub mod solar;
pub mod wind;
