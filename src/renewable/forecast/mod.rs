//! Renewable energy forecasting models.
//!
//! - [`persistence`] — Naive (last-value) and diurnal (same-hour-yesterday) persistence,
//!   skill score vs. climatology
//! - [`arima`]       — AR(p) Yule-Walker estimator, ARIMA(p,d,0), AIC-based order selection
//! - [`nn_bridge`]   — [`ForecastModel`](nn_bridge::ForecastModel) trait + adapters for
//!   persistence, ARIMA, ensemble, and external neural-network models
pub mod arima;
pub mod ensemble;
pub mod nn_bridge;
pub mod persistence;
pub mod probabilistic;
