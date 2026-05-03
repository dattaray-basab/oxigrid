//! Renewable energy forecasting models.
//!
//! - [`persistence`] — Naive (last-value) and diurnal (same-hour-yesterday) persistence,
//!   skill score vs. climatology
//! - [`arima`]       — AR(p) Yule-Walker estimator, ARIMA(p,d,0), AIC-based order selection
//! - [`nn_bridge`]   — [`ForecastModel`](nn_bridge::ForecastModel) trait + adapters for
//!   persistence, ARIMA, ensemble, and external neural-network models
pub mod advanced_ensemble;
pub mod arima;
pub mod ensemble;
pub mod ensemble_v2;
pub mod load_forecast;
pub mod nn_bridge;
pub mod nn_forecast;
pub mod persistence;
pub mod probabilistic;
pub mod weather_coupling;
