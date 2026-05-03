//! I/O utilities for OxiGrid data: CSV, JSON serialization, and digital twin export.
//!
//! - [`csv`]          — CSV time-series import/export for measurement and forecast data
//! - [`serialize`]    — serde helpers and custom serializers for all core types
//! - [`oxirs_bridge`] — JSON-LD digital twin export (`ToDigitalTwin` trait,
//!   `DigitalTwinModel`, `DtAsset`); compatible with oxirs / RDF / JSON-LD consumers
pub mod csv;
pub mod matpower_export;
pub mod oxirs_bridge;
pub mod pmu;
pub mod serialize;
pub mod timeseries;
pub mod visualization;
