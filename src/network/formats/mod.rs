//! Network data format parsers.
//!
//! | Parser | Module | Format |
//! |--------|--------|--------|
//! | MATPOWER `.m` | [`matpower`]    | MATPOWER Case Format v2 (bus/branch/gen sections) |
//! | IEEE CDF       | [`ieee_cdf`]    | IEEE Common Data Format (bus/branch cards) |
//! | pandapower     | [`pandapower`]  | pandapower JSON (bus/line/trafo/gen tables) |
pub mod ieee_cdf;
pub mod matpower;
pub mod pandapower;
