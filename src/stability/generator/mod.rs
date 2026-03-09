//! Generator dynamic models for power system stability analysis.
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`classical`] | Constant E' behind X'd — simplest model, used for SMIB studies |
//! | [`detailed`]  | 4th-order two-axis (d-q) model with subtransient reactances |
//! | [`governor`]  | TGOV1 steam governor (lead-lag), droop speed governor |
//! | [`avr`]       | IEEE Type 1 / EXDC1 automatic voltage regulator (Vref → Efd) |
pub mod avr;
pub mod classical;
pub mod detailed;
pub mod governor;
pub mod pss;
