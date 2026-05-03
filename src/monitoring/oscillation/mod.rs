//! Real-time power system oscillation monitoring using Prony analysis.
//!
//! Split from the original `oscillation.rs` to comply with the 2000-line policy.
//! All public types are re-exported here to preserve the original module API.

mod functions;
mod trait_impls;
pub mod types;

// Re-export all public types to maintain original API
pub use types::*;
