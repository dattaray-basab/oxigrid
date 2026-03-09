//! Power system harmonics: THD analysis, IEEE 519 compliance, passive filter design,
//! harmonic source identification, and harmonic mitigation strategies.
//!
//! # Modules
//! - [`analysis`]              — THD, THVD, Goertzel DFT, IEEE 519-2022 voltage compliance
//! - [`filter`]                — single-tuned and high-pass passive filter design (RLC)
//! - [`standards`]             — IEC 61000-3-2 current limits, IEEE 519-2022 voltage limits
//! - [`mitigation`]            — passive/active/hybrid filter design, APF control, cost analysis
//! - [`source_identification`] — current injection, power direction, pattern matching, hybrid
pub mod analysis;
pub mod filter;
pub mod flicker;
pub mod mitigation;
pub mod source_identification;
pub mod standards;
pub use mitigation::*;
pub use source_identification::{
    HarmonicMeasurement, HarmonicSource, HarmonicSourceIdentifier, HarmonicSourceType,
    IdentificationMethod, IdentificationResult, SourceConfidence, SourceFingerprint,
};
