//! Power system harmonics: THD analysis, IEEE 519 compliance, passive filter design,
//! harmonic source identification, and harmonic mitigation strategies.
//!
//! # Modules
//! - [`analysis`]              — THD, THVD, Goertzel DFT, IEEE 519-2022 voltage compliance
//! - [`filter`]                — single-tuned and high-pass passive filter design (RLC)
//! - [`standards`]             — IEC 61000-3-2 current limits, IEEE 519-2022 voltage limits
//! - [`mitigation`]            — passive/active/hybrid filter design, APF control, cost analysis
//! - [`source_identification`] — current injection, power direction, pattern matching, hybrid
//!
//! ## Mathematical background
//!
//! **Total Harmonic Distortion (THD)**:
//!
//! ```text
//! THD = sqrt(Σ_{h=2}^{H_max} V_h²) / V_1
//! ```
//!
//! where V_1 is the fundamental RMS voltage and V_h is the h-th harmonic RMS.
//!
//! **K-factor** (transformer derating under non-sinusoidal load):
//!
//! ```text
//! K = Σ_{h=1}^{H_max} I_h² · h²  /  Σ_{h=1}^{H_max} I_h²
//! ```
//!
//! **IEEE 519-2022** voltage distortion limits (at PCC):
//! - Buses below 1 kV: THD_V ≤ 8 %
//! - 1–69 kV: THD_V ≤ 5 %
//! - 69–161 kV: THD_V ≤ 2.5 %
//! - Above 161 kV: THD_V ≤ 1.5 %
pub mod active_filter;
pub mod analysis;
pub mod filter;
pub mod flicker;
pub mod harmonic_pf;
pub mod mitigation;
pub mod source_identification;
pub mod standards;
pub use mitigation::*;
pub use source_identification::{
    HarmonicMeasurement, HarmonicSource, HarmonicSourceIdentifier, HarmonicSourceType,
    IdentificationMethod, IdentificationResult, SourceConfidence, SourceFingerprint,
};
