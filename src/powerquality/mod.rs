//! Comprehensive power quality analysis.
//!
//! This module extends the harmonic analysis in [`crate::harmonics`] with a
//! broader set of power quality (PQ) functions covering the complete scope of
//! IEEE 1159-2019, EN 50160, and IEEE 519-2022.
//!
//! # Modules
//!
//! | Module | Contents |
//! |--------|----------|
//! | [`sag_swell`]  | IEEE 1159 sag/swell/interruption detection, ITIC & SEMI F47 |
//! | [`waveform`]   | Waveform distortion: THD, K-factor, crest factor, interharmonics |
//! | [`indices`]    | EN 50160 compliance, IEEE 519 limits, PQ distribution indices |
//! | [`events`]     | PQ event classifier, severity rating, event summary statistics |
//!
//! # Quick-start
//!
//! ```rust,ignore
//! use oxigrid::powerquality::{
//!     sag_swell::{half_cycle_rms, detect_voltage_events},
//!     waveform::analyze_waveform,
//!     indices::{En50160Limits, check_en50160_compliance},
//!     events::PqEventClassifier,
//! };
//! ```
pub mod events;
pub mod indices;
pub mod ml_classifier;
pub mod sag_swell;
pub mod standards_compliance;
pub mod waveform;
pub use ml_classifier::{
    ClassificationResult, ClassifierMethod, ConfusionMatrix, DecisionTreeConfig, DistanceMetric,
    KnnConfig, PqClassifier, PqEventClass, PqFeatures, TrainingSample, TreeNode,
};
pub use standards_compliance::*;
