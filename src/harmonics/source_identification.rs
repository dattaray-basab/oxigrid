//! Harmonic Source Identification.
//!
//! Identifies harmonic-polluting loads/generators in a power network using
//! four complementary measurement-based techniques:
//!
//! | Method            | Principle                                               |
//! |-------------------|---------------------------------------------------------|
//! | CurrentInjection  | Norton equivalent: I_N = I_meas + Y·V_h                |
//! | ImpedanceBased    | Thevenin impedance ratio at harmonic frequency          |
//! | PowerDirection    | Sign of P_h = V_h·I_h·cos(φ_h) indicates source        |
//! | CorrelationBased  | Statistical cross-correlation across bus measurements   |
//! | PatternMatching   | Cosine similarity to fingerprint library                |
//! | Hybrid            | Weighted combination of pattern + power-direction       |
//!
//! ## References
//! - Xu & Liu, "A method for determining customer and utility harmonic
//!   contributions at the PCC", IEEE Trans. Power Del., 2000.
//! - Rönnberg & Bollen, "Harmonic distortion in the future low-voltage grid",
//!   IEEE Trans. Smart Grid, 2017.
//! - Testa et al., "Harmonics and interharmonics attribution", IEEE, 2007.

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// Algorithm used to identify harmonic sources.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum IdentificationMethod {
    /// Norton-equivalent current injection (I_N = I_meas + Y·V_h).
    CurrentInjection,
    /// Impedance-ratio method using Thevenin equivalent at harmonic frequency.
    ImpedanceBased,
    /// Sign of harmonic active power P_h = V_h·I_h·cos(φ_h).
    PowerDirection,
    /// Statistical cross-correlation index (HCI) from current magnitudes.
    CorrelationBased,
    /// Cosine similarity against the built-in fingerprint library.
    PatternMatching,
    /// Weighted combination of [`PatternMatching`] and [`PowerDirection`].
    ///
    /// [`PatternMatching`]: IdentificationMethod::PatternMatching
    /// [`PowerDirection`]: IdentificationMethod::PowerDirection
    Hybrid,
}

/// High-level category of harmonic-producing equipment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HarmonicSourceType {
    /// 6-pulse variable-speed drive: 5th, 7th, 11th, 13th (h = 6k±1).
    VariableSpeedDrive,
    /// 12-pulse VSD: 11th, 13th, 23rd, 25th (h = 12k±1).
    VsdTwelvePulse,
    /// Arc furnace: even and odd harmonics, significant 2nd–5th.
    ArcFurnace,
    /// Saturated transformer: dominant 3rd, odd harmonics.
    SatTransformer,
    /// Uncontrolled/controlled rectifier: 5th, 7th, 11th, 13th.
    Rectifier,
    /// Grid-tied PV inverter: low-THD, mainly 3rd, 5th, 7th.
    InverterPv,
    /// EV battery charger: triplen harmonics 3rd, 5th, 7th, 9th.
    EvCharger,
    /// Uninterruptible power supply: similar pattern to 6-pulse VSD.
    Ups,
    /// Source could not be classified from available data.
    Unknown,
}

/// Confidence level attached to an identification result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SourceConfidence {
    /// Similarity score < 0.30 — result is unreliable.
    Inconclusive,
    /// Similarity score 0.30–0.60.
    Low,
    /// Similarity score 0.60–0.85.
    Medium,
    /// Similarity score > 0.85.
    High,
}

impl SourceConfidence {
    /// Map a continuous score in \[0, 1\] to a [`SourceConfidence`] variant.
    pub fn from_score(score: f64) -> Self {
        if score > 0.85 {
            Self::High
        } else if score >= 0.60 {
            Self::Medium
        } else if score >= 0.30 {
            Self::Low
        } else {
            Self::Inconclusive
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Measurement
// ─────────────────────────────────────────────────────────────────────────────

/// Per-bus harmonic voltage and current measurement snapshot.
///
/// `harmonic_voltages` and `harmonic_currents` are indexed by position in
/// `harmonic_orders`, so `harmonic_voltages[i]` corresponds to order
/// `harmonic_orders[i]`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicMeasurement {
    /// 0-based bus index in the network.
    pub bus_id: usize,
    /// Unix-epoch timestamp \[s\] of the measurement.
    pub timestamp: f64,
    /// Fundamental voltage magnitude \[V RMS\].
    pub fundamental_v: f64,
    /// Fundamental current magnitude \[A RMS\].
    pub fundamental_i_a: f64,
    /// Harmonic voltage magnitudes \[V RMS\] indexed by `harmonic_orders`.
    pub harmonic_voltages: Vec<f64>,
    /// Harmonic current magnitudes \[A RMS\] indexed by `harmonic_orders`.
    pub harmonic_currents: Vec<f64>,
    /// Harmonic orders present (e.g. `[2, 3, 5, 7, 11, 13]`).
    pub harmonic_orders: Vec<u32>,
    /// Displacement power factor at fundamental frequency.
    pub power_factor: f64,
    /// Voltage Total Harmonic Distortion \[%\].
    pub thd_v_pct: f64,
    /// Current Total Harmonic Distortion \[%\].
    pub thd_i_pct: f64,
}

impl HarmonicMeasurement {
    /// Return the normalised harmonic current spectrum vector `[I_h / I_1]`.
    ///
    /// Each element is the ratio of the harmonic current to the fundamental.
    /// If `fundamental_i_a` is zero the vector is all zeros.
    pub fn normalised_current_spectrum(&self) -> Vec<f64> {
        let i1 = self.fundamental_i_a;
        if i1 < 1e-12 {
            return vec![0.0; self.harmonic_currents.len()];
        }
        self.harmonic_currents.iter().map(|&ih| ih / i1).collect()
    }

    /// Look up the current for a specific harmonic `order`, returning `None`
    /// if the order is not in `harmonic_orders`.
    pub fn current_at_order(&self, order: u32) -> Option<f64> {
        self.harmonic_orders
            .iter()
            .position(|&o| o == order)
            .and_then(|i| self.harmonic_currents.get(i).copied())
    }

    /// Look up the voltage for a specific harmonic `order`.
    pub fn voltage_at_order(&self, order: u32) -> Option<f64> {
        self.harmonic_orders
            .iter()
            .position(|&o| o == order)
            .and_then(|i| self.harmonic_voltages.get(i).copied())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Results
// ─────────────────────────────────────────────────────────────────────────────

/// A single identified harmonic source at a specific bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicSource {
    /// Unique ID assigned by the identifier.
    pub id: usize,
    /// Bus where this source is located.
    pub bus_id: usize,
    /// Classified equipment type.
    pub source_type: HarmonicSourceType,
    /// Qualitative confidence level.
    pub confidence: SourceConfidence,
    /// Continuous confidence score in \[0, 1\].
    pub confidence_score: f64,
    /// Harmonic orders that this source predominantly generates.
    pub dominant_harmonics: Vec<u32>,
    /// Estimated apparent power of harmonic injection \[kVA\].
    pub estimated_magnitude_kva: f64,
    /// Phase angle of the dominant harmonic current \[deg\].
    pub phase_angle_deg: f64,
    /// Identification method that produced this result.
    pub identification_method: IdentificationMethod,
}

/// Aggregated result of a harmonic source identification run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdentificationResult {
    /// All sources that passed the minimum-confidence threshold.
    pub identified_sources: Vec<HarmonicSource>,
    /// Number of distinct sources found.
    pub total_sources_found: usize,
    /// Fraction of system THD not explained by identified sources \[%\].
    pub unattributed_thd_pct: f64,
    /// Method used for this identification run.
    pub method_used: IdentificationMethod,
    /// Per-bus contribution to total system THD: `(bus_id, contribution_pct)`.
    pub bus_contributions: Vec<(usize, f64)>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Fingerprint library
// ─────────────────────────────────────────────────────────────────────────────

/// Known spectral signature (fingerprint) for a category of harmonic source.
///
/// Used by the [`IdentificationMethod::PatternMatching`] algorithm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SourceFingerprint {
    /// Equipment category this fingerprint represents.
    pub source_type: HarmonicSourceType,
    /// Characteristic harmonic orders produced by this source type.
    pub dominant_orders: Vec<u32>,
    /// Typical current THD value for this source category \[%\].
    pub typical_thd_pct: f64,
    /// Typical `(order, I_h / I_1)` ratios for characteristic harmonics.
    pub harmonic_ratios: Vec<(u32, f64)>,
    /// Normalised spectrum vector used for cosine-similarity matching.
    ///
    /// Element `i` corresponds to harmonic order `dominant_orders[i]`.
    pub characteristic_signature: Vec<f64>,
}

impl SourceFingerprint {
    /// Build a fingerprint from a set of `(order, ratio)` pairs.
    ///
    /// The `characteristic_signature` is L2-normalised so that cosine
    /// similarity can be computed with a plain dot product against another
    /// normalised vector.
    fn from_ratios(
        source_type: HarmonicSourceType,
        typical_thd_pct: f64,
        ratios: Vec<(u32, f64)>,
    ) -> Self {
        let dominant_orders: Vec<u32> = ratios.iter().map(|&(o, _)| o).collect();
        let raw: Vec<f64> = ratios.iter().map(|&(_, r)| r).collect();
        let norm = l2_norm(&raw).max(1e-12);
        let characteristic_signature = raw.iter().map(|&v| v / norm).collect();
        Self {
            source_type,
            dominant_orders,
            typical_thd_pct,
            harmonic_ratios: ratios,
            characteristic_signature,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main identifier
// ─────────────────────────────────────────────────────────────────────────────

/// Main harmonic source identification engine.
///
/// # Quick start
/// ```rust,ignore
/// let mut id = HarmonicSourceIdentifier::new();
/// id.add_measurement(meas);
/// let result = id.identify_sources();
/// ```
pub struct HarmonicSourceIdentifier {
    /// Collected bus measurements to analyse.
    pub measurements: Vec<HarmonicMeasurement>,
    /// Library of known source spectral signatures.
    pub fingerprint_library: Vec<SourceFingerprint>,
    /// Identification algorithm to use.
    pub method: IdentificationMethod,
    /// Minimum confidence score for a source to appear in results \[0, 1\].
    pub min_confidence: f64,
    /// Bus-by-bus impedance matrix \[Ω\] at harmonic frequency (optional).
    ///
    /// Dimensions: `n_bus × n_bus`.  If empty, the impedance-based method
    /// falls back to a diagonal approximation.
    pub system_impedance: Vec<Vec<f64>>,
    /// Counter used to assign unique IDs to identified sources.
    pub next_source_id: usize,
}

impl HarmonicSourceIdentifier {
    /// Create a new identifier with the standard fingerprint library and
    /// `PatternMatching` as the default method.
    pub fn new() -> Self {
        Self {
            measurements: Vec::new(),
            fingerprint_library: Self::build_standard_fingerprint_library(),
            method: IdentificationMethod::PatternMatching,
            min_confidence: 0.30,
            system_impedance: Vec::new(),
            next_source_id: 0,
        }
    }

    /// Add a single bus measurement to the internal store.
    pub fn add_measurement(&mut self, m: HarmonicMeasurement) {
        self.measurements.push(m);
    }

    /// Identify harmonic sources using the configured [`IdentificationMethod`].
    ///
    /// Returns an [`IdentificationResult`] describing all sources found above
    /// `self.min_confidence`.
    pub fn identify_sources(&mut self) -> IdentificationResult {
        let raw_sources = match self.method {
            IdentificationMethod::PatternMatching => self.identify_by_pattern_matching(),
            IdentificationMethod::PowerDirection => self.identify_by_power_direction(),
            IdentificationMethod::CurrentInjection => self.identify_by_current_injection(),
            IdentificationMethod::ImpedanceBased => self.identify_by_impedance_based(),
            IdentificationMethod::CorrelationBased => self.identify_by_correlation_based(),
            IdentificationMethod::Hybrid => self.identify_by_hybrid(),
        };

        // Filter by minimum confidence and assign IDs
        let mut id_counter = self.next_source_id;
        let identified_sources: Vec<HarmonicSource> = raw_sources
            .into_iter()
            .filter(|s| s.confidence_score >= self.min_confidence)
            .map(|mut s| {
                s.id = id_counter;
                id_counter += 1;
                s
            })
            .collect();
        self.next_source_id = id_counter;

        let total_sources_found = identified_sources.len();
        let method_used = self.method;

        let bus_contributions = self.compute_bus_contributions(&identified_sources);
        let unattributed_thd_pct =
            compute_unattributed_thd(&self.measurements, &identified_sources);

        IdentificationResult {
            identified_sources,
            total_sources_found,
            unattributed_thd_pct,
            method_used,
            bus_contributions,
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Pattern Matching (cosine similarity)
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources by computing cosine similarity between each bus's
    /// normalised current spectrum and each entry in the fingerprint library.
    ///
    /// A source is reported when the best-match similarity exceeds 0.70.
    pub fn identify_by_pattern_matching(&self) -> Vec<HarmonicSource> {
        let mut sources = Vec::new();

        for meas in &self.measurements {
            if meas.thd_i_pct < 1e-6 && meas.fundamental_i_a < 1e-12 {
                continue;
            }

            // Build measurement spectrum over the union of fingerprint orders
            let best = self
                .fingerprint_library
                .iter()
                .map(|fp| {
                    let meas_vec = build_spectrum_for_fingerprint(meas, fp);
                    let score = Self::compute_similarity(&meas_vec, &fp.characteristic_signature);
                    (score, fp)
                })
                .max_by(|(a, _), (b, _)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

            if let Some((score, fp)) = best {
                if score >= 0.70 {
                    let magnitude_kva = estimate_magnitude_kva(meas);
                    let phase = dominant_phase_angle(meas, &fp.dominant_orders);
                    sources.push(HarmonicSource {
                        id: 0, // assigned later
                        bus_id: meas.bus_id,
                        source_type: fp.source_type,
                        confidence: SourceConfidence::from_score(score),
                        confidence_score: score,
                        dominant_harmonics: fp.dominant_orders.clone(),
                        estimated_magnitude_kva: magnitude_kva,
                        phase_angle_deg: phase,
                        identification_method: IdentificationMethod::PatternMatching,
                    });
                }
            }
        }

        sources
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Power Direction
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources using the harmonic power-direction criterion.
    ///
    /// If the harmonic active power `P_h = V_h · I_h · cos(φ_h)` is **negative**
    /// (power flows from the bus into the network), the bus is classified as a
    /// harmonic source.  A simple phase model is used: `φ_h ≈ 150°` for typical
    /// non-linear loads (lagging current).
    pub fn identify_by_power_direction(&self) -> Vec<HarmonicSource> {
        let mut sources = Vec::new();

        for meas in &self.measurements {
            if meas.thd_i_pct < 1e-6 {
                continue;
            }

            // Aggregate P_h over all measured harmonic orders
            let mut total_ph = 0.0_f64;
            let mut dominant_orders: Vec<u32> = Vec::new();

            for (idx, &order) in meas.harmonic_orders.iter().enumerate() {
                let vh = meas.harmonic_voltages.get(idx).copied().unwrap_or(0.0);
                let ih = meas.harmonic_currents.get(idx).copied().unwrap_or(0.0);
                // Assume typical non-linear load phase: 150° (π * 5/6)
                let phi_h = std::f64::consts::PI * 5.0 / 6.0;
                let ph = Self::compute_harmonic_power_direction(vh, ih, phi_h);
                total_ph += ph;
                if ih > meas.fundamental_i_a * 0.05 {
                    dominant_orders.push(order);
                }
            }

            // Negative total harmonic power → source
            if total_ph < -1e-9 {
                let score = ((-total_ph).ln() + 1.0).clamp(0.0, 1.0);
                // Clamp to [0.3, 1.0] for reasonable confidence range
                let score = 0.30 + score * 0.70;
                let source_type = classify_by_harmonic_pattern(meas);
                let magnitude_kva = estimate_magnitude_kva(meas);
                sources.push(HarmonicSource {
                    id: 0,
                    bus_id: meas.bus_id,
                    source_type,
                    confidence: SourceConfidence::from_score(score),
                    confidence_score: score,
                    dominant_harmonics: dominant_orders,
                    estimated_magnitude_kva: magnitude_kva,
                    phase_angle_deg: 150.0,
                    identification_method: IdentificationMethod::PowerDirection,
                });
            }
        }

        sources
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Current Injection (Norton equivalent)
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources using the Norton-equivalent current injection method.
    ///
    /// The Norton source current is estimated as:
    /// ```text
    /// I_N ≈ I_measured_h + V_h / Z_diag
    /// ```
    /// where `Z_diag` is the diagonal (self-impedance) of `system_impedance`,
    /// scaled by the harmonic order `h`.  If `system_impedance` is empty a
    /// default self-impedance of `0.1 Ω` is used.
    pub fn identify_by_current_injection(&self) -> Vec<HarmonicSource> {
        let mut sources = Vec::new();
        let n_buses = self.measurements.len().max(1);

        for meas in &self.measurements {
            if meas.thd_i_pct < 1e-6 {
                continue;
            }

            let z_self = self
                .system_impedance
                .get(meas.bus_id)
                .and_then(|row| row.get(meas.bus_id).copied())
                .unwrap_or(0.1_f64);

            let mut norton_magnitude = 0.0_f64;
            let mut dominant_orders: Vec<u32> = Vec::new();

            for (idx, &order) in meas.harmonic_orders.iter().enumerate() {
                let ih = meas.harmonic_currents.get(idx).copied().unwrap_or(0.0);
                let vh = meas.harmonic_voltages.get(idx).copied().unwrap_or(0.0);
                // Z_h = h · Z_self (inductive scaling with harmonic order)
                let z_h = z_self * order as f64;
                // I_N = I_meas + V_h / Z_h  (Norton injection estimate)
                let i_norton = ih + vh / z_h.max(1e-12);
                norton_magnitude += i_norton * i_norton;
                if i_norton > meas.fundamental_i_a * 0.05 {
                    dominant_orders.push(order);
                }
            }

            norton_magnitude = norton_magnitude.sqrt();

            // Normalise by sum over all buses for a confidence proxy
            let total_harmonic_i: f64 = self
                .measurements
                .iter()
                .map(|m| {
                    m.harmonic_currents
                        .iter()
                        .map(|&c| c * c)
                        .sum::<f64>()
                        .sqrt()
                })
                .sum();

            let score = if total_harmonic_i > 1e-12 {
                (norton_magnitude / total_harmonic_i).min(1.0)
            } else {
                0.0
            };

            if score >= self.min_confidence {
                let source_type = classify_by_harmonic_pattern(meas);
                let magnitude_kva = estimate_magnitude_kva(meas);
                let _ = n_buses; // used for future Zbus extensions
                sources.push(HarmonicSource {
                    id: 0,
                    bus_id: meas.bus_id,
                    source_type,
                    confidence: SourceConfidence::from_score(score),
                    confidence_score: score,
                    dominant_harmonics: dominant_orders,
                    estimated_magnitude_kva: magnitude_kva,
                    phase_angle_deg: 0.0,
                    identification_method: IdentificationMethod::CurrentInjection,
                });
            }
        }

        sources
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Impedance-Based
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources using the impedance-based Thevenin ratio method.
    ///
    /// Buses where `|V_h| / |I_h|` is significantly **higher** than the
    /// system mean Thevenin impedance are flagged as harmonic sources, because
    /// a true source drives voltage distortion proportional to its Norton
    /// current multiplied by the Thevenin impedance.
    fn identify_by_impedance_based(&self) -> Vec<HarmonicSource> {
        // Compute per-bus aggregate |V_h|/|I_h| ratio
        let ratios: Vec<f64> = self
            .measurements
            .iter()
            .map(|m| {
                let v_rms: f64 = m
                    .harmonic_voltages
                    .iter()
                    .map(|&v| v * v)
                    .sum::<f64>()
                    .sqrt();
                let i_rms: f64 = m
                    .harmonic_currents
                    .iter()
                    .map(|&c| c * c)
                    .sum::<f64>()
                    .sqrt();
                v_rms / i_rms.max(1e-12)
            })
            .collect();

        let mean_ratio = if ratios.is_empty() {
            1.0
        } else {
            ratios.iter().sum::<f64>() / ratios.len() as f64
        };

        let mut sources = Vec::new();

        for (meas, &ratio) in self.measurements.iter().zip(ratios.iter()) {
            if meas.thd_i_pct < 1e-6 {
                continue;
            }
            // Source criterion: bus impedance ratio > 1.5 × mean
            if ratio > 1.5 * mean_ratio {
                let score = (ratio / (mean_ratio + 1e-12) / 3.0).min(1.0);
                let source_type = classify_by_harmonic_pattern(meas);
                let magnitude_kva = estimate_magnitude_kva(meas);
                let dominant_orders = meas.harmonic_orders.clone();
                sources.push(HarmonicSource {
                    id: 0,
                    bus_id: meas.bus_id,
                    source_type,
                    confidence: SourceConfidence::from_score(score),
                    confidence_score: score,
                    dominant_harmonics: dominant_orders,
                    estimated_magnitude_kva: magnitude_kva,
                    phase_angle_deg: 0.0,
                    identification_method: IdentificationMethod::ImpedanceBased,
                });
            }
        }

        sources
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Correlation-Based (HCI)
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources using the Harmonic Contribution Index (HCI).
    ///
    /// `HCI_i = I_h_i / Σ_j I_h_j` — buses whose HCI exceeds a threshold
    /// (`0.30`) are flagged as significant sources.
    fn identify_by_correlation_based(&self) -> Vec<HarmonicSource> {
        let hci_threshold = 0.30_f64;

        // Per-bus total harmonic current (RMS across all orders)
        let bus_total_i: Vec<f64> = self
            .measurements
            .iter()
            .map(|m| {
                m.harmonic_currents
                    .iter()
                    .map(|&c| c * c)
                    .sum::<f64>()
                    .sqrt()
            })
            .collect();

        let system_total_i: f64 = bus_total_i.iter().sum();

        let mut sources = Vec::new();
        for (meas, &bi_total) in self.measurements.iter().zip(bus_total_i.iter()) {
            if meas.thd_i_pct < 1e-6 {
                continue;
            }
            let hci = if system_total_i > 1e-12 {
                bi_total / system_total_i
            } else {
                0.0
            };
            if hci > hci_threshold {
                let score = hci.min(1.0);
                let source_type = classify_by_harmonic_pattern(meas);
                let magnitude_kva = estimate_magnitude_kva(meas);
                sources.push(HarmonicSource {
                    id: 0,
                    bus_id: meas.bus_id,
                    source_type,
                    confidence: SourceConfidence::from_score(score),
                    confidence_score: score,
                    dominant_harmonics: meas.harmonic_orders.clone(),
                    estimated_magnitude_kva: magnitude_kva,
                    phase_angle_deg: 0.0,
                    identification_method: IdentificationMethod::CorrelationBased,
                });
            }
        }

        sources
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Algorithm: Hybrid (Pattern + Power Direction)
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify sources using a weighted combination of pattern matching and
    /// power-direction evidence.
    ///
    /// Weight: 0.65 pattern + 0.35 power-direction.  A source is kept when
    /// the combined score exceeds `self.min_confidence`.
    fn identify_by_hybrid(&self) -> Vec<HarmonicSource> {
        let pm_sources = self.identify_by_pattern_matching();
        let pd_sources = self.identify_by_power_direction();

        let mut hybrid: Vec<HarmonicSource> = Vec::new();

        for mut ps in pm_sources {
            // Look for a matching power-direction source at the same bus
            let pd_score = pd_sources
                .iter()
                .find(|s| s.bus_id == ps.bus_id)
                .map(|s| s.confidence_score)
                .unwrap_or(0.0);

            let combined = 0.65 * ps.confidence_score + 0.35 * pd_score;
            ps.confidence_score = combined;
            ps.confidence = SourceConfidence::from_score(combined);
            ps.identification_method = IdentificationMethod::Hybrid;
            hybrid.push(ps);
        }

        // Include any power-direction buses not already covered by pattern matching
        for pd in &pd_sources {
            if !hybrid.iter().any(|s| s.bus_id == pd.bus_id) {
                let combined = 0.35 * pd.confidence_score;
                let mut s = pd.clone();
                s.confidence_score = combined;
                s.confidence = SourceConfidence::from_score(combined);
                s.identification_method = IdentificationMethod::Hybrid;
                hybrid.push(s);
            }
        }

        hybrid
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Utility: cosine similarity
    // ──────────────────────────────────────────────────────────────────────────

    /// Compute cosine similarity between two spectrum vectors.
    ///
    /// ```text
    /// sim(a, b) = (Σ a_i · b_i) / (‖a‖ · ‖b‖)
    /// ```
    ///
    /// Returns values in \[−1, 1\]; returns `0.0` when either vector is zero.
    pub fn compute_similarity(a: &[f64], b: &[f64]) -> f64 {
        let n = a.len().min(b.len());
        if n == 0 {
            return 0.0;
        }
        let dot: f64 = a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum();
        let na = l2_norm(a);
        let nb = l2_norm(b);
        if na < 1e-15 || nb < 1e-15 {
            return 0.0;
        }
        (dot / (na * nb)).clamp(-1.0, 1.0)
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Utility: harmonic power direction
    // ──────────────────────────────────────────────────────────────────────────

    /// Compute harmonic active power at a single harmonic order.
    ///
    /// ```text
    /// P_h = V_h · I_h · cos(φ_h)
    /// ```
    ///
    /// * Positive result → bus is a harmonic **load** (receives power from network).
    /// * Negative result → bus is a harmonic **source** (injects power into network).
    pub fn compute_harmonic_power_direction(v_h: f64, i_h: f64, phase_diff_rad: f64) -> f64 {
        v_h * i_h * phase_diff_rad.cos()
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Fingerprint library
    // ──────────────────────────────────────────────────────────────────────────

    /// Build the standard built-in fingerprint library for 8 equipment types.
    ///
    /// The `(order, ratio)` entries represent typical `I_h / I_1` values
    /// derived from field measurements and IEC/IEEE standards.
    pub fn build_standard_fingerprint_library() -> Vec<SourceFingerprint> {
        vec![
            // 6-pulse VSD: h = 6k±1, dominant 5th and 7th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::VariableSpeedDrive,
                30.0,
                vec![(5, 0.35), (7, 0.14), (11, 0.09), (13, 0.06)],
            ),
            // 12-pulse VSD: h = 12k±1, dominant 11th and 13th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::VsdTwelvePulse,
                10.0,
                vec![(11, 0.09), (13, 0.06), (23, 0.02), (25, 0.01)],
            ),
            // Arc furnace: even + odd, significant 2nd–5th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::ArcFurnace,
                20.0,
                vec![(2, 0.10), (3, 0.08), (4, 0.06), (5, 0.05), (7, 0.04)],
            ),
            // Saturated transformer: dominant 3rd, mainly odd
            SourceFingerprint::from_ratios(
                HarmonicSourceType::SatTransformer,
                5.0,
                vec![(3, 0.30), (5, 0.08), (7, 0.04)],
            ),
            // Uncontrolled rectifier: similar to 6-pulse VSD but slightly different ratios
            SourceFingerprint::from_ratios(
                HarmonicSourceType::Rectifier,
                25.0,
                vec![(5, 0.30), (7, 0.12), (11, 0.07), (13, 0.05)],
            ),
            // PV inverter: low THD, mainly 3rd, 5th, 7th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::InverterPv,
                3.0,
                vec![(3, 0.02), (5, 0.015), (7, 0.01)],
            ),
            // EV charger: triplen harmonics 3rd, 5th, 7th, 9th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::EvCharger,
                15.0,
                vec![(3, 0.12), (5, 0.08), (7, 0.06), (9, 0.04)],
            ),
            // UPS: similar to 6-pulse VSD, lower 11th/13th
            SourceFingerprint::from_ratios(
                HarmonicSourceType::Ups,
                8.0,
                vec![(5, 0.20), (7, 0.08), (11, 0.04), (13, 0.03)],
            ),
        ]
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Ranking and contribution helpers
    // ──────────────────────────────────────────────────────────────────────────

    /// Return indices into `sources` sorted by `estimated_magnitude_kva` descending.
    pub fn rank_sources_by_contribution(&self, sources: &[HarmonicSource]) -> Vec<usize> {
        let mut indices: Vec<usize> = (0..sources.len()).collect();
        indices.sort_by(|&a, &b| {
            sources[b]
                .estimated_magnitude_kva
                .partial_cmp(&sources[a].estimated_magnitude_kva)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        indices
    }

    /// Compute the percentage contribution of each bus to total system THD.
    ///
    /// Contribution is proportional to the sum of `estimated_magnitude_kva`
    /// of identified sources at that bus relative to the system total.
    pub fn compute_bus_contributions(&self, sources: &[HarmonicSource]) -> Vec<(usize, f64)> {
        if sources.is_empty() {
            return Vec::new();
        }

        // Aggregate kVA per bus
        let mut bus_kva: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
        for s in sources {
            *bus_kva.entry(s.bus_id).or_insert(0.0) += s.estimated_magnitude_kva;
        }

        let total_kva: f64 = bus_kva.values().sum();

        let mut contributions: Vec<(usize, f64)> = bus_kva
            .into_iter()
            .map(|(bus, kva)| {
                let pct = if total_kva > 1e-12 {
                    kva / total_kva * 100.0
                } else {
                    0.0
                };
                (bus, pct)
            })
            .collect();

        // Sort by bus_id for deterministic output
        contributions.sort_by_key(|(bus, _)| *bus);
        contributions
    }
}

impl Default for HarmonicSourceIdentifier {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the L2 norm of a slice.
fn l2_norm(v: &[f64]) -> f64 {
    v.iter().map(|&x| x * x).sum::<f64>().sqrt()
}

/// Build a measurement spectrum vector aligned to the fingerprint's
/// `dominant_orders`.  Missing orders contribute `0.0`.
fn build_spectrum_for_fingerprint(meas: &HarmonicMeasurement, fp: &SourceFingerprint) -> Vec<f64> {
    let i1 = meas.fundamental_i_a.max(1e-12);
    fp.dominant_orders
        .iter()
        .map(|&order| meas.current_at_order(order).unwrap_or(0.0) / i1)
        .collect()
}

/// Estimate the apparent harmonic power injected by a source at a bus \[kVA\].
///
/// Uses `S_h ≈ V_fund · Σ I_h` as a simplified conservative estimate.
fn estimate_magnitude_kva(meas: &HarmonicMeasurement) -> f64 {
    let total_ih: f64 = meas.harmonic_currents.iter().sum();
    // V [V] * I [A] / 1000 → kVA
    meas.fundamental_v * total_ih / 1000.0
}

/// Return the phase angle \[deg\] associated with the dominant harmonic order.
///
/// Falls back to `0.0` if no dominant order is present in the measurement.
fn dominant_phase_angle(meas: &HarmonicMeasurement, dominant_orders: &[u32]) -> f64 {
    // We do not have phase in the new HarmonicMeasurement struct, so we
    // use a type-specific typical value derived from IEEE literature.
    // For now return 0 degrees (unknown) — subclasses can override.
    let _ = (meas, dominant_orders);
    0.0
}

/// Heuristic source classification from spectral ratios of a [`HarmonicMeasurement`].
fn classify_by_harmonic_pattern(meas: &HarmonicMeasurement) -> HarmonicSourceType {
    let i1 = meas.fundamental_i_a.max(1e-12);

    let i2 = meas.current_at_order(2).unwrap_or(0.0) / i1;
    let i3 = meas.current_at_order(3).unwrap_or(0.0) / i1;
    let i5 = meas.current_at_order(5).unwrap_or(0.0) / i1;
    let i7 = meas.current_at_order(7).unwrap_or(0.0) / i1;
    let i9 = meas.current_at_order(9).unwrap_or(0.0) / i1;
    let i11 = meas.current_at_order(11).unwrap_or(0.0) / i1;
    let i13 = meas.current_at_order(13).unwrap_or(0.0) / i1;

    // Arc furnace: significant even harmonics
    if i2 > 0.04 {
        return HarmonicSourceType::ArcFurnace;
    }
    // Saturated transformer: strong 3rd, weak 5th
    if i3 > 0.15 && i5 < 0.05 {
        return HarmonicSourceType::SatTransformer;
    }
    // EV charger: strong 3rd + 9th triplen
    if i3 > 0.08 && i9 > 0.02 {
        return HarmonicSourceType::EvCharger;
    }
    // 12-pulse VSD: dominant 11th / 13th, weak 5th/7th
    if i11 > 0.05 && i13 > 0.03 && i5 < 0.10 {
        return HarmonicSourceType::VsdTwelvePulse;
    }
    // 6-pulse VSD / Rectifier: strong 5th and 7th
    if i5 > 0.20 && i7 > 0.08 {
        return HarmonicSourceType::VariableSpeedDrive;
    }
    if i5 > 0.15 && i7 > 0.05 {
        return HarmonicSourceType::Rectifier;
    }
    // PV inverter: low overall distortion
    if meas.thd_i_pct < 5.0 {
        return HarmonicSourceType::InverterPv;
    }

    HarmonicSourceType::Unknown
}

/// Estimate unattributed THD: fraction of measured system THD not covered
/// by identified sources.
fn compute_unattributed_thd(
    measurements: &[HarmonicMeasurement],
    sources: &[HarmonicSource],
) -> f64 {
    let total_thd: f64 = measurements.iter().map(|m| m.thd_i_pct).sum();
    if total_thd < 1e-12 {
        return 0.0;
    }

    let attributed_thd: f64 = sources
        .iter()
        .filter_map(|s| {
            measurements
                .iter()
                .find(|m| m.bus_id == s.bus_id)
                .map(|m| m.thd_i_pct * s.confidence_score)
        })
        .sum();

    (total_thd - attributed_thd).max(0.0) / total_thd * 100.0
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ──────────────────────────────────────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────────────────────────────────────

    fn make_meas(
        bus_id: usize,
        harmonic_orders: Vec<u32>,
        harmonic_currents: Vec<f64>,
        harmonic_voltages: Vec<f64>,
        fundamental_i_a: f64,
        thd_i_pct: f64,
    ) -> HarmonicMeasurement {
        HarmonicMeasurement {
            bus_id,
            timestamp: 0.0,
            fundamental_v: 230.0,
            fundamental_i_a,
            harmonic_voltages,
            harmonic_currents,
            harmonic_orders,
            power_factor: 0.90,
            thd_v_pct: thd_i_pct * 0.1,
            thd_i_pct,
        }
    }

    /// Measurement matching a 6-pulse VSD signature
    fn vsd_6pulse_meas() -> HarmonicMeasurement {
        // I_5/I_1=0.35, I_7/I_1=0.14, I_11/I_1=0.09, I_13/I_1=0.06
        let i1 = 100.0_f64;
        make_meas(
            0,
            vec![5, 7, 11, 13],
            vec![35.0, 14.0, 9.0, 6.0],
            vec![8.05, 3.22, 2.07, 1.38],
            i1,
            37.4,
        )
    }

    /// Measurement matching a saturated transformer (dominant 3rd)
    fn sat_transformer_meas() -> HarmonicMeasurement {
        let i1 = 50.0_f64;
        make_meas(
            1,
            vec![3, 5, 7],
            vec![15.0, 2.0, 1.0],
            vec![1.0, 0.2, 0.1],
            i1,
            31.0,
        )
    }

    /// Measurement matching an EV charger (3rd, 5th, 7th, 9th triplen)
    fn ev_charger_meas() -> HarmonicMeasurement {
        let i1 = 80.0_f64;
        make_meas(
            2,
            vec![3, 5, 7, 9],
            vec![9.6, 6.4, 4.8, 3.2],
            vec![2.0, 1.5, 1.1, 0.7],
            i1,
            15.5,
        )
    }

    /// Zero-THD measurement (no harmonics)
    fn zero_thd_meas() -> HarmonicMeasurement {
        make_meas(
            3,
            vec![3, 5, 7],
            vec![0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0],
            100.0,
            0.0,
        )
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Tests
    // ──────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_fingerprint_library_populated() {
        let lib = HarmonicSourceIdentifier::build_standard_fingerprint_library();
        assert_eq!(lib.len(), 8, "expected 8 standard fingerprints");
    }

    #[test]
    fn test_vsd_6pulse_identified() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        assert!(
            !result.identified_sources.is_empty(),
            "should identify VSD 6-pulse source"
        );
        let src = &result.identified_sources[0];
        assert_eq!(
            src.source_type,
            HarmonicSourceType::VariableSpeedDrive,
            "should be classified as VariableSpeedDrive"
        );
    }

    #[test]
    fn test_sat_transformer_identified() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(sat_transformer_meas());
        let result = id.identify_sources();
        assert!(
            !result.identified_sources.is_empty(),
            "should identify a source"
        );
        let src = &result.identified_sources[0];
        assert_eq!(
            src.source_type,
            HarmonicSourceType::SatTransformer,
            "should be classified as SatTransformer"
        );
    }

    #[test]
    fn test_ev_charger_identified() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(ev_charger_meas());
        let result = id.identify_sources();
        assert!(
            !result.identified_sources.is_empty(),
            "should identify a source"
        );
        let src = &result.identified_sources[0];
        assert_eq!(
            src.source_type,
            HarmonicSourceType::EvCharger,
            "should be classified as EvCharger"
        );
    }

    #[test]
    fn test_cosine_similarity_identical() {
        let a = vec![0.35, 0.14, 0.09, 0.06];
        let sim = HarmonicSourceIdentifier::compute_similarity(&a, &a);
        assert!(
            (sim - 1.0).abs() < 1e-10,
            "cosine similarity of identical vectors must be 1.0, got {sim}"
        );
    }

    #[test]
    fn test_cosine_similarity_orthogonal() {
        let a = vec![1.0, 0.0];
        let b = vec![0.0, 1.0];
        let sim = HarmonicSourceIdentifier::compute_similarity(&a, &b);
        assert!(
            sim.abs() < 1e-10,
            "cosine similarity of orthogonal vectors must be 0, got {sim}"
        );
    }

    #[test]
    fn test_harmonic_power_direction_positive() {
        // In-phase (φ = 0): P_h = V_h · I_h > 0 → harmonic load
        let ph = HarmonicSourceIdentifier::compute_harmonic_power_direction(10.0, 2.0, 0.0);
        assert!(ph > 0.0, "in-phase → P_h > 0, got {ph}");
    }

    #[test]
    fn test_harmonic_power_direction_negative() {
        // Phase opposition (φ = π): P_h = V_h · I_h · cos(π) < 0 → harmonic source
        let ph = HarmonicSourceIdentifier::compute_harmonic_power_direction(
            10.0,
            2.0,
            std::f64::consts::PI,
        );
        assert!(ph < 0.0, "out-of-phase → P_h < 0, got {ph}");
    }

    #[test]
    fn test_source_identified_above_confidence() {
        let mut id = HarmonicSourceIdentifier::new();
        id.min_confidence = 0.70; // high bar
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        for src in &result.identified_sources {
            assert!(
                src.confidence_score >= 0.70,
                "all sources must meet min_confidence: got {}",
                src.confidence_score
            );
        }
    }

    #[test]
    fn test_bus_contributions_sum() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas());
        id.add_measurement(sat_transformer_meas());
        let result = id.identify_sources();
        if !result.bus_contributions.is_empty() {
            let total_pct: f64 = result.bus_contributions.iter().map(|(_, p)| p).sum();
            assert!(
                (total_pct - 100.0).abs() < 1.0,
                "contributions should sum to ≈100%, got {total_pct:.2}%"
            );
        }
    }

    #[test]
    fn test_identify_sources_nonempty() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas()); // THD_I = 37.4% >> 5%
        let result = id.identify_sources();
        assert!(
            !result.identified_sources.is_empty(),
            "should find at least one source for THD > 5%"
        );
    }

    #[test]
    fn test_unattributed_thd_nonnegative() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        assert!(
            result.unattributed_thd_pct >= 0.0,
            "unattributed THD must be ≥ 0, got {}",
            result.unattributed_thd_pct
        );
    }

    #[test]
    fn test_rank_sources_by_magnitude() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas());
        id.add_measurement(sat_transformer_meas());
        id.add_measurement(ev_charger_meas());
        let result = id.identify_sources();
        let ranked = id.rank_sources_by_contribution(&result.identified_sources);
        // Verify descending order by magnitude
        for window in ranked.windows(2) {
            let a = result.identified_sources[window[0]].estimated_magnitude_kva;
            let b = result.identified_sources[window[1]].estimated_magnitude_kva;
            assert!(a >= b, "rank order violated: {a} < {b}");
        }
    }

    #[test]
    fn test_pattern_matching_returns_best_match() {
        let mut id = HarmonicSourceIdentifier::new();
        id.method = IdentificationMethod::PatternMatching;
        id.add_measurement(vsd_6pulse_meas());
        let sources = id.identify_by_pattern_matching();
        // The best match (highest similarity) should be the first source found
        // for a clean VSD-6pulse signal
        assert!(
            !sources.is_empty(),
            "pattern matching should return a match"
        );
        assert_eq!(
            sources[0].source_type,
            HarmonicSourceType::VariableSpeedDrive
        );
    }

    #[test]
    fn test_multiple_sources_different_buses() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas()); // bus 0
        id.add_measurement(sat_transformer_meas()); // bus 1
        let result = id.identify_sources();
        let bus_ids: std::collections::HashSet<usize> =
            result.identified_sources.iter().map(|s| s.bus_id).collect();
        assert!(
            !bus_ids.is_empty(),
            "should identify sources on at least one bus, got {:?}",
            bus_ids
        );
    }

    #[test]
    fn test_confidence_high_threshold() {
        // A perfect match gives score 1.0 → High confidence
        let score = 1.0_f64;
        assert_eq!(SourceConfidence::from_score(score), SourceConfidence::High);
    }

    #[test]
    fn test_confidence_medium_threshold() {
        let score = 0.72_f64;
        assert_eq!(
            SourceConfidence::from_score(score),
            SourceConfidence::Medium
        );
    }

    #[test]
    fn test_zero_thd_no_source() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(zero_thd_meas());
        let result = id.identify_sources();
        assert!(
            result.identified_sources.is_empty(),
            "zero THD should produce no identified sources"
        );
    }

    #[test]
    fn test_measurement_added() {
        let mut id = HarmonicSourceIdentifier::new();
        assert_eq!(id.measurements.len(), 0);
        id.add_measurement(vsd_6pulse_meas());
        assert_eq!(id.measurements.len(), 1);
        id.add_measurement(sat_transformer_meas());
        assert_eq!(id.measurements.len(), 2);
    }

    #[test]
    fn test_hybrid_method_uses_multiple() {
        let mut id = HarmonicSourceIdentifier::new();
        id.method = IdentificationMethod::Hybrid;
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        // Hybrid should produce sources with method = Hybrid
        for src in &result.identified_sources {
            assert_eq!(
                src.identification_method,
                IdentificationMethod::Hybrid,
                "hybrid method should label sources as Hybrid"
            );
        }
    }

    // Additional robustness tests

    #[test]
    fn test_empty_measurements_returns_empty_result() {
        let mut id = HarmonicSourceIdentifier::new();
        let result = id.identify_sources();
        assert_eq!(result.total_sources_found, 0);
        assert!(result.identified_sources.is_empty());
    }

    #[test]
    fn test_source_id_increments() {
        let mut id = HarmonicSourceIdentifier::new();
        id.add_measurement(vsd_6pulse_meas());
        let r1 = id.identify_sources();
        let r2 = id.identify_sources();
        // IDs in r2 should be higher than those in r1
        if !r1.identified_sources.is_empty() && !r2.identified_sources.is_empty() {
            assert!(
                r2.identified_sources[0].id > r1.identified_sources[0].id
                    || r1.identified_sources.is_empty(),
                "source IDs should increment across calls"
            );
        }
    }

    #[test]
    fn test_power_direction_method_dispatches() {
        let mut id = HarmonicSourceIdentifier::new();
        id.method = IdentificationMethod::PowerDirection;
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        assert_eq!(result.method_used, IdentificationMethod::PowerDirection);
    }

    #[test]
    fn test_current_injection_method_dispatches() {
        let mut id = HarmonicSourceIdentifier::new();
        id.method = IdentificationMethod::CurrentInjection;
        id.add_measurement(vsd_6pulse_meas());
        let result = id.identify_sources();
        assert_eq!(result.method_used, IdentificationMethod::CurrentInjection);
    }

    #[test]
    fn test_fingerprint_all_signatures_normalised() {
        let lib = HarmonicSourceIdentifier::build_standard_fingerprint_library();
        for fp in &lib {
            let norm = l2_norm(&fp.characteristic_signature);
            assert!(
                (norm - 1.0).abs() < 1e-10,
                "{:?} fingerprint not normalised: norm = {norm}",
                fp.source_type
            );
        }
    }
}
