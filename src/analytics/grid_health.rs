//! Grid health scoring: composite index computation, threshold classification,
//! and multi-component aggregation for power system health monitoring.
//!
//! # Overview
//!
//! The module provides a single-pass pipeline:
//!
//! 1. Collect [`ComponentHealth`] readings (one per monitored component).
//! 2. Build a [`GridHealthScorer`] with optional per-category weights.
//! 3. Call [`GridHealthScorer::compute`] to obtain a [`GridHealthReport`] with
//!    – a composite score in `[0.0, 100.0]`,
//!    – per-component classification ([`HealthStatus`]),
//!    – and a sorted list of the most critical components.
//!
//! # Standards basis
//!
//! Thresholds follow common utility practice and IEC 60300 / IEEE 1910
//! condition-assessment guidance:
//! - Score ≥ 80 → Healthy
//! - 60 ≤ score < 80 → Warning
//! - score < 60 → Critical

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Three-level health status classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum HealthStatus {
    /// Score < 60 — immediate intervention required.
    Critical,
    /// 60 ≤ score < 80 — degraded; action recommended.
    Warning,
    /// Score ≥ 80 — operating within normal bounds.
    Healthy,
}

impl HealthStatus {
    /// Classify a `[0, 100]` score into a [`HealthStatus`].
    ///
    /// # Arguments
    ///
    /// * `score` — composite health score in `[0.0, 100.0]`.
    pub fn from_score(score: f64) -> Self {
        if score >= 80.0 {
            Self::Healthy
        } else if (60.0..80.0).contains(&score) {
            Self::Warning
        } else {
            Self::Critical
        }
    }

    /// Return `true` when the status is [`HealthStatus::Critical`].
    pub fn is_critical(self) -> bool {
        self == Self::Critical
    }
}

/// Category of a monitored component, used for weight-grouped aggregation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ComponentCategory {
    /// High-voltage transmission equipment.
    Transmission,
    /// Medium/low-voltage distribution equipment.
    Distribution,
    /// Generation assets (thermal, renewable, storage).
    Generation,
    /// Protection and control devices.
    Protection,
    /// Communication and SCADA infrastructure.
    Communication,
}

/// A single-component health snapshot supplied by the caller.
///
/// All scores are in `[0.0, 100.0]`.  The `weight` is relative within its
/// [`ComponentCategory`]; all weights within a category are normalised before
/// aggregation so they need not sum to 1.
#[derive(Debug, Clone)]
pub struct ComponentHealth {
    /// Unique component identifier (e.g. asset tag).
    pub id: String,
    /// Descriptive name.
    pub name: String,
    /// Equipment category.
    pub category: ComponentCategory,
    /// Raw health score from the underlying condition monitor `[0, 100]`.
    pub score: f64,
    /// Relative importance weight within its category (must be > 0).
    pub weight: f64,
    /// Unix-epoch timestamp of this reading (seconds).
    pub timestamp_s: f64,
}

/// Aggregation result for a single [`ComponentCategory`].
#[derive(Debug, Clone)]
pub struct CategoryAggregate {
    /// The category being summarised.
    pub category: ComponentCategory,
    /// Weighted-average health score for all components in this category.
    pub weighted_score: f64,
    /// Overall status classification for this category.
    pub status: HealthStatus,
    /// Number of components in this category.
    pub component_count: usize,
    /// Number of components classified as [`HealthStatus::Critical`].
    pub critical_count: usize,
}

/// Full grid health report returned by [`GridHealthScorer::compute`].
#[derive(Debug, Clone)]
pub struct GridHealthReport {
    /// Composite grid health score in `[0.0, 100.0]`.
    pub composite_score: f64,
    /// Overall grid status derived from `composite_score`.
    pub overall_status: HealthStatus,
    /// Per-category aggregates.
    pub categories: Vec<CategoryAggregate>,
    /// Ids of components with `Critical` status, sorted by ascending score
    /// (worst-first).
    pub critical_components: Vec<String>,
    /// Total number of components processed.
    pub total_components: usize,
    /// Count of components in each status bucket.
    pub healthy_count: usize,
    /// Count of Warning-status components.
    pub warning_count: usize,
    /// Count of Critical-status components.
    pub critical_count: usize,
}

impl GridHealthReport {
    /// Return `true` when no component is in a [`HealthStatus::Critical`] state.
    pub fn all_healthy(&self) -> bool {
        self.critical_count == 0 && self.warning_count == 0
    }

    /// Return the fraction of components in [`HealthStatus::Critical`] state.
    pub fn critical_fraction(&self) -> f64 {
        if self.total_components == 0 {
            return 0.0;
        }
        self.critical_count as f64 / self.total_components as f64
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Per-category weight used when computing the composite score.
///
/// Default: all categories have equal weight 1.0.
#[derive(Debug, Clone)]
pub struct CategoryWeight {
    /// Weight for [`ComponentCategory::Transmission`].
    pub transmission: f64,
    /// Weight for [`ComponentCategory::Distribution`].
    pub distribution: f64,
    /// Weight for [`ComponentCategory::Generation`].
    pub generation: f64,
    /// Weight for [`ComponentCategory::Protection`].
    pub protection: f64,
    /// Weight for [`ComponentCategory::Communication`].
    pub communication: f64,
}

impl Default for CategoryWeight {
    fn default() -> Self {
        Self {
            transmission: 1.0,
            distribution: 1.0,
            generation: 1.0,
            protection: 1.0,
            communication: 1.0,
        }
    }
}

impl CategoryWeight {
    /// Look up the configured weight for a given `category`.
    pub fn get(&self, category: ComponentCategory) -> f64 {
        match category {
            ComponentCategory::Transmission => self.transmission,
            ComponentCategory::Distribution => self.distribution,
            ComponentCategory::Generation => self.generation,
            ComponentCategory::Protection => self.protection,
            ComponentCategory::Communication => self.communication,
        }
    }
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors that can occur during health score computation.
#[derive(Debug, Clone, PartialEq)]
pub enum GridHealthError {
    /// No component readings were supplied.
    NoComponents,
    /// A component has an invalid (non-positive) weight.
    InvalidWeight {
        /// Component id that triggered the error.
        component_id: String,
    },
    /// A component score is outside `[0, 100]`.
    ScoreOutOfRange {
        /// Component id that triggered the error.
        component_id: String,
        /// The offending score value.
        score: f64,
    },
}

impl std::fmt::Display for GridHealthError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoComponents => write!(f, "no component health readings supplied"),
            Self::InvalidWeight { component_id } => {
                write!(f, "component '{}' has non-positive weight", component_id)
            }
            Self::ScoreOutOfRange {
                component_id,
                score,
            } => {
                write!(
                    f,
                    "component '{}' has score {:.2} outside [0, 100]",
                    component_id, score
                )
            }
        }
    }
}

impl std::error::Error for GridHealthError {}

// ---------------------------------------------------------------------------
// Scorer
// ---------------------------------------------------------------------------

/// Computes a composite grid health score from a collection of component readings.
///
/// The algorithm is a two-level weighted average:
///
/// 1. Within each [`ComponentCategory`], a weight-normalised average is computed
///    over all component scores.
/// 2. Across categories, each category average is weighted by the corresponding
///    [`CategoryWeight`] entry.
///
/// Categories that have no components are ignored in the top-level average.
#[derive(Debug, Clone, Default)]
pub struct GridHealthScorer {
    /// Category importance weights (can be tuned per-utility).
    pub category_weights: CategoryWeight,
}

impl GridHealthScorer {
    /// Create a scorer with default (equal) category weights.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a scorer with explicit category weights.
    pub fn with_weights(weights: CategoryWeight) -> Self {
        Self {
            category_weights: weights,
        }
    }

    /// Compute the full [`GridHealthReport`] from a slice of component readings.
    ///
    /// # Errors
    ///
    /// Returns [`GridHealthError::NoComponents`] when `components` is empty.
    /// Returns [`GridHealthError::InvalidWeight`] for any component with weight ≤ 0.
    /// Returns [`GridHealthError::ScoreOutOfRange`] for any score outside `[0, 100]`.
    pub fn compute(
        &self,
        components: &[ComponentHealth],
    ) -> Result<GridHealthReport, GridHealthError> {
        if components.is_empty() {
            return Err(GridHealthError::NoComponents);
        }

        // Validate inputs
        for c in components {
            if c.weight <= 0.0 {
                return Err(GridHealthError::InvalidWeight {
                    component_id: c.id.clone(),
                });
            }
            if !(0.0..=100.0).contains(&c.score) {
                return Err(GridHealthError::ScoreOutOfRange {
                    component_id: c.id.clone(),
                    score: c.score,
                });
            }
        }

        // Group components by category
        let mut by_category: HashMap<u8, Vec<&ComponentHealth>> = HashMap::new();
        for c in components {
            by_category
                .entry(category_key(c.category))
                .or_default()
                .push(c);
        }

        // Per-category aggregation
        let mut categories: Vec<CategoryAggregate> = Vec::new();
        for (key, members) in &by_category {
            let cat = category_from_key(*key);
            let total_weight: f64 = members.iter().map(|m| m.weight).sum();
            let weighted_score: f64 = members.iter().map(|m| m.weight * m.score).sum::<f64>()
                / total_weight.max(f64::EPSILON);
            let critical_count = members
                .iter()
                .filter(|m| HealthStatus::from_score(m.score).is_critical())
                .count();
            categories.push(CategoryAggregate {
                category: cat,
                weighted_score,
                status: HealthStatus::from_score(weighted_score),
                component_count: members.len(),
                critical_count,
            });
        }

        // Composite score: weight each category aggregate by CategoryWeight
        let mut numerator = 0.0_f64;
        let mut denominator = 0.0_f64;
        for agg in &categories {
            let w = self.category_weights.get(agg.category);
            numerator += w * agg.weighted_score;
            denominator += w;
        }
        let composite_score = if denominator > f64::EPSILON {
            numerator / denominator
        } else {
            0.0
        };

        // Status buckets and critical list
        let mut healthy_count = 0usize;
        let mut warning_count = 0usize;
        let mut critical_count = 0usize;
        let mut critical_components: Vec<(String, f64)> = Vec::new();

        for c in components {
            match HealthStatus::from_score(c.score) {
                HealthStatus::Healthy => healthy_count += 1,
                HealthStatus::Warning => warning_count += 1,
                HealthStatus::Critical => {
                    critical_count += 1;
                    critical_components.push((c.id.clone(), c.score));
                }
            }
        }

        // Sort critical list worst-first (ascending score)
        critical_components
            .sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let critical_ids: Vec<String> = critical_components.into_iter().map(|(id, _)| id).collect();

        // Sort categories for deterministic output
        categories.sort_by_key(|a| category_key(a.category));

        Ok(GridHealthReport {
            composite_score,
            overall_status: HealthStatus::from_score(composite_score),
            categories,
            critical_components: critical_ids,
            total_components: components.len(),
            healthy_count,
            warning_count,
            critical_count,
        })
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Map a [`ComponentCategory`] to a stable u8 key for `HashMap` keying.
fn category_key(cat: ComponentCategory) -> u8 {
    match cat {
        ComponentCategory::Transmission => 0,
        ComponentCategory::Distribution => 1,
        ComponentCategory::Generation => 2,
        ComponentCategory::Protection => 3,
        ComponentCategory::Communication => 4,
    }
}

/// Reverse mapping from the stable key back to [`ComponentCategory`].
fn category_from_key(key: u8) -> ComponentCategory {
    match key {
        0 => ComponentCategory::Transmission,
        1 => ComponentCategory::Distribution,
        2 => ComponentCategory::Generation,
        3 => ComponentCategory::Protection,
        4 => ComponentCategory::Communication,
        _ => ComponentCategory::Communication, // unreachable in practice
    }
}

// ---------------------------------------------------------------------------
// Public utility functions
// ---------------------------------------------------------------------------

/// Compute the weighted average score of a heterogeneous component slice.
///
/// Returns `None` when `components` is empty or all weights are zero.
pub fn weighted_average_score(components: &[ComponentHealth]) -> Option<f64> {
    if components.is_empty() {
        return None;
    }
    let total_weight: f64 = components.iter().map(|c| c.weight).sum();
    if total_weight <= f64::EPSILON {
        return None;
    }
    let numerator: f64 = components.iter().map(|c| c.weight * c.score).sum();
    Some(numerator / total_weight)
}

/// Return the ids of components whose score is at or below `threshold`.
///
/// The returned vector is sorted by ascending score (worst first).
pub fn components_below_threshold(components: &[ComponentHealth], threshold: f64) -> Vec<&str> {
    let mut below: Vec<(&str, f64)> = components
        .iter()
        .filter(|c| c.score <= threshold)
        .map(|c| (c.id.as_str(), c.score))
        .collect();
    below.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    below.into_iter().map(|(id, _)| id).collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper constructors ───────────────────────────────────────────────────

    fn make_component(
        id: &str,
        category: ComponentCategory,
        score: f64,
        weight: f64,
    ) -> ComponentHealth {
        ComponentHealth {
            id: id.to_string(),
            name: format!("Component {}", id),
            category,
            score,
            weight,
            timestamp_s: 0.0,
        }
    }

    // ── Test 1: HealthStatus classification boundaries ───────────────────────

    #[test]
    fn test_health_status_boundary_classification() {
        // Exact boundary values must map to the documented thresholds.
        assert_eq!(HealthStatus::from_score(100.0), HealthStatus::Healthy);
        assert_eq!(HealthStatus::from_score(80.0), HealthStatus::Healthy);
        assert_eq!(HealthStatus::from_score(79.9), HealthStatus::Warning);
        assert_eq!(HealthStatus::from_score(60.0), HealthStatus::Warning);
        assert_eq!(HealthStatus::from_score(59.9), HealthStatus::Critical);
        assert_eq!(HealthStatus::from_score(0.0), HealthStatus::Critical);
    }

    // ── Test 2: Empty input returns NoComponents error ────────────────────────

    #[test]
    fn test_empty_components_returns_error() {
        let scorer = GridHealthScorer::new();
        let result = scorer.compute(&[]);
        assert!(
            matches!(result, Err(GridHealthError::NoComponents)),
            "empty component list must yield NoComponents error"
        );
    }

    // ── Test 3: All-healthy components → Healthy status ──────────────────────

    #[test]
    fn test_all_healthy_components_yield_healthy_report() {
        let components = vec![
            make_component("T1", ComponentCategory::Transmission, 90.0, 1.0),
            make_component("T2", ComponentCategory::Transmission, 85.0, 1.0),
            make_component("G1", ComponentCategory::Generation, 95.0, 1.0),
        ];
        let report = GridHealthScorer::new()
            .compute(&components)
            .expect("valid components must not error");

        assert_eq!(report.overall_status, HealthStatus::Healthy);
        assert!(
            report.all_healthy(),
            "no warning/critical components expected"
        );
        assert_eq!(report.critical_count, 0);
        assert_eq!(report.warning_count, 0);
        assert_eq!(report.healthy_count, 3);
    }

    // ── Test 4: Score computation is deterministic and correct ────────────────

    #[test]
    fn test_composite_score_single_category_equal_weights() {
        // Three equal-weight Transmission components with scores 70, 80, 90.
        // Expected composite = (70 + 80 + 90) / 3 = 80.0
        let components = vec![
            make_component("A", ComponentCategory::Transmission, 70.0, 1.0),
            make_component("B", ComponentCategory::Transmission, 80.0, 1.0),
            make_component("C", ComponentCategory::Transmission, 90.0, 1.0),
        ];
        let report = GridHealthScorer::new()
            .compute(&components)
            .expect("valid inputs");

        assert!(
            (report.composite_score - 80.0).abs() < 1e-9,
            "composite score should be 80.0, got {:.4}",
            report.composite_score
        );
    }

    // ── Test 5: Weighted average within a category respects weights ───────────

    #[test]
    fn test_category_weighted_average_non_uniform_weights() {
        // Two Generation components: score=40 weight=3 and score=100 weight=1
        // Weighted avg = (40*3 + 100*1) / (3+1) = 220/4 = 55.0 → Critical
        let components = vec![
            make_component("G_low", ComponentCategory::Generation, 40.0, 3.0),
            make_component("G_high", ComponentCategory::Generation, 100.0, 1.0),
        ];
        let report = GridHealthScorer::new()
            .compute(&components)
            .expect("valid inputs");

        let gen_agg = report
            .categories
            .iter()
            .find(|a| a.category == ComponentCategory::Generation)
            .expect("Generation category must be present");

        assert!(
            (gen_agg.weighted_score - 55.0).abs() < 1e-9,
            "generation weighted score should be 55.0, got {:.4}",
            gen_agg.weighted_score
        );
        assert_eq!(
            gen_agg.status,
            HealthStatus::Critical,
            "55.0 < 60.0 must classify as Critical"
        );
    }

    // ── Test 6: Critical components list is sorted worst-first ───────────────

    #[test]
    fn test_critical_components_sorted_worst_first() {
        // Three Critical components with different scores; expect them
        // in ascending score order (worst first).
        let components = vec![
            make_component("C30", ComponentCategory::Protection, 30.0, 1.0),
            make_component("C10", ComponentCategory::Protection, 10.0, 1.0),
            make_component("C50", ComponentCategory::Protection, 50.0, 1.0),
        ];
        let report = GridHealthScorer::new()
            .compute(&components)
            .expect("valid inputs");

        assert_eq!(report.critical_count, 3);
        assert_eq!(
            report.critical_components,
            vec!["C10", "C30", "C50"],
            "critical components must be sorted ascending by score"
        );
    }

    // ── Test 7: Multi-category aggregation with explicit category weights ─────

    #[test]
    fn test_multi_category_composite_with_custom_weights() {
        // Transmission score = 90, Protection score = 30.
        // With transmission_weight=1, protection_weight=2:
        // composite = (1*90 + 2*30) / (1+2) = 150/3 = 50.0 → Critical
        let components = vec![
            make_component("TX", ComponentCategory::Transmission, 90.0, 1.0),
            make_component("PR", ComponentCategory::Protection, 30.0, 1.0),
        ];
        let weights = CategoryWeight {
            transmission: 1.0,
            distribution: 1.0,
            generation: 1.0,
            protection: 2.0,
            communication: 1.0,
        };
        let scorer = GridHealthScorer::with_weights(weights);
        let report = scorer.compute(&components).expect("valid inputs");

        assert!(
            (report.composite_score - 50.0).abs() < 1e-9,
            "composite should be 50.0, got {:.4}",
            report.composite_score
        );
        assert_eq!(
            report.overall_status,
            HealthStatus::Critical,
            "50.0 must classify as Critical"
        );
    }

    // ── Test 8: `weighted_average_score` utility function ────────────────────

    #[test]
    fn test_weighted_average_score_utility() {
        let components = vec![
            make_component("X1", ComponentCategory::Distribution, 60.0, 2.0),
            make_component("X2", ComponentCategory::Distribution, 80.0, 2.0),
        ];
        // avg = (60*2 + 80*2) / (2+2) = 280/4 = 70.0
        let avg = weighted_average_score(&components).expect("non-empty list must return Some");
        assert!(
            (avg - 70.0).abs() < 1e-9,
            "weighted average should be 70.0, got {:.4}",
            avg
        );

        // Empty slice must return None
        assert!(
            weighted_average_score(&[]).is_none(),
            "empty slice must return None"
        );
    }

    // ── Test 9: `components_below_threshold` edge cases ──────────────────────

    #[test]
    fn test_components_below_threshold_sorted_and_edge_cases() {
        let components = vec![
            make_component("H1", ComponentCategory::Communication, 90.0, 1.0),
            make_component("W1", ComponentCategory::Communication, 70.0, 1.0),
            make_component("C1", ComponentCategory::Communication, 50.0, 1.0),
            make_component("C2", ComponentCategory::Communication, 30.0, 1.0),
        ];

        let below_60 = components_below_threshold(&components, 60.0);
        // Scores 50 and 30 are ≤ 60; sorted worst-first → ["C2", "C1"]
        assert_eq!(below_60, vec!["C2", "C1"], "must return worst-first");

        // Threshold 100 catches all components
        let all_below = components_below_threshold(&components, 100.0);
        assert_eq!(
            all_below.len(),
            4,
            "threshold 100 should include all components"
        );

        // Threshold 0 catches no components (no score == 0 in this set)
        let none_below = components_below_threshold(&components, -1.0);
        assert!(
            none_below.is_empty(),
            "threshold -1 should match no components"
        );
    }

    // ── Test 10: `critical_fraction` computation ──────────────────────────────

    #[test]
    fn test_critical_fraction_and_status_counts() {
        // 1 Healthy (score=90), 1 Warning (score=65), 2 Critical (score=50, score=20)
        let components = vec![
            make_component("H", ComponentCategory::Transmission, 90.0, 1.0),
            make_component("W", ComponentCategory::Transmission, 65.0, 1.0),
            make_component("C1", ComponentCategory::Transmission, 50.0, 1.0),
            make_component("C2", ComponentCategory::Transmission, 20.0, 1.0),
        ];
        let report = GridHealthScorer::new()
            .compute(&components)
            .expect("valid inputs");

        assert_eq!(report.total_components, 4);
        assert_eq!(report.healthy_count, 1);
        assert_eq!(report.warning_count, 1);
        assert_eq!(report.critical_count, 2);
        assert!(
            (report.critical_fraction() - 0.5).abs() < 1e-9,
            "2 out of 4 components critical → fraction 0.5"
        );
    }

    // ── Test 11: Score out of range returns error ─────────────────────────────

    #[test]
    fn test_score_out_of_range_returns_error() {
        let bad = vec![make_component(
            "X",
            ComponentCategory::Generation,
            101.0,
            1.0,
        )];
        let result = GridHealthScorer::new().compute(&bad);
        assert!(
            matches!(result, Err(GridHealthError::ScoreOutOfRange { .. })),
            "score > 100 must yield ScoreOutOfRange error"
        );

        let neg = vec![make_component(
            "Y",
            ComponentCategory::Generation,
            -0.1,
            1.0,
        )];
        let result2 = GridHealthScorer::new().compute(&neg);
        assert!(
            matches!(result2, Err(GridHealthError::ScoreOutOfRange { .. })),
            "negative score must yield ScoreOutOfRange error"
        );
    }

    // ── Test 12: is_critical helper and HealthStatus ordering ─────────────────

    #[test]
    fn test_health_status_is_critical_and_ordering() {
        assert!(HealthStatus::Critical.is_critical());
        assert!(!HealthStatus::Warning.is_critical());
        assert!(!HealthStatus::Healthy.is_critical());

        // Ord: Critical < Warning < Healthy (ascending severity → higher is better)
        assert!(HealthStatus::Critical < HealthStatus::Warning);
        assert!(HealthStatus::Warning < HealthStatus::Healthy);
    }
}
