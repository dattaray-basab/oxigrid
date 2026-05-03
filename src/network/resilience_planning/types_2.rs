//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::functions::{ep_default_failure_prob, ep_lcg_f64, ep_lcg_next, extreme_event_key};
use super::types::{
    ComponentVulnerability, ExtremHardeningMeasure, ExtremeEventType, ExtremeHardeningOption,
    ExtremeResilienceMetrics, ExtremeResiliencePlan, RepairDistribution, RestorationPlan,
};

/// Main planner for grid resilience hardening against extreme weather events.
///
/// Orchestrates vulnerability assessment, Monte Carlo resilience simulation,
/// hardening measure generation, and budget-constrained plan optimisation.
#[derive(Debug, Clone)]
pub struct ResiliencePlanner {
    /// Vulnerability characterisations for all grid components.
    pub components: Vec<ComponentVulnerability>,
    /// Total number of customers served.
    pub n_customers: u64,
    /// Peak system load (MW).
    pub peak_load_mw: f64,
    /// Value of lost load (USD/MWh). Default 10 000.
    pub voll_usd_per_mwh: f64,
    /// Planning horizon for NPV (years). Default 20.
    pub planning_horizon_years: f64,
    /// Discount rate for NPV. Default 0.05.
    pub discount_rate: f64,
    /// Probability of at least one extreme event per year.
    pub annual_event_probability: f64,
}
impl ResiliencePlanner {
    /// Create a new [`ResiliencePlanner`] with default financial parameters.
    pub fn new(
        components: Vec<ComponentVulnerability>,
        n_customers: u64,
        peak_load_mw: f64,
    ) -> Self {
        Self {
            components,
            n_customers,
            peak_load_mw,
            voll_usd_per_mwh: 10_000.0,
            planning_horizon_years: 20.0,
            discount_rate: 0.05,
            annual_event_probability: 0.10,
        }
    }
    /// Return `(component_id, failure_probability)` for each component.
    ///
    /// Looks up the component's own probability map first, then falls back to
    /// the built-in fragility table.
    pub fn assess_vulnerability(&self, event: &ExtremeEventType) -> Vec<(usize, f64)> {
        let key = extreme_event_key(event);
        self.components
            .iter()
            .map(|c| {
                let prob = c
                    .failure_probability
                    .get(&key)
                    .copied()
                    .unwrap_or_else(|| ep_default_failure_prob(&c.component_type, &key));
                (c.component_id, prob)
            })
            .collect()
    }
    /// Compute analytical baseline resilience metrics for an event.
    pub fn compute_baseline_metrics(&self, event: &ExtremeEventType) -> ExtremeResilienceMetrics {
        let vulns = self.assess_vulnerability(event);
        if self.components.is_empty() {
            return ExtremeResilienceMetrics {
                expected_loss_mwh: 0.0,
                expected_repair_time_h: 0.0,
                resilience_trapezoid_area: 24.0,
                rapidity: 1.0,
                robustness: 1.0,
                redundancy: 1.0,
                resourcefulness: 1.0,
            };
        }
        let total_crit: f64 = self.components.iter().map(|c| c.criticality_score).sum();
        let total_crit = total_crit.max(1e-9);
        let loss_fraction: f64 = vulns
            .iter()
            .zip(self.components.iter())
            .map(|((_, prob), comp)| prob * comp.criticality_score)
            .sum::<f64>()
            / total_crit;
        let expected_loss_mwh = loss_fraction * self.peak_load_mw * 24.0;
        let expected_repair_time_h: f64 = self
            .components
            .iter()
            .zip(vulns.iter())
            .map(|(c, (_, prob))| prob * c.criticality_score * c.repair_time_h)
            .sum::<f64>()
            / total_crit;
        let robustness = (1.0 - loss_fraction).max(0.0);
        let t_event = 24.0_f64;
        let t_restore = (expected_repair_time_h / 2.0).max(1.0);
        let resilience_trapezoid_area =
            0.5 * (1.0 + robustness) * t_event + 0.5 * (robustness + 1.0) * t_restore;
        let rapidity = if expected_repair_time_h > 1e-9 {
            (robustness / expected_repair_time_h).min(1.0)
        } else {
            1.0
        };
        let redundancy = if self.components.is_empty() {
            1.0
        } else {
            vulns.iter().filter(|(_, p)| *p < 0.5).count() as f64 / self.components.len() as f64
        };
        let resourcefulness = if self.components.is_empty() {
            1.0
        } else {
            self.components
                .iter()
                .filter(|c| c.repair_time_h <= 24.0)
                .count() as f64
                / self.components.len() as f64
        };
        ExtremeResilienceMetrics {
            expected_loss_mwh,
            expected_repair_time_h,
            resilience_trapezoid_area,
            rapidity,
            robustness,
            redundancy,
            resourcefulness,
        }
    }
    /// Estimate resilience metrics via Monte Carlo simulation.
    ///
    /// Uses a deterministic LCG (seed 42) for reproducibility.
    pub fn simulate_event(
        &self,
        event: &ExtremeEventType,
        n_trials: usize,
    ) -> ExtremeResilienceMetrics {
        if self.components.is_empty() || n_trials == 0 {
            return ExtremeResilienceMetrics {
                expected_loss_mwh: 0.0,
                expected_repair_time_h: 0.0,
                resilience_trapezoid_area: 24.0,
                rapidity: 1.0,
                robustness: 1.0,
                redundancy: 1.0,
                resourcefulness: 1.0,
            };
        }
        let vulns = self.assess_vulnerability(event);
        let total_crit: f64 = self.components.iter().map(|c| c.criticality_score).sum();
        let total_crit = total_crit.max(1e-9);
        let mut state: u64 = 42;
        let mut sum_loss = 0.0_f64;
        let mut min_perf = 1.0_f64;
        let mut redundancy_count = 0_usize;
        for _ in 0..n_trials {
            let mut failed_crit = 0.0_f64;
            for ((_, prob), comp) in vulns.iter().zip(self.components.iter()) {
                state = ep_lcg_next(state);
                if ep_lcg_f64(state) < *prob {
                    failed_crit += comp.criticality_score;
                }
            }
            let loss_frac = (failed_crit / total_crit).min(1.0);
            let perf = 1.0 - loss_frac;
            sum_loss += loss_frac;
            if perf < min_perf {
                min_perf = perf;
            }
            if perf >= 0.5 {
                redundancy_count += 1;
            }
        }
        let mean_loss = sum_loss / n_trials as f64;
        let expected_loss_mwh = mean_loss * self.peak_load_mw * 24.0;
        let robustness = min_perf.max(0.0);
        let expected_repair_time_h: f64 = self
            .components
            .iter()
            .zip(vulns.iter())
            .map(|(c, (_, prob))| prob * c.criticality_score * c.repair_time_h)
            .sum::<f64>()
            / total_crit;
        let t_event = 24.0_f64;
        let t_restore = (expected_repair_time_h / 2.0).max(1.0);
        let resilience_trapezoid_area =
            0.5 * (1.0 + robustness) * t_event + 0.5 * (robustness + 1.0) * t_restore;
        let rapidity = if expected_repair_time_h > 1e-9 {
            (robustness / expected_repair_time_h).min(1.0)
        } else {
            1.0
        };
        let redundancy = redundancy_count as f64 / n_trials as f64;
        let resourcefulness = if self.components.is_empty() {
            1.0
        } else {
            self.components
                .iter()
                .filter(|c| c.repair_time_h <= 24.0)
                .count() as f64
                / self.components.len() as f64
        };
        ExtremeResilienceMetrics {
            expected_loss_mwh,
            expected_repair_time_h,
            resilience_trapezoid_area,
            rapidity,
            robustness,
            redundancy,
            resourcefulness,
        }
    }
    /// Compute the NPV of damage reduction for a hardening measure and event.
    pub fn compute_measure_benefit(
        &self,
        measure: &ExtremHardeningMeasure,
        event: &ExtremeEventType,
    ) -> f64 {
        let key = extreme_event_key(event);
        let damage: f64 = measure
            .target_component_ids
            .iter()
            .filter_map(|&id| self.components.iter().find(|c| c.component_id == id))
            .map(|comp| {
                let base_prob = comp
                    .failure_probability
                    .get(&key)
                    .copied()
                    .unwrap_or_else(|| ep_default_failure_prob(&comp.component_type, &key));
                base_prob
                    * comp.criticality_score
                    * self.peak_load_mw
                    * 24.0
                    * self.voll_usd_per_mwh
            })
            .sum();
        let annual_savings =
            self.annual_event_probability * measure.failure_prob_reduction * damage;
        let r = self.discount_rate;
        let t = self.planning_horizon_years;
        if r.abs() < 1e-12 {
            annual_savings * t
        } else {
            annual_savings * (1.0 - (1.0 + r).powf(-t)) / r
        }
    }
    /// Total life-cycle cost of a measure (capital + NPV of OPEX).
    pub(crate) fn ep_measure_cost(&self, measure: &ExtremHardeningMeasure) -> f64 {
        let r = self.discount_rate;
        let t = self.planning_horizon_years;
        let npv_opex = if r.abs() < 1e-12 {
            measure.annual_opex_usd * t
        } else {
            measure.annual_opex_usd * (1.0 - (1.0 + r).powf(-t)) / r
        };
        measure.capital_cost_usd + npv_opex
    }
    /// Compute BCR for a measure averaged across representative events.
    fn ep_measure_bcr(&self, measure: &ExtremHardeningMeasure) -> f64 {
        let events = [
            ExtremeEventType::Hurricane { category: 3 },
            ExtremeEventType::IceStorm {
                ice_thickness_mm: 25.0,
            },
            ExtremeEventType::Wildfire {
                severity: FireSeverity::High,
            },
            ExtremeEventType::Earthquake { magnitude: 6.5 },
            ExtremeEventType::Flood { depth_m: 2.0 },
        ];
        let total_benefit: f64 = events
            .iter()
            .map(|e| self.compute_measure_benefit(measure, e))
            .sum();
        let avg_benefit = total_benefit / events.len() as f64;
        let total_cost = self.ep_measure_cost(measure);
        if total_cost < 1e-9 {
            0.0
        } else {
            avg_benefit / total_cost
        }
    }
    /// Rank hardening measures by benefit-cost ratio (descending).
    ///
    /// Returns `(measure_index, bcr)` pairs.
    pub fn rank_hardening_options(
        &self,
        available: &[ExtremHardeningMeasure],
    ) -> Vec<(usize, f64)> {
        let mut ranked: Vec<(usize, f64)> = available
            .iter()
            .enumerate()
            .map(|(i, m)| (i, self.ep_measure_bcr(m)))
            .collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        ranked
    }
    /// Select measures within `budget_usd` using a greedy knapsack by BCR.
    pub fn optimize_hardening_budget(
        &self,
        measures: &[ExtremHardeningMeasure],
        budget_usd: f64,
    ) -> ExtremeResiliencePlan {
        let ranked = self.rank_hardening_options(measures);
        let mut selected: Vec<ExtremHardeningMeasure> = Vec::new();
        let mut remaining = budget_usd;
        let mut total_cost = 0.0_f64;
        let mut total_benefit = 0.0_f64;
        for (idx, _bcr) in &ranked {
            let m = &measures[*idx];
            let cost = self.ep_measure_cost(m);
            if cost <= remaining {
                remaining -= cost;
                total_cost += cost;
                let benefit: f64 = [
                    ExtremeEventType::Hurricane { category: 3 },
                    ExtremeEventType::IceStorm {
                        ice_thickness_mm: 25.0,
                    },
                    ExtremeEventType::Wildfire {
                        severity: FireSeverity::High,
                    },
                ]
                .iter()
                .map(|e| self.compute_measure_benefit(m, e))
                .sum::<f64>()
                    / 3.0;
                total_benefit += benefit;
                selected.push(m.clone());
            }
        }
        let bcr = if total_cost < 1e-9 {
            0.0
        } else {
            total_benefit / total_cost
        };
        let rep_event = ExtremeEventType::Hurricane { category: 3 };
        let improvement = self.compute_baseline_metrics(&rep_event);
        ExtremeResiliencePlan {
            measures: selected,
            total_cost_usd: total_cost,
            expected_benefit_usd: total_benefit,
            benefit_cost_ratio: bcr,
            resilience_improvement: improvement,
        }
    }
    /// Automatically generate candidate hardening measures for all components.
    pub fn generate_hardening_measures(&self) -> Vec<ExtremHardeningMeasure> {
        let mut measures = Vec::new();
        for comp in &self.components {
            let ids = vec![comp.component_id];
            match comp.component_type.as_str() {
                "overhead_line" => {
                    let cap = 50_000.0_f64;
                    measures.push(ExtremHardeningMeasure {
                        option: ExtremeHardeningOption::StormHardenedPole,
                        target_component_ids: ids.clone(),
                        capital_cost_usd: cap,
                        annual_opex_usd: cap * 0.02,
                        failure_prob_reduction: 0.5,
                        repair_time_reduction_h: 4.0,
                        implementation_years: 2,
                    });
                    let cap2 = 200_000.0_f64;
                    measures.push(ExtremHardeningMeasure {
                        option: ExtremeHardeningOption::UndergroundCable,
                        target_component_ids: ids.clone(),
                        capital_cost_usd: cap2,
                        annual_opex_usd: cap2 * 0.02,
                        failure_prob_reduction: 0.8,
                        repair_time_reduction_h: 8.0,
                        implementation_years: 2,
                    });
                }
                "substation" | "transformer" => {
                    let cap = 100_000.0_f64;
                    measures.push(ExtremHardeningMeasure {
                        option: ExtremeHardeningOption::SubstationFloodBarrier,
                        target_component_ids: ids.clone(),
                        capital_cost_usd: cap,
                        annual_opex_usd: cap * 0.02,
                        failure_prob_reduction: 0.4,
                        repair_time_reduction_h: 0.0,
                        implementation_years: 2,
                    });
                    let cap2 = 300_000.0_f64;
                    measures.push(ExtremHardeningMeasure {
                        option: ExtremeHardeningOption::MobileSubstation,
                        target_component_ids: ids.clone(),
                        capital_cost_usd: cap2,
                        annual_opex_usd: cap2 * 0.02,
                        failure_prob_reduction: 0.6,
                        repair_time_reduction_h: 12.0,
                        implementation_years: 2,
                    });
                }
                _ => {
                    let cap = 500_000.0_f64;
                    measures.push(ExtremHardeningMeasure {
                        option: ExtremeHardeningOption::MicroGrid,
                        target_component_ids: ids.clone(),
                        capital_cost_usd: cap,
                        annual_opex_usd: cap * 0.02,
                        failure_prob_reduction: 0.5,
                        repair_time_reduction_h: 10.0,
                        implementation_years: 3,
                    });
                }
            }
            let cap_sw = 20_000.0_f64;
            measures.push(ExtremHardeningMeasure {
                option: ExtremeHardeningOption::SmartSwitching,
                target_component_ids: ids,
                capital_cost_usd: cap_sw,
                annual_opex_usd: cap_sw * 0.02,
                failure_prob_reduction: 0.2,
                repair_time_reduction_h: 2.0,
                implementation_years: 2,
            });
        }
        measures
    }
    /// Compute a composite resilience index in [0, 1].
    ///
    /// Weights: robustness 25 %, normalised loss 25 %, redundancy 25 %,
    /// resourcefulness 25 %.
    pub fn compute_resilience_index(&self, metrics: &ExtremeResilienceMetrics) -> f64 {
        let max_loss = self.peak_load_mw * 24.0;
        let norm_loss = if max_loss < 1e-9 {
            1.0
        } else {
            1.0 - (metrics.expected_loss_mwh / max_loss).min(1.0)
        };
        (0.25 * metrics.robustness
            + 0.25 * norm_loss
            + 0.25 * metrics.redundancy
            + 0.25 * metrics.resourcefulness)
            .clamp(0.0, 1.0)
    }
}
/// Type of climate hazard that can impact grid infrastructure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HazardType {
    /// Tropical cyclone or hurricane.
    Hurricane,
    /// Freezing-rain ice accumulation event.
    IceStorm,
    /// Wildland–urban interface fire.
    Wildfire,
    /// Riverine or coastal inundation.
    Flood,
    /// Prolonged high-temperature event causing conductor overload.
    ExtremeHeat,
    /// Ground motion from seismic activity.
    Earthquake,
}
/// Qualitative severity classification for a hazard event.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HazardSeverity {
    /// Minor impact, rapidly self-correcting.
    Low,
    /// Moderate impact, standard repair timescales.
    Medium,
    /// Significant damage requiring extended restoration.
    High,
    /// Catastrophic damage with long-term consequences.
    Extreme,
}
/// A single phase in the restoration sequence.
#[derive(Debug, Clone)]
pub struct RestorationPhase {
    /// Descriptive name of the phase.
    pub phase_name: String,
    /// Phase duration \[h\].
    pub duration_h: f64,
    /// Cumulative load restored at end of this phase \[MW\].
    pub load_restored_mw: f64,
    /// Crews allocated during this phase.
    pub crew_count: usize,
}
/// Restoration time estimator for a storm-damage scenario.
#[derive(Debug, Clone)]
pub struct RestorationEstimator {
    /// Number of repair crews available.
    pub crew_count: usize,
    /// Time for crews to travel to the damaged area \[h\].
    pub crew_travel_time_h: f64,
    /// Stochastic repair duration model.
    pub repair_time_distribution: RepairDistribution,
    /// Time required to assess the extent of damage \[h\].
    pub damage_assessment_time_h: f64,
}
impl RestorationEstimator {
    /// Mean of the triangular repair distribution \[h\].
    ///
    /// `E[T] = (min + mode + max) / 3`
    pub fn expected_repair_time_h(&self) -> f64 {
        let d = &self.repair_time_distribution;
        (d.min_h + d.mode_h + d.max_h) / 3.0
    }
    /// Analytical percentile of the triangular distribution plus travel and
    /// assessment overhead \[h\].
    ///
    /// `alpha` must be in \[0, 1\].
    ///
    /// Uses the standard triangular CDF inverse:
    /// - if `α ≤ (c−a)/(b−a)`: `t = a + √(α·(b−a)·(c−a))`
    /// - otherwise:             `t = b − √((1−α)·(b−a)·(b−c))`
    pub fn percentile_repair_time_h(&self, alpha: f64) -> f64 {
        let d = &self.repair_time_distribution;
        let a = d.min_h;
        let b = d.max_h;
        let c = d.mode_h;
        let alpha = alpha.clamp(0.0, 1.0);
        let range = (b - a).max(f64::EPSILON);
        let fc = (c - a) / range;
        let repair_t = if alpha <= fc {
            a + (alpha * range * (c - a).max(0.0)).sqrt()
        } else {
            b - ((1.0 - alpha) * range * (b - c).max(0.0)).sqrt()
        };
        self.crew_travel_time_h + self.damage_assessment_time_h + repair_t
    }
    /// Build a phased restoration plan for `damaged_elements` faults with
    /// specified priority and total load.
    ///
    /// # Arguments
    /// * `damaged_elements` – number of faulted components
    /// * `priority_loads_mw` – MW of highest-priority loads (hospitals,
    ///   emergency services) \[MW\]
    /// * `total_load_mw` – total load to be restored \[MW\]
    pub fn restoration_sequence(
        &self,
        damaged_elements: usize,
        priority_loads_mw: f64,
        total_load_mw: f64,
    ) -> RestorationPlan {
        let mean_repair = self.expected_repair_time_h();
        let safe_crews = self.crew_count.max(1) as f64;
        let bulk_elements = damaged_elements.saturating_sub(1) as f64;
        let phase1 = RestorationPhase {
            phase_name: "Damage Assessment".to_string(),
            duration_h: self.damage_assessment_time_h,
            load_restored_mw: 0.0,
            crew_count: 0,
        };
        let phase2 = RestorationPhase {
            phase_name: "Crew Dispatch".to_string(),
            duration_h: self.crew_travel_time_h,
            load_restored_mw: 0.0,
            crew_count: self.crew_count,
        };
        let phase3 = RestorationPhase {
            phase_name: "Priority Load Restoration".to_string(),
            duration_h: mean_repair,
            load_restored_mw: priority_loads_mw,
            crew_count: self.crew_count,
        };
        let phase4_duration = if bulk_elements > 0.0 {
            mean_repair * bulk_elements / safe_crews
        } else {
            0.0
        };
        let phase4 = RestorationPhase {
            phase_name: "Full Restoration".to_string(),
            duration_h: phase4_duration,
            load_restored_mw: (total_load_mw - priority_loads_mw).max(0.0),
            crew_count: self.crew_count,
        };
        let total_time_h =
            phase1.duration_h + phase2.duration_h + phase3.duration_h + phase4.duration_h;
        let saidi_contribution_h =
            total_time_h * priority_loads_mw / total_load_mw.max(f64::EPSILON);
        RestorationPlan {
            phases: vec![phase1, phase2, phase3, phase4],
            total_time_h,
            saidi_contribution_h,
        }
    }
}
/// Wildfire severity classification.
#[derive(Debug, Clone, PartialEq)]
pub enum FireSeverity {
    /// Low severity wildfire.
    Low,
    /// Moderate severity wildfire.
    Moderate,
    /// High severity wildfire.
    High,
    /// Extreme (catastrophic) severity wildfire.
    Extreme,
}
