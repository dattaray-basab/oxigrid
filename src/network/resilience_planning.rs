//! Grid Resilience Planning and Hardening.
//!
//! Implements IEEE Std 2030 resilience trapezoid, N-k contingency analysis,
//! hardening investment optimisation, microgrid resilience modelling,
//! restoration time estimation, and climate risk assessment.
//!
//! # Units
//! - Time: \[h\]
//! - Power: \[MW\]
//! - Energy: \[MWh\]
//! - Cost: \[\$\]
//! - Failure rate: \[1/year\]
//! - Performance: \[0-1\] (p.u.)

// ─────────────────────────────────────────────────────────────────────────────
// Section 1: Resilience Trapezoid (IEEE Std 2030)
// ─────────────────────────────────────────────────────────────────────────────

/// IEEE Std 2030 resilience trapezoid characterising a grid event in four
/// time-stamped phases: degradation, disturbed steady-state, recovery, and
/// post-recovery.
///
/// All times are in \[h\] and performance values are dimensionless \[0-1\].
#[derive(Debug, Clone)]
pub struct ResiliencePlanningTrapezoid {
    /// Event onset time \[h\].
    pub t0: f64,
    /// Time at which the disturbed (nadir) state is first reached \[h\].
    pub t1: f64,
    /// Time at which the restorative phase begins \[h\].
    pub t2: f64,
    /// Time of full (post-event) recovery \[h\].
    pub t3: f64,
    /// Pre-event system performance \[0-1\].
    pub phi0: f64,
    /// Nadir (minimum) performance during the disturbed state \[0-1\].
    pub phi1: f64,
    /// Post-event (final) system performance \[0-1\].
    pub phi3: f64,
}

impl ResiliencePlanningTrapezoid {
    /// Construct a new resilience trapezoid.
    ///
    /// # Arguments
    /// * `t0` – event onset \[h\]
    /// * `t1` – nadir time \[h\]
    /// * `t2` – recovery start \[h\]
    /// * `t3` – full recovery time \[h\]
    /// * `phi0` – pre-event performance \[0-1\]
    /// * `phi1` – nadir performance \[0-1\]
    /// * `phi3` – post-event performance \[0-1\]
    pub fn new(t0: f64, t1: f64, t2: f64, t3: f64, phi0: f64, phi1: f64, phi3: f64) -> Self {
        Self {
            t0,
            t1,
            t2,
            t3,
            phi0,
            phi1,
            phi3,
        }
    }

    /// Area of lost performance (energy not served) under the resilience
    /// trapezoid \[p.u.·h\].
    ///
    /// Computed as the integral of `(phi0 - phi(t))` over `[t0, t3]` using
    /// three piecewise-linear segments:
    ///
    /// 1. **Degradation** `[t0, t1]` — linear drop from `phi0` to `phi1`
    /// 2. **Disturbed** `[t1, t2]` — flat at `phi1`
    /// 3. **Recovery** `[t2, t3]` — linear rise from `phi1` to `phi3`
    pub fn resilience_triangle_area(&self) -> f64 {
        let drop_phase = 0.5 * (self.phi0 - self.phi1) * (self.t1 - self.t0).max(0.0);
        let flat_phase = (self.phi0 - self.phi1) * (self.t2 - self.t1).max(0.0);
        let recovery_phase = 0.5
            * ((self.phi0 - self.phi1) + (self.phi0 - self.phi3))
            * (self.t3 - self.t2).max(0.0);
        drop_phase + flat_phase + recovery_phase
    }

    /// Rate of performance recovery during the restoration phase \[1/h\].
    ///
    /// Returns 0 if `t3 <= t2`.
    pub fn recovery_rate(&self) -> f64 {
        let dt = self.t3 - self.t2;
        if dt > 0.0 {
            (self.phi3 - self.phi1) / dt
        } else {
            0.0
        }
    }

    /// Absorptive capacity — ratio of nadir to pre-event performance \[0-1\].
    ///
    /// A value of 1 means no degradation occurred; a value near 0 means total
    /// collapse.
    pub fn absorptive_capacity(&self) -> f64 {
        self.phi1 / self.phi0.max(f64::EPSILON)
    }

    /// Duration of the restorative phase \[h\].
    pub fn restorative_capacity(&self) -> f64 {
        (self.t3 - self.t2).max(0.0)
    }

    /// Composite resilience index \[0-1\].
    ///
    /// Weighted combination of:
    /// - 40 % absorptive capacity (`phi1/phi0`)
    /// - 30 % post-recovery ratio (`phi3/phi0`)
    /// - 30 % normalised preserved-energy fraction
    pub fn resilience_index(&self) -> f64 {
        let absorptive = self.absorptive_capacity();
        let recovery_ratio = self.phi3 / self.phi0.max(f64::EPSILON);
        let total_window = (self.t3 - self.t0).max(f64::EPSILON);
        let max_loss = self.phi0 * total_window;
        let loss_fraction = self.resilience_triangle_area() / max_loss.max(f64::EPSILON);
        let preserved = (1.0 - loss_fraction).clamp(0.0, 1.0);
        let index = 0.4 * absorptive + 0.3 * recovery_ratio + 0.3 * preserved;
        index.clamp(0.0, 1.0)
    }

    /// Overall rapidity of recovery from event onset to full restoration \[1/h\].
    ///
    /// Represents the average performance improvement rate over the whole event
    /// window.
    pub fn rapidity(&self) -> f64 {
        let dt = self.t3 - self.t0;
        if dt > 0.0 {
            (self.phi3 - self.phi1) / dt
        } else {
            0.0
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 2: N-k Contingency Analysis
// ─────────────────────────────────────────────────────────────────────────────

/// N-k contingency analysis controller.
///
/// Evaluates system resilience under simultaneous loss of up to `k` elements
/// from a network of `N` total elements.
#[derive(Debug, Clone)]
pub struct NkContingencyAnalysis {
    /// Total number of network elements (N).
    pub network_elements: usize,
    /// Contingency level k (1, 2, or 3).
    pub k_level: usize,
    /// Performance threshold below which a contingency is classified as
    /// critical \[0-1\].
    pub critical_threshold: f64,
}

/// Result of evaluating a single N-k contingency scenario.
#[derive(Debug, Clone)]
pub struct ContingencyOutcomeResult {
    /// Identifiers of the elements removed in this contingency.
    pub element_ids: Vec<usize>,
    /// System performance after the contingency \[0-1\].
    pub performance: f64,
    /// Estimated load lost \[MW\].
    pub load_lost_mw: f64,
    /// True when `performance < critical_threshold`.
    pub is_critical: bool,
    /// Estimated time to restore full service \[h\].
    pub recovery_time_h: f64,
}

impl NkContingencyAnalysis {
    /// Create an analyser with default `critical_threshold = 0.8`.
    pub fn new(n: usize, k: usize) -> Self {
        Self {
            network_elements: n,
            k_level: k,
            critical_threshold: 0.8,
        }
    }

    /// Number of distinct N-k combinations C(N, k).
    ///
    /// Uses saturating arithmetic to avoid overflow for large N.
    pub fn combination_count(&self) -> u64 {
        combinations(self.network_elements, self.k_level)
    }

    /// Evaluate performance for one contingency scenario.
    ///
    /// The load-lost estimate uses a simplified linear model:
    /// `load_lost = |elements| × 10 × (1 − performance)` \[MW\].
    pub fn analyze_contingency(
        &self,
        element_ids: &[usize],
        network_performance: f64,
    ) -> ContingencyOutcomeResult {
        let load_lost_mw = element_ids.len() as f64 * 10.0 * (1.0 - network_performance);
        let is_critical = network_performance < self.critical_threshold;
        let recovery_time_h = load_lost_mw * 0.5;
        ContingencyOutcomeResult {
            element_ids: element_ids.to_vec(),
            performance: network_performance,
            load_lost_mw,
            is_critical,
            recovery_time_h,
        }
    }

    /// Return up to `top_n` element IDs ranked by descending load impact.
    ///
    /// Contingency results are sorted by `load_lost_mw` (highest first);
    /// element IDs are collected in order without repetition.
    pub fn most_critical_elements(
        &self,
        results: &[ContingencyOutcomeResult],
        top_n: usize,
    ) -> Vec<usize> {
        let mut sorted: Vec<&ContingencyOutcomeResult> = results.iter().collect();
        sorted.sort_by(|a, b| {
            b.load_lost_mw
                .partial_cmp(&a.load_lost_mw)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut seen = std::collections::HashSet::new();
        let mut out = Vec::new();
        for res in sorted {
            for &id in &res.element_ids {
                if seen.insert(id) {
                    out.push(id);
                    if out.len() >= top_n {
                        return out;
                    }
                }
            }
        }
        out
    }

    /// Fraction of contingency scenarios that are non-critical \[0-1\].
    pub fn system_adequacy(&self, results: &[ContingencyOutcomeResult]) -> f64 {
        if results.is_empty() {
            return 1.0;
        }
        let non_critical = results.iter().filter(|r| !r.is_critical).count();
        non_critical as f64 / results.len() as f64
    }
}

/// Compute the binomial coefficient C(n, k) with saturating arithmetic.
fn combinations(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result.saturating_mul((n - i) as u64);
        result /= (i + 1) as u64;
    }
    result
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 3: Hardening Investment Optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Value of lost load used in benefit calculations \[$/MWh\].
const VOLL_DOLLARS_PER_MWH: f64 = 10_000.0;

/// Classification of a grid element subject to hardening.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElementType {
    /// High-voltage transmission line or cable.
    Transmission,
    /// Medium/low-voltage distribution feeder.
    Distribution,
    /// HV/MV substation.
    Substation,
    /// Generating unit.
    Generator,
}

/// A single hardening measure that can be applied to a `GridElement`.
#[derive(Debug, Clone)]
pub struct HardeningOption {
    /// Descriptive name of the hardening measure.
    pub name: String,
    /// Upfront capital cost \[\$\].
    pub cost: f64,
    /// Fractional reduction in failure rate \[0-1\].
    pub failure_rate_reduction: f64,
    /// Fractional reduction in mean time to repair (MTTR) \[0-1\].
    pub mttr_reduction: f64,
    /// Residual (salvage) value as fraction of `cost` at end of planning
    /// horizon \[0-1\].
    pub residual_value_pct: f64,
}

/// A grid element that may be hardened.
#[derive(Debug, Clone)]
pub struct GridElement {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Classification of this element.
    pub element_type: ElementType,
    /// Failure rate \[1/year\].
    pub failure_rate_per_year: f64,
    /// Mean time to repair \[h\].
    pub mttr_h: f64,
    /// Available hardening options for this element.
    pub hardening_options: Vec<HardeningOption>,
    /// Average load served by this element \[MW\].
    pub load_served_mw: f64,
}

/// Output of the hardening investment optimiser.
#[derive(Debug, Clone)]
pub struct GridHardeningPlan {
    /// Selected `(element_id, option_index)` pairs.
    pub selected_options: Vec<(usize, usize)>,
    /// Total capital expenditure \[\$\].
    pub total_cost: f64,
    /// Expected reduction in energy-not-served \[MWh/year\].
    pub expected_ens_reduction_mwh_per_year: f64,
    /// Net present value of all benefits over the planning horizon \[\$\].
    pub npv_benefit: f64,
    /// Portfolio benefit-cost ratio (dimensionless).
    pub benefit_cost_ratio: f64,
}

/// Greedy hardening investment optimiser operating within a fixed budget.
#[derive(Debug, Clone)]
pub struct HardeningOptimizer {
    /// Elements available for hardening.
    pub elements: Vec<GridElement>,
    /// Total investment budget \[\$\].
    pub budget: f64,
    /// Planning horizon \[years\].
    pub planning_horizon_years: f64,
    /// Annual discount rate (e.g. 0.05 for 5 %).
    pub discount_rate: f64,
}

impl HardeningOptimizer {
    /// Expected energy not served for an un-hardened element \[MWh/year\].
    ///
    /// `ENS = λ × MTTR × load_MW`
    ///
    /// where `λ` is in \[1/year\] and `MTTR` is in \[h\].
    pub fn expected_energy_not_served_mwh_per_year(&self, element: &GridElement) -> f64 {
        element.failure_rate_per_year * element.mttr_h * element.load_served_mw
    }

    /// Benefit-cost ratio for applying `option` to `element`.
    ///
    /// `BCR = NPV_benefit / net_cost`
    ///
    /// where the net cost deducts the present-value of the residual (salvage)
    /// value at end of planning horizon.
    pub fn benefit_cost_ratio(&self, element: &GridElement, option: &HardeningOption) -> f64 {
        let old_ens = element.failure_rate_per_year * element.mttr_h * element.load_served_mw;
        let new_lambda = element.failure_rate_per_year * (1.0 - option.failure_rate_reduction);
        let new_mttr = element.mttr_h * (1.0 - option.mttr_reduction);
        let new_ens = new_lambda * new_mttr * element.load_served_mw;
        let ens_reduction = (old_ens - new_ens).max(0.0);
        let annual_benefit = ens_reduction * VOLL_DOLLARS_PER_MWH;
        let npv = self.npv_annuity(annual_benefit);
        let residual_pv = option.cost * option.residual_value_pct
            / (1.0 + self.discount_rate).powf(self.planning_horizon_years);
        let net_cost = (option.cost - residual_pv).max(f64::EPSILON);
        npv / net_cost
    }

    /// Greedy optimiser: select hardening options ranked by BCR until the
    /// budget is exhausted.
    pub fn optimize_greedy(&self) -> GridHardeningPlan {
        // Build flat list of (element_idx, option_idx, bcr, cost, ens_reduction)
        struct Candidate {
            element_id: usize,
            option_idx: usize,
            bcr: f64,
            cost: f64,
            ens_reduction: f64,
            npv: f64,
        }

        let mut candidates: Vec<Candidate> = Vec::new();
        for element in &self.elements {
            let old_ens = element.failure_rate_per_year * element.mttr_h * element.load_served_mw;
            for (opt_idx, option) in element.hardening_options.iter().enumerate() {
                let new_lambda =
                    element.failure_rate_per_year * (1.0 - option.failure_rate_reduction);
                let new_mttr = element.mttr_h * (1.0 - option.mttr_reduction);
                let new_ens = new_lambda * new_mttr * element.load_served_mw;
                let ens_reduction = (old_ens - new_ens).max(0.0);
                let annual_benefit = ens_reduction * VOLL_DOLLARS_PER_MWH;
                let npv = self.npv_annuity(annual_benefit);
                let residual_pv = option.cost * option.residual_value_pct
                    / (1.0 + self.discount_rate).powf(self.planning_horizon_years);
                let net_cost = (option.cost - residual_pv).max(f64::EPSILON);
                let bcr = npv / net_cost;
                candidates.push(Candidate {
                    element_id: element.id,
                    option_idx: opt_idx,
                    bcr,
                    cost: option.cost,
                    ens_reduction,
                    npv,
                });
            }
        }

        // Sort by BCR descending
        candidates.sort_by(|a, b| {
            b.bcr
                .partial_cmp(&a.bcr)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut selected: Vec<(usize, usize)> = Vec::new();
        let mut total_cost = 0.0_f64;
        let mut total_ens_reduction = 0.0_f64;
        let mut total_npv = 0.0_f64;
        // Track which elements have already had an option selected (one per element)
        let mut used_elements = std::collections::HashSet::new();

        for c in &candidates {
            if used_elements.contains(&c.element_id) {
                continue;
            }
            if total_cost + c.cost <= self.budget {
                selected.push((c.element_id, c.option_idx));
                total_cost += c.cost;
                total_ens_reduction += c.ens_reduction;
                total_npv += c.npv;
                used_elements.insert(c.element_id);
            }
        }

        let portfolio_bcr = if total_cost > f64::EPSILON {
            total_npv / total_cost
        } else {
            0.0
        };

        GridHardeningPlan {
            selected_options: selected,
            total_cost,
            expected_ens_reduction_mwh_per_year: total_ens_reduction,
            npv_benefit: total_npv,
            benefit_cost_ratio: portfolio_bcr,
        }
    }

    /// Present value of a level annuity of `annual_cash_flow` \[\$/year\] over
    /// `planning_horizon_years` at `discount_rate`.
    fn npv_annuity(&self, annual_cash_flow: f64) -> f64 {
        let r = self.discount_rate;
        let n = self.planning_horizon_years;
        if r.abs() < f64::EPSILON {
            return annual_cash_flow * n;
        }
        annual_cash_flow * (1.0 - (1.0 + r).powf(-n)) / r
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 4: Resilience Microgrids
// ─────────────────────────────────────────────────────────────────────────────

/// A grid-connected microgrid dimensioned to maintain critical loads during
/// extended grid outages.
#[derive(Debug, Clone)]
pub struct ResilienceMicrogrid {
    /// Unique identifier.
    pub id: usize,
    /// Geographic or administrative location label.
    pub location: String,
    /// Peak critical load to be served during islanded operation \[MW\].
    pub critical_load_mw: f64,
    /// Installed PV capacity \[MW\].
    pub pv_mw: f64,
    /// Usable battery energy capacity \[MWh\].
    pub battery_mwh: f64,
    /// Battery maximum power rating \[MW\].
    pub battery_mw: f64,
    /// Diesel (or gas) generator capacity \[MW\].
    pub diesel_mw: f64,
    /// Target islanded autonomy \[days\].
    pub autonomy_days: f64,
}

/// Result of a microgrid sizing check.
#[derive(Debug, Clone, PartialEq)]
pub enum SizingStatus {
    /// All sizing criteria are met.
    Adequate,
    /// Battery energy capacity is insufficient.
    BatteryUndersized {
        /// Energy shortfall \[MWh\].
        deficit_mwh: f64,
    },
    /// Combined generator + battery power is insufficient.
    GeneratorUndersized {
        /// Power shortfall \[MW\].
        deficit_mw: f64,
    },
}

impl ResilienceMicrogrid {
    /// Returns `true` when combined dispatchable power meets the critical load.
    pub fn can_island(&self) -> bool {
        (self.battery_mw + self.diesel_mw) >= self.critical_load_mw
    }

    /// Estimated islanding duration under nominal conditions \[h\].
    ///
    /// `duration = (battery_mwh × 0.9 + diesel_mw × 24 × autonomy_days) / critical_load_mw`
    pub fn islanding_duration_h(&self) -> f64 {
        let stored_energy = self.battery_mwh * 0.9;
        let diesel_energy = self.diesel_mw * 24.0 * self.autonomy_days;
        (stored_energy + diesel_energy) / self.critical_load_mw.max(f64::EPSILON)
    }

    /// Average load served per hour during a grid outage of `grid_outage_h`
    /// \[MW\].
    ///
    /// Capped at `critical_load_mw`.
    pub fn load_served_during_outage(&self, grid_outage_h: f64) -> f64 {
        let safe_h = grid_outage_h.max(f64::EPSILON);
        let available_energy = self.battery_mwh * 0.9 + self.diesel_mw * safe_h;
        let avg_power = available_energy / safe_h;
        avg_power.min(self.critical_load_mw)
    }

    /// Check whether the microgrid is adequately sized for its stated autonomy
    /// target.
    ///
    /// Battery must supply at least 20 % of the critical load energy over the
    /// autonomy period; dispatchable power must meet peak critical load.
    pub fn sizing_check(&self) -> SizingStatus {
        let required_battery_mwh = self.critical_load_mw * self.autonomy_days * 24.0 * 0.2;
        if self.battery_mwh < required_battery_mwh {
            return SizingStatus::BatteryUndersized {
                deficit_mwh: required_battery_mwh - self.battery_mwh,
            };
        }
        let available_power = self.battery_mw + self.diesel_mw;
        if available_power < self.critical_load_mw {
            return SizingStatus::GeneratorUndersized {
                deficit_mw: self.critical_load_mw - available_power,
            };
        }
        SizingStatus::Adequate
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 5: Restoration Time Estimation
// ─────────────────────────────────────────────────────────────────────────────

/// Parameters of a triangular probability distribution for repair durations.
#[derive(Debug, Clone)]
pub struct RepairDistribution {
    /// Optimistic (minimum) repair time \[h\].
    pub min_h: f64,
    /// Most likely (mode) repair time \[h\].
    pub mode_h: f64,
    /// Pessimistic (maximum) repair time \[h\].
    pub max_h: f64,
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

/// Full restoration plan output.
#[derive(Debug, Clone)]
pub struct RestorationPlan {
    /// Ordered restoration phases.
    pub phases: Vec<RestorationPhase>,
    /// Total elapsed time from event to full restoration \[h\].
    pub total_time_h: f64,
    /// Estimated SAIDI contribution (average hours interrupted per customer)
    /// \[h\].
    pub saidi_contribution_h: f64,
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

// ─────────────────────────────────────────────────────────────────────────────
// Section 6: Climate Risk Assessment
// ─────────────────────────────────────────────────────────────────────────────

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

/// A single climate hazard with frequency and consequence parameters.
#[derive(Debug, Clone)]
pub struct ClimateHazard {
    /// Type of hazard.
    pub hazard_type: HazardType,
    /// Annual exceedance probability (0–1).
    pub annual_probability: f64,
    /// Qualitative severity category.
    pub severity: HazardSeverity,
    /// Fraction of network elements that fall within the hazard footprint
    /// \[0-1\].
    pub affected_fraction: f64,
    /// Fraction of affected elements expected to sustain damage requiring
    /// repair \[0-1\].
    pub damage_fraction: f64,
}

/// Multi-hazard climate risk assessor for a grid in a defined geographic
/// region.
#[derive(Debug, Clone)]
pub struct ClimateRiskAssessor {
    /// Region identifier or name.
    pub region: String,
    /// Set of applicable climate hazards.
    pub hazards: Vec<ClimateHazard>,
}

impl ClimateRiskAssessor {
    /// Expected annual damage as a percentage of the network asset base \[%\].
    ///
    /// `EAD% = Σ_h p_h × severity_factor_h × affected_h × damage_h × 100`
    pub fn annual_expected_damage_pct(&self) -> f64 {
        self.hazards
            .iter()
            .map(|h| {
                let sf = severity_factor(&h.severity);
                h.annual_probability * sf * h.affected_fraction * h.damage_fraction * 100.0
            })
            .sum()
    }

    /// The hazard that contributes most to annual expected damage.
    pub fn most_critical_hazard(&self) -> Option<&ClimateHazard> {
        self.hazards.iter().max_by(|a, b| {
            let score_a = a.annual_probability
                * severity_factor(&a.severity)
                * a.affected_fraction
                * a.damage_fraction;
            let score_b = b.annual_probability
                * severity_factor(&b.severity)
                * b.affected_fraction
                * b.damage_fraction;
            score_a
                .partial_cmp(&score_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Composite resilience score \[0–100\]; 100 represents zero climate risk.
    pub fn resilience_score(&self) -> f64 {
        (100.0 - self.annual_expected_damage_pct()).clamp(0.0, 100.0)
    }

    /// Suggested hardening actions based on the hazard portfolio.
    ///
    /// Returns one recommendation per distinct hazard type present.
    pub fn recommended_hardening(&self) -> Vec<String> {
        let mut seen = std::collections::HashSet::new();
        let mut recs = Vec::new();
        for h in &self.hazards {
            let key = hazard_type_key(&h.hazard_type);
            if seen.insert(key) {
                let rec = match &h.hazard_type {
                    HazardType::Hurricane => "Underground cabling in coastal areas",
                    HazardType::IceStorm => "Ice-resistant conductors and de-icing systems",
                    HazardType::Wildfire => "Fire-resistant poles and vegetation management",
                    HazardType::Flood => "Elevate substations above flood level",
                    HazardType::ExtremeHeat => "Uprated conductor ratings for high temperature",
                    HazardType::Earthquake => "Seismic bracing for substation equipment",
                };
                recs.push(rec.to_string());
            }
        }
        recs
    }
}

/// Map `HazardSeverity` to a dimensionless multiplicative factor.
fn severity_factor(severity: &HazardSeverity) -> f64 {
    match severity {
        HazardSeverity::Low => 0.1,
        HazardSeverity::Medium => 0.3,
        HazardSeverity::High => 0.6,
        HazardSeverity::Extreme => 1.0,
    }
}

/// Stable string key for deduplicating hazard types.
fn hazard_type_key(ht: &HazardType) -> &'static str {
    match ht {
        HazardType::Hurricane => "hurricane",
        HazardType::IceStorm => "ice_storm",
        HazardType::Wildfire => "wildfire",
        HazardType::Flood => "flood",
        HazardType::ExtremeHeat => "extreme_heat",
        HazardType::Earthquake => "earthquake",
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Section 7: Extreme-Event Resilience Planning
// ─────────────────────────────────────────────────────────────────────────────

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

/// Extreme weather and hazard event types threatening power grid infrastructure.
#[derive(Debug, Clone)]
pub enum ExtremeEventType {
    /// Hurricane with Saffir-Simpson category 1–5.
    Hurricane {
        /// Saffir-Simpson category (1–5).
        category: u8,
    },
    /// Ice storm characterised by ice accumulation.
    IceStorm {
        /// Ice thickness in mm.
        ice_thickness_mm: f64,
    },
    /// Wildfire with qualitative severity.
    Wildfire {
        /// Wildfire severity level.
        severity: FireSeverity,
    },
    /// Earthquake characterised by Richter magnitude.
    Earthquake {
        /// Richter magnitude.
        magnitude: f64,
    },
    /// Flood characterised by inundation depth.
    Flood {
        /// Flood depth in metres.
        depth_m: f64,
    },
    /// Heat wave characterised by peak temperature.
    HeatWave {
        /// Peak ambient temperature °C.
        peak_temp_c: f64,
    },
    /// Cyber attack on grid control infrastructure.
    CyberAttack {
        /// Attack vector description.
        attack_vector: String,
    },
}

/// Infrastructure hardening options for improving grid resilience against
/// extreme events.
#[derive(Debug, Clone, PartialEq)]
pub enum ExtremeHardeningOption {
    /// Replace overhead conductors with underground cables.
    UndergroundCable,
    /// Replace standard poles with storm-hardened steel/composite poles.
    StormHardenedPole,
    /// Remove vegetation posing a threat to overhead lines.
    VegetationManagement,
    /// Install flood barriers around substations.
    SubstationFloodBarrier,
    /// Deploy a mobile substation for rapid restoration.
    MobileSubstation,
    /// Establish a local microgrid with islanding capability.
    MicroGrid,
    /// Install a backup diesel or gas generator.
    BackupGenerator,
    /// Deploy smart switching to auto-isolate faults.
    SmartSwitching,
    /// Install sectionalizing switches to limit outage extent.
    SectionalizingSwitch,
}

/// Vulnerability characterisation for a single grid component under extreme
/// events.
#[derive(Debug, Clone)]
pub struct ComponentVulnerability {
    /// Unique identifier (index into the network asset list).
    pub component_id: usize,
    /// Component type, e.g. `"overhead_line"`, `"substation"`, `"transformer"`.
    pub component_type: String,
    /// Event-key-indexed failure probabilities in [0, 1].
    ///
    /// Keys follow the canonical form produced by [`extreme_event_key`]:
    /// e.g. `"Hurricane_3"`, `"IceStorm_25mm"`, `"Wildfire_High"`.
    pub failure_probability: std::collections::HashMap<String, f64>,
    /// Mean time to repair the component after failure (hours).
    pub repair_time_h: f64,
    /// Estimated replacement cost (USD).
    pub replacement_cost_usd: f64,
    /// Criticality score [0, 1] proportional to load served by this component.
    pub criticality_score: f64,
}

/// Four-dimensional resilience metrics for a grid under an extreme event.
#[derive(Debug, Clone)]
pub struct ExtremeResilienceMetrics {
    /// Expected energy not served during the event (MWh).
    pub expected_loss_mwh: f64,
    /// Expected time to restore full service (hours).
    pub expected_repair_time_h: f64,
    /// Area under the performance curve during the event (p.u.·hours).
    pub resilience_trapezoid_area: f64,
    /// Recovery speed: performance fraction recovered per hour.
    pub rapidity: f64,
    /// Minimum performance fraction during the event.
    pub robustness: f64,
    /// Fraction of load served via alternate paths.
    pub redundancy: f64,
    /// Fraction of failed components restorable within 24 h.
    pub resourcefulness: f64,
}

/// A concrete hardening measure that can be applied to one or more components.
#[derive(Debug, Clone)]
pub struct ExtremHardeningMeasure {
    /// Type of hardening intervention.
    pub option: ExtremeHardeningOption,
    /// IDs of targeted components.
    pub target_component_ids: Vec<usize>,
    /// Upfront capital expenditure (USD).
    pub capital_cost_usd: f64,
    /// Annual operating expenditure (USD).
    pub annual_opex_usd: f64,
    /// Multiplicative reduction in failure probability (e.g. 0.5 → 50 %).
    pub failure_prob_reduction: f64,
    /// Reduction in mean repair time (hours).
    pub repair_time_reduction_h: f64,
    /// Years required to implement.
    pub implementation_years: u8,
}

/// Prioritised resilience hardening plan produced by budget optimisation.
#[derive(Debug, Clone)]
pub struct ExtremeResiliencePlan {
    /// Selected hardening measures.
    pub measures: Vec<ExtremHardeningMeasure>,
    /// Total NPV cost of selected measures (USD).
    pub total_cost_usd: f64,
    /// NPV of expected damage reduction (USD).
    pub expected_benefit_usd: f64,
    /// Benefit-cost ratio.
    pub benefit_cost_ratio: f64,
    /// Post-hardening resilience metrics (representative event).
    pub resilience_improvement: ExtremeResilienceMetrics,
}

/// Advance one LCG step (Knuth multiplicative congruential generator).
#[inline]
fn ep_lcg_next(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64)
}

/// Convert a raw LCG state to a uniform sample in [0, 1).
#[inline]
fn ep_lcg_f64(state: u64) -> f64 {
    (state >> 11) as f64 / (1u64 << 53) as f64
}

/// Convert an [`ExtremeEventType`] to a canonical string key used in
/// failure probability maps.
///
/// # Examples
/// ```
/// use oxigrid::network::resilience_planning::{ExtremeEventType, extreme_event_key};
/// assert_eq!(extreme_event_key(&ExtremeEventType::Hurricane { category: 3 }), "Hurricane_3");
/// ```
pub fn extreme_event_key(event: &ExtremeEventType) -> String {
    match event {
        ExtremeEventType::Hurricane { category } => format!("Hurricane_{}", category),
        ExtremeEventType::IceStorm { ice_thickness_mm } => {
            if *ice_thickness_mm <= 5.0 {
                "IceStorm_5mm".to_string()
            } else if *ice_thickness_mm <= 25.0 {
                "IceStorm_25mm".to_string()
            } else {
                "IceStorm_50mm".to_string()
            }
        }
        ExtremeEventType::Wildfire { severity } => match severity {
            FireSeverity::Low => "Wildfire_Low".to_string(),
            FireSeverity::Moderate => "Wildfire_Moderate".to_string(),
            FireSeverity::High => "Wildfire_High".to_string(),
            FireSeverity::Extreme => "Wildfire_Extreme".to_string(),
        },
        ExtremeEventType::Earthquake { magnitude } => {
            if *magnitude < 5.0 {
                "Earthquake_Minor".to_string()
            } else if *magnitude < 7.0 {
                "Earthquake_Moderate".to_string()
            } else {
                "Earthquake_Major".to_string()
            }
        }
        ExtremeEventType::Flood { depth_m } => {
            if *depth_m < 1.0 {
                "Flood_Minor".to_string()
            } else if *depth_m < 3.0 {
                "Flood_Moderate".to_string()
            } else {
                "Flood_Major".to_string()
            }
        }
        ExtremeEventType::HeatWave { .. } => "HeatWave".to_string(),
        ExtremeEventType::CyberAttack { .. } => "CyberAttack".to_string(),
    }
}

/// Return a default failure probability for a component type given an event
/// key, based on empirical fragility data.
fn ep_default_failure_prob(component_type: &str, key: &str) -> f64 {
    match component_type {
        "overhead_line" => match key {
            "Hurricane_1" => 0.05,
            "Hurricane_2" => 0.10,
            "Hurricane_3" => 0.20,
            "Hurricane_4" => 0.40,
            "Hurricane_5" => 0.60,
            "IceStorm_5mm" => 0.05,
            "IceStorm_25mm" => 0.15,
            "IceStorm_50mm" => 0.35,
            "Wildfire_Low" => 0.05,
            "Wildfire_Moderate" => 0.15,
            "Wildfire_High" => 0.30,
            "Wildfire_Extreme" => 0.55,
            "Earthquake_Minor" => 0.02,
            "Earthquake_Moderate" => 0.10,
            "Earthquake_Major" => 0.25,
            "Flood_Minor" => 0.02,
            "Flood_Moderate" => 0.05,
            "Flood_Major" => 0.10,
            "HeatWave" => 0.02,
            "CyberAttack" => 0.01,
            _ => 0.05,
        },
        "substation" => match key {
            "Hurricane_1" => 0.01,
            "Hurricane_2" => 0.02,
            "Hurricane_3" => 0.05,
            "Hurricane_4" => 0.12,
            "Hurricane_5" => 0.25,
            "IceStorm_5mm" => 0.01,
            "IceStorm_25mm" => 0.03,
            "IceStorm_50mm" => 0.08,
            "Wildfire_Low" => 0.01,
            "Wildfire_Moderate" => 0.03,
            "Wildfire_High" => 0.05,
            "Wildfire_Extreme" => 0.10,
            "Earthquake_Minor" => 0.01,
            "Earthquake_Moderate" => 0.06,
            "Earthquake_Major" => 0.15,
            "Flood_Minor" => 0.03,
            "Flood_Moderate" => 0.10,
            "Flood_Major" => 0.20,
            "HeatWave" => 0.01,
            "CyberAttack" => 0.05,
            _ => 0.05,
        },
        "transformer" => (ep_default_failure_prob("substation", key) * 1.2).min(1.0),
        _ => 0.05,
    }
}

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
    fn ep_measure_cost(&self, measure: &ExtremHardeningMeasure) -> f64 {
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
            // Smart switching for every component
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

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── ResiliencePlanningTrapezoid ──────────────────────────────────────────

    fn sample_trapezoid() -> ResiliencePlanningTrapezoid {
        // t0=0, t1=1, t2=3, t3=8; phi0=1.0, phi1=0.4, phi3=0.9
        ResiliencePlanningTrapezoid::new(0.0, 1.0, 3.0, 8.0, 1.0, 0.4, 0.9)
    }

    #[test]
    fn trapezoid_resilience_index_in_range() {
        let t = sample_trapezoid();
        let idx = t.resilience_index();
        assert!(idx >= 0.0, "index must be >= 0, got {idx}");
        assert!(idx <= 1.0, "index must be <= 1, got {idx}");
    }

    #[test]
    fn trapezoid_recovery_rate_positive() {
        let t = sample_trapezoid(); // phi3=0.9 > phi1=0.4
        assert!(t.recovery_rate() > 0.0, "recovery_rate should be positive");
    }

    #[test]
    fn trapezoid_absorptive_capacity_le_1() {
        let t = sample_trapezoid(); // phi1=0.4 <= phi0=1.0
        let ac = t.absorptive_capacity();
        assert!(ac <= 1.0, "absorptive_capacity must be <= 1, got {ac}");
        assert!(ac >= 0.0);
    }

    #[test]
    fn trapezoid_rapidity_positive() {
        let t = sample_trapezoid();
        assert!(t.rapidity() > 0.0);
    }

    #[test]
    fn trapezoid_triangle_area_positive() {
        let t = sample_trapezoid();
        assert!(t.resilience_triangle_area() > 0.0);
    }

    #[test]
    fn trapezoid_restorative_capacity_correct() {
        let t = sample_trapezoid(); // t3-t2 = 5
        let rc = t.restorative_capacity();
        assert!((rc - 5.0).abs() < 1e-10, "expected 5.0, got {rc}");
    }

    // ── NkContingencyAnalysis ────────────────────────────────────────────────

    #[test]
    fn nk_combination_count_n10_k2_is_45() {
        let nk = NkContingencyAnalysis::new(10, 2);
        assert_eq!(nk.combination_count(), 45);
    }

    #[test]
    fn nk_combination_count_n5_k0_is_1() {
        let nk = NkContingencyAnalysis::new(5, 0);
        assert_eq!(nk.combination_count(), 1);
    }

    #[test]
    fn nk_most_critical_by_load_lost() {
        let nk = NkContingencyAnalysis::new(10, 1);
        let r1 = nk.analyze_contingency(&[1], 0.5); // load_lost = 5.0
        let r2 = nk.analyze_contingency(&[2], 0.9); // load_lost = 1.0
        let r3 = nk.analyze_contingency(&[3], 0.3); // load_lost = 7.0
        let critical = nk.most_critical_elements(&[r1, r2, r3], 2);
        // element 3 first (7.0), element 1 second (5.0)
        assert_eq!(critical[0], 3);
        assert_eq!(critical[1], 1);
    }

    #[test]
    fn nk_system_adequacy_all_ok() {
        let nk = NkContingencyAnalysis::new(5, 1);
        let r1 = nk.analyze_contingency(&[0], 0.95);
        let r2 = nk.analyze_contingency(&[1], 0.90);
        let adequacy = nk.system_adequacy(&[r1, r2]);
        assert!(
            (adequacy - 1.0).abs() < 1e-10,
            "all non-critical → adequacy=1"
        );
    }

    #[test]
    fn nk_system_adequacy_partial() {
        let nk = NkContingencyAnalysis::new(5, 1);
        let r1 = nk.analyze_contingency(&[0], 0.95); // not critical
        let r2 = nk.analyze_contingency(&[1], 0.50); // critical (<0.8)
        let adequacy = nk.system_adequacy(&[r1, r2]);
        assert!((adequacy - 0.5).abs() < 1e-10);
    }

    // ── HardeningOptimizer ───────────────────────────────────────────────────

    fn sample_element() -> GridElement {
        GridElement {
            id: 0,
            name: "Feeder-A".to_string(),
            element_type: ElementType::Distribution,
            failure_rate_per_year: 2.0, // 2 faults/year
            mttr_h: 4.0,                // 4 h MTTR
            load_served_mw: 5.0,        // 5 MW
            hardening_options: vec![HardeningOption {
                name: "Underground cable".to_string(),
                cost: 50_000.0,
                failure_rate_reduction: 0.8,
                mttr_reduction: 0.5,
                residual_value_pct: 0.1,
            }],
        }
    }

    fn sample_optimizer() -> HardeningOptimizer {
        HardeningOptimizer {
            elements: vec![sample_element()],
            budget: 100_000.0,
            planning_horizon_years: 20.0,
            discount_rate: 0.05,
        }
    }

    #[test]
    fn hardening_ens_formula() {
        let opt = sample_optimizer();
        let el = &opt.elements[0];
        // ENS = 2.0 * 4.0 * 5.0 = 40 MWh/year
        let ens = opt.expected_energy_not_served_mwh_per_year(el);
        assert!((ens - 40.0).abs() < 1e-10, "expected 40, got {ens}");
    }

    #[test]
    fn hardening_greedy_within_budget() {
        let opt = sample_optimizer();
        let plan = opt.optimize_greedy();
        assert!(plan.total_cost <= opt.budget, "plan cost exceeds budget");
        assert!(!plan.selected_options.is_empty());
    }

    #[test]
    fn hardening_bcr_positive() {
        let opt = sample_optimizer();
        let el = &opt.elements[0];
        let option = &el.hardening_options[0];
        let bcr = opt.benefit_cost_ratio(el, option);
        assert!(bcr > 0.0, "BCR should be positive, got {bcr}");
    }

    // ── ResilienceMicrogrid ──────────────────────────────────────────────────

    fn adequate_microgrid() -> ResilienceMicrogrid {
        ResilienceMicrogrid {
            id: 1,
            location: "Hospital District".to_string(),
            critical_load_mw: 2.0,
            pv_mw: 1.0,
            battery_mwh: 10.0,
            battery_mw: 1.5,
            diesel_mw: 1.0,
            autonomy_days: 3.0,
        }
    }

    #[test]
    fn microgrid_can_island_with_resources() {
        let mg = adequate_microgrid();
        // battery_mw(1.5) + diesel_mw(1.0) = 2.5 >= critical_load(2.0)
        assert!(mg.can_island());
    }

    #[test]
    fn microgrid_sizing_adequate() {
        let mg = adequate_microgrid();
        // required battery = 2.0 * 3 * 24 * 0.2 = 28.8 MWh; battery=10 < 28.8 → undersized
        // So this microgrid actually fails the battery check. Let's use a bigger battery.
        let mg2 = ResilienceMicrogrid {
            battery_mwh: 50.0,
            ..mg
        };
        assert_eq!(mg2.sizing_check(), SizingStatus::Adequate);
    }

    #[test]
    fn microgrid_sizing_battery_undersized() {
        let mg = ResilienceMicrogrid {
            id: 2,
            location: "Test".to_string(),
            critical_load_mw: 10.0,
            pv_mw: 2.0,
            battery_mwh: 1.0, // very small
            battery_mw: 5.0,
            diesel_mw: 6.0,
            autonomy_days: 2.0,
        };
        // required = 10 * 2 * 24 * 0.2 = 96 MWh; actual=1 → undersized
        match mg.sizing_check() {
            SizingStatus::BatteryUndersized { deficit_mwh } => {
                assert!(deficit_mwh > 0.0)
            }
            other => panic!("expected BatteryUndersized, got {other:?}"),
        }
    }

    // ── RestorationEstimator ─────────────────────────────────────────────────

    fn sample_estimator() -> RestorationEstimator {
        RestorationEstimator {
            crew_count: 4,
            crew_travel_time_h: 1.0,
            repair_time_distribution: RepairDistribution {
                min_h: 2.0,
                mode_h: 4.0,
                max_h: 9.0,
            },
            damage_assessment_time_h: 0.5,
        }
    }

    #[test]
    fn restoration_expected_repair_time_triangular_mean() {
        let est = sample_estimator();
        // (2+4+9)/3 = 5.0
        let mean = est.expected_repair_time_h();
        assert!((mean - 5.0).abs() < 1e-10, "expected 5.0, got {mean}");
    }

    #[test]
    fn restoration_percentile_above_min() {
        let est = sample_estimator();
        let p50 = est.percentile_repair_time_h(0.5);
        // Must exceed crew_travel + assessment + min_repair = 1+0.5+2 = 3.5
        assert!(p50 > 3.5, "p50={p50} should exceed 3.5");
    }

    #[test]
    fn restoration_sequence_total_time_positive() {
        let est = sample_estimator();
        let plan = est.restoration_sequence(5, 3.0, 10.0);
        assert!(plan.total_time_h > 0.0);
        assert_eq!(plan.phases.len(), 4);
    }

    // ── ClimateRiskAssessor ──────────────────────────────────────────────────

    fn sample_assessor() -> ClimateRiskAssessor {
        ClimateRiskAssessor {
            region: "Gulf Coast".to_string(),
            hazards: vec![
                ClimateHazard {
                    hazard_type: HazardType::Hurricane,
                    annual_probability: 0.05,
                    severity: HazardSeverity::High,
                    affected_fraction: 0.4,
                    damage_fraction: 0.3,
                },
                ClimateHazard {
                    hazard_type: HazardType::Flood,
                    annual_probability: 0.1,
                    severity: HazardSeverity::Medium,
                    affected_fraction: 0.2,
                    damage_fraction: 0.15,
                },
            ],
        }
    }

    #[test]
    fn climate_risk_damage_positive() {
        let ca = sample_assessor();
        let dmg = ca.annual_expected_damage_pct();
        assert!(dmg > 0.0, "expected damage should be positive, got {dmg}");
    }

    #[test]
    fn climate_risk_resilience_score_in_range() {
        let ca = sample_assessor();
        let score = ca.resilience_score();
        assert!(
            (0.0..=100.0).contains(&score),
            "score {score} out of [0,100]"
        );
    }

    #[test]
    fn climate_risk_most_critical_hazard_is_hurricane() {
        let ca = sample_assessor();
        let critical = ca.most_critical_hazard().expect("should have a hazard");
        assert_eq!(critical.hazard_type, HazardType::Hurricane);
    }

    #[test]
    fn climate_risk_recommendations_nonempty() {
        let ca = sample_assessor();
        let recs = ca.recommended_hardening();
        assert!(!recs.is_empty());
        // Exactly 2 distinct hazard types → 2 recs
        assert_eq!(recs.len(), 2);
    }

    // ── ExtremeEvent ResiliencePlanner ───────────────────────────────────────

    fn ep_line(id: usize, crit: f64, repair_h: f64) -> ComponentVulnerability {
        ComponentVulnerability {
            component_id: id,
            component_type: "overhead_line".to_string(),
            failure_probability: std::collections::HashMap::new(),
            repair_time_h: repair_h,
            replacement_cost_usd: 50_000.0,
            criticality_score: crit,
        }
    }

    fn ep_sub(id: usize, crit: f64) -> ComponentVulnerability {
        ComponentVulnerability {
            component_id: id,
            component_type: "substation".to_string(),
            failure_probability: std::collections::HashMap::new(),
            repair_time_h: 48.0,
            replacement_cost_usd: 500_000.0,
            criticality_score: crit,
        }
    }

    fn ep_planner() -> ResiliencePlanner {
        let comps = vec![ep_line(0, 0.5, 8.0), ep_line(1, 0.3, 12.0), ep_sub(2, 0.2)];
        let mut p = ResiliencePlanner::new(comps, 5000, 100.0);
        p.annual_event_probability = 0.10;
        p
    }

    #[test]
    fn test_vulnerability_assessment_hurricane() {
        let planner = ep_planner();
        let v1 = planner.assess_vulnerability(&ExtremeEventType::Hurricane { category: 1 });
        let v5 = planner.assess_vulnerability(&ExtremeEventType::Hurricane { category: 5 });
        let p1 = v1
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        let p5 = v5
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        assert!(p5 > p1, "cat 5 must exceed cat 1: {} vs {}", p5, p1);
    }

    #[test]
    fn test_vulnerability_assessment_ice_storm() {
        let planner = ep_planner();
        let v = planner.assess_vulnerability(&ExtremeEventType::IceStorm {
            ice_thickness_mm: 25.0,
        });
        let prob = v
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        assert!(prob > 0.0 && prob <= 1.0, "IceStorm_25mm prob={}", prob);
    }

    #[test]
    fn test_baseline_metrics_no_events() {
        let mut comp = ep_line(0, 1.0, 4.0);
        comp.failure_probability
            .insert("Hurricane_3".to_string(), 0.0);
        let planner = ResiliencePlanner::new(vec![comp], 1000, 50.0);
        let m = planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 3 });
        assert!(
            (m.robustness - 1.0).abs() < 1e-6,
            "robustness={}",
            m.robustness
        );
    }

    #[test]
    fn test_monte_carlo_convergence() {
        let planner = ep_planner();
        let event = ExtremeEventType::Hurricane { category: 3 };
        let m100 = planner.simulate_event(&event, 100);
        let m2000 = planner.simulate_event(&event, 2000);
        let diff = (m100.robustness - m2000.robustness).abs();
        assert!(diff < 0.5, "convergence diff={:.4}", diff);
    }

    #[test]
    fn test_resilience_trapezoid_formula() {
        let robustness = 0.6_f64;
        let t_event = 24.0_f64;
        let t_restore = 12.0_f64;
        let area = 0.5 * (1.0 + robustness) * t_event + 0.5 * (robustness + 1.0) * t_restore;
        assert!((area - 28.8).abs() < 1e-6, "area={}", area);
    }

    #[test]
    fn test_hardening_bcr_ranking() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let ranked = planner.rank_hardening_options(&measures);
        assert!(!ranked.is_empty());
        for i in 1..ranked.len() {
            assert!(
                ranked[i - 1].1 >= ranked[i].1 - 1e-12,
                "not descending at {}",
                i
            );
        }
    }

    #[test]
    fn test_greedy_budget_optimization() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 150_000.0);
        assert!(
            plan.total_cost_usd <= 150_000.0 + 1e-6,
            "cost={}",
            plan.total_cost_usd
        );
    }

    #[test]
    fn test_greedy_selects_best_bcr() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let ranked = planner.rank_hardening_options(&measures);
        let plan = planner.optimize_hardening_budget(&measures, 1e9);
        if !plan.measures.is_empty() && !ranked.is_empty() {
            let best_opt = &measures[ranked[0].0].option;
            assert_eq!(&plan.measures[0].option, best_opt);
        }
    }

    #[test]
    fn test_compute_measure_benefit() {
        let planner = ep_planner();
        let m = ExtremHardeningMeasure {
            option: ExtremeHardeningOption::StormHardenedPole,
            target_component_ids: vec![0],
            capital_cost_usd: 50_000.0,
            annual_opex_usd: 1_000.0,
            failure_prob_reduction: 0.5,
            repair_time_reduction_h: 4.0,
            implementation_years: 2,
        };
        let b = planner.compute_measure_benefit(&m, &ExtremeEventType::Hurricane { category: 3 });
        assert!(b > 0.0, "benefit={}", b);
    }

    #[test]
    fn test_resilience_index_bounds() {
        let planner = ep_planner();
        let metrics =
            planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 3 });
        let idx = planner.compute_resilience_index(&metrics);
        assert!((0.0..=1.0).contains(&idx), "index={}", idx);
    }

    #[test]
    fn test_generate_hardening_measures() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        assert!(!measures.is_empty());
        assert!(measures.len() >= planner.components.len());
    }

    #[test]
    fn test_plan_total_cost() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 1e9);
        let manual: f64 = plan
            .measures
            .iter()
            .map(|m| planner.ep_measure_cost(m))
            .sum();
        assert!((plan.total_cost_usd - manual).abs() < 1.0, "cost mismatch");
    }

    #[test]
    fn test_plan_bcr_positive() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 500_000.0);
        assert!(
            plan.benefit_cost_ratio >= 0.0,
            "bcr={}",
            plan.benefit_cost_ratio
        );
    }

    #[test]
    fn test_redundancy_metric() {
        let mut comp = ep_line(0, 1.0, 2.0);
        comp.failure_probability
            .insert("Hurricane_1".to_string(), 0.01);
        let planner = ResiliencePlanner::new(vec![comp], 100, 10.0);
        let m = planner.simulate_event(&ExtremeEventType::Hurricane { category: 1 }, 500);
        assert!(m.redundancy > 0.5, "redundancy={}", m.redundancy);
    }

    #[test]
    fn test_rapidity_positive() {
        let planner = ep_planner();
        let m = planner.compute_baseline_metrics(&ExtremeEventType::Hurricane { category: 2 });
        assert!(m.rapidity >= 0.0, "rapidity={}", m.rapidity);
    }

    #[test]
    fn test_hurricane_categories() {
        let p1 = ep_default_failure_prob("overhead_line", "Hurricane_1");
        let p5 = ep_default_failure_prob("overhead_line", "Hurricane_5");
        assert!(p5 > p1, "cat5={} must exceed cat1={}", p5, p1);
    }

    #[test]
    fn test_empty_components() {
        let planner = ResiliencePlanner::new(vec![], 0, 0.0);
        let event = ExtremeEventType::Hurricane { category: 3 };
        assert!(planner.assess_vulnerability(&event).is_empty());
        let m = planner.compute_baseline_metrics(&event);
        assert_eq!(m.robustness, 1.0);
        let mc = planner.simulate_event(&event, 100);
        assert_eq!(mc.robustness, 1.0);
    }

    #[test]
    fn test_zero_budget() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 0.0);
        assert!(
            plan.measures.is_empty(),
            "expected empty plan, got {}",
            plan.measures.len()
        );
        assert_eq!(plan.total_cost_usd, 0.0);
    }

    #[test]
    fn test_full_budget() {
        let planner = ep_planner();
        let measures = planner.generate_hardening_measures();
        let plan = planner.optimize_hardening_budget(&measures, 1e12);
        let ranked = planner.rank_hardening_options(&measures);
        let n_pos = ranked.iter().filter(|(_, bcr)| *bcr > 0.0).count();
        assert_eq!(plan.measures.len(), n_pos, "expected {} measures", n_pos);
    }

    #[test]
    fn test_voll_impact() {
        let event = ExtremeEventType::Hurricane { category: 3 };
        let m = ExtremHardeningMeasure {
            option: ExtremeHardeningOption::StormHardenedPole,
            target_component_ids: vec![0],
            capital_cost_usd: 50_000.0,
            annual_opex_usd: 1_000.0,
            failure_prob_reduction: 0.5,
            repair_time_reduction_h: 4.0,
            implementation_years: 2,
        };
        let mut low = ep_planner();
        low.voll_usd_per_mwh = 1_000.0;
        let mut high = ep_planner();
        high.voll_usd_per_mwh = 50_000.0;
        let b_low = low.compute_measure_benefit(&m, &event);
        let b_high = high.compute_measure_benefit(&m, &event);
        assert!(b_high > b_low, "high={} low={}", b_high, b_low);
    }

    #[test]
    fn test_extreme_event_key_hurricane() {
        assert_eq!(
            extreme_event_key(&ExtremeEventType::Hurricane { category: 1 }),
            "Hurricane_1"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::Hurricane { category: 5 }),
            "Hurricane_5"
        );
    }

    #[test]
    fn test_extreme_event_key_ice_storm() {
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 5.0
            }),
            "IceStorm_5mm"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 25.0
            }),
            "IceStorm_25mm"
        );
        assert_eq!(
            extreme_event_key(&ExtremeEventType::IceStorm {
                ice_thickness_mm: 50.0
            }),
            "IceStorm_50mm"
        );
    }

    #[test]
    fn test_transformer_vs_substation_prob() {
        let p_sub = ep_default_failure_prob("substation", "Hurricane_3");
        let p_tf = ep_default_failure_prob("transformer", "Hurricane_3");
        assert!(p_tf >= p_sub, "transformer={} sub={}", p_tf, p_sub);
    }

    #[test]
    fn test_resilience_index_perfect() {
        let planner = ResiliencePlanner::new(vec![], 0, 100.0);
        let m = ExtremeResilienceMetrics {
            expected_loss_mwh: 0.0,
            expected_repair_time_h: 0.0,
            resilience_trapezoid_area: 48.0,
            rapidity: 1.0,
            robustness: 1.0,
            redundancy: 1.0,
            resourcefulness: 1.0,
        };
        let idx = planner.compute_resilience_index(&m);
        assert!((idx - 1.0).abs() < 1e-9, "idx={}", idx);
    }
}
