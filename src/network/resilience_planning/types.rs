//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::functions::{combinations, hazard_type_key, severity_factor, VOLL_DOLLARS_PER_MWH};
use super::types_2::{FireSeverity, HazardSeverity, HazardType, RestorationPhase};

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
        candidates.sort_by(|a, b| {
            b.bcr
                .partial_cmp(&a.bcr)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut selected: Vec<(usize, usize)> = Vec::new();
        let mut total_cost = 0.0_f64;
        let mut total_ens_reduction = 0.0_f64;
        let mut total_npv = 0.0_f64;
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
    /// Keys follow the canonical form produced by `extreme_event_key`:
    /// e.g. `"Hurricane_3"`, `"IceStorm_25mm"`, `"Wildfire_High"`.
    pub failure_probability: std::collections::HashMap<String, f64>,
    /// Mean time to repair the component after failure (hours).
    pub repair_time_h: f64,
    /// Estimated replacement cost (USD).
    pub replacement_cost_usd: f64,
    /// Criticality score [0, 1] proportional to load served by this component.
    pub criticality_score: f64,
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
