/// Demand Response Flexibility Characterization.
///
/// Models the flexibility envelope of different DR program types, covering
/// load shifting, load shedding, valley filling, peak clipping, direct load
/// control, interruptible loads, and demand bidding.
///
/// # Flexibility Types
///
/// - **LoadShift**: Moves consumption between time periods (energy-neutral).
/// - **LoadShed**: Permanent reduction (no rebound).
/// - **LoadCurtailment**: Temporary reduction with rebound effect.
/// - **ValleyFilling**: Increases load in off-peak periods.
/// - **PeakClipping**: Reduces load during peak hours.
/// - **DirectLoadControl**: Utility-controlled interruption.
/// - **InterruptibleLoad**: Contractual curtailment obligation.
/// - **DemandBidding**: Customer-submitted bids to wholesale market.
///
/// # Merit Order Dispatch
///
/// Resources are sorted by willingness-to-pay (curtailment cost) ascending
/// and dispatched in order until the target reduction is met.
///
/// # Rebound Modelling
///
/// After curtailment, load rebounds according to an exponential decay:
///   rebound(t) = dispatched_kw × recovery_factor × exp(-t / τ)
/// where τ = recovery_time_h / 3, and t is in quarter-hour increments.
///
/// # References
/// - FERC Order 745 (2011): demand response compensation in organised markets.
/// - IEA, "Harnessing Variable Renewables", 2011.
/// - Strbac, "Demand side management: Benefits and challenges", Energy Policy 2008.
use std::collections::HashMap;

use serde::{Deserialize, Serialize};

// ─── Enums ────────────────────────────────────────────────────────────────────

/// Flexibility program type — characterises the nature of the demand response.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FlexibilityType {
    /// Shift load between time periods (energy conserved).
    LoadShift,
    /// Permanent load reduction with no rebound.
    LoadShed,
    /// Temporary curtailment followed by rebound consumption.
    LoadCurtailment,
    /// Increase off-peak consumption (valley filling).
    ValleyFilling,
    /// Reduce peak-hour consumption.
    PeakClipping,
    /// Utility/aggregator directly controls the load.
    DirectLoadControl,
    /// Contractual obligation to curtail on request.
    InterruptibleLoad,
    /// Customer submits bids to wholesale or flexibility market.
    DemandBidding,
}

/// Customer segment — determines default elasticity and behaviour profiles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CustomerSegment {
    /// Large industrial facility (e.g., smelter, cement plant).
    LargeIndustrial,
    /// Small commercial premises (e.g., retail shop, small office).
    SmallCommercial,
    /// Large commercial building (e.g., shopping centre, hotel).
    LargeCommercial,
    /// Residential household.
    Residential,
    /// Agricultural operation (e.g., irrigation pumps).
    Agricultural,
    /// Data centre with UPS backup.
    DataCenter,
    /// Electric vehicle charging station or fleet.
    ElectricVehicle,
    /// Grid-connected battery storage system.
    BatteryStorage,
}

/// Activation mode — how the DR event is triggered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ActivationMode {
    /// Operator manually issues a curtailment request.
    Manual,
    /// Automatic dispatch based on real-time signals.
    Automatic,
    /// Pre-programmed schedule (time-of-use).
    Scheduled,
    /// Customer responds autonomously to a published price signal.
    PriceSignal,
}

// ─── Structs ──────────────────────────────────────────────────────────────────

/// A single demand response flexibility resource (load or aggregation of loads).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityResource {
    /// Unique resource identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Customer segment classification.
    pub segment: CustomerSegment,
    /// Flexibility program type.
    pub flex_type: FlexibilityType,
    /// Baseline (uncontrolled) power consumption `kW`.
    pub baseline_kw: f64,
    /// Maximum achievable load reduction `kW`.
    pub max_reduction_kw: f64,
    /// Maximum load increase capability for valley filling `kW`.
    pub max_increase_kw: f64,
    /// Minimum event duration `h`.
    pub min_duration_h: f64,
    /// Maximum event duration `h`.
    pub max_duration_h: f64,
    /// Duration over which rebound load recovers after curtailment `h`.
    pub recovery_time_h: f64,
    /// Fraction of curtailed energy that rebounds (0 = no rebound, 1 = full).
    pub recovery_factor: f64,
    /// How the DR event is activated.
    pub activation_mode: ActivationMode,
    /// Advance notice required before dispatch `minutes`.
    pub notification_time_min: f64,
    /// Maximum DR events permitted per day.
    pub max_events_per_day: u8,
    /// Maximum DR events permitted per year.
    pub max_events_per_year: u16,
    /// Customer's willingness to accept curtailment [$/kWh curtailed].
    pub willingness_to_pay_usd_per_kwh: f64,
}

impl FlexibilityResource {
    /// Construct a large industrial interruptible load resource with typical parameters.
    pub fn large_industrial(id: usize) -> Self {
        Self {
            id,
            name: format!("LargeIndustrial-{id}"),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::InterruptibleLoad,
            baseline_kw: 500.0,
            max_reduction_kw: 400.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 8.0,
            recovery_time_h: 2.0,
            recovery_factor: 0.3,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 30.0,
            max_events_per_day: 2,
            max_events_per_year: 50,
            willingness_to_pay_usd_per_kwh: 0.15,
        }
    }

    /// Construct a residential load curtailment resource with typical parameters.
    pub fn residential(id: usize) -> Self {
        Self {
            id,
            name: format!("Residential-{id}"),
            segment: CustomerSegment::Residential,
            flex_type: FlexibilityType::LoadCurtailment,
            baseline_kw: 3.0,
            max_reduction_kw: 1.0,
            max_increase_kw: 0.5,
            min_duration_h: 0.5,
            max_duration_h: 2.0,
            recovery_time_h: 1.0,
            recovery_factor: 0.6,
            activation_mode: ActivationMode::PriceSignal,
            notification_time_min: 5.0,
            max_events_per_day: 4,
            max_events_per_year: 200,
            willingness_to_pay_usd_per_kwh: 0.05,
        }
    }

    /// Construct an EV fleet valley-filling resource with typical parameters.
    pub fn ev_fleet(id: usize) -> Self {
        Self {
            id,
            name: format!("EVFleet-{id}"),
            segment: CustomerSegment::ElectricVehicle,
            flex_type: FlexibilityType::ValleyFilling,
            baseline_kw: 50.0,
            max_reduction_kw: 40.0,
            max_increase_kw: 60.0,
            min_duration_h: 0.25,
            max_duration_h: 4.0,
            recovery_time_h: 0.0,
            recovery_factor: 0.0,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 0.0,
            max_events_per_day: 8,
            max_events_per_year: 365,
            willingness_to_pay_usd_per_kwh: 0.08,
        }
    }
}

/// Per-resource, per-hour flexibility envelope covering the planning horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityEnvelope {
    /// Resource this envelope belongs to.
    pub resource_id: usize,
    /// Number of hours in the planning horizon.
    pub time_horizon_h: usize,
    /// Available upward flexibility (load increase) per hour `kW`.
    pub upward_flex_kw: Vec<f64>,
    /// Available downward flexibility (load reduction) per hour `kW`.
    pub downward_flex_kw: Vec<f64>,
    /// Expected rebound load after curtailment per hour `kW`.
    pub rebound_kw: Vec<f64>,
    /// Curtailment cost per hour [$/kWh].
    pub cost_usd_per_kwh: Vec<f64>,
}

/// A dispatch signal sent to a flexibility resource requesting curtailment or increase.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispatchSignal {
    /// Target resource ID.
    pub resource_id: usize,
    /// Hour (0-indexed) at which dispatch begins.
    pub start_hour: usize,
    /// Requested dispatch duration `h`.
    pub duration_h: f64,
    /// Requested power: positive = curtail, negative = increase `kW`.
    pub requested_kw: f64,
    /// Actual dispatched power after feasibility checks `kW`.
    pub actual_kw: f64,
    /// Curtailment cost for this dispatch event [$].
    pub cost_usd: f64,
    /// Whether the dispatch is feasible given all operational constraints.
    pub feasible: bool,
}

/// Aggregated portfolio of flexibility resources with combined envelope.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityPortfolio {
    /// Individual resources in the portfolio.
    pub resources: Vec<FlexibilityResource>,
    /// Hour-by-hour aggregate flexibility envelope across all resources.
    pub aggregated_envelope: FlexibilityEnvelope,
    /// Sum of all resource baseline loads `kW`.
    pub total_baseline_kw: f64,
    /// Sum of all maximum reductions `kW`.
    pub total_max_reduction_kw: f64,
    /// Sum of all maximum increases `kW`.
    pub total_max_increase_kw: f64,
}

/// Econometric elasticity model for a customer segment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElasticityModel {
    /// Customer segment this model applies to.
    pub segment: CustomerSegment,
    /// Own-price elasticity of demand (negative: demand falls as price rises).
    pub own_price_elasticity: f64,
    /// Cross-time-of-use elasticity (substitution between peak and off-peak).
    pub cross_time_elasticity: f64,
    /// Income elasticity of demand.
    pub income_elasticity: f64,
}

/// A flexibility bid submitted to a wholesale or local flexibility market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityBid {
    /// Resource submitting the bid.
    pub resource_id: usize,
    /// Hour for which the bid is valid.
    pub hour: usize,
    /// Quantity offered `kW`.
    pub quantity_kw: f64,
    /// Bid price [$/kWh].
    pub price_usd_per_kwh: f64,
    /// Flexibility type offered.
    pub flex_type: FlexibilityType,
    /// Whether the bid was accepted by the market clearing algorithm.
    pub accepted: bool,
}

// ─── Main aggregator ──────────────────────────────────────────────────────────

/// Aggregates a portfolio of flexibility resources and provides dispatch,
/// envelope computation, rebound modelling, and price-response estimation.
///
/// # Example
/// ```rust,ignore
/// let resources = vec![FlexibilityResource::large_industrial(0)];
/// let mut agg = FlexibilityAggregator::new(resources, 24);
/// let signals = agg.aggregate_dispatch(200.0, 10);
/// ```
pub struct FlexibilityAggregator {
    /// Enrolled flexibility resources.
    pub resources: Vec<FlexibilityResource>,
    /// Own-price elasticity of demand per customer segment (negative values).
    pub price_elasticity: HashMap<CustomerSegment, f64>,
    /// Planning horizon in hours.
    pub time_horizon_h: usize,
}

impl FlexibilityAggregator {
    /// Create a new aggregator with default per-segment price elasticities.
    ///
    /// Default own-price elasticities (all negative):
    /// - LargeIndustrial: -0.50
    /// - SmallCommercial: -0.30
    /// - LargeCommercial: -0.35
    /// - Residential: -0.15
    /// - Agricultural: -0.20
    /// - DataCenter: -0.05
    /// - ElectricVehicle: -0.60
    /// - BatteryStorage: -0.80
    pub fn new(resources: Vec<FlexibilityResource>, time_horizon_h: usize) -> Self {
        let mut price_elasticity = HashMap::new();
        price_elasticity.insert(CustomerSegment::LargeIndustrial, -0.50);
        price_elasticity.insert(CustomerSegment::SmallCommercial, -0.30);
        price_elasticity.insert(CustomerSegment::LargeCommercial, -0.35);
        price_elasticity.insert(CustomerSegment::Residential, -0.15);
        price_elasticity.insert(CustomerSegment::Agricultural, -0.20);
        price_elasticity.insert(CustomerSegment::DataCenter, -0.05);
        price_elasticity.insert(CustomerSegment::ElectricVehicle, -0.60);
        price_elasticity.insert(CustomerSegment::BatteryStorage, -0.80);
        Self {
            resources,
            price_elasticity,
            time_horizon_h,
        }
    }

    /// Compute the hour-by-hour flexibility envelope for a single resource.
    ///
    /// Downward flexibility uses a diurnal capacity factor (1.0 during peak
    /// hours 8–19, 0.6 off-peak).  EV resources receive a 1.2× upward boost
    /// during overnight hours 22–05.
    pub fn compute_envelope(&self, resource_id: usize) -> FlexibilityEnvelope {
        let n = self.time_horizon_h;
        let r = match self.resources.iter().find(|r| r.id == resource_id) {
            Some(r) => r,
            None => {
                return FlexibilityEnvelope {
                    resource_id,
                    time_horizon_h: n,
                    upward_flex_kw: vec![0.0; n],
                    downward_flex_kw: vec![0.0; n],
                    rebound_kw: vec![0.0; n],
                    cost_usd_per_kwh: vec![0.0; n],
                };
            }
        };

        let mut upward = Vec::with_capacity(n);
        let mut downward = Vec::with_capacity(n);
        let mut rebound = Vec::with_capacity(n);
        let mut cost = Vec::with_capacity(n);

        for h in 0..n {
            let hour_of_day = h % 24;

            // Diurnal capacity factor for downward flex
            let cap_factor = if (8..20).contains(&hour_of_day) {
                1.0
            } else {
                0.6
            };

            let down_kw = r.max_reduction_kw.min(r.baseline_kw * cap_factor);

            // Upward flex: EV gets overnight boost, others are flat
            let up_kw = if r.segment == CustomerSegment::ElectricVehicle {
                if !(6..22).contains(&hour_of_day) {
                    r.max_increase_kw * 1.2
                } else {
                    r.max_increase_kw * 0.8
                }
            } else {
                r.max_increase_kw
            };

            upward.push(up_kw);
            downward.push(down_kw);
            rebound.push(0.0); // populated after actual dispatch via compute_rebound
            cost.push(r.willingness_to_pay_usd_per_kwh);
        }

        FlexibilityEnvelope {
            resource_id,
            time_horizon_h: n,
            upward_flex_kw: upward,
            downward_flex_kw: downward,
            rebound_kw: rebound,
            cost_usd_per_kwh: cost,
        }
    }

    /// Aggregate all resource envelopes into a single portfolio envelope.
    ///
    /// The aggregated cost is the capacity-weighted average of individual costs.
    pub fn aggregate_portfolio(&self) -> FlexibilityPortfolio {
        let n = self.time_horizon_h;
        let mut up_agg = vec![0.0f64; n];
        let mut down_agg = vec![0.0f64; n];
        let mut rebound_agg = vec![0.0f64; n];
        let mut cost_num = vec![0.0f64; n]; // numerator for weighted average

        for r in &self.resources {
            let env = self.compute_envelope(r.id);
            for h in 0..n {
                up_agg[h] += env.upward_flex_kw[h];
                down_agg[h] += env.downward_flex_kw[h];
                rebound_agg[h] += env.rebound_kw[h];
                cost_num[h] += env.cost_usd_per_kwh[h] * env.downward_flex_kw[h];
            }
        }

        // Capacity-weighted average cost; zero when no downward flex is available
        let cost_agg: Vec<f64> = (0..n)
            .map(|h| {
                if down_agg[h] > 1e-9 {
                    cost_num[h] / down_agg[h]
                } else {
                    0.0
                }
            })
            .collect();

        let total_baseline_kw = self.resources.iter().map(|r| r.baseline_kw).sum();
        let total_max_reduction_kw = self.resources.iter().map(|r| r.max_reduction_kw).sum();
        let total_max_increase_kw = self.resources.iter().map(|r| r.max_increase_kw).sum();

        FlexibilityPortfolio {
            resources: self.resources.clone(),
            aggregated_envelope: FlexibilityEnvelope {
                resource_id: 0,
                time_horizon_h: n,
                upward_flex_kw: up_agg,
                downward_flex_kw: down_agg,
                rebound_kw: rebound_agg,
                cost_usd_per_kwh: cost_agg,
            },
            total_baseline_kw,
            total_max_reduction_kw,
            total_max_increase_kw,
        }
    }

    /// Check feasibility of a dispatch signal and return the verified result.
    ///
    /// - Positive `requested_kw` → curtailment (clamped to `max_reduction_kw`).
    /// - Negative `requested_kw` → load increase (clamped to `max_increase_kw`).
    /// - Duration below `min_duration_h` → infeasible.
    /// - Duration above `max_duration_h` → clamped (still feasible).
    pub fn dispatch(&self, resource_id: usize, signal: &DispatchSignal) -> DispatchSignal {
        let r = match self.resources.iter().find(|r| r.id == resource_id) {
            Some(r) => r,
            None => {
                return DispatchSignal {
                    resource_id,
                    start_hour: signal.start_hour,
                    duration_h: signal.duration_h,
                    requested_kw: signal.requested_kw,
                    actual_kw: 0.0,
                    cost_usd: 0.0,
                    feasible: false,
                };
            }
        };

        // Duration feasibility check
        if signal.duration_h < r.min_duration_h {
            return DispatchSignal {
                resource_id,
                start_hour: signal.start_hour,
                duration_h: signal.duration_h,
                requested_kw: signal.requested_kw,
                actual_kw: 0.0,
                cost_usd: 0.0,
                feasible: false,
            };
        }

        // Clamp duration to maximum
        let eff_duration = signal.duration_h.min(r.max_duration_h);

        // Power feasibility: clamp to resource capability
        let actual_kw = if signal.requested_kw > 0.0 {
            signal.requested_kw.min(r.max_reduction_kw)
        } else if signal.requested_kw < 0.0 {
            signal.requested_kw.max(-r.max_increase_kw)
        } else {
            0.0
        };

        let cost_usd = actual_kw.abs() * eff_duration * r.willingness_to_pay_usd_per_kwh;
        let feasible = actual_kw.abs() > 1e-9;

        DispatchSignal {
            resource_id,
            start_hour: signal.start_hour,
            duration_h: eff_duration,
            requested_kw: signal.requested_kw,
            actual_kw,
            cost_usd,
            feasible,
        }
    }

    /// Merit-order dispatch across all resources to meet a curtailment target.
    ///
    /// Resources with `max_reduction_kw > 0` are sorted by
    /// `willingness_to_pay_usd_per_kwh` ascending (cheapest first) and
    /// dispatched in order until `target_kw` is met or all are exhausted.
    pub fn aggregate_dispatch(&mut self, target_kw: f64, hour: usize) -> Vec<DispatchSignal> {
        // Collect resource IDs with available reduction, sorted cheapest-first
        let mut order: Vec<usize> = self
            .resources
            .iter()
            .filter(|r| r.max_reduction_kw > 0.0)
            .map(|r| r.id)
            .collect();

        order.sort_by(|&a, &b| {
            let ca = self
                .resources
                .iter()
                .find(|r| r.id == a)
                .map(|r| r.willingness_to_pay_usd_per_kwh)
                .unwrap_or(f64::MAX);
            let cb = self
                .resources
                .iter()
                .find(|r| r.id == b)
                .map(|r| r.willingness_to_pay_usd_per_kwh)
                .unwrap_or(f64::MAX);
            ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut signals = Vec::new();
        let mut remaining = target_kw;

        for id in order {
            if remaining <= 1e-9 {
                break;
            }
            let (max_red, min_dur, max_dur, wtp) = {
                let r = match self.resources.iter().find(|r| r.id == id) {
                    Some(r) => r,
                    None => continue,
                };
                (
                    r.max_reduction_kw,
                    r.min_duration_h,
                    r.max_duration_h,
                    r.willingness_to_pay_usd_per_kwh,
                )
            };

            let amount = remaining.min(max_red);
            // Effective dispatch duration: at least min, at most max
            let duration = 1.0_f64.max(min_dur).min(max_dur);
            let cost = amount * duration * wtp;

            signals.push(DispatchSignal {
                resource_id: id,
                start_hour: hour,
                duration_h: duration,
                requested_kw: amount,
                actual_kw: amount,
                cost_usd: cost,
                feasible: true,
            });
            remaining -= amount;
        }

        signals
    }

    /// Compute the rebound load profile (quarter-hour resolution) after a dispatch event.
    ///
    /// Uses an exponential decay model:
    ///   rebound(t) = actual_kw × recovery_factor × exp(-t / τ)
    /// where τ = recovery_time_h × 4 / 3 expressed in quarter-hour units, and t
    /// indexes quarter-hour steps from 0 to `ceil(recovery_time_h × 4)`.
    pub fn compute_rebound(&self, signal: &DispatchSignal) -> Vec<f64> {
        let r = match self.resources.iter().find(|r| r.id == signal.resource_id) {
            Some(r) => r,
            None => return Vec::new(),
        };

        let n_steps = ((r.recovery_time_h * 4.0).ceil() as usize).max(1);

        if r.recovery_factor <= 0.0 || signal.actual_kw.abs() < 1e-9 {
            return vec![0.0; n_steps];
        }

        // τ in quarter-hour units: recovery_time_h × 4 / 3
        let tau = if r.recovery_time_h > 1e-9 {
            (r.recovery_time_h * 4.0) / 3.0
        } else {
            1.0
        };

        (0..n_steps)
            .map(|t| signal.actual_kw * r.recovery_factor * (-(t as f64) / tau).exp())
            .collect()
    }

    /// Estimate aggregate load change `kW` from a price signal at a given hour.
    ///
    /// Applies own-price elasticity: ΔL = ε × L_baseline × ΔP/P_ref,
    /// where P_ref = 0.10 $/kWh.  Sums across all resources.
    /// Returns a negative value when price exceeds the reference (demand falls).
    pub fn estimate_price_response(&self, price_usd_per_kwh: f64, _hour: usize) -> f64 {
        const REF_PRICE: f64 = 0.10;
        let delta_ratio = (price_usd_per_kwh - REF_PRICE) / REF_PRICE;

        self.resources
            .iter()
            .map(|r| {
                let eps = self
                    .price_elasticity
                    .get(&r.segment)
                    .copied()
                    .unwrap_or(-0.15);
                eps * r.baseline_kw * delta_ratio
            })
            .sum()
    }

    /// Build a merit-order cost curve for a given hour.
    ///
    /// Returns `(cumulative_kw, cost_usd_per_kwh)` pairs sorted by cost ascending,
    /// representing the aggregate supply curve of curtailment flexibility.
    pub fn compute_cost_curve(&self, hour: usize) -> Vec<(f64, f64)> {
        let h_idx = if self.time_horizon_h > 0 {
            hour.min(self.time_horizon_h - 1)
        } else {
            0
        };

        let mut steps: Vec<(f64, f64)> = self
            .resources
            .iter()
            .filter_map(|r| {
                let env = self.compute_envelope(r.id);
                let qty = env.downward_flex_kw.get(h_idx).copied().unwrap_or(0.0);
                let c = env.cost_usd_per_kwh.get(h_idx).copied().unwrap_or(0.0);
                if qty > 1e-9 {
                    Some((qty, c))
                } else {
                    None
                }
            })
            .collect();

        steps.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut cumulative_kw = 0.0;
        steps
            .into_iter()
            .map(|(qty, c)| {
                cumulative_kw += qty;
                (cumulative_kw, c)
            })
            .collect()
    }

    /// Generate a 24-hour aggregate baseline load profile from all enrolled resources.
    ///
    /// Applies a diurnal load factor profile:
    /// - 00–05 h: 0.60 (night off-peak)
    /// - 06–08 h: 0.80 (morning ramp)
    /// - 09–17 h: 1.00 (business-day peak)
    /// - 18–21 h: 0.90 (evening demand)
    /// - 22–23 h: 0.70 (late evening)
    pub fn generate_daily_baseline(&self) -> Vec<f64> {
        (0..24)
            .map(|h| {
                let factor = hourly_load_factor(h);
                self.resources.iter().map(|r| r.baseline_kw * factor).sum()
            })
            .collect()
    }
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

/// Diurnal load factor for hour-of-day (0–23).
fn hourly_load_factor(h: usize) -> f64 {
    match h {
        0..=5 => 0.6,
        6..=8 => 0.8,
        9..=17 => 1.0,
        18..=21 => 0.9,
        _ => 0.7, // 22–23
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_industrial() -> FlexibilityResource {
        FlexibilityResource {
            id: 1,
            name: "Ind-1".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::InterruptibleLoad,
            baseline_kw: 500.0,
            max_reduction_kw: 400.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 8.0,
            recovery_time_h: 2.0,
            recovery_factor: 0.3,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 30.0,
            max_events_per_day: 2,
            max_events_per_year: 50,
            willingness_to_pay_usd_per_kwh: 0.15,
        }
    }

    fn make_residential() -> FlexibilityResource {
        FlexibilityResource {
            id: 2,
            name: "Res-2".to_string(),
            segment: CustomerSegment::Residential,
            flex_type: FlexibilityType::LoadCurtailment,
            baseline_kw: 3.0,
            max_reduction_kw: 1.0,
            max_increase_kw: 0.5,
            min_duration_h: 0.5,
            max_duration_h: 2.0,
            recovery_time_h: 1.0,
            recovery_factor: 0.6,
            activation_mode: ActivationMode::PriceSignal,
            notification_time_min: 5.0,
            max_events_per_day: 4,
            max_events_per_year: 200,
            willingness_to_pay_usd_per_kwh: 0.05,
        }
    }

    fn aggregator_with_two() -> FlexibilityAggregator {
        FlexibilityAggregator::new(vec![make_industrial(), make_residential()], 24)
    }

    // ── Envelope tests ────────────────────────────────────────────────────────

    #[test]
    fn test_envelope_downward_flex() {
        let agg = aggregator_with_two();
        let env = agg.compute_envelope(1);
        for (h, &d) in env.downward_flex_kw.iter().enumerate() {
            assert!(
                d <= 400.0 + 1e-9,
                "Hour {h}: downward flex {d:.2} exceeds max_reduction_kw=400"
            );
        }
    }

    #[test]
    fn test_envelope_upward_flex() {
        let ev = FlexibilityResource::ev_fleet(3);
        let agg = FlexibilityAggregator::new(vec![ev], 24);
        let env = agg.compute_envelope(3);
        // EV gets at most 1.2x max_increase_kw = 72 kW
        let max_allowed = 60.0 * 1.2 + 1e-9;
        for (h, &u) in env.upward_flex_kw.iter().enumerate() {
            assert!(
                u <= max_allowed,
                "Hour {h}: upward flex {u:.2} exceeds max_allowed={max_allowed:.2}"
            );
        }
    }

    #[test]
    fn test_envelope_length_matches_horizon() {
        let agg = FlexibilityAggregator::new(vec![make_industrial()], 48);
        let env = agg.compute_envelope(1);
        assert_eq!(env.downward_flex_kw.len(), 48);
        assert_eq!(env.upward_flex_kw.len(), 48);
        assert_eq!(env.cost_usd_per_kwh.len(), 48);
    }

    // ── Rebound tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_rebound_profile_sum() {
        let agg = aggregator_with_two();
        let sig = DispatchSignal {
            resource_id: 1,
            start_hour: 10,
            duration_h: 2.0,
            requested_kw: 100.0,
            actual_kw: 100.0,
            cost_usd: 30.0,
            feasible: true,
        };
        let rebound = agg.compute_rebound(&sig);
        let total: f64 = rebound.iter().sum();
        assert!(
            total > 0.0,
            "Rebound total should be positive, got {total:.4}"
        );
        // Upper bound: initial value * n_steps (if it didn't decay)
        let initial = 100.0 * 0.3;
        assert!(
            total < initial * rebound.len() as f64,
            "Rebound total {total:.4} suspiciously large"
        );
    }

    #[test]
    fn test_rebound_exponential_decay() {
        let res = FlexibilityResource {
            id: 10,
            name: "DecayTest".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::LoadCurtailment,
            baseline_kw: 100.0,
            max_reduction_kw: 80.0,
            max_increase_kw: 0.0,
            min_duration_h: 0.5,
            max_duration_h: 4.0,
            recovery_time_h: 3.0,
            recovery_factor: 0.5,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 10.0,
            max_events_per_day: 3,
            max_events_per_year: 100,
            willingness_to_pay_usd_per_kwh: 0.12,
        };
        let agg = FlexibilityAggregator::new(vec![res], 24);
        let sig = DispatchSignal {
            resource_id: 10,
            start_hour: 8,
            duration_h: 1.0,
            requested_kw: 60.0,
            actual_kw: 60.0,
            cost_usd: 7.2,
            feasible: true,
        };
        let rebound = agg.compute_rebound(&sig);
        assert!(
            rebound.len() >= 2,
            "Rebound profile too short: {}",
            rebound.len()
        );
        for i in 1..rebound.len() {
            assert!(
                rebound[i] <= rebound[i - 1] + 1e-9,
                "Rebound not monotonically decreasing at step {i}: {} > {}",
                rebound[i],
                rebound[i - 1]
            );
        }
    }

    #[test]
    fn test_rebound_zero_when_no_recovery_factor() {
        let res = FlexibilityResource {
            id: 11,
            name: "NoRebound".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::LoadShed,
            baseline_kw: 200.0,
            max_reduction_kw: 150.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 6.0,
            recovery_time_h: 2.0,
            recovery_factor: 0.0,
            activation_mode: ActivationMode::Manual,
            notification_time_min: 60.0,
            max_events_per_day: 1,
            max_events_per_year: 20,
            willingness_to_pay_usd_per_kwh: 0.20,
        };
        let agg = FlexibilityAggregator::new(vec![res], 24);
        let sig = DispatchSignal {
            resource_id: 11,
            start_hour: 12,
            duration_h: 2.0,
            requested_kw: 100.0,
            actual_kw: 100.0,
            cost_usd: 40.0,
            feasible: true,
        };
        let rebound = agg.compute_rebound(&sig);
        let total: f64 = rebound.iter().sum();
        assert!(
            total.abs() < 1e-9,
            "Expected zero rebound for recovery_factor=0, got {total:.6}"
        );
    }

    // ── Merit order / dispatch tests ──────────────────────────────────────────

    #[test]
    fn test_merit_order_sort() {
        let cheap = FlexibilityResource {
            id: 20,
            name: "Cheap".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::InterruptibleLoad,
            baseline_kw: 100.0,
            max_reduction_kw: 80.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 4.0,
            recovery_time_h: 1.0,
            recovery_factor: 0.2,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 15.0,
            max_events_per_day: 3,
            max_events_per_year: 60,
            willingness_to_pay_usd_per_kwh: 0.05,
        };
        let expensive = FlexibilityResource {
            id: 21,
            name: "Expensive".to_string(),
            segment: CustomerSegment::Residential,
            flex_type: FlexibilityType::PeakClipping,
            baseline_kw: 5.0,
            max_reduction_kw: 3.0,
            max_increase_kw: 0.0,
            min_duration_h: 0.5,
            max_duration_h: 2.0,
            recovery_time_h: 0.5,
            recovery_factor: 0.5,
            activation_mode: ActivationMode::PriceSignal,
            notification_time_min: 5.0,
            max_events_per_day: 4,
            max_events_per_year: 200,
            willingness_to_pay_usd_per_kwh: 0.30,
        };
        // Add expensive before cheap — dispatch must still serve cheap first
        let mut agg = FlexibilityAggregator::new(vec![expensive, cheap], 24);
        let signals = agg.aggregate_dispatch(50.0, 10);
        assert!(!signals.is_empty(), "Expected at least one dispatch signal");
        assert_eq!(
            signals[0].resource_id, 20,
            "Cheapest resource (id=20) must be dispatched first"
        );
    }

    #[test]
    fn test_aggregate_dispatch_target() {
        let mut agg = aggregator_with_two();
        let signals = agg.aggregate_dispatch(200.0, 8);
        let total: f64 = signals.iter().map(|s| s.actual_kw).sum();
        // Available: 400 + 1 = 401 kW; target = 200 → should be exactly met
        assert!(
            (total - 200.0).abs() < 1.0,
            "Expected ~200 kW dispatched, got {total:.2}"
        );
    }

    #[test]
    fn test_dispatch_feasibility_max_reduction() {
        let agg = aggregator_with_two();
        let sig = DispatchSignal {
            resource_id: 1,
            start_hour: 10,
            duration_h: 2.0,
            requested_kw: 9999.0,
            actual_kw: 0.0,
            cost_usd: 0.0,
            feasible: false,
        };
        let result = agg.dispatch(1, &sig);
        assert!(
            result.actual_kw <= 400.0 + 1e-9,
            "actual_kw {} exceeds max_reduction_kw=400",
            result.actual_kw
        );
        assert!(result.feasible, "Should be feasible with clamped power");
    }

    // ── Cost curve tests ──────────────────────────────────────────────────────

    #[test]
    fn test_cost_curve_monotone() {
        let agg = aggregator_with_two();
        let curve = agg.compute_cost_curve(10);
        for i in 1..curve.len() {
            assert!(
                curve[i].1 >= curve[i - 1].1 - 1e-9,
                "Cost curve not monotone at index {i}: {:.4} < {:.4}",
                curve[i].1,
                curve[i - 1].1
            );
        }
    }

    #[test]
    fn test_cost_curve_non_empty() {
        let agg = aggregator_with_two();
        let curve = agg.compute_cost_curve(10);
        assert!(
            !curve.is_empty(),
            "Cost curve should have at least one entry"
        );
    }

    // ── Portfolio aggregation tests ───────────────────────────────────────────

    #[test]
    fn test_portfolio_aggregation() {
        let agg = aggregator_with_two();
        let portfolio = agg.aggregate_portfolio();
        let expected_baseline = 500.0 + 3.0;
        assert!(
            (portfolio.total_baseline_kw - expected_baseline).abs() < 1e-9,
            "Expected total baseline {expected_baseline:.2}, got {:.2}",
            portfolio.total_baseline_kw
        );
    }

    #[test]
    fn test_portfolio_max_reduction() {
        let agg = aggregator_with_two();
        let portfolio = agg.aggregate_portfolio();
        let expected = 400.0 + 1.0;
        assert!(
            (portfolio.total_max_reduction_kw - expected).abs() < 1e-9,
            "Expected total max_reduction {expected:.2}, got {:.2}",
            portfolio.total_max_reduction_kw
        );
    }

    // ── Price response tests ──────────────────────────────────────────────────

    #[test]
    fn test_price_response_negative() {
        // Price > ref_price (0.10) → demand decreases → negative delta
        let agg = aggregator_with_two();
        let delta = agg.estimate_price_response(0.30, 10);
        assert!(
            delta < 0.0,
            "Higher price should yield negative load change, got {delta:.4}"
        );
    }

    #[test]
    fn test_price_response_magnitude() {
        let r = FlexibilityResource {
            id: 30,
            name: "ElasticTest".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::PeakClipping,
            baseline_kw: 100.0,
            max_reduction_kw: 80.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 4.0,
            recovery_time_h: 1.0,
            recovery_factor: 0.0,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 15.0,
            max_events_per_day: 3,
            max_events_per_year: 60,
            willingness_to_pay_usd_per_kwh: 0.12,
        };
        let mut agg = FlexibilityAggregator::new(vec![r], 24);
        // LargeIndustrial default elasticity = -0.5
        let delta1 = agg.estimate_price_response(0.20, 0);

        // Double the elasticity → double the response
        agg.price_elasticity
            .insert(CustomerSegment::LargeIndustrial, -1.0);
        let delta2 = agg.estimate_price_response(0.20, 0);

        assert!(
            (delta2 / delta1 - 2.0).abs() < 1e-9,
            "Doubling elasticity should double response: ratio={:.6}",
            delta2 / delta1
        );
    }

    // ── Segment-specific tests ────────────────────────────────────────────────

    #[test]
    fn test_industrial_segment_high_flex() {
        let ind = make_industrial();
        let res = make_residential();
        assert!(
            ind.max_reduction_kw > res.max_reduction_kw,
            "Industrial max_reduction_kw ({:.1}) should exceed residential ({:.1})",
            ind.max_reduction_kw,
            res.max_reduction_kw
        );
    }

    #[test]
    fn test_residential_segment_low_flex() {
        let res = make_residential();
        assert!(
            res.baseline_kw < 10.0,
            "Residential baseline should be small, got {:.2} kW",
            res.baseline_kw
        );
        assert!(
            res.max_reduction_kw <= res.baseline_kw,
            "Residential max_reduction ({:.2}) must not exceed baseline ({:.2})",
            res.max_reduction_kw,
            res.baseline_kw
        );
    }

    // ── Multi-resource dispatch merit order ───────────────────────────────────

    #[test]
    fn test_dispatch_multiple_resources() {
        let r1 = FlexibilityResource {
            id: 40,
            name: "R1".to_string(),
            segment: CustomerSegment::LargeIndustrial,
            flex_type: FlexibilityType::InterruptibleLoad,
            baseline_kw: 200.0,
            max_reduction_kw: 150.0,
            max_increase_kw: 0.0,
            min_duration_h: 1.0,
            max_duration_h: 6.0,
            recovery_time_h: 1.0,
            recovery_factor: 0.2,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 10.0,
            max_events_per_day: 2,
            max_events_per_year: 50,
            willingness_to_pay_usd_per_kwh: 0.10,
        };
        let r2 = FlexibilityResource {
            id: 41,
            name: "R2".to_string(),
            segment: CustomerSegment::LargeCommercial,
            flex_type: FlexibilityType::DirectLoadControl,
            baseline_kw: 100.0,
            max_reduction_kw: 80.0,
            max_increase_kw: 0.0,
            min_duration_h: 0.5,
            max_duration_h: 4.0,
            recovery_time_h: 0.5,
            recovery_factor: 0.3,
            activation_mode: ActivationMode::Automatic,
            notification_time_min: 5.0,
            max_events_per_day: 3,
            max_events_per_year: 100,
            willingness_to_pay_usd_per_kwh: 0.20,
        };
        let r3 = FlexibilityResource {
            id: 42,
            name: "R3".to_string(),
            segment: CustomerSegment::SmallCommercial,
            flex_type: FlexibilityType::DemandBidding,
            baseline_kw: 30.0,
            max_reduction_kw: 20.0,
            max_increase_kw: 0.0,
            min_duration_h: 0.5,
            max_duration_h: 2.0,
            recovery_time_h: 0.5,
            recovery_factor: 0.4,
            activation_mode: ActivationMode::PriceSignal,
            notification_time_min: 0.0,
            max_events_per_day: 4,
            max_events_per_year: 200,
            willingness_to_pay_usd_per_kwh: 0.30,
        };
        // Provide resources in non-merit-order (r3 first)
        let mut agg = FlexibilityAggregator::new(vec![r3, r2, r1], 24);
        let signals = agg.aggregate_dispatch(200.0, 12);
        // Verify costs are non-decreasing in dispatch order
        let wtp = |id: usize| -> f64 {
            agg.resources
                .iter()
                .find(|r| r.id == id)
                .map(|r| r.willingness_to_pay_usd_per_kwh)
                .unwrap_or(0.0)
        };
        for i in 1..signals.len() {
            let c_prev = wtp(signals[i - 1].resource_id);
            let c_curr = wtp(signals[i].resource_id);
            assert!(
                c_curr >= c_prev - 1e-9,
                "Merit order violated at index {i}: wtp={c_curr:.3} < {c_prev:.3}"
            );
        }
    }

    // ── Operational constraint tests ──────────────────────────────────────────

    #[test]
    fn test_event_limit_constraint() {
        // Verify max_events_per_day and max_events_per_year fields are accessible
        let r = make_industrial();
        assert_eq!(
            r.max_events_per_day, 2,
            "Expected 2 events/day for industrial"
        );
        assert_eq!(
            r.max_events_per_year, 50,
            "Expected 50 events/year for industrial"
        );
    }

    #[test]
    fn test_notification_time_check() {
        let ind = make_industrial();
        assert!(
            ind.notification_time_min > 0.0,
            "Industrial resource should require advance notice, got {:.1} min",
            ind.notification_time_min
        );
        let ev = FlexibilityResource::ev_fleet(99);
        assert!(
            ev.notification_time_min < 1e-9,
            "EV fleet should need zero advance notice"
        );
    }

    #[test]
    fn test_min_duration_constraint() {
        let agg = aggregator_with_two();
        // Industrial min_duration_h = 1.0; request 0.1 h → infeasible
        let sig = DispatchSignal {
            resource_id: 1,
            start_hour: 10,
            duration_h: 0.1,
            requested_kw: 100.0,
            actual_kw: 0.0,
            cost_usd: 0.0,
            feasible: false,
        };
        let result = agg.dispatch(1, &sig);
        assert!(
            !result.feasible,
            "Dispatch below min_duration_h should be infeasible"
        );
        assert!(
            result.actual_kw.abs() < 1e-9,
            "actual_kw should be 0 for infeasible dispatch, got {:.4}",
            result.actual_kw
        );
    }

    #[test]
    fn test_max_duration_constraint() {
        let agg = aggregator_with_two();
        // Industrial max_duration_h = 8.0; request 100 h → clamped
        let sig = DispatchSignal {
            resource_id: 1,
            start_hour: 10,
            duration_h: 100.0,
            requested_kw: 100.0,
            actual_kw: 0.0,
            cost_usd: 0.0,
            feasible: false,
        };
        let result = agg.dispatch(1, &sig);
        assert!(
            result.duration_h <= 8.0 + 1e-9,
            "Duration should be clamped to max_duration_h=8.0, got {:.2}",
            result.duration_h
        );
        assert!(
            result.feasible,
            "Should still be feasible after duration clamp"
        );
    }

    // ── Valley filling test ───────────────────────────────────────────────────

    #[test]
    fn test_valley_filling_positive() {
        let vf = FlexibilityResource {
            id: 50,
            name: "VF-Battery".to_string(),
            segment: CustomerSegment::BatteryStorage,
            flex_type: FlexibilityType::ValleyFilling,
            baseline_kw: 0.0,
            max_reduction_kw: 0.0,
            max_increase_kw: 100.0,
            min_duration_h: 0.5,
            max_duration_h: 4.0,
            recovery_time_h: 0.0,
            recovery_factor: 0.0,
            activation_mode: ActivationMode::Scheduled,
            notification_time_min: 0.0,
            max_events_per_day: 4,
            max_events_per_year: 365,
            willingness_to_pay_usd_per_kwh: 0.02,
        };
        assert_eq!(vf.flex_type, FlexibilityType::ValleyFilling);
        assert!(
            vf.max_increase_kw > 0.0,
            "ValleyFilling resource should have positive max_increase_kw"
        );
        let agg = FlexibilityAggregator::new(vec![vf], 24);
        let env = agg.compute_envelope(50);
        let any_positive = env.upward_flex_kw.iter().any(|&u| u > 0.0);
        assert!(
            any_positive,
            "ValleyFilling envelope should have positive upward flex hours"
        );
    }

    // ── Daily baseline test ───────────────────────────────────────────────────

    #[test]
    fn test_daily_baseline_hours() {
        let agg = aggregator_with_two();
        let baseline = agg.generate_daily_baseline();
        assert_eq!(
            baseline.len(),
            24,
            "Daily baseline should return exactly 24 values, got {}",
            baseline.len()
        );
    }

    #[test]
    fn test_daily_baseline_positive() {
        let agg = aggregator_with_two();
        let baseline = agg.generate_daily_baseline();
        for (h, &v) in baseline.iter().enumerate() {
            assert!(
                v > 0.0,
                "Baseline at hour {h} should be positive, got {v:.4}"
            );
        }
    }
}
