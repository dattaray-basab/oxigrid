//! Demand Response Aggregator (RRR).
//!
//! Coordinates multiple demand-response programs across a portfolio of
//! enrolled customers. Supports direct load control, interruptible load,
//! automated OpenADR, time-of-use, peak-time-rebate, and capacity-auction
//! programs.
//!
//! # Key operations
//!
//! - Enroll and manage DR customers
//! - Dispatch DR events (select, notify, simulate response with LCG noise)
//! - Compute portfolio capacity \[MW\] and forecast response
//! - Track historical performance

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors from the DR aggregator.
#[derive(Debug, Error)]
pub enum DrError {
    /// No customers enrolled.
    #[error("no customers enrolled in aggregator {0}")]
    NoCustomers(usize),
    /// Target reduction could not be fully met.
    #[error("insufficient DR capacity: requested {requested:.2} MW, available {available:.2} MW")]
    InsufficientCapacity { requested: f64, available: f64 },
    /// A customer with the given ID is already enrolled.
    #[error("customer {0} is already enrolled")]
    AlreadyEnrolled(usize),
    /// Negative or zero target reduction provided.
    #[error("invalid target reduction {0:.2} MW (must be positive)")]
    InvalidTarget(f64),
    /// Notification lead time exceeds allowed value.
    #[error("notification lead time {0:.2} h exceeds configured limit {1:.2} h")]
    LeadTimeExceeded(f64, f64),
}

// ── Program and customer types ────────────────────────────────────────────────

/// Type of demand-response program.
#[derive(Debug, Clone, PartialEq)]
pub enum DrProgramType {
    /// Aggregator directly controls customer loads.
    DirectLoadControl,
    /// Customer curtails consumption on aggregator signal.
    InterruptibleLoad,
    /// Automated demand response via OpenADR protocol.
    AutoDemandResponse,
    /// Customer responds to time-of-use price signals.
    TimeOfUse,
    /// Customer earns rebates for reducing peak demand.
    PeakTimRebates,
    /// Customer commits day-ahead capacity in an auction.
    CapacityAuction,
}

/// Method used to establish the customer's load baseline.
#[derive(Debug, Clone, PartialEq)]
pub enum BaselineMethod {
    /// Average of matching-day 10-of-10 window (same day type, past 10 days).
    DayMatching10of10,
    /// Adjusted 3 of 5 highest days in past 10-day window.
    Adjusted3of5,
    /// Regression-adjusted baseline on outdoor temperature.
    WeatherNormalized,
    /// Individual customer-specific baseline model.
    CustomerSpecific,
}

/// Market segment for a DR customer.
#[derive(Debug, Clone, PartialEq)]
pub enum CustomerSegment {
    /// Large industrial (>1 MW) load.
    Industrial,
    /// Commercial building / retail.
    Commercial,
    /// Residential (aggregated).
    Residential,
    /// Agricultural pumping / irrigation.
    Agricultural,
}

// ── Customer ──────────────────────────────────────────────────────────────────

/// An enrolled demand-response customer.
pub struct DrCustomer {
    /// Unique customer identifier.
    pub id: usize,
    /// Customer name or site label.
    pub name: String,
    /// Market segment classification.
    pub segment: CustomerSegment,
    /// Programs this customer is enrolled in.
    pub enrolled_programs: Vec<DrProgramType>,
    /// Hourly baseline load profile \[kW\].
    pub baseline_load_kw: Vec<f64>,
    /// Maximum achievable load reduction \[kW\].
    pub dr_potential_kw: f64,
    /// Latency from notification to load response \[min\].
    pub response_time_min: f64,
    /// Historical compliance rate \[0–1\].
    pub reliability_pct: f64,
    /// Notification delivery channel (email, SMS, API, etc.).
    pub notification_preference: String,
}

impl DrCustomer {
    /// Compute expected reduction contribution \[kW\] accounting for
    /// reliability.
    pub fn expected_reduction_kw(&self) -> f64 {
        self.dr_potential_kw * self.reliability_pct
    }
}

// ── Event ─────────────────────────────────────────────────────────────────────

/// A demand-response dispatch event.
pub struct DrEvent {
    /// Unique event identifier.
    pub event_id: u64,
    /// DR program being activated.
    pub program: DrProgramType,
    /// Aggregate load reduction target \[MW\].
    pub target_reduction_mw: f64,
    /// Event duration \[h\].
    pub duration_h: f64,
    /// Starting hour of the event (0-based).
    pub start_hour: usize,
    /// Advance notification time provided \[h\].
    pub notification_time_h: f64,
    /// Customer incentive payment \[USD/kW\].
    pub incentive_usd_per_kw: f64,
    /// Optional real-time price signal \[USD/kWh\].
    pub price_signal_usd_per_kwh: Option<f64>,
}

// ── Event result ──────────────────────────────────────────────────────────────

/// Outcome of a dispatched DR event.
pub struct DrEventResult {
    /// Event identifier.
    pub event_id: u64,
    /// Actual aggregate load reduction achieved \[MW\].
    pub actual_reduction_mw: f64,
    /// Number of customers selected for the event.
    pub enrolled_customers: usize,
    /// Number of customers that responded (reliability ≥ random draw).
    pub responding_customers: usize,
    /// Compliance rate across selected customers \[%\].
    pub compliance_rate_pct: f64,
    /// Total incentive payments made \[USD\].
    pub total_payment_usd: f64,
    /// All-in cost per MW of achieved reduction \[USD/MW\].
    pub cost_per_mw_usd: f64,
    /// Accuracy of baseline estimates \[%\] (simulated as function of method).
    pub baseline_accuracy_pct: f64,
    /// Energy curtailed over the event window \[MWh\].
    pub curtailed_energy_mwh: f64,
}

// ── Aggregator config ─────────────────────────────────────────────────────────

/// Configuration for the `DrAggregator`.
pub struct DrAggregatorConfig {
    /// Total customers served by the aggregator's utility territory.
    pub n_customers: usize,
    /// Aggregator identifier.
    pub aggregator_id: usize,
    /// Programs offered by this aggregator.
    pub program_types: Vec<DrProgramType>,
    /// Minimum required notification lead time for events \[h\].
    pub notification_lead_time_h: f64,
    /// Nominal event duration \[h\].
    pub dispatch_duration_h: f64,
    /// Maximum number of DR events per calendar month.
    pub max_events_per_month: usize,
    /// Default baseline computation method.
    pub baseline_method: BaselineMethod,
}

// ── Performance summary ────────────────────────────────────────────────────────

/// Aggregate statistics across all historical DR events.
pub struct DrPerformanceSummary {
    /// Total number of events dispatched.
    pub n_events: usize,
    /// Average customer compliance rate \[%\].
    pub avg_compliance_pct: f64,
    /// Cumulative energy curtailed \[MWh\].
    pub total_curtailed_mwh: f64,
    /// Cumulative incentive payments \[USD\].
    pub total_cost_usd: f64,
    /// Average cost per MW of reduction across all events \[USD/MW\].
    pub avg_cost_per_mw_usd: f64,
}

// ── LCG helper ────────────────────────────────────────────────────────────────

/// Linear congruential generator state for reproducible noise.
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Advance the LCG and return next pseudo-random `f64` in `[0.0, 1.0)`.
    fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005_u64)
            .wrapping_add(1_442_695_040_888_963_407_u64);
        // Use upper 32 bits for float.
        (self.state >> 32) as f64 / u32::MAX as f64
    }
}

// ── Aggregator ────────────────────────────────────────────────────────────────

/// Demand response aggregator managing a portfolio of enrolled customers.
pub struct DrAggregator {
    config: DrAggregatorConfig,
    customers: Vec<DrCustomer>,
    event_history: Vec<(DrEvent, DrEventResult)>,
}

impl DrAggregator {
    /// Create an empty aggregator with the given configuration.
    pub fn new(config: DrAggregatorConfig) -> Self {
        Self {
            config,
            customers: Vec::new(),
            event_history: Vec::new(),
        }
    }

    /// Enroll a customer in the aggregator portfolio.
    ///
    /// # Errors
    ///
    /// Returns [`DrError::AlreadyEnrolled`] if a customer with the same ID
    /// is already in the portfolio.
    pub fn enroll_customer(&mut self, customer: DrCustomer) -> Result<(), DrError> {
        if self.customers.iter().any(|c| c.id == customer.id) {
            return Err(DrError::AlreadyEnrolled(customer.id));
        }
        self.customers.push(customer);
        Ok(())
    }

    /// Dispatch a DR event: select customers, simulate response, record result.
    ///
    /// # Errors
    ///
    /// - [`DrError::NoCustomers`] — no customers enrolled.
    /// - [`DrError::InvalidTarget`] — target ≤ 0.
    /// - [`DrError::LeadTimeExceeded`] — notification lead time is too short.
    /// - [`DrError::InsufficientCapacity`] — enrolled portfolio cannot meet target.
    pub fn dispatch_event(&mut self, event: DrEvent) -> Result<DrEventResult, DrError> {
        if self.customers.is_empty() {
            return Err(DrError::NoCustomers(self.config.aggregator_id));
        }
        if event.target_reduction_mw <= 0.0 {
            return Err(DrError::InvalidTarget(event.target_reduction_mw));
        }
        if event.notification_time_h < self.config.notification_lead_time_h {
            return Err(DrError::LeadTimeExceeded(
                event.notification_time_h,
                self.config.notification_lead_time_h,
            ));
        }

        let target_kw = event.target_reduction_mw * 1000.0;

        // 1. Filter eligible customers.
        //    Eligible: enrolled in the event's program AND response time fits
        //    within notification window.
        let max_response_min = event.notification_time_h * 60.0;
        let eligible: Vec<&DrCustomer> = self
            .customers
            .iter()
            .filter(|c| {
                c.enrolled_programs.contains(&event.program)
                    && c.response_time_min <= max_response_min
                    && c.dr_potential_kw > 0.0
            })
            .collect();

        // 2. Sort by reliability descending (most reliable first).
        let mut eligible_sorted = eligible;
        eligible_sorted.sort_by(|a, b| {
            b.reliability_pct
                .partial_cmp(&a.reliability_pct)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // 3. Select customers until target is met.
        let mut selected: Vec<&DrCustomer> = Vec::new();
        let mut committed_kw = 0.0;
        for c in &eligible_sorted {
            selected.push(c);
            committed_kw += c.expected_reduction_kw();
            if committed_kw >= target_kw {
                break;
            }
        }

        if committed_kw < target_kw - 1e-6 {
            return Err(DrError::InsufficientCapacity {
                requested: event.target_reduction_mw,
                available: committed_kw / 1000.0,
            });
        }

        // 4. Simulate actual response with LCG noise.
        //    Each customer responds with probability = reliability_pct.
        let mut lcg = Lcg::new(event.event_id ^ 0xDEAD_BEEF_CAFE_1234);
        let mut actual_kw = 0.0;
        let mut responding = 0usize;
        let mut total_payment_usd = 0.0;

        for c in &selected {
            let draw = lcg.next_f64();
            if draw < c.reliability_pct {
                // Customer responds: scale by a small noise factor [0.9, 1.1].
                let noise = 0.9 + 0.2 * lcg.next_f64();
                let contributed = (c.dr_potential_kw * noise).max(0.0);
                actual_kw += contributed;
                total_payment_usd += contributed * event.incentive_usd_per_kw;
                responding += 1;
            }
        }

        let enrolled_customers = selected.len();
        let compliance_rate_pct = if enrolled_customers > 0 {
            100.0 * responding as f64 / enrolled_customers as f64
        } else {
            0.0
        };

        let actual_mw = actual_kw / 1000.0;
        let cost_per_mw_usd = if actual_mw > 1e-6 {
            total_payment_usd / actual_mw
        } else {
            0.0
        };

        // Baseline accuracy depends on method (simulated heuristic).
        let baseline_accuracy_pct = match self.config.baseline_method {
            BaselineMethod::DayMatching10of10 => 92.0,
            BaselineMethod::Adjusted3of5 => 88.0,
            BaselineMethod::WeatherNormalized => 94.0,
            BaselineMethod::CustomerSpecific => 96.0,
        };

        let curtailed_energy_mwh = actual_mw * event.duration_h;

        let result = DrEventResult {
            event_id: event.event_id,
            actual_reduction_mw: actual_mw,
            enrolled_customers,
            responding_customers: responding,
            compliance_rate_pct,
            total_payment_usd,
            cost_per_mw_usd,
            baseline_accuracy_pct,
            curtailed_energy_mwh,
        };

        self.event_history.push((event, result));
        // Return a view of the last result.
        let last = self.event_history.last().expect("just pushed");
        Ok(DrEventResult {
            event_id: last.1.event_id,
            actual_reduction_mw: last.1.actual_reduction_mw,
            enrolled_customers: last.1.enrolled_customers,
            responding_customers: last.1.responding_customers,
            compliance_rate_pct: last.1.compliance_rate_pct,
            total_payment_usd: last.1.total_payment_usd,
            cost_per_mw_usd: last.1.cost_per_mw_usd,
            baseline_accuracy_pct: last.1.baseline_accuracy_pct,
            curtailed_energy_mwh: last.1.curtailed_energy_mwh,
        })
    }

    /// Total available DR capacity across all enrolled customers \[MW\].
    ///
    /// This is the sum of `dr_potential_kw` (nameplate, not reliability-weighted).
    pub fn portfolio_capacity_mw(&self) -> f64 {
        self.customers
            .iter()
            .map(|c| c.dr_potential_kw)
            .sum::<f64>()
            / 1000.0
    }

    /// Forecast expected portfolio response \[MW\] for a given target.
    ///
    /// Selects customers greedily (highest reliability first) until the target
    /// is met or the portfolio is exhausted. Returns the reliability-weighted
    /// capacity of selected customers.
    pub fn forecast_response(&self, target_mw: f64) -> f64 {
        let target_kw = target_mw * 1000.0;
        let mut sorted = self.customers.iter().collect::<Vec<_>>();
        sorted.sort_by(|a, b| {
            b.reliability_pct
                .partial_cmp(&a.reliability_pct)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut forecast_kw = 0.0;
        let mut committed = 0.0;
        for c in sorted {
            if committed >= target_kw {
                break;
            }
            let expected = c.expected_reduction_kw();
            forecast_kw += expected;
            committed += c.dr_potential_kw;
        }
        (forecast_kw / 1000.0).min(target_mw)
    }

    /// Compute performance statistics from all historical events.
    pub fn performance_summary(&self) -> DrPerformanceSummary {
        let n_events = self.event_history.len();
        if n_events == 0 {
            return DrPerformanceSummary {
                n_events: 0,
                avg_compliance_pct: 0.0,
                total_curtailed_mwh: 0.0,
                total_cost_usd: 0.0,
                avg_cost_per_mw_usd: 0.0,
            };
        }

        let avg_compliance_pct = self
            .event_history
            .iter()
            .map(|(_, r)| r.compliance_rate_pct)
            .sum::<f64>()
            / n_events as f64;
        let total_curtailed_mwh: f64 = self
            .event_history
            .iter()
            .map(|(_, r)| r.curtailed_energy_mwh)
            .sum();
        let total_cost_usd: f64 = self
            .event_history
            .iter()
            .map(|(_, r)| r.total_payment_usd)
            .sum();
        let avg_cost_per_mw_usd = if n_events > 0 {
            self.event_history
                .iter()
                .map(|(_, r)| r.cost_per_mw_usd)
                .sum::<f64>()
                / n_events as f64
        } else {
            0.0
        };

        DrPerformanceSummary {
            n_events,
            avg_compliance_pct,
            total_curtailed_mwh,
            total_cost_usd,
            avg_cost_per_mw_usd,
        }
    }

    /// Reference to the enrolled customer list.
    pub fn customers(&self) -> &[DrCustomer] {
        &self.customers
    }

    /// Reference to the event history.
    pub fn event_history(&self) -> &[(DrEvent, DrEventResult)] {
        &self.event_history
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> DrAggregatorConfig {
        DrAggregatorConfig {
            n_customers: 100,
            aggregator_id: 1,
            program_types: vec![DrProgramType::DirectLoadControl],
            notification_lead_time_h: 0.5,
            dispatch_duration_h: 2.0,
            max_events_per_month: 10,
            baseline_method: BaselineMethod::DayMatching10of10,
        }
    }

    fn make_customer(id: usize, potential_kw: f64, reliability: f64) -> DrCustomer {
        DrCustomer {
            id,
            name: format!("Customer-{id}"),
            segment: CustomerSegment::Commercial,
            enrolled_programs: vec![DrProgramType::DirectLoadControl],
            baseline_load_kw: vec![100.0; 24],
            dr_potential_kw: potential_kw,
            response_time_min: 10.0,
            reliability_pct: reliability,
            notification_preference: String::from("email"),
        }
    }

    fn make_event(target_mw: f64) -> DrEvent {
        DrEvent {
            event_id: 42,
            program: DrProgramType::DirectLoadControl,
            target_reduction_mw: target_mw,
            duration_h: 2.0,
            start_hour: 14,
            notification_time_h: 1.0,
            incentive_usd_per_kw: 50.0,
            price_signal_usd_per_kwh: None,
        }
    }

    /// Enrolled customer appears in portfolio.
    #[test]
    fn test_enrollment_adds_customer() {
        let mut agg = DrAggregator::new(default_config());
        agg.enroll_customer(make_customer(1, 500.0, 0.9))
            .expect("enrollment should succeed");
        assert_eq!(agg.customers().len(), 1, "Customer count must be 1");
        assert_eq!(agg.customers()[0].id, 1);
    }

    /// Duplicate enrollment returns AlreadyEnrolled error.
    #[test]
    fn test_duplicate_enrollment_error() {
        let mut agg = DrAggregator::new(default_config());
        agg.enroll_customer(make_customer(1, 500.0, 0.9))
            .expect("first enrollment ok");
        let result = agg.enroll_customer(make_customer(1, 300.0, 0.8));
        assert!(
            matches!(result, Err(DrError::AlreadyEnrolled(1))),
            "Expected AlreadyEnrolled error"
        );
    }

    /// Portfolio capacity is the sum of dr_potential_kw in MW.
    #[test]
    fn test_portfolio_capacity() {
        let mut agg = DrAggregator::new(default_config());
        agg.enroll_customer(make_customer(1, 1000.0, 0.9))
            .expect("ok");
        agg.enroll_customer(make_customer(2, 2000.0, 0.8))
            .expect("ok");
        let cap = agg.portfolio_capacity_mw();
        assert!(
            (cap - 3.0).abs() < 1e-9,
            "Portfolio capacity should be 3.0 MW, got {:.4}",
            cap
        );
    }

    /// Event dispatch achieves non-zero reduction when customers are reliable.
    #[test]
    fn test_event_dispatch_reduction() {
        let mut agg = DrAggregator::new(default_config());
        // Enroll many reliable customers with 500 kW each.
        for i in 0..10 {
            agg.enroll_customer(make_customer(i, 500.0, 1.0))
                .expect("ok");
        }
        // Target 2 MW: 4 customers × 500 kW each.
        let result = agg.dispatch_event(make_event(2.0)).expect("dispatch ok");
        assert!(
            result.actual_reduction_mw > 0.0,
            "Actual reduction must be positive, got {:.4}",
            result.actual_reduction_mw
        );
        assert!(
            result.enrolled_customers >= 4,
            "At least 4 customers needed for 2 MW"
        );
    }

    /// Compliance rate is computed correctly from responding / enrolled.
    #[test]
    fn test_compliance_rate_computed() {
        let mut agg = DrAggregator::new(default_config());
        // Use reliability 1.0 → all respond → 100% compliance.
        for i in 0..5 {
            agg.enroll_customer(make_customer(i, 600.0, 1.0))
                .expect("ok");
        }
        let result = agg.dispatch_event(make_event(2.0)).expect("dispatch ok");
        // With reliability=1.0 and LCG draw < 1.0 always → all respond.
        assert!(
            result.compliance_rate_pct > 50.0,
            "Compliance with reliable customers must be > 50%, got {:.2}%",
            result.compliance_rate_pct
        );
    }

    /// Performance summary matches individual event data.
    #[test]
    fn test_performance_summary() {
        let mut agg = DrAggregator::new(default_config());
        for i in 0..8 {
            agg.enroll_customer(make_customer(i, 500.0, 1.0))
                .expect("ok");
        }
        // Dispatch two events.
        let mut ev1 = make_event(1.0);
        ev1.event_id = 1;
        let mut ev2 = make_event(1.5);
        ev2.event_id = 2;

        let r1 = agg.dispatch_event(ev1).expect("event 1 ok");
        let r2 = agg.dispatch_event(ev2).expect("event 2 ok");

        let summary = agg.performance_summary();
        assert_eq!(summary.n_events, 2, "Must have 2 events");

        let expected_mwh = r1.curtailed_energy_mwh + r2.curtailed_energy_mwh;
        assert!(
            (summary.total_curtailed_mwh - expected_mwh).abs() < 1e-9,
            "Curtailed MWh mismatch: expected {:.4}, got {:.4}",
            expected_mwh,
            summary.total_curtailed_mwh
        );
    }

    /// Forecast response is non-zero for a feasible portfolio.
    #[test]
    fn test_forecast_response_feasible() {
        let mut agg = DrAggregator::new(default_config());
        for i in 0..5 {
            agg.enroll_customer(make_customer(i, 1000.0, 0.9))
                .expect("ok");
        }
        let forecast = agg.forecast_response(2.0);
        assert!(
            forecast > 0.0,
            "Forecast must be positive for 5 MW portfolio targeting 2 MW"
        );
        assert!(
            forecast <= 2.0 + 1e-9,
            "Forecast must not exceed target, got {:.4}",
            forecast
        );
    }

    /// Insufficient capacity returns error.
    #[test]
    fn test_insufficient_capacity_error() {
        let mut agg = DrAggregator::new(default_config());
        agg.enroll_customer(make_customer(1, 100.0, 0.9))
            .expect("ok");
        // Target 10 MW with only 0.1 MW enrolled.
        let result = agg.dispatch_event(make_event(10.0));
        assert!(
            matches!(result, Err(DrError::InsufficientCapacity { .. })),
            "Expected InsufficientCapacity error"
        );
    }
}
