//! Demand response market integration and comprehensive DR program management.
//!
//! # Overview
//!
//! Demand response (DR) resources can participate in electricity markets
//! by offering load curtailment or shifting at a price. This module provides:
//!
//! - Basic DR offer/portfolio clearing ([`DrPortfolio`], [`DrOffer`])
//! - Comprehensive DR program management ([`DrProgramPortfolio`], [`DrCustomer`])
//! - Price-responsive demand elasticity (TOU/RTP)
//! - Baseline computation (D+1 same-hour average)
//! - Rebound load estimation
//! - VOLL (Value of Lost Load) estimation
//! - Cost-benefit analysis
//!
//! # References
//! - Albadi, M.H. & El-Saadany, E.F., "A summary of demand response in electricity markets",
//!   Electric Power Systems Research, 78(11), 1989-1996, 2008
//! - FERC, "Assessment of Demand Response and Advanced Metering", 2022

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// DR program types
// ─────────────────────────────────────────────────────────────────────────────

/// Types of demand response programs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrProgramType {
    /// Direct load control (utility cycles equipment on/off).
    DirectLoadControl,
    /// Interruptible / curtailable service at request.
    Interruptible,
    /// Time-of-use pricing response.
    TimeOfUse,
    /// Critical peak pricing (high prices during grid stress events).
    CriticalPeakPricing,
    /// Real-time pricing with hourly or sub-hourly price signals.
    RealTimePricing,
    /// Emergency demand response (event-based curtailment).
    EmergencyDr,
}

// ─────────────────────────────────────────────────────────────────────────────
// DR offer / bid
// ─────────────────────────────────────────────────────────────────────────────

/// A demand response offer — load willing to curtail at a price.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrOffer {
    /// Load / aggregator identifier.
    pub load_id: usize,
    /// Bus location.
    pub bus_id: usize,
    /// DR program type.
    pub program_type: DrProgramType,
    /// Maximum curtailment capacity offered \[MW\].
    pub curtailment_mw: f64,
    /// Offer price [$/MWh] — compensation required for curtailment.
    pub offer_price: f64,
    /// Value of lost load estimate [$/MWh] (upper bound on compensation).
    pub voll: f64,
    /// Minimum advance notice required \[minutes\].
    pub notice_minutes: f64,
    /// Maximum curtailment duration \[hours\].
    pub max_duration_h: f64,
    /// Rebound load fraction — fraction of curtailed energy consumed later (0–1).
    pub rebound_fraction: f64,
}

impl DrOffer {
    /// Create a simple DR offer with default VOLL of 10,000 $/MWh.
    pub fn simple(load_id: usize, bus_id: usize, curtailment_mw: f64, offer_price: f64) -> Self {
        Self {
            load_id,
            bus_id,
            program_type: DrProgramType::Interruptible,
            curtailment_mw,
            offer_price,
            voll: 10_000.0,
            notice_minutes: 10.0,
            max_duration_h: 4.0,
            rebound_fraction: 0.0,
        }
    }

    /// Rebound load \[MW\] that appears after curtailment ends.
    pub fn rebound_mw(&self) -> f64 {
        self.curtailment_mw * self.rebound_fraction
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DR portfolio
// ─────────────────────────────────────────────────────────────────────────────

/// A portfolio of demand response offers for market participation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrPortfolio {
    /// Individual DR offers in this portfolio.
    pub offers: Vec<DrOffer>,
}

impl DrPortfolio {
    /// Create a new empty DR portfolio.
    pub fn new() -> Self {
        Self { offers: Vec::new() }
    }

    /// Add a DR offer to the portfolio.
    pub fn add_offer(&mut self, offer: DrOffer) {
        self.offers.push(offer);
    }

    /// Total curtailment capacity available \[MW\].
    pub fn total_curtailment_mw(&self) -> f64 {
        self.offers.iter().map(|o| o.curtailment_mw).sum()
    }

    /// Merit-order sorted offers (cheapest curtailment first).
    pub fn merit_order(&self) -> Vec<&DrOffer> {
        let mut sorted: Vec<&DrOffer> = self.offers.iter().collect();
        sorted.sort_by(|a, b| {
            a.offer_price
                .partial_cmp(&b.offer_price)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        sorted
    }

    /// Estimate Value of Lost Load for the portfolio (capacity-weighted average) [$/MWh].
    pub fn portfolio_voll(&self) -> f64 {
        let total_mw = self.total_curtailment_mw();
        if total_mw < 1e-9 {
            return 0.0;
        }
        self.offers
            .iter()
            .map(|o| o.voll * o.curtailment_mw)
            .sum::<f64>()
            / total_mw
    }

    /// Clear DR offers up to a curtailment target at clearing price [$/MWh].
    ///
    /// Returns (cleared offers, clearing price, total curtailment \[MW\]).
    pub fn clear_dr(&self, curtailment_target_mw: f64, max_price: f64) -> Result<DrClearingResult> {
        if curtailment_target_mw < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "Curtailment target must be non-negative".to_string(),
            ));
        }

        let mut cleared = Vec::new();
        let mut total_curtailed = 0.0;
        let mut clearing_price = 0.0;

        for offer in self.merit_order() {
            if offer.offer_price > max_price {
                break;
            }
            if total_curtailed >= curtailment_target_mw - 1e-9 {
                break;
            }

            let available = curtailment_target_mw - total_curtailed;
            let cleared_mw = offer.curtailment_mw.min(available);

            cleared.push(DrClearedOffer {
                load_id: offer.load_id,
                bus_id: offer.bus_id,
                cleared_mw,
                offer_price: offer.offer_price,
                rebound_mw: cleared_mw * offer.rebound_fraction,
            });

            total_curtailed += cleared_mw;
            clearing_price = offer.offer_price;
        }

        let cost = cleared.iter().map(|c| c.cleared_mw * c.offer_price).sum();

        Ok(DrClearingResult {
            cleared_offers: cleared,
            clearing_price,
            total_curtailment_mw: total_curtailed,
            total_cost: cost,
        })
    }

    /// Price elasticity analysis: estimate demand reduction for price increase.
    ///
    /// Uses constant elasticity of demand: ΔQ/Q = ε × ΔP/P.
    /// Returns estimated MW reduction for a price signal.
    pub fn price_elasticity_response(
        baseline_load_mw: f64,
        baseline_price: f64,
        new_price: f64,
        elasticity: f64,
    ) -> f64 {
        if baseline_price < 1e-9 || baseline_load_mw < 0.0 {
            return 0.0;
        }
        let price_ratio = (new_price - baseline_price) / baseline_price;
        let load_change = baseline_load_mw * elasticity * price_ratio;
        // Negative elasticity → demand decreases as price rises
        (-load_change).max(0.0).min(baseline_load_mw)
    }
}

impl Default for DrPortfolio {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Clearing result
// ─────────────────────────────────────────────────────────────────────────────

/// A DR offer that was cleared in the market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrClearedOffer {
    /// Load identifier.
    pub load_id: usize,
    /// Bus location.
    pub bus_id: usize,
    /// Curtailment dispatched \[MW\].
    pub cleared_mw: f64,
    /// Price at which this offer was accepted [$/MWh].
    pub offer_price: f64,
    /// Expected rebound load after curtailment ends \[MW\].
    pub rebound_mw: f64,
}

/// Result of a DR clearing run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrClearingResult {
    /// Cleared DR offers, sorted by offer price.
    pub cleared_offers: Vec<DrClearedOffer>,
    /// Market clearing price [$/MWh] (price of the marginal cleared offer).
    pub clearing_price: f64,
    /// Total curtailment procured \[MW\].
    pub total_curtailment_mw: f64,
    /// Total payment to DR providers [$].
    pub total_cost: f64,
}

impl DrClearingResult {
    /// Cost-benefit ratio: total_cost / (total_curtailment_mw × VOLL savings per hour).
    ///
    /// A ratio < 1.0 indicates DR is cheaper than the Value of Lost Load.
    pub fn cost_benefit_ratio(&self, voll: f64, duration_h: f64) -> f64 {
        let benefit = self.total_curtailment_mw * voll * duration_h;
        if benefit < 1e-9 {
            return f64::INFINITY;
        }
        self.total_cost * duration_h / benefit
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dr_portfolio_merit_order() {
        let mut portfolio = DrPortfolio::new();
        portfolio.add_offer(DrOffer::simple(0, 0, 10.0, 50.0));
        portfolio.add_offer(DrOffer::simple(1, 0, 20.0, 30.0));
        portfolio.add_offer(DrOffer::simple(2, 0, 15.0, 40.0));

        let merit = portfolio.merit_order();
        assert!((merit[0].offer_price - 30.0).abs() < 1e-9);
        assert!((merit[1].offer_price - 40.0).abs() < 1e-9);
        assert!((merit[2].offer_price - 50.0).abs() < 1e-9);
    }

    #[test]
    fn test_dr_clearing_partial() {
        let mut portfolio = DrPortfolio::new();
        portfolio.add_offer(DrOffer::simple(0, 0, 10.0, 30.0));
        portfolio.add_offer(DrOffer::simple(1, 0, 20.0, 50.0));

        let result = portfolio.clear_dr(15.0, 100.0).unwrap();
        assert!((result.total_curtailment_mw - 15.0).abs() < 1e-6);
        assert_eq!(result.cleared_offers.len(), 2);
    }

    #[test]
    fn test_dr_clearing_price_cap() {
        let mut portfolio = DrPortfolio::new();
        portfolio.add_offer(DrOffer::simple(0, 0, 10.0, 30.0));
        portfolio.add_offer(DrOffer::simple(1, 0, 20.0, 80.0));

        // Max price 60: only first offer should clear
        let result = portfolio.clear_dr(25.0, 60.0).unwrap();
        assert!((result.total_curtailment_mw - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_price_elasticity_response() {
        // -0.3 elasticity: 10% price increase → 3% demand reduction
        let reduction = DrPortfolio::price_elasticity_response(100.0, 50.0, 55.0, -0.3);
        assert!(
            (reduction - 3.0).abs() < 0.01,
            "Expected 3 MW reduction, got {reduction:.4}"
        );
    }

    #[test]
    fn test_portfolio_voll() {
        let mut portfolio = DrPortfolio::new();
        let mut offer1 = DrOffer::simple(0, 0, 10.0, 30.0);
        offer1.voll = 5000.0;
        let mut offer2 = DrOffer::simple(1, 0, 10.0, 40.0);
        offer2.voll = 15000.0;
        portfolio.add_offer(offer1);
        portfolio.add_offer(offer2);

        // Capacity-weighted: (10 * 5000 + 10 * 15000) / 20 = 10000
        assert!((portfolio.portfolio_voll() - 10000.0).abs() < 1e-6);
    }
}

// =============================================================================
// Comprehensive DR Program Management
// =============================================================================

/// Extended DR program types (superset of basic DrProgramType).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DrProgramKind {
    /// Utility directly controls customer appliances (HVAC, water heaters)
    DirectLoadControl,
    /// Customer agrees contractually to curtail on operator request
    InterruptibleLoad,
    /// Time-of-use pricing: predefined high/low price periods (no obligation)
    TimeOfUse,
    /// Critical peak pricing: very high prices during declared peak events
    CriticalPeakPricing,
    /// Real-time pricing: 5-minute interval prices (spot market exposure)
    RealTimePricing,
    /// Emergency DR: reliability-based curtailment during grid emergencies
    EmergencyDemandResponse,
    /// Economic DR: voluntary curtailment when energy prices exceed threshold
    EconomicDemandResponse,
}

/// Trigger condition for a DR dispatch event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DrEventTrigger {
    /// Price exceeds threshold — economic curtailment
    Price { threshold_per_mwh: f64 },
    /// Reserve margin below required level — reliability curtailment
    Reliability { reserve_shortage_mw: f64 },
    /// Branch loading exceeds thermal rating — congestion management
    Congestion { branch_idx: usize, loading_pct: f64 },
    /// Operator manual dispatch
    Operator,
}

/// Enrolled demand response customer for program-level management.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrCustomer {
    /// Unique customer identifier
    pub customer_id: usize,
    /// Bus where customer is connected
    pub bus: usize,
    /// DR program this customer is enrolled in
    pub program: DrProgramKind,
    /// Baseline demand (average non-DR consumption) \[MW\]
    pub baseline_mw: f64,
    /// Maximum curtailment available \[MW\]
    pub max_curtailment_mw: f64,
    /// Minimum curtailment if called (usually 0) \[MW\]
    pub min_curtailment_mw: f64,
    /// Maximum DR events allowed per calendar year
    pub max_events_per_year: usize,
    /// Maximum duration per single event \[hours\]
    pub max_duration_h: f64,
    /// Required advance notice time \[minutes\]
    pub notice_time_min: f64,
    /// Minimum rest period between events \[hours\]
    pub recovery_time_h: f64,
    /// Incentive payment rate \[$/MWh curtailed\]
    pub incentive_rate: f64,
    /// Own-price elasticity of demand (typically -0.1 to -0.5)
    pub elasticity: f64,
    /// Time (hours from start) when last event ended; None if no prior event
    pub last_event_time: Option<f64>,
    /// Number of DR events called this calendar year
    pub events_this_year: usize,
}

impl DrCustomer {
    /// Check whether this customer is eligible for dispatch at `current_time_h`.
    pub fn is_eligible(&self, current_time_h: f64) -> bool {
        if self.events_this_year >= self.max_events_per_year {
            return false;
        }
        if let Some(last_t) = self.last_event_time {
            if current_time_h - last_t < self.recovery_time_h {
                return false;
            }
        }
        self.max_curtailment_mw > 1e-9
    }
}

/// Result of a comprehensive DR dispatch call.
#[derive(Debug, Clone)]
pub struct DrProgramDispatchResult {
    /// Customer IDs actually dispatched (those that responded)
    pub dispatched_customers: Vec<usize>,
    /// Curtailment per dispatched customer \[MW\]
    pub curtailment_per_customer: Vec<f64>,
    /// Total curtailment achieved \[MW\]
    pub total_curtailment_mw: f64,
    /// Total incentive payments \[$\]
    pub total_cost: f64,
    /// Number of customers called (including non-responders)
    pub n_customers_called: usize,
    /// Fraction of called customers that responded (0..1)
    pub response_rate: f64,
}

/// DR program portfolio manager — collection of enrolled customers.
pub struct DrProgramPortfolio {
    /// Enrolled customers
    pub customers: Vec<DrCustomer>,
    /// Program type governing dispatch rules
    pub program_type: DrProgramKind,
    /// Maximum simultaneous curtailment across all customers \[MW\]
    pub max_simultaneous_mw: f64,
    /// Counter for sequential event IDs
    pub event_counter: usize,
}

impl DrProgramPortfolio {
    /// Create a new DR program portfolio.
    pub fn new(customers: Vec<DrCustomer>, program: DrProgramKind, max_mw: f64) -> Self {
        Self {
            customers,
            program_type: program,
            max_simultaneous_mw: max_mw,
            event_counter: 0,
        }
    }

    /// Dispatch DR customers to achieve `target_mw` curtailment.
    ///
    /// # Algorithm
    /// 1. Filter eligible customers (recovery time, event count, non-zero capability)
    /// 2. Sort by incentive_rate ascending (cheapest-first dispatch)
    /// 3. Call customers until target_mw achieved or portfolio exhausted
    /// 4. Apply response factor: 85% of called customers actually respond
    /// 5. Update customer state (last_event_time, events_this_year)
    /// 6. Cap total curtailment at max_simultaneous_mw
    pub fn dispatch(
        &mut self,
        target_mw: f64,
        current_time_h: f64,
        _trigger: DrEventTrigger,
    ) -> Result<DrProgramDispatchResult> {
        if target_mw < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "DR dispatch target cannot be negative".to_string(),
            ));
        }

        let effective_target = target_mw.min(self.max_simultaneous_mw);

        let mut eligible_idx: Vec<usize> = self
            .customers
            .iter()
            .enumerate()
            .filter(|(_, c)| c.is_eligible(current_time_h))
            .map(|(i, _)| i)
            .collect();

        eligible_idx.sort_by(|&a, &b| {
            self.customers[a]
                .incentive_rate
                .partial_cmp(&self.customers[b].incentive_rate)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        const RESPONSE_FACTOR: f64 = 0.85;

        let mut dispatched_customers: Vec<usize> = Vec::new();
        let mut curtailment_per_customer: Vec<f64> = Vec::new();
        let mut remaining = effective_target;
        let mut n_called = 0_usize;
        let mut total_curtailment = 0.0_f64;
        let mut total_cost = 0.0_f64;

        for idx in &eligible_idx {
            if remaining <= 1e-9 {
                break;
            }
            n_called += 1;
            let customer = &self.customers[*idx];
            let raw_curtailment = customer.max_curtailment_mw.min(remaining);
            let actual_curtailment = raw_curtailment * RESPONSE_FACTOR;

            if actual_curtailment > 1e-9 {
                dispatched_customers.push(customer.customer_id);
                curtailment_per_customer.push(actual_curtailment);
                total_cost += actual_curtailment * customer.incentive_rate;
                total_curtailment += actual_curtailment;
                remaining -= actual_curtailment;
            }
        }

        // Update customer state
        for cust_id in &dispatched_customers {
            if let Some(c) = self
                .customers
                .iter_mut()
                .find(|c| c.customer_id == *cust_id)
            {
                c.last_event_time = Some(current_time_h);
                c.events_this_year = c.events_this_year.saturating_add(1);
            }
        }

        let response_rate = if n_called > 0 {
            dispatched_customers.len() as f64 / n_called as f64
        } else {
            0.0
        };

        self.event_counter = self.event_counter.saturating_add(1);

        Ok(DrProgramDispatchResult {
            dispatched_customers,
            curtailment_per_customer,
            total_curtailment_mw: total_curtailment,
            total_cost,
            n_customers_called: n_called,
            response_rate,
        })
    }

    /// Compute price-responsive demand using own-price elasticity.
    ///
    /// Q_new = Q_base × (1 + ε × (P - P_ref) / P_ref)
    ///
    /// Result is clamped to [0, Q_base].
    pub fn price_responsive_demand(
        &self,
        price_mwh: f64,
        baseline_mw: f64,
        price_elasticity: f64,
        reference_price_mwh: f64,
    ) -> f64 {
        let ref_price = reference_price_mwh.max(1e-12);
        let relative_change = (price_mwh - ref_price) / ref_price;
        let demand = baseline_mw * (1.0 + price_elasticity * relative_change);
        demand.clamp(0.0, baseline_mw)
    }

    /// Build the flexibility supply curve for all eligible customers at `current_time_h`.
    ///
    /// Returns `(cumulative_mw, incentive_rate_$/MWh)` pairs sorted by price ascending.
    pub fn flexibility_supply_curve(&self, current_time_h: f64) -> Vec<(f64, f64)> {
        let mut eligible: Vec<&DrCustomer> = self
            .customers
            .iter()
            .filter(|c| c.is_eligible(current_time_h))
            .collect();

        eligible.sort_by(|a, b| {
            a.incentive_rate
                .partial_cmp(&b.incentive_rate)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut cumulative = 0.0_f64;
        eligible
            .into_iter()
            .map(|c| {
                cumulative += c.max_curtailment_mw;
                (cumulative, c.incentive_rate)
            })
            .collect()
    }

    /// Compute day-ahead baseline using the D+1 same-hour average method.
    ///
    /// # Arguments
    /// - `historical_demand` — flat array of hourly demands (length = days × 24)
    /// - `n_days`            — number of past days to average
    /// - `hour`              — hour-of-day (0..23)
    pub fn compute_baseline(historical_demand: &[f64], n_days: usize, hour: usize) -> f64 {
        if n_days == 0 || hour >= 24 {
            return 0.0;
        }
        let n_available = historical_demand.len() / 24;
        let days_used = n_days.min(n_available);
        if days_used == 0 {
            return 0.0;
        }

        let sum: f64 = (0..days_used)
            .filter_map(|day| {
                let idx = day * 24 + hour;
                historical_demand.get(idx).copied()
            })
            .sum();

        let count = (0..days_used)
            .filter(|&day| day * 24 + hour < historical_demand.len())
            .count();

        if count == 0 {
            0.0
        } else {
            sum / count as f64
        }
    }

    /// Estimate rebound load after a DR event ends.
    ///
    /// Returns vec of length `ceil(rebound_duration_h)` with MW per hour of rebound.
    pub fn rebound_load(
        curtailment_mw: f64,
        rebound_factor: f64,
        rebound_duration_h: f64,
    ) -> Vec<f64> {
        let duration = rebound_duration_h.max(1.0);
        let n_periods = duration.ceil() as usize;
        let total_rebound_mw = curtailment_mw * rebound_factor.clamp(0.0, 1.0);
        let per_hour = total_rebound_mw / duration;
        vec![per_hour; n_periods]
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Value of Lost Load (VOLL)
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate Value of Lost Load (VOLL) for a given customer sector and outage duration.
///
/// # Arguments
/// - `sector`     — customer type: "residential", "commercial", "industrial"
/// - `duration_h` — outage duration \[hours\]
///
/// # Returns
/// VOLL in \[$/MWh\]
pub fn compute_voll(sector: &str, duration_h: f64) -> f64 {
    let base_voll: f64 = match sector {
        "residential" => 10_000.0,
        "commercial" => 15_000.0,
        "industrial" => 25_000.0,
        _ => 10_000.0,
    };
    let duration_factor = (1.0 + 0.1 * duration_h.max(0.0)).min(3.0);
    base_voll * duration_factor
}

/// Compute cost-benefit analysis for a DR program dispatch.
///
/// # Returns
/// `(benefit, cost, net_benefit)` in dollars.
pub fn dr_cost_benefit(
    curtailment_mwh: f64,
    avoided_cost_per_mwh: f64,
    incentive_cost_per_mwh: f64,
    admin_cost: f64,
) -> (f64, f64, f64) {
    let benefit = curtailment_mwh * avoided_cost_per_mwh;
    let cost = curtailment_mwh * incentive_cost_per_mwh + admin_cost;
    let net_benefit = benefit - cost;
    (benefit, cost, net_benefit)
}

#[cfg(test)]
mod program_tests {
    use super::*;

    fn make_customer(id: usize, max_mw: f64, rate: f64, recovery_h: f64) -> DrCustomer {
        DrCustomer {
            customer_id: id,
            bus: 1,
            program: DrProgramKind::InterruptibleLoad,
            baseline_mw: max_mw * 2.0,
            max_curtailment_mw: max_mw,
            min_curtailment_mw: 0.0,
            max_events_per_year: 30,
            max_duration_h: 4.0,
            notice_time_min: 10.0,
            recovery_time_h: recovery_h,
            incentive_rate: rate,
            elasticity: -0.2,
            last_event_time: None,
            events_this_year: 0,
        }
    }

    #[test]
    fn test_dr_dispatch_meets_target() {
        let customers = vec![
            make_customer(0, 50.0, 10.0, 2.0),
            make_customer(1, 50.0, 15.0, 2.0),
            make_customer(2, 50.0, 20.0, 2.0),
        ];
        let mut portfolio =
            DrProgramPortfolio::new(customers, DrProgramKind::InterruptibleLoad, 200.0);
        let result = portfolio
            .dispatch(100.0, 10.0, DrEventTrigger::Operator)
            .expect("Dispatch should succeed");

        assert!(
            result.total_curtailment_mw > 0.0,
            "Should curtail some load: {:.2}",
            result.total_curtailment_mw
        );
        assert!(result.n_customers_called > 0);
    }

    #[test]
    fn test_dr_price_response_elasticity() {
        let customers = vec![make_customer(0, 50.0, 10.0, 2.0)];
        let portfolio = DrProgramPortfolio::new(customers, DrProgramKind::RealTimePricing, 100.0);

        let baseline = 100.0;
        let reference = 50.0;
        let high_price = 100.0;
        let elasticity = -0.3;

        let demand = portfolio.price_responsive_demand(high_price, baseline, elasticity, reference);
        assert!(
            demand < baseline,
            "Demand should decrease at higher price: {demand:.2}"
        );
        assert!(demand > 0.0, "Demand should remain positive: {demand:.2}");
        assert!(
            (demand - 70.0).abs() < 1e-6,
            "Expected 70 MW, got {demand:.2}"
        );
    }

    #[test]
    fn test_dr_eligibility_recovery_time() {
        let mut c = make_customer(0, 50.0, 10.0, 4.0);
        c.last_event_time = Some(0.0);

        assert!(!c.is_eligible(1.0), "Not eligible before recovery");
        assert!(c.is_eligible(5.0), "Eligible after recovery");

        let customers = vec![{
            let mut c2 = make_customer(0, 50.0, 10.0, 4.0);
            c2.last_event_time = Some(0.0);
            c2
        }];
        let mut portfolio =
            DrProgramPortfolio::new(customers, DrProgramKind::InterruptibleLoad, 100.0);
        let result = portfolio
            .dispatch(50.0, 1.0, DrEventTrigger::Operator)
            .expect("Dispatch should not error");
        assert_eq!(
            result.total_curtailment_mw, 0.0,
            "No eligible customers → zero curtailment"
        );
    }

    #[test]
    fn test_dr_supply_curve_sorted() {
        let customers = vec![
            make_customer(0, 30.0, 25.0, 2.0),
            make_customer(1, 20.0, 10.0, 2.0),
            make_customer(2, 40.0, 15.0, 2.0),
        ];
        let portfolio = DrProgramPortfolio::new(customers, DrProgramKind::InterruptibleLoad, 200.0);
        let curve = portfolio.flexibility_supply_curve(0.0);

        assert!(!curve.is_empty(), "Supply curve should not be empty");
        for i in 1..curve.len() {
            assert!(
                curve[i - 1].1 <= curve[i].1,
                "Prices must be sorted ascending"
            );
            assert!(
                curve[i - 1].0 <= curve[i].0,
                "Cumulative volume must be non-decreasing"
            );
        }
    }

    #[test]
    fn test_dr_baseline_computation() {
        let mut hist = vec![100.0_f64; 72]; // 3 days
        hist[12] = 150.0;
        hist[24 + 12] = 160.0;
        hist[48 + 12] = 170.0;

        let baseline = DrProgramPortfolio::compute_baseline(&hist, 3, 12);
        let expected = (150.0 + 160.0 + 170.0) / 3.0;
        assert!(
            (baseline - expected).abs() < 1e-6,
            "Baseline should be {expected:.2}, got {baseline:.2}"
        );
    }

    #[test]
    fn test_voll_positive() {
        let voll = compute_voll("residential", 1.0);
        assert!(voll > 0.0, "VOLL should be positive: {voll:.2}");
        assert!(voll >= 10_000.0, "Residential VOLL >= 10,000 $/MWh");
    }

    #[test]
    fn test_dr_cost_benefit() {
        let (benefit, cost, net) = dr_cost_benefit(10.0, 100.0, 20.0, 50.0);
        assert!((benefit - 1000.0).abs() < 1e-6, "Benefit={benefit:.2}");
        assert!((cost - 250.0).abs() < 1e-6, "Cost={cost:.2}");
        assert!((net - 750.0).abs() < 1e-6, "Net={net:.2}");
    }
}
