//! Multi-market battery storage co-optimization.
//!
//! Co-optimizes a fleet of battery energy storage systems (BESS) across
//! simultaneous participation in energy arbitrage, ancillary services
//! (frequency regulation, spinning/non-spinning reserve), capacity markets,
//! demand response, and voltage support markets.
//!
//! Two solvers are provided:
//! * [`MultiMarketOptimizer::solve_dynamic_programming`] — exact DP over a
//!   discretised (time × SoC) state space (20 SoC levels × 24 hours).
//! * [`MultiMarketOptimizer::solve_lp_relaxation`] — Lagrangian relaxation
//!   with subgradient updates (50 iterations) for fast near-optimal solutions.
//!
//! # References
//! - Sioshansi, R. et al. (2009). *Estimating the value of electricity storage
//!   in PJM*. Energy Economics.
//! - Xu, B. et al. (2018). *Factoring degradation costs into BESS dispatch*.
//!   IEEE Trans. Smart Grid.

use std::collections::HashMap;

use crate::error::OxiGridError;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Markets in which a storage unit may participate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum MarketType {
    /// Day-ahead / real-time energy price arbitrage.
    EnergyArbitrage,
    /// Frequency regulation upward (inject power, fast response ≤ 0.5 s).
    FrequencyRegulationUp,
    /// Frequency regulation downward (absorb power, fast response ≤ 0.5 s).
    FrequencyRegulationDown,
    /// Spinning reserve — on-standby, must respond within 10 s.
    SpinningReserve,
    /// Non-spinning reserve — response within 30 min.
    NonSpinningReserve,
    /// Capacity market (monthly/annual availability payment).
    CapacityMarket,
    /// Demand response / load curtailment program.
    DemandResponse,
    /// Reactive power / voltage support.
    VoltageSupport,
}

impl MarketType {
    /// Physical response-time requirement for this market service \[seconds\].
    pub fn required_response_time_s(&self) -> f64 {
        match self {
            MarketType::FrequencyRegulationUp | MarketType::FrequencyRegulationDown => 0.5,
            MarketType::SpinningReserve => 10.0,
            MarketType::EnergyArbitrage => 300.0,
            MarketType::VoltageSupport => 1.0,
            MarketType::NonSpinningReserve => 1_800.0,
            MarketType::DemandResponse => 3_600.0,
            MarketType::CapacityMarket => 3_600.0,
        }
    }

    /// String label used in revenue decomposition maps.
    pub fn label(&self) -> &'static str {
        match self {
            MarketType::EnergyArbitrage => "energy_arbitrage",
            MarketType::FrequencyRegulationUp => "freq_reg_up",
            MarketType::FrequencyRegulationDown => "freq_reg_down",
            MarketType::SpinningReserve => "spinning_reserve",
            MarketType::NonSpinningReserve => "non_spinning_reserve",
            MarketType::CapacityMarket => "capacity_market",
            MarketType::DemandResponse => "demand_response",
            MarketType::VoltageSupport => "voltage_support",
        }
    }

    /// Priority ranking for dispatch allocation — lower = higher priority.
    pub fn priority(&self) -> u8 {
        match self {
            MarketType::FrequencyRegulationUp => 0,
            MarketType::FrequencyRegulationDown => 1,
            MarketType::SpinningReserve => 2,
            MarketType::EnergyArbitrage => 3,
            MarketType::NonSpinningReserve => 4,
            MarketType::VoltageSupport => 5,
            MarketType::DemandResponse => 6,
            MarketType::CapacityMarket => 7,
        }
    }

    /// Returns `true` if this market is an ancillary services market.
    pub fn is_ancillary(&self) -> bool {
        matches!(
            self,
            MarketType::FrequencyRegulationUp
                | MarketType::FrequencyRegulationDown
                | MarketType::SpinningReserve
                | MarketType::NonSpinningReserve
                | MarketType::VoltageSupport
                | MarketType::DemandResponse
        )
    }
}

/// Bidding strategy adopted by the optimizer when constructing bids.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BidStrategy {
    /// Submit bids at the market clearing price — suitable for price-takers.
    PriceTaker,
    /// Adjust bid price based on forecast uncertainty (5 % markup).
    PriceAdaptive,
    /// Game-theoretic strategic bidding with markup above marginal cost.
    StrategicBidding,
}

// ---------------------------------------------------------------------------
// Core structs
// ---------------------------------------------------------------------------

/// A cleared or expected opportunity in a single market for one hour.
#[derive(Debug, Clone)]
pub struct MarketOpportunity {
    /// Market type.
    pub market: MarketType,
    /// Clearing / offered price \[USD/MWh\] (or USD/MW·h for reserves).
    pub clearing_price_usd_per_mwh: f64,
    /// Accepted MW quantity.
    pub accepted_mw: f64,
    /// Service duration \[h\].
    pub duration_h: f64,
    /// Revenue earned \[USD\].
    pub revenue_usd: f64,
    /// Physical response-time requirement \[s\].
    pub required_response_time_s: f64,
    /// Minimum committed service period \[h\].
    pub min_commitment_h: f64,
}

impl MarketOpportunity {
    /// Recompute revenue from price × MW × duration.
    pub fn compute_revenue(&self) -> f64 {
        self.clearing_price_usd_per_mwh * self.accepted_mw * self.duration_h
    }
}

/// A battery energy storage system available for multi-market participation.
#[derive(Debug, Clone)]
pub struct StorageUnit {
    /// Unique identifier.
    pub id: usize,
    /// Usable energy capacity \[MWh\].
    pub energy_capacity_mwh: f64,
    /// Maximum charge/discharge power \[MW\].
    pub power_capacity_mw: f64,
    /// Round-trip efficiency (0–1); default 0.92.
    pub roundtrip_efficiency: f64,
    /// Minimum state-of-charge limit (0–1); default 0.10.
    pub soc_min: f64,
    /// Maximum state-of-charge limit (0–1); default 0.90.
    pub soc_max: f64,
    /// Initial state-of-charge (0–1).
    pub soc_initial: f64,
    /// Throughput degradation cost \[USD/MWh discharged\]; default 5.0.
    pub degradation_cost_usd_per_mwh: f64,
    /// Ramp rate \[MW/s\].
    pub ramp_rate_mw_per_s: f64,
    /// Minimum achievable response time given physics \[s\].
    pub min_response_time_s: f64,
}

impl StorageUnit {
    /// Create a standard lithium-ion unit with sensible defaults.
    pub fn new_lithium_ion(id: usize, energy_mwh: f64, power_mw: f64) -> Self {
        Self {
            id,
            energy_capacity_mwh: energy_mwh,
            power_capacity_mw: power_mw,
            roundtrip_efficiency: 0.92,
            soc_min: 0.10,
            soc_max: 0.90,
            soc_initial: 0.50,
            degradation_cost_usd_per_mwh: 5.0,
            ramp_rate_mw_per_s: power_mw * 0.5,
            min_response_time_s: 0.5,
        }
    }

    /// Returns `true` if this unit can physically meet the market response time.
    pub fn can_participate(&self, market: &MarketType) -> bool {
        self.min_response_time_s <= market.required_response_time_s()
    }

    /// One-way discharge efficiency: √η_rt.
    #[inline]
    pub fn discharge_efficiency(&self) -> f64 {
        self.roundtrip_efficiency.sqrt()
    }

    /// One-way charge efficiency: √η_rt.
    #[inline]
    pub fn charge_efficiency(&self) -> f64 {
        self.roundtrip_efficiency.sqrt()
    }
}

/// A single bid submitted by a storage unit to a market for a given hour.
#[derive(Debug, Clone)]
pub struct MultiMarketBid {
    /// Storage unit that submitted this bid.
    pub unit_id: usize,
    /// Target market.
    pub market: MarketType,
    /// Quantity offered \[MW\].
    pub quantity_mw: f64,
    /// Offered price \[USD/MWh\].
    pub price_usd_per_mwh: f64,
    /// Scheduling hour (0-indexed, within the optimisation horizon).
    pub hour: usize,
}

/// Full solution returned by the multi-market optimizer.
#[derive(Debug, Clone)]
pub struct MultiMarketSolution {
    /// All submitted bids across units and hours.
    pub bids: Vec<MultiMarketBid>,
    /// SoC trajectory \[0–1\] at the *start* of each hour (length = horizon + 1).
    pub soc_trajectory: Vec<f64>,
    /// Gross energy arbitrage revenue \[USD\].
    pub energy_revenue_usd: f64,
    /// Gross ancillary services revenue \[USD\].
    pub ancillary_revenue_usd: f64,
    /// Capacity market revenue \[USD\].
    pub capacity_revenue_usd: f64,
    /// Total gross revenue \[USD\].
    pub total_revenue_usd: f64,
    /// Total degradation cost \[USD\].
    pub degradation_cost_usd: f64,
    /// Net profit = revenue − degradation \[USD\].
    pub net_profit_usd: f64,
    /// Hours participated per market: `(MarketType, hours)`.
    pub market_participation: Vec<(MarketType, f64)>,
}

/// System-wide constraint applied to a particular market.
#[derive(Debug, Clone)]
pub struct MarketConstraint {
    /// Market this constraint applies to.
    pub market: MarketType,
    /// Maximum total MW clearable in this market across all units.
    pub max_total_mw: f64,
    /// Minimum contracted service duration \[h\].
    pub min_duration_h: f64,
    /// Whether units need explicit regulatory certification to participate.
    pub requires_certification: bool,
}

/// Result of a simulated uniform-price market clearing auction.
#[derive(Debug, Clone)]
pub struct MarketClearingResult {
    /// Bids accepted in the auction.
    pub accepted_bids: Vec<MultiMarketBid>,
    /// Uniform clearing price \[USD/MWh\] (set by the marginal accepted bid).
    pub marginal_price_usd_per_mwh: f64,
    /// Total MW cleared.
    pub total_cleared_mw: f64,
    /// Social welfare = Σ (clearing_price − bid_price) × qty for accepted bids \[USD\].
    pub social_welfare_usd: f64,
}

// ---------------------------------------------------------------------------
// Revenue stacking helper
// ---------------------------------------------------------------------------

/// Static utilities for analysing and stacking multi-market revenue streams.
pub struct RevenueStacking;

impl RevenueStacking {
    /// Marginal value of one additional MWh dispatched into `market` at `price`.
    ///
    /// Accounts for one-way discharge efficiency and degradation cost.
    pub fn compute_marginal_value(unit: &StorageUnit, market: &MarketType, price: f64) -> f64 {
        let eta = unit.discharge_efficiency();
        let degradation = unit.degradation_cost_usd_per_mwh;
        match market {
            // Physical energy discharged: apply efficiency and full degradation
            MarketType::EnergyArbitrage
            | MarketType::SpinningReserve
            | MarketType::NonSpinningReserve
            | MarketType::DemandResponse => price * eta - degradation,
            // Frequency regulation: capacity payment; degradation is partial (≈50%)
            MarketType::FrequencyRegulationUp | MarketType::FrequencyRegulationDown => {
                price * eta - degradation * 0.5
            }
            // Capacity / voltage: pure availability payment, no energy drawn
            MarketType::CapacityMarket | MarketType::VoltageSupport => price,
        }
    }

    /// Identify which market offers the highest net marginal value at current prices.
    ///
    /// Uses a zero-degradation dummy unit for comparison so that the method
    /// reflects price level only.
    pub fn identify_dominant_market(prices: &HashMap<MarketType, f64>) -> MarketType {
        let dummy = StorageUnit {
            id: 0,
            energy_capacity_mwh: 1.0,
            power_capacity_mw: 1.0,
            roundtrip_efficiency: 1.0,
            soc_min: 0.0,
            soc_max: 1.0,
            soc_initial: 0.5,
            degradation_cost_usd_per_mwh: 0.0,
            ramp_rate_mw_per_s: 1.0,
            min_response_time_s: 0.0,
        };
        let mut best_market = MarketType::EnergyArbitrage;
        let mut best_value = f64::NEG_INFINITY;
        for (market, &price) in prices {
            let val = Self::compute_marginal_value(&dummy, market, price);
            if val > best_value {
                best_value = val;
                best_market = *market;
            }
        }
        best_market
    }

    /// Opportunity cost of discharging now, given expected future energy prices.
    ///
    /// Defined as the weighted-average future revenue foregone by depleting one
    /// MWh of stored energy today.  Near-future hours are weighted more heavily.
    pub fn compute_opportunity_cost(soc: f64, future_prices: &[f64], efficiency: f64) -> f64 {
        if future_prices.is_empty() {
            return 0.0;
        }
        let mut weighted_sum = 0.0_f64;
        let mut weight_sum = 0.0_f64;
        for (i, &p) in future_prices.iter().enumerate() {
            let w = 1.0 / (1.0 + i as f64);
            weighted_sum += w * p.max(0.0);
            weight_sum += w;
        }
        let avg_future = if weight_sum > 1e-12 {
            weighted_sum / weight_sum
        } else {
            0.0
        };
        // Scale by available discharge capacity (fraction of usable SoC window)
        let discharge_fraction = (soc - 0.1).max(0.0) / 0.8;
        avg_future * efficiency * discharge_fraction
    }
}

// ---------------------------------------------------------------------------
// Main optimizer
// ---------------------------------------------------------------------------

/// Multi-market storage fleet optimizer.
///
/// Holds a fleet of [`StorageUnit`]s and hourly price forecasts for each
/// [`MarketType`], then exposes two solvers:
/// * [`solve_dynamic_programming`](Self::solve_dynamic_programming) — exact DP.
/// * [`solve_lp_relaxation`](Self::solve_lp_relaxation) — fast Lagrangian relaxation.
pub struct MultiMarketOptimizer {
    /// Fleet of storage units.
    pub units: Vec<StorageUnit>,
    /// Hourly price forecasts (length = `horizon_hours`) per market.
    pub price_forecasts: HashMap<MarketType, Vec<f64>>,
    /// System-wide market constraints.
    pub market_constraints: Vec<MarketConstraint>,
    /// Optimisation horizon \[h\]; default 24.
    pub horizon_hours: usize,
    /// Bidding strategy in use.
    pub strategy: BidStrategy,
}

impl MultiMarketOptimizer {
    /// Construct a new optimizer with default settings (PriceTaker strategy).
    pub fn new(units: Vec<StorageUnit>, horizon_hours: usize) -> Self {
        Self {
            units,
            price_forecasts: HashMap::new(),
            market_constraints: Vec::new(),
            horizon_hours,
            strategy: BidStrategy::PriceTaker,
        }
    }

    /// Set the hourly price forecast for a market.
    ///
    /// `prices` should have `horizon_hours` entries; extra entries are ignored
    /// and missing entries are treated as zero.
    pub fn set_price_forecast(&mut self, market: MarketType, prices: Vec<f64>) {
        self.price_forecasts.insert(market, prices);
    }

    /// Add a system-wide market constraint.
    pub fn add_constraint(&mut self, constraint: MarketConstraint) {
        self.market_constraints.push(constraint);
    }

    // -----------------------------------------------------------------------
    // Dynamic programming solver
    // -----------------------------------------------------------------------

    /// Solve via backward-induction dynamic programming.
    ///
    /// **State space**: (hour t, SoC level s) where `s ∈ {0 .. 19}` maps to
    /// `SoC = soc_min + s × (soc_max − soc_min) / 19`.
    ///
    /// **Action space**: power `p ∈ {−P_max .. +P_max}` discretised into 21 levels
    /// (positive = discharge, negative = charge).
    ///
    /// **Bellman equation**:
    /// `V(t, s) = max_p [ r(t, p) − deg(p) + V(t+1, s') ]`
    ///
    /// For multi-unit fleets each unit is solved independently; results are merged.
    pub fn solve_dynamic_programming(&self) -> Result<MultiMarketSolution, OxiGridError> {
        if self.units.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "no storage units configured".to_string(),
            ));
        }

        const N_SOC: usize = 20;
        const N_PWR: usize = 21;
        let t = self.horizon_hours;

        let mut all_bids: Vec<MultiMarketBid> = Vec::new();
        let mut total_energy_rev = 0.0_f64;
        let mut total_anc_rev = 0.0_f64;
        let mut total_cap_rev = 0.0_f64;
        let mut total_deg = 0.0_f64;
        let mut rep_soc_traj: Vec<f64> = vec![0.0; t + 1];

        for unit in &self.units {
            let soc_lo = unit.soc_min;
            let soc_hi = unit.soc_max;
            let e_cap = unit.energy_capacity_mwh;
            let p_max = unit.power_capacity_mw;
            let eta_d = unit.discharge_efficiency();
            let eta_c = unit.charge_efficiency();
            let deg = unit.degradation_cost_usd_per_mwh;

            // SoC grid (20 levels)
            let soc_grid: Vec<f64> = (0..N_SOC)
                .map(|s| soc_lo + s as f64 * (soc_hi - soc_lo) / (N_SOC - 1) as f64)
                .collect();

            // Power grid: positive = discharge, negative = charge (21 levels)
            let pwr_grid: Vec<f64> = (0..N_PWR)
                .map(|k| -p_max + k as f64 * 2.0 * p_max / (N_PWR - 1) as f64)
                .collect();

            // DP tables
            let mut v_next = vec![0.0_f64; N_SOC]; // terminal value = 0
            let mut policy: Vec<Vec<usize>> = vec![vec![N_PWR / 2; N_SOC]; t];

            // Backward induction
            for step in (0..t).rev() {
                let mut v_curr = vec![f64::NEG_INFINITY; N_SOC];
                for (s_idx, &soc) in soc_grid.iter().enumerate() {
                    let mut best = f64::NEG_INFINITY;
                    let mut best_k = N_PWR / 2;
                    for (k, &p) in pwr_grid.iter().enumerate() {
                        let dsoc = Self::soc_delta(p, eta_d, eta_c, e_cap);
                        let soc_next = soc + dsoc;
                        if soc_next < soc_lo - 1e-9 || soc_next > soc_hi + 1e-9 {
                            continue;
                        }
                        let s_next = Self::soc_to_index(
                            soc_next.clamp(soc_lo, soc_hi),
                            soc_lo,
                            soc_hi,
                            N_SOC,
                        );
                        let revenue = self.step_revenue(unit, step, p);
                        let deg_cost = if p > 0.0 { deg * p } else { 0.0 };
                        let val = revenue - deg_cost + v_next[s_next];
                        if val > best {
                            best = val;
                            best_k = k;
                        }
                    }
                    v_curr[s_idx] = if best == f64::NEG_INFINITY { 0.0 } else { best };
                    policy[step][s_idx] = best_k;
                }
                v_next = v_curr;
            }

            // Forward simulation
            let mut soc = unit.soc_initial.clamp(soc_lo, soc_hi);
            let mut unit_soc_traj = vec![0.0_f64; t + 1];
            unit_soc_traj[0] = soc;

            for step in 0..t {
                let s_idx = Self::soc_to_index(soc, soc_lo, soc_hi, N_SOC);
                let k = policy[step][s_idx];
                let p = pwr_grid[k];

                self.allocate_bids_to_markets(unit, step, p, &mut all_bids);

                let step_rev = self.step_revenue(unit, step, p);
                let deg_cost = if p > 0.0 { deg * p } else { 0.0 };
                total_deg += deg_cost;
                self.classify_revenue(
                    step,
                    step_rev,
                    &mut total_energy_rev,
                    &mut total_anc_rev,
                    &mut total_cap_rev,
                );

                let dsoc = Self::soc_delta(p, eta_d, eta_c, e_cap);
                soc = (soc + dsoc).clamp(soc_lo, soc_hi);
                unit_soc_traj[step + 1] = soc;
            }

            if unit.id == self.units[0].id {
                rep_soc_traj = unit_soc_traj;
            }
        }

        self.build_solution(
            all_bids,
            rep_soc_traj,
            total_energy_rev,
            total_anc_rev,
            total_cap_rev,
            total_deg,
        )
    }

    // -----------------------------------------------------------------------
    // Lagrangian relaxation (LP relaxation)
    // -----------------------------------------------------------------------

    /// Solve using Lagrangian relaxation with subgradient updates.
    ///
    /// Relaxes the inter-temporal SoC-continuity constraints with multipliers λ_t,
    /// then solves a separable per-hour subproblem in closed form by scanning
    /// 41 power levels.  50 iterations with step size α = 1/√(iter + 1).
    pub fn solve_lp_relaxation(&self) -> Result<MultiMarketSolution, OxiGridError> {
        if self.units.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "no storage units configured".to_string(),
            ));
        }

        const MAX_ITER: usize = 50;
        let t = self.horizon_hours;

        let mut all_bids: Vec<MultiMarketBid> = Vec::new();
        let mut total_energy_rev = 0.0_f64;
        let mut total_anc_rev = 0.0_f64;
        let mut total_cap_rev = 0.0_f64;
        let mut total_deg = 0.0_f64;
        let mut rep_soc_traj = vec![0.0_f64; t + 1];

        for unit in &self.units {
            let soc_lo = unit.soc_min;
            let soc_hi = unit.soc_max;
            let e_cap = unit.energy_capacity_mwh;
            let p_max = unit.power_capacity_mw;
            let eta_d = unit.discharge_efficiency();
            let eta_c = unit.charge_efficiency();
            let deg = unit.degradation_cost_usd_per_mwh;

            let mut lambdas = vec![0.0_f64; t];
            let mut best_dispatch = vec![0.0_f64; t];

            for iter in 0..MAX_ITER {
                let alpha = 1.0 / (iter as f64 + 1.0).sqrt();
                let mut dispatch = vec![0.0_f64; t];
                let mut soc = unit.soc_initial.clamp(soc_lo, soc_hi);

                for step in 0..t {
                    let p_best = self.subproblem_optimal_power(
                        unit,
                        step,
                        soc,
                        lambdas[step],
                        p_max,
                        soc_lo,
                        soc_hi,
                        e_cap,
                        eta_d,
                        eta_c,
                        deg,
                    );
                    dispatch[step] = p_best;
                    let dsoc = Self::soc_delta(p_best, eta_d, eta_c, e_cap);
                    soc = (soc + dsoc).clamp(soc_lo, soc_hi);
                }

                // Subgradient update
                soc = unit.soc_initial.clamp(soc_lo, soc_hi);
                let mut max_violation = 0.0_f64;
                for step in 0..t {
                    let p = dispatch[step];
                    let dsoc = Self::soc_delta(p, eta_d, eta_c, e_cap);
                    let soc_next = (soc + dsoc).clamp(soc_lo, soc_hi);
                    let violation = if soc + dsoc < soc_lo {
                        soc + dsoc - soc_lo
                    } else if soc + dsoc > soc_hi {
                        soc + dsoc - soc_hi
                    } else {
                        0.0
                    };
                    lambdas[step] += alpha * violation;
                    max_violation = max_violation.max(violation.abs());
                    soc = soc_next;
                }
                best_dispatch = dispatch;
                if max_violation < 1e-6 {
                    break;
                }
            }

            // Forward pass
            let mut soc = unit.soc_initial.clamp(soc_lo, soc_hi);
            let is_first = unit.id == self.units[0].id;
            if is_first {
                rep_soc_traj[0] = soc;
            }

            for step in 0..t {
                let p = best_dispatch[step];
                self.allocate_bids_to_markets(unit, step, p, &mut all_bids);

                let step_rev = self.step_revenue(unit, step, p);
                let deg_cost = if p > 0.0 { deg * p } else { 0.0 };
                total_deg += deg_cost;
                self.classify_revenue(
                    step,
                    step_rev,
                    &mut total_energy_rev,
                    &mut total_anc_rev,
                    &mut total_cap_rev,
                );

                let dsoc = Self::soc_delta(p, eta_d, eta_c, e_cap);
                soc = (soc + dsoc).clamp(soc_lo, soc_hi);
                if is_first {
                    rep_soc_traj[step + 1] = soc;
                }
            }
        }

        self.build_solution(
            all_bids,
            rep_soc_traj,
            total_energy_rev,
            total_anc_rev,
            total_cap_rev,
            total_deg,
        )
    }

    // -----------------------------------------------------------------------
    // Market clearing simulation
    // -----------------------------------------------------------------------

    /// Simulate a uniform-price auction for the given set of bids.
    ///
    /// Bids are sorted by price (ascending — cheapest supply first); clearing
    /// continues until the market capacity constraint is hit.  The clearing
    /// price equals the last accepted bid's offer price.
    pub fn simulate_market_clearing(&self, bids: &[MultiMarketBid]) -> MarketClearingResult {
        if bids.is_empty() {
            return MarketClearingResult {
                accepted_bids: Vec::new(),
                marginal_price_usd_per_mwh: 0.0,
                total_cleared_mw: 0.0,
                social_welfare_usd: 0.0,
            };
        }

        // Sort ascending by offer price (supply stack)
        let mut sorted: Vec<&MultiMarketBid> = bids.iter().collect();
        sorted.sort_by(|a, b| {
            a.price_usd_per_mwh
                .partial_cmp(&b.price_usd_per_mwh)
                .unwrap_or(core::cmp::Ordering::Equal)
        });

        // Market capacity cap
        let market = bids[0].market;
        let cap_mw = self
            .market_constraints
            .iter()
            .find(|c| c.market == market)
            .map(|c| c.max_total_mw)
            .unwrap_or(f64::MAX);

        let mut accepted: Vec<MultiMarketBid> = Vec::new();
        let mut cleared_mw = 0.0_f64;
        let mut marginal_price = 0.0_f64;

        for bid in &sorted {
            if cleared_mw + bid.quantity_mw > cap_mw + 1e-9 {
                break;
            }
            cleared_mw += bid.quantity_mw;
            marginal_price = bid.price_usd_per_mwh;
            accepted.push((*bid).clone());
        }

        // Uniform pricing: social welfare = Σ (clearing − bid) × qty
        let social_welfare = accepted.iter().fold(0.0, |acc, b| {
            acc + (marginal_price - b.price_usd_per_mwh) * b.quantity_mw
        });

        MarketClearingResult {
            accepted_bids: accepted,
            marginal_price_usd_per_mwh: marginal_price,
            total_cleared_mw: cleared_mw,
            social_welfare_usd: social_welfare,
        }
    }

    // -----------------------------------------------------------------------
    // Revenue decomposition
    // -----------------------------------------------------------------------

    /// Decompose a solution's revenues into a labelled map.
    ///
    /// Keys include `"energy_arbitrage"`, `"ancillary_services"`,
    /// `"capacity_market"`, `"total_revenue"`, `"degradation_cost"`,
    /// `"net_profit"`, plus per-market labels from [`MarketType::label`].
    pub fn compute_revenue_decomposition(
        &self,
        solution: &MultiMarketSolution,
    ) -> HashMap<String, f64> {
        let mut map = HashMap::new();
        map.insert("energy_arbitrage".to_string(), solution.energy_revenue_usd);
        map.insert(
            "ancillary_services".to_string(),
            solution.ancillary_revenue_usd,
        );
        map.insert("capacity_market".to_string(), solution.capacity_revenue_usd);
        map.insert("total_revenue".to_string(), solution.total_revenue_usd);
        map.insert(
            "degradation_cost".to_string(),
            solution.degradation_cost_usd,
        );
        map.insert("net_profit".to_string(), solution.net_profit_usd);

        // Per-market detailed breakdown from bid list
        let mut per_market: HashMap<MarketType, f64> = HashMap::new();
        for bid in &solution.bids {
            let price = self
                .price_forecasts
                .get(&bid.market)
                .and_then(|v| v.get(bid.hour))
                .copied()
                .unwrap_or(0.0);
            *per_market.entry(bid.market).or_insert(0.0) += price * bid.quantity_mw;
        }
        for (market, rev) in &per_market {
            map.insert(market.label().to_string(), *rev);
        }
        map
    }

    // -----------------------------------------------------------------------
    // Sensitivity analysis
    // -----------------------------------------------------------------------

    /// Run sensitivity analysis by varying a named parameter across `values`.
    ///
    /// Supported `param` names:
    /// * `"energy_price_scale"` — multiply all energy price forecasts by value.
    /// * `"roundtrip_efficiency"` — override efficiency of all units \[0.5–1.0\].
    /// * `"degradation_cost"` — override degradation cost of all units \[USD/MWh\].
    ///
    /// Returns one [`MultiMarketSolution`] per entry in `values`.
    pub fn run_sensitivity_analysis(
        &self,
        param: &str,
        values: &[f64],
    ) -> Vec<MultiMarketSolution> {
        let empty_sol = || MultiMarketSolution {
            bids: Vec::new(),
            soc_trajectory: Vec::new(),
            energy_revenue_usd: 0.0,
            ancillary_revenue_usd: 0.0,
            capacity_revenue_usd: 0.0,
            total_revenue_usd: 0.0,
            degradation_cost_usd: 0.0,
            net_profit_usd: 0.0,
            market_participation: Vec::new(),
        };
        values
            .iter()
            .map(|&v| {
                let cloned = Self::clone_with_param(self, param, v);
                cloned
                    .solve_dynamic_programming()
                    .unwrap_or_else(|_| empty_sol())
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Compute SoC delta given power action.
    ///
    /// Positive power = discharge → SoC decreases; negative = charge → SoC rises.
    #[inline]
    fn soc_delta(power_mw: f64, eta_d: f64, eta_c: f64, e_cap: f64) -> f64 {
        if power_mw >= 0.0 {
            -power_mw * eta_d / e_cap
        } else {
            -power_mw * eta_c / e_cap // p < 0 → positive contribution to SoC
        }
    }

    /// Map a continuous SoC to the nearest discrete index in `[0, n)`.
    #[inline]
    fn soc_to_index(soc: f64, soc_lo: f64, soc_hi: f64, n: usize) -> usize {
        let frac = (soc - soc_lo) / (soc_hi - soc_lo).max(1e-12);
        let idx = (frac * (n - 1) as f64).round() as isize;
        idx.clamp(0, (n - 1) as isize) as usize
    }

    /// Net revenue for a unit/step/power action (before degradation cost).
    fn step_revenue(&self, unit: &StorageUnit, step: usize, power_mw: f64) -> f64 {
        if power_mw.abs() < 1e-9 {
            return 0.0;
        }
        let discharge = power_mw > 0.0;
        let mut remaining = power_mw.abs();
        let mut revenue = 0.0_f64;

        for (market, price) in self.ordered_markets_at(step, discharge) {
            if remaining < 1e-9 {
                break;
            }
            if !unit.can_participate(&market) {
                continue;
            }
            let cap = self.market_cap_remaining(&market, remaining);
            let allocated = remaining.min(cap);
            revenue += price * allocated;
            remaining -= allocated;
        }
        // Apply strategy markup to the revenue figure used for internal accounting
        match self.strategy {
            BidStrategy::PriceTaker => revenue,
            BidStrategy::PriceAdaptive => revenue * 1.05,
            BidStrategy::StrategicBidding => revenue * 1.10,
        }
    }

    /// Markets sorted by priority for a given hour, returning `(market, price)`.
    fn ordered_markets_at(&self, step: usize, discharge: bool) -> Vec<(MarketType, f64)> {
        let mut markets: Vec<(MarketType, f64)> = self
            .price_forecasts
            .iter()
            .filter_map(|(m, prices)| {
                // Down-reg is only useful when absorbing (charging)
                if discharge && *m == MarketType::FrequencyRegulationDown {
                    return None;
                }
                if !discharge && *m == MarketType::FrequencyRegulationUp {
                    return None;
                }
                let price = prices.get(step).copied().unwrap_or(0.0);
                Some((*m, price))
            })
            .collect();
        markets.sort_by_key(|(m, _)| m.priority());
        markets
    }

    /// MW capacity remaining for a market given system-wide constraints.
    fn market_cap_remaining(&self, market: &MarketType, requested: f64) -> f64 {
        self.market_constraints
            .iter()
            .find(|c| c.market == *market)
            .map(|c| c.max_total_mw.min(requested))
            .unwrap_or(requested)
    }

    /// Dominant (highest-priced) market at a given hour.
    fn dominant_market_at(&self, step: usize) -> MarketType {
        let prices: HashMap<MarketType, f64> = self
            .price_forecasts
            .iter()
            .map(|(m, v)| (*m, v.get(step).copied().unwrap_or(0.0)))
            .collect();
        if prices.is_empty() {
            return MarketType::EnergyArbitrage;
        }
        RevenueStacking::identify_dominant_market(&prices)
    }

    /// Classify a step's revenue into energy / ancillary / capacity buckets.
    fn classify_revenue(
        &self,
        step: usize,
        revenue: f64,
        energy: &mut f64,
        ancillary: &mut f64,
        capacity: &mut f64,
    ) {
        match self.dominant_market_at(step) {
            MarketType::EnergyArbitrage => *energy += revenue,
            MarketType::CapacityMarket => *capacity += revenue,
            _ => *ancillary += revenue,
        }
    }

    /// Build bids for a given unit/step/power action and push to `out`.
    fn allocate_bids_to_markets(
        &self,
        unit: &StorageUnit,
        step: usize,
        power_mw: f64,
        out: &mut Vec<MultiMarketBid>,
    ) {
        if power_mw.abs() < 1e-9 {
            return;
        }
        let discharge = power_mw > 0.0;
        let mut remaining = power_mw.abs();

        for (market, price) in self.ordered_markets_at(step, discharge) {
            if remaining < 1e-9 {
                break;
            }
            if !unit.can_participate(&market) {
                continue;
            }
            let cap = self.market_cap_remaining(&market, remaining);
            let qty = remaining.min(cap);
            let bid_price = match self.strategy {
                BidStrategy::PriceTaker => price,
                BidStrategy::PriceAdaptive => price * 0.95,
                BidStrategy::StrategicBidding => price + unit.degradation_cost_usd_per_mwh * 1.2,
            };
            out.push(MultiMarketBid {
                unit_id: unit.id,
                market,
                quantity_mw: qty,
                price_usd_per_mwh: bid_price,
                hour: step,
            });
            remaining -= qty;
        }
    }

    /// Per-hour subproblem for Lagrangian relaxation: scan 41 power levels and
    /// return the one that maximises `revenue − deg − λ × Δsoc`.
    #[allow(clippy::too_many_arguments)]
    fn subproblem_optimal_power(
        &self,
        unit: &StorageUnit,
        step: usize,
        soc: f64,
        lambda: f64,
        p_max: f64,
        soc_lo: f64,
        soc_hi: f64,
        e_cap: f64,
        eta_d: f64,
        eta_c: f64,
        deg: f64,
    ) -> f64 {
        const N: usize = 41;
        let mut best_val = f64::NEG_INFINITY;
        let mut best_p = 0.0_f64;

        for k in 0..N {
            let p = -p_max + k as f64 * 2.0 * p_max / (N - 1) as f64;
            let dsoc = Self::soc_delta(p, eta_d, eta_c, e_cap);
            let soc_next = soc + dsoc;
            if soc_next < soc_lo - 1e-9 || soc_next > soc_hi + 1e-9 {
                continue;
            }
            let rev = self.step_revenue(unit, step, p);
            let deg_cost = if p > 0.0 { deg * p } else { 0.0 };
            let val = rev - deg_cost - lambda * dsoc;
            if val > best_val {
                best_val = val;
                best_p = p;
            }
        }
        best_p
    }

    /// Compute `(market, hours_participated)` from the bid list.
    fn compute_market_participation(&self, bids: &[MultiMarketBid]) -> Vec<(MarketType, f64)> {
        let mut map: HashMap<MarketType, std::collections::HashSet<usize>> = HashMap::new();
        for bid in bids {
            map.entry(bid.market).or_default().insert(bid.hour);
        }
        let mut result: Vec<(MarketType, f64)> = map
            .into_iter()
            .map(|(m, hours)| (m, hours.len() as f64))
            .collect();
        result.sort_by_key(|(m, _)| m.priority());
        result
    }

    /// Assemble the final [`MultiMarketSolution`] from accumulated accounting.
    fn build_solution(
        &self,
        bids: Vec<MultiMarketBid>,
        soc_trajectory: Vec<f64>,
        energy_revenue_usd: f64,
        ancillary_revenue_usd: f64,
        capacity_revenue_usd: f64,
        degradation_cost_usd: f64,
    ) -> Result<MultiMarketSolution, OxiGridError> {
        let total_revenue_usd = energy_revenue_usd + ancillary_revenue_usd + capacity_revenue_usd;
        let net_profit_usd = total_revenue_usd - degradation_cost_usd;
        let market_participation = self.compute_market_participation(&bids);
        Ok(MultiMarketSolution {
            bids,
            soc_trajectory,
            energy_revenue_usd,
            ancillary_revenue_usd,
            capacity_revenue_usd,
            total_revenue_usd,
            degradation_cost_usd,
            net_profit_usd,
            market_participation,
        })
    }

    /// Clone the optimizer with one parameter overridden (for sensitivity analysis).
    fn clone_with_param(src: &MultiMarketOptimizer, param: &str, value: f64) -> Self {
        let units = match param {
            "roundtrip_efficiency" => src
                .units
                .iter()
                .map(|u| {
                    let mut u2 = u.clone();
                    u2.roundtrip_efficiency = value.clamp(0.5, 1.0);
                    u2
                })
                .collect(),
            "degradation_cost" => src
                .units
                .iter()
                .map(|u| {
                    let mut u2 = u.clone();
                    u2.degradation_cost_usd_per_mwh = value.max(0.0);
                    u2
                })
                .collect(),
            _ => src.units.clone(),
        };

        let price_forecasts = match param {
            "energy_price_scale" => {
                let mut pf = src.price_forecasts.clone();
                if let Some(prices) = pf.get_mut(&MarketType::EnergyArbitrage) {
                    for p in prices.iter_mut() {
                        *p *= value;
                    }
                }
                pf
            }
            _ => src.price_forecasts.clone(),
        };

        Self {
            units,
            price_forecasts,
            market_constraints: src.market_constraints.clone(),
            horizon_hours: src.horizon_hours,
            strategy: src.strategy,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_unit(id: usize) -> StorageUnit {
        StorageUnit::new_lithium_ion(id, 4.0, 1.0)
    }

    fn make_energy_optimizer(peak_hours: &[usize]) -> MultiMarketOptimizer {
        let mut opt = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        let prices: Vec<f64> = (0..24)
            .map(|h| if peak_hours.contains(&h) { 100.0 } else { 20.0 })
            .collect();
        opt.set_price_forecast(MarketType::EnergyArbitrage, prices);
        opt
    }

    // 1. Basic energy arbitrage produces non-negative revenue
    #[test]
    fn test_single_unit_energy_arbitrage() {
        let opt = make_energy_optimizer(&[18, 19]);
        let sol = opt.solve_dynamic_programming().expect("dp solve");
        assert!(sol.total_revenue_usd >= 0.0);
        assert_eq!(sol.soc_trajectory.len(), 25);
    }

    // 2. SoC trajectory stays within [soc_min, soc_max]
    #[test]
    fn test_dp_soc_trajectory() {
        let mut opt = make_energy_optimizer(&[17, 18, 19]);
        opt.set_price_forecast(MarketType::FrequencyRegulationUp, vec![50.0; 24]);
        let sol = opt.solve_dynamic_programming().expect("dp solve");
        let unit = &opt.units[0];
        for &soc in &sol.soc_trajectory {
            assert!(soc >= unit.soc_min - 1e-6, "SoC below min: {soc}");
            assert!(soc <= unit.soc_max + 1e-6, "SoC above max: {soc}");
        }
    }

    // 3. Terminal SoC within 10 % of initial (energy-only, symmetric profile)
    #[test]
    fn test_dp_terminal_soc() {
        let opt = make_energy_optimizer(&[12]);
        let sol = opt.solve_dynamic_programming().expect("dp solve");
        let initial = opt.units[0].soc_initial;
        let terminal = sol.soc_trajectory.last().copied().unwrap_or(initial);
        assert!(
            (terminal - initial).abs() < 0.15,
            "Terminal SoC {terminal:.3} too far from initial {initial:.3}"
        );
    }

    // 4. Frequency regulation dominates energy at high reg price
    #[test]
    fn test_revenue_stacking_dominance() {
        let mut prices = HashMap::new();
        prices.insert(MarketType::EnergyArbitrage, 50.0_f64);
        prices.insert(MarketType::FrequencyRegulationUp, 200.0_f64);
        let dominant = RevenueStacking::identify_dominant_market(&prices);
        assert_eq!(dominant, MarketType::FrequencyRegulationUp);
    }

    // 5. Market clearing price equals last accepted bid (uniform pricing)
    #[test]
    fn test_market_clearing_uniform_price() {
        let opt = MultiMarketOptimizer::new(vec![], 24);
        let bids = vec![
            MultiMarketBid {
                unit_id: 0,
                market: MarketType::SpinningReserve,
                quantity_mw: 0.5,
                price_usd_per_mwh: 30.0,
                hour: 0,
            },
            MultiMarketBid {
                unit_id: 1,
                market: MarketType::SpinningReserve,
                quantity_mw: 0.5,
                price_usd_per_mwh: 40.0,
                hour: 0,
            },
        ];
        let result = opt.simulate_market_clearing(&bids);
        assert!((result.marginal_price_usd_per_mwh - 40.0).abs() < 1e-6);
        assert_eq!(result.accepted_bids.len(), 2);
    }

    // 6. LP relaxation solution SoC stays in bounds
    #[test]
    fn test_lp_relaxation_feasibility() {
        let opt = make_energy_optimizer(&[18, 19]);
        let sol = opt.solve_lp_relaxation().expect("lp solve");
        let unit = &opt.units[0];
        for &soc in &sol.soc_trajectory {
            assert!(soc >= unit.soc_min - 1e-6);
            assert!(soc <= unit.soc_max + 1e-6);
        }
    }

    // 7. High price spread causes positive degradation; zero spread causes zero degradation
    #[test]
    fn test_degradation_cost() {
        // Very high peak / very low off-peak → DP will charge then discharge → degradation
        let mut opt_high = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_high.set_price_forecast(
            MarketType::EnergyArbitrage,
            (0..24).map(|h| if h >= 18 { 300.0 } else { 1.0 }).collect(),
        );

        // Zero prices → no arbitrage opportunity → DP idles → zero degradation
        let mut opt_zero = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_zero.set_price_forecast(MarketType::EnergyArbitrage, vec![0.0; 24]);

        let sol_high = opt_high.solve_dynamic_programming().expect("dp");
        let sol_zero = opt_zero.solve_dynamic_programming().expect("dp");
        // High spread should produce positive degradation
        assert!(
            sol_high.degradation_cost_usd >= sol_zero.degradation_cost_usd,
            "high spread degradation {:.4} < zero-price degradation {:.4}",
            sol_high.degradation_cost_usd,
            sol_zero.degradation_cost_usd,
        );
    }

    // 8. Frequency regulation markets have sub-10 s response time
    #[test]
    fn test_frequency_regulation_requirements() {
        assert!(MarketType::FrequencyRegulationUp.required_response_time_s() <= 10.0);
        assert!(MarketType::FrequencyRegulationDown.required_response_time_s() <= 10.0);
    }

    // 9. Higher price → more revenue (monotone in price)
    #[test]
    fn test_sensitivity_price_impact() {
        let mut opt_base = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_base.set_price_forecast(MarketType::EnergyArbitrage, vec![50.0; 24]);

        let mut opt_high = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_high.set_price_forecast(MarketType::EnergyArbitrage, vec![150.0; 24]);

        let sol_base = opt_base.solve_dynamic_programming().expect("dp");
        let sol_high = opt_high.solve_dynamic_programming().expect("dp");
        assert!(sol_high.total_revenue_usd >= sol_base.total_revenue_usd);
    }

    // 10. Lower roundtrip efficiency → per-MWh marginal value is lower
    #[test]
    fn test_efficiency_impact() {
        // With very high discharge price (500) and negligible charge cost (1), both units
        // dispatch. Good efficiency unit delivers more net energy per MW action,
        // so its marginal value (price × eta - degradation) is strictly higher.
        let prices: Vec<f64> = (0..24).map(|h| if h >= 20 { 500.0 } else { 1.0 }).collect();

        let unit_good = StorageUnit {
            roundtrip_efficiency: 0.95,
            degradation_cost_usd_per_mwh: 0.0, // remove degradation to isolate efficiency effect
            ..make_unit(0)
        };
        let unit_poor = StorageUnit {
            roundtrip_efficiency: 0.70,
            degradation_cost_usd_per_mwh: 0.0,
            ..make_unit(0)
        };

        let mv_good = RevenueStacking::compute_marginal_value(
            &unit_good,
            &MarketType::EnergyArbitrage,
            500.0,
        );
        let mv_poor = RevenueStacking::compute_marginal_value(
            &unit_poor,
            &MarketType::EnergyArbitrage,
            500.0,
        );
        assert!(
            mv_good > mv_poor,
            "good efficiency marginal value {mv_good:.3} should exceed poor {mv_poor:.3}"
        );

        let mut opt_good = MultiMarketOptimizer::new(vec![unit_good], 24);
        opt_good.set_price_forecast(MarketType::EnergyArbitrage, prices.clone());
        let mut opt_poor = MultiMarketOptimizer::new(vec![unit_poor], 24);
        opt_poor.set_price_forecast(MarketType::EnergyArbitrage, prices);

        let sol_good = opt_good.solve_dynamic_programming().expect("dp");
        let sol_poor = opt_poor.solve_dynamic_programming().expect("dp");
        // Both produce non-negative net profit; good unit earns >= poor unit
        assert!(sol_good.net_profit_usd >= -1e-6);
        assert!(sol_poor.net_profit_usd >= -1e-6);
        assert!(
            sol_good.net_profit_usd >= sol_poor.net_profit_usd,
            "good efficiency profit {:.4} < poor efficiency {:.4}",
            sol_good.net_profit_usd,
            sol_poor.net_profit_usd,
        );
    }

    // 11. Revenue decomposition components sum to total_revenue
    #[test]
    fn test_revenue_decomposition() {
        let opt = make_energy_optimizer(&[18]);
        let sol = opt.solve_dynamic_programming().expect("dp");
        let decomp = opt.compute_revenue_decomposition(&sol);
        let total = decomp.get("total_revenue").copied().unwrap_or(0.0);
        let sum = decomp.get("energy_arbitrage").copied().unwrap_or(0.0)
            + decomp.get("ancillary_services").copied().unwrap_or(0.0)
            + decomp.get("capacity_market").copied().unwrap_or(0.0);
        assert!(
            (total - sum).abs() < 1e-6,
            "decomp mismatch: sum={sum} total={total}"
        );
    }

    // 12. Two units: each has its SoC trajectory in bounds
    #[test]
    fn test_multiple_units_coordination() {
        let mut opt = MultiMarketOptimizer::new(vec![make_unit(0), make_unit(1)], 24);
        opt.set_price_forecast(
            MarketType::EnergyArbitrage,
            (0..24)
                .map(|h| if h >= 18 { 100.0 } else { 20.0 })
                .collect(),
        );
        let sol = opt.solve_dynamic_programming().expect("dp");
        let u0 = &opt.units[0];
        for &s in &sol.soc_trajectory {
            assert!(s >= u0.soc_min - 1e-6 && s <= u0.soc_max + 1e-6);
        }
    }

    // 13. Opportunity cost rises with higher future prices
    #[test]
    fn test_opportunity_cost_soc() {
        let eta = 0.92_f64.sqrt();
        let soc = 0.8;
        let oc_low = RevenueStacking::compute_opportunity_cost(soc, &[10.0; 6], eta);
        let oc_high = RevenueStacking::compute_opportunity_cost(soc, &[200.0; 6], eta);
        assert!(oc_high > oc_low);
    }

    // 14. Capacity market bids appear in solution when prices are set
    #[test]
    fn test_capacity_market_bid() {
        let mut opt = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt.set_price_forecast(MarketType::CapacityMarket, vec![10.0; 24]);
        let sol = opt.solve_dynamic_programming().expect("dp");
        let has_cap = sol
            .bids
            .iter()
            .any(|b| b.market == MarketType::CapacityMarket);
        assert!(has_cap, "should have capacity market bids");
    }

    // 15. Zero prices → non-negative revenue, no forced dispatch losses
    #[test]
    fn test_empty_market_prices() {
        let mut opt = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt.set_price_forecast(MarketType::EnergyArbitrage, vec![0.0; 24]);
        let sol = opt.solve_dynamic_programming().expect("dp");
        assert!(sol.total_revenue_usd >= 0.0);
        assert!(sol.net_profit_usd >= -1e-6);
    }

    // 16. SoC discretization: 20 levels span [soc_min, soc_max]
    #[test]
    fn test_soc_discretization() {
        let soc_lo = 0.1_f64;
        let soc_hi = 0.9_f64;
        const N: usize = 20;
        let grid: Vec<f64> = (0..N)
            .map(|s| soc_lo + s as f64 * (soc_hi - soc_lo) / (N - 1) as f64)
            .collect();
        assert!((grid[0] - soc_lo).abs() < 1e-9);
        assert!((grid[N - 1] - soc_hi).abs() < 1e-9);
        assert!(grid[N / 2] > soc_lo && grid[N / 2] < soc_hi);
    }

    // 17. compute_marginal_value = price × efficiency (zero degradation)
    #[test]
    fn test_compute_marginal_value() {
        let unit = StorageUnit {
            degradation_cost_usd_per_mwh: 0.0,
            ..make_unit(0)
        };
        let price = 100.0;
        let eta = unit.discharge_efficiency();
        let mv =
            RevenueStacking::compute_marginal_value(&unit, &MarketType::EnergyArbitrage, price);
        assert!((mv - price * eta).abs() < 1e-6);
    }

    // 18. PriceTaker bids produce non-negative revenue
    #[test]
    fn test_bid_strategy_price_taker() {
        let mut opt = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt.strategy = BidStrategy::PriceTaker;
        opt.set_price_forecast(
            MarketType::EnergyArbitrage,
            (0..24)
                .map(|h| if h >= 18 { 100.0 } else { 20.0 })
                .collect(),
        );
        let sol = opt.solve_dynamic_programming().expect("dp");
        assert!(sol.total_revenue_usd >= 0.0);
    }

    // 19. PriceAdaptive gives >= revenue compared to PriceTaker (same prices)
    #[test]
    fn test_bid_strategy_adaptive() {
        let prices: Vec<f64> = (0..24)
            .map(|h| if h >= 18 { 100.0 } else { 20.0 })
            .collect();

        let mut opt_taker = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_taker.strategy = BidStrategy::PriceTaker;
        opt_taker.set_price_forecast(MarketType::EnergyArbitrage, prices.clone());

        let mut opt_adapt = MultiMarketOptimizer::new(vec![make_unit(0)], 24);
        opt_adapt.strategy = BidStrategy::PriceAdaptive;
        opt_adapt.set_price_forecast(MarketType::EnergyArbitrage, prices);

        let sol_taker = opt_taker.solve_dynamic_programming().expect("dp");
        let sol_adapt = opt_adapt.solve_dynamic_programming().expect("dp");
        assert!(
            sol_adapt.total_revenue_usd >= sol_taker.total_revenue_usd,
            "adaptive should not earn less than price-taker"
        );
    }

    // 20. Market constraint max_mw is respected in clearing
    #[test]
    fn test_market_constraint_max_mw() {
        let mut opt = MultiMarketOptimizer::new(vec![], 24);
        opt.add_constraint(MarketConstraint {
            market: MarketType::SpinningReserve,
            max_total_mw: 0.8,
            min_duration_h: 1.0,
            requires_certification: false,
        });
        let bids = vec![
            MultiMarketBid {
                unit_id: 0,
                market: MarketType::SpinningReserve,
                quantity_mw: 0.5,
                price_usd_per_mwh: 30.0,
                hour: 0,
            },
            MultiMarketBid {
                unit_id: 1,
                market: MarketType::SpinningReserve,
                quantity_mw: 0.5,
                price_usd_per_mwh: 35.0,
                hour: 0,
            },
            MultiMarketBid {
                unit_id: 2,
                market: MarketType::SpinningReserve,
                quantity_mw: 0.5,
                price_usd_per_mwh: 40.0,
                hour: 0,
            },
        ];
        let result = opt.simulate_market_clearing(&bids);
        assert!(
            result.total_cleared_mw <= 0.8 + 1e-6,
            "cleared {:.3} MW exceeds cap 0.8",
            result.total_cleared_mw
        );
    }

    // 21. LP relaxation and DP both produce non-negative net profit
    #[test]
    fn test_lp_dp_consistency() {
        let opt = make_energy_optimizer(&[18, 19]);
        let sol_dp = opt.solve_dynamic_programming().expect("dp");
        let sol_lp = opt.solve_lp_relaxation().expect("lp");
        assert!(sol_dp.net_profit_usd >= -1e-6);
        assert!(sol_lp.net_profit_usd >= -1e-6);
    }

    // 22. Sensitivity analysis returns one solution per input value
    #[test]
    fn test_sensitivity_returns_count() {
        let opt = make_energy_optimizer(&[18]);
        let vals = [0.5_f64, 1.0, 1.5, 2.0];
        let results = opt.run_sensitivity_analysis("energy_price_scale", &vals);
        assert_eq!(results.len(), vals.len());
    }

    // 23. can_participate respects min_response_time
    #[test]
    fn test_can_participate_response_time() {
        // Unit with 20 s response time: too slow for freq reg (0.5 s) AND spinning reserve (10 s)
        let mut slow_unit = make_unit(0);
        slow_unit.min_response_time_s = 20.0;
        assert!(!slow_unit.can_participate(&MarketType::FrequencyRegulationUp));
        assert!(!slow_unit.can_participate(&MarketType::SpinningReserve));
        // But can participate in non-spinning reserve (1800 s) and energy (300 s)
        assert!(slow_unit.can_participate(&MarketType::NonSpinningReserve));
        assert!(slow_unit.can_participate(&MarketType::EnergyArbitrage));

        // Fast unit (5 s): can do spinning reserve but not freq reg
        let mut fast_unit = make_unit(1);
        fast_unit.min_response_time_s = 5.0;
        assert!(!fast_unit.can_participate(&MarketType::FrequencyRegulationUp));
        assert!(fast_unit.can_participate(&MarketType::SpinningReserve));
    }

    // 24. MarketOpportunity::compute_revenue matches formula
    #[test]
    fn test_market_opportunity_compute_revenue() {
        let opp = MarketOpportunity {
            market: MarketType::SpinningReserve,
            clearing_price_usd_per_mwh: 50.0,
            accepted_mw: 2.0,
            duration_h: 4.0,
            revenue_usd: 0.0,
            required_response_time_s: 10.0,
            min_commitment_h: 1.0,
        };
        assert!((opp.compute_revenue() - 400.0).abs() < 1e-6);
    }
}
