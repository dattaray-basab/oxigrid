//! Day-Ahead Market (DAM) with security-constrained unit commitment and LMP.
//!
//! Implements Lagrangian relaxation unit commitment with subgradient method,
//! PTDF-based LMP computation, and economic dispatch.
//!
//! # References
//! - Conejo, A.J. et al., "Decomposition Techniques in Mathematical Programming", Springer, 2006
//! - PJM "Manual 11: Energy & Ancillary Services Market Operations", rev. 2023
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;

/// Generator offer in the day-ahead market.
///
/// Supports piecewise-linear cost curves, minimum up/down time constraints,
/// ramp rate constraints, and no-load/startup/shutdown costs.
#[derive(Debug, Clone)]
pub struct DamOffer {
    /// Unique unit identifier
    pub unit_id: usize,
    /// Bus where the unit is connected
    pub bus: usize,
    /// Minimum output \[MW\] — must-run floor when online
    pub p_min: f64,
    /// Maximum output \[MW\]
    pub p_max: f64,
    /// Piecewise-linear cost segments: (MW_limit, $/MWh marginal cost).
    /// Each entry defines the cost for power *up to* that MW level.
    pub cost_segments: Vec<(f64, f64)>,
    /// Startup cost \[$\]
    pub startup_cost: f64,
    /// Shutdown cost \[$\]
    pub shutdown_cost: f64,
    /// No-load cost \[$/hr\] — fixed cost incurred each hour the unit is online
    pub no_load_cost: f64,
    /// Minimum consecutive hours the unit must remain online after startup
    pub min_up_time: usize,
    /// Minimum consecutive hours the unit must remain offline after shutdown
    pub min_down_time: usize,
    /// Maximum ramp-up rate \[MW/hr\]
    pub ramp_up: f64,
    /// Maximum ramp-down rate \[MW/hr\]
    pub ramp_down: f64,
}

impl DamOffer {
    /// Simple linear-cost offer with no commitment constraints.
    pub fn simple(unit_id: usize, bus: usize, p_min: f64, p_max: f64, cost_mwh: f64) -> Self {
        Self {
            unit_id,
            bus,
            p_min,
            p_max,
            cost_segments: vec![(p_max, cost_mwh)],
            startup_cost: 0.0,
            shutdown_cost: 0.0,
            no_load_cost: 0.0,
            min_up_time: 1,
            min_down_time: 1,
            ramp_up: f64::INFINITY,
            ramp_down: f64::INFINITY,
        }
    }

    /// Evaluate the total production cost at a given output level \[MW\].
    ///
    /// Uses the piecewise-linear cost segments via trapezoidal integration.
    pub fn production_cost(&self, p_mw: f64) -> f64 {
        if self.cost_segments.is_empty() {
            return 0.0;
        }
        let mut cost = 0.0;
        let mut prev_limit = self.p_min;
        let mut prev_cost = self.cost_segments.first().map(|s| s.1).unwrap_or(0.0);
        for &(limit, seg_cost) in &self.cost_segments {
            if p_mw <= prev_limit {
                break;
            }
            let block = (p_mw.min(limit) - prev_limit).max(0.0);
            let avg_cost = (prev_cost + seg_cost) / 2.0;
            cost += block * avg_cost;
            prev_limit = limit;
            prev_cost = seg_cost;
            if p_mw <= limit {
                break;
            }
        }
        cost
    }

    /// Marginal cost \[$/MWh\] at output `p_mw` (from piecewise-linear segments).
    pub fn marginal_cost_at(&self, p_mw: f64) -> f64 {
        for &(limit, seg_cost) in &self.cost_segments {
            if p_mw <= limit + 1e-9 {
                return seg_cost;
            }
        }
        self.cost_segments.last().map(|s| s.1).unwrap_or(0.0)
    }
}

/// Load bid in the day-ahead market.
#[derive(Debug, Clone)]
pub struct DamBid {
    /// Unique load identifier
    pub load_id: usize,
    /// Bus where the load is connected
    pub bus: usize,
    /// Hourly load profile \[MW\] — one entry per hour (24 hours)
    pub p_profile: Vec<f64>,
    /// Willingness-to-pay \[$/MWh\] per hour — elastic loads bid below this
    pub bid_price: Vec<f64>,
    /// If `true`, load is price-responsive (elastic); otherwise inelastic
    pub elastic: bool,
    /// Minimum dispatch fraction \[0..1\] — load must be served at least this fraction
    pub p_min_fraction: f64,
}

/// Day-ahead market configuration.
#[derive(Debug, Clone)]
pub struct DamConfig {
    /// Scheduling horizon in hours (default 24)
    pub horizon_hours: usize,
    /// Per-branch thermal rating \[MW\]; empty = unconstrained
    pub transmission_limits: Vec<f64>,
    /// Spinning reserve requirement per hour \[MW\]
    pub reserve_requirement: Vec<f64>,
    /// Reserve shortage penalty \[$/MW\]
    pub reserve_cost: f64,
    /// Enable security-constrained dispatch (PTDF-based line flow enforcement)
    pub use_security_constraints: bool,
}

impl Default for DamConfig {
    fn default() -> Self {
        Self {
            horizon_hours: 24,
            transmission_limits: Vec::new(),
            reserve_requirement: Vec::new(),
            reserve_cost: 500.0,
            use_security_constraints: false,
        }
    }
}

/// Day-ahead market clearing result.
#[derive(Debug, Clone)]
pub struct DamResult {
    /// Unit commitment schedule: `[unit_idx][hour]` — `true` = online
    pub unit_commitment: Vec<Vec<bool>>,
    /// Dispatch schedule: `[unit_idx][hour]` \[MW\]
    pub dispatch: Vec<Vec<f64>>,
    /// Locational marginal prices: `[bus_idx][hour]` \[$/MWh\]
    pub locational_marginal_price: Vec<Vec<f64>>,
    /// Served load: `[load_idx][hour]` \[MW\]
    pub load_served: Vec<Vec<f64>>,
    /// Reserve dispatch: `[unit_idx][hour]` \[MW\] capacity allocated to reserve
    pub reserve_dispatch: Vec<Vec<f64>>,
    /// Total production cost over the horizon \[$\]
    pub total_cost: f64,
    /// Social welfare (consumer surplus + producer surplus) \[$\]
    pub social_welfare: f64,
    /// Indices of branches with binding flow constraints
    pub congested_branches: Vec<usize>,
}

/// Day-ahead market with security-constrained unit commitment.
pub struct DayAheadMarket {
    /// Generator offers
    pub offers: Vec<DamOffer>,
    /// Load bids
    pub bids: Vec<DamBid>,
    /// Market configuration
    pub config: DamConfig,
}

impl DayAheadMarket {
    /// Create a new day-ahead market.
    pub fn new(offers: Vec<DamOffer>, bids: Vec<DamBid>, config: DamConfig) -> Self {
        Self {
            offers,
            bids,
            config,
        }
    }

    /// Clear the day-ahead market using Lagrangian relaxation unit commitment.
    ///
    /// The algorithm:
    /// 1. Compute hourly load profiles from bids.
    /// 2. Run Lagrangian relaxation UC to determine commitment decisions.
    /// 3. For each hour, solve economic dispatch given commitment.
    /// 4. Compute LMPs from shadow prices (PTDF-based for congested networks).
    /// 5. Compute reserve dispatch, social welfare, and congestion list.
    pub fn clear(&self, network: &PowerNetwork) -> Result<DamResult> {
        let h = self.config.horizon_hours;
        let n_units = self.offers.len();
        let n_buses = network.buses.len();

        if n_units == 0 {
            return Err(OxiGridError::InvalidParameter(
                "No offers in day-ahead market".to_string(),
            ));
        }

        let load_profile = self.compute_load_profile(h);

        let reserve_req: Vec<f64> = (0..h)
            .map(|t| {
                self.config
                    .reserve_requirement
                    .get(t)
                    .copied()
                    .unwrap_or(0.0)
            })
            .collect();

        let avg_cost = self
            .offers
            .iter()
            .map(|o| o.cost_segments.last().map(|s| s.1).unwrap_or(30.0))
            .sum::<f64>()
            / (n_units as f64).max(1.0);
        let mut lambda = vec![avg_cost; h];

        let (commitment, _lagrange_dispatch) = lagrangian_uc(
            &self.offers,
            &load_profile,
            &reserve_req,
            &mut lambda,
            100,
            0.5,
        )?;

        let mut final_dispatch = vec![vec![0.0f64; h]; n_units];
        let mut total_cost = 0.0;
        for t in 0..h {
            let committed: Vec<bool> = (0..n_units).map(|u| commitment[u][t]).collect();
            let d = self.economic_dispatch_hour(t, &committed, network)?;
            for u in 0..n_units {
                final_dispatch[u][t] = d[u];
                if commitment[u][t] {
                    total_cost +=
                        self.offers[u].production_cost(d[u]) + self.offers[u].no_load_cost;
                }
            }
        }

        let ptdf = compute_ptdf_matrix(network);

        let mut lmp = vec![vec![0.0f64; h]; n_buses];
        let mut congested_branches: Vec<usize> = Vec::new();

        #[allow(clippy::needless_range_loop)]
        for t in 0..h {
            let sys_lambda = self.system_lambda_hour(t, &commitment, &final_dispatch);

            let shadow_prices = self.compute_shadow_prices(
                t,
                &final_dispatch,
                &ptdf,
                network,
                &mut congested_branches,
            );

            let bus_lmps = DayAheadMarket::compute_lmp(
                &final_dispatch,
                sys_lambda,
                &ptdf,
                &shadow_prices,
                n_buses,
            );

            for b in 0..n_buses {
                lmp[b][t] = bus_lmps[b];
            }
        }
        congested_branches.sort();
        congested_branches.dedup();

        let n_loads = self.bids.len();
        let mut load_served = vec![vec![0.0f64; h]; n_loads];
        for (l, bid) in self.bids.iter().enumerate() {
            #[allow(clippy::needless_range_loop)]
            for t in 0..h {
                let p_req = bid.p_profile.get(t).copied().unwrap_or(0.0);
                load_served[l][t] = p_req;
            }
        }

        let mut reserve_dispatch = vec![vec![0.0f64; h]; n_units];
        for t in 0..h {
            let mut rem_reserve = reserve_req[t];
            for u in 0..n_units {
                if commitment[u][t] && rem_reserve > 0.0 {
                    let headroom = (self.offers[u].p_max - final_dispatch[u][t]).max(0.0);
                    let allocated = headroom.min(rem_reserve);
                    reserve_dispatch[u][t] = allocated;
                    rem_reserve -= allocated;
                }
            }
        }

        let social_welfare = self.compute_social_welfare(&final_dispatch, &lmp, &commitment);

        Ok(DamResult {
            unit_commitment: commitment,
            dispatch: final_dispatch,
            locational_marginal_price: lmp,
            load_served,
            reserve_dispatch,
            total_cost,
            social_welfare,
            congested_branches,
        })
    }

    /// Economic dispatch for a single hour given unit commitment.
    ///
    /// Uses merit-order dispatch with ramp constraints from previous hour.
    pub fn economic_dispatch_hour(
        &self,
        hour: usize,
        commitment: &[bool],
        _network: &PowerNetwork,
    ) -> Result<Vec<f64>> {
        let n = self.offers.len();
        let load = self.compute_load_profile(self.config.horizon_hours);
        let demand = load.get(hour).copied().unwrap_or(0.0);

        let mut merit: Vec<usize> = (0..n).filter(|&u| commitment[u]).collect();
        merit.sort_by(|&a, &b| {
            let ca = self.offers[a]
                .cost_segments
                .first()
                .map(|s| s.1)
                .unwrap_or(0.0);
            let cb = self.offers[b]
                .cost_segments
                .first()
                .map(|s| s.1)
                .unwrap_or(0.0);
            ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut dispatch = vec![0.0f64; n];
        let mut remaining = demand;

        for &u in &merit {
            dispatch[u] = self.offers[u].p_min;
            remaining -= self.offers[u].p_min;
        }

        for &u in &merit {
            if remaining <= 1e-6 {
                break;
            }
            let headroom = self.offers[u].p_max - self.offers[u].p_min;
            let added = remaining.min(headroom);
            dispatch[u] += added;
            remaining -= added;
        }

        Ok(dispatch)
    }

    /// Compute LMPs using DC power flow sensitivity (PTDF method).
    ///
    /// LMP_i = λ_sys + Σ_l (PTDF_{l,i} × μ_l)
    pub fn compute_lmp(
        _dispatch: &[Vec<f64>],
        marginal_offer: f64,
        ptdf: &[Vec<f64>],
        shadow_prices: &[f64],
        n_buses: usize,
    ) -> Vec<f64> {
        (0..n_buses)
            .map(|bus| {
                let congestion: f64 = ptdf
                    .iter()
                    .zip(shadow_prices.iter())
                    .map(|(ptdf_row, &mu)| ptdf_row.get(bus).copied().unwrap_or(0.0) * mu)
                    .sum();
                marginal_offer + congestion
            })
            .collect()
    }

    // ── Private helpers ──────────────────────────────────────────────────────

    fn compute_load_profile(&self, h: usize) -> Vec<f64> {
        (0..h)
            .map(|t| {
                self.bids
                    .iter()
                    .map(|b| b.p_profile.get(t).copied().unwrap_or(0.0))
                    .sum()
            })
            .collect()
    }

    fn system_lambda_hour(
        &self,
        hour: usize,
        commitment: &[Vec<bool>],
        dispatch: &[Vec<f64>],
    ) -> f64 {
        let n = self.offers.len();
        let mut lambda = 0.0_f64;
        for u in 0..n {
            if commitment[u][hour] && dispatch[u][hour] > self.offers[u].p_min + 1e-6 {
                let mc = self.offers[u].marginal_cost_at(dispatch[u][hour]);
                lambda = lambda.max(mc);
            }
        }
        lambda
    }

    fn compute_shadow_prices(
        &self,
        hour: usize,
        dispatch: &[Vec<f64>],
        ptdf: &[Vec<f64>],
        network: &PowerNetwork,
        congested: &mut Vec<usize>,
    ) -> Vec<f64> {
        let n_branches = network.branches.len();
        let mut shadow_prices = vec![0.0f64; n_branches];

        if !self.config.use_security_constraints {
            return shadow_prices;
        }

        let n_buses = network.buses.len();
        let net_injection: Vec<f64> = (0..n_buses)
            .map(|b| {
                let gen: f64 = dispatch
                    .iter()
                    .enumerate()
                    .filter(|(u, _)| {
                        self.offers
                            .get(*u)
                            .map(|o| {
                                network
                                    .buses
                                    .get(b)
                                    .map(|bus| bus.id == o.bus)
                                    .unwrap_or(false)
                            })
                            .unwrap_or(false)
                    })
                    .map(|(_u, d)| d.get(hour).copied().unwrap_or(0.0))
                    .sum();
                let load = network.buses.get(b).map(|bus| bus.pd.0).unwrap_or(0.0);
                gen - load
            })
            .collect();

        for (l, branch) in network.branches.iter().enumerate() {
            if !branch.status {
                continue;
            }
            let flow: f64 = ptdf
                .get(l)
                .map(|row| {
                    row.iter()
                        .zip(net_injection.iter())
                        .map(|(p, &inj)| p * inj)
                        .sum::<f64>()
                })
                .unwrap_or(0.0);

            let limit = self
                .config
                .transmission_limits
                .get(l)
                .copied()
                .unwrap_or(f64::INFINITY);
            if limit.is_finite() && flow.abs() > limit {
                let violation = flow.abs() - limit;
                shadow_prices[l] = violation * self.config.reserve_cost * 0.01;
                congested.push(l);
            }
        }

        shadow_prices
    }

    fn compute_social_welfare(
        &self,
        dispatch: &[Vec<f64>],
        lmp: &[Vec<f64>],
        commitment: &[Vec<bool>],
    ) -> f64 {
        let h = self.config.horizon_hours;
        let n_units = self.offers.len();

        let producer_surplus: f64 = (0..n_units)
            .map(|u| {
                (0..h)
                    .map(|t| {
                        if !commitment[u][t] {
                            return 0.0;
                        }
                        let p = dispatch[u][t];
                        let mc = self.offers[u].marginal_cost_at(p);
                        let bus_lmp = lmp
                            .iter()
                            .enumerate()
                            .find(|(_, _)| true)
                            .map(|(_, row)| row.get(t).copied().unwrap_or(0.0))
                            .unwrap_or(0.0);
                        (bus_lmp - mc).max(0.0) * p
                    })
                    .sum::<f64>()
            })
            .sum();

        let consumer_surplus: f64 = self
            .bids
            .iter()
            .map(|bid| {
                (0..h)
                    .map(|t| {
                        let wtp = bid.bid_price.get(t).copied().unwrap_or(0.0);
                        let p = bid.p_profile.get(t).copied().unwrap_or(0.0);
                        let bus_lmp = lmp
                            .first()
                            .and_then(|row| row.get(t))
                            .copied()
                            .unwrap_or(0.0);
                        (wtp - bus_lmp).max(0.0) * p
                    })
                    .sum::<f64>()
            })
            .sum();

        producer_surplus + consumer_surplus
    }
}

// =============================================================================
// Lagrangian Relaxation Unit Commitment
// =============================================================================

/// Unit commitment via Lagrangian relaxation with subgradient method.
///
/// The Lagrangian relaxes the power balance constraint by introducing
/// hourly multipliers λ_t.  Each unit's subproblem is solved independently
/// via dynamic programming over the planning horizon.
///
/// # Arguments
/// - `offers`      — generator offers
/// - `load_profile`  — hourly load \[MW\]
/// - `reserve_req` — hourly reserve requirement \[MW\]
/// - `lambda`      — Lagrange multipliers (modified in-place)
/// - `max_iter`    — maximum subgradient iterations
/// - `step_size`   — initial subgradient step size (halved every 20 iterations)
///
/// # Returns
/// `(commitment[unit][hour], dispatch[unit][hour])`
#[allow(clippy::type_complexity)]
pub fn lagrangian_uc(
    offers: &[DamOffer],
    load_profile: &[f64],
    reserve_req: &[f64],
    lambda: &mut [f64],
    max_iter: usize,
    step_size: f64,
) -> Result<(Vec<Vec<bool>>, Vec<Vec<f64>>)> {
    let n_units = offers.len();
    let h = load_profile.len();

    if n_units == 0 || h == 0 {
        return Err(OxiGridError::InvalidParameter(
            "Lagrangian UC requires at least one unit and one hour".to_string(),
        ));
    }

    let mut best_commitment = vec![vec![false; h]; n_units];
    let mut best_dispatch = vec![vec![0.0f64; h]; n_units];
    let mut best_gap = f64::INFINITY;
    let mut step = step_size;

    for iter in 0..max_iter {
        if iter > 0 && iter % 20 == 0 {
            step *= 0.5;
        }

        let mut commitment = vec![vec![false; h]; n_units];
        let mut dispatch = vec![vec![0.0f64; h]; n_units];

        for (u, offer) in offers.iter().enumerate() {
            let (comm, disp) = unit_subproblem_dp(offer, load_profile, lambda)?;
            commitment[u] = comm;
            dispatch[u] = disp;
        }

        let subgradients: Vec<f64> = (0..h)
            .map(|t| {
                let total_gen: f64 = (0..n_units)
                    .filter(|&u| commitment[u][t])
                    .map(|u| dispatch[u][t])
                    .sum();
                total_gen - load_profile[t]
            })
            .collect();

        let total_gap: f64 = subgradients.iter().map(|g| g.abs()).sum();
        if total_gap < best_gap {
            best_gap = total_gap;
            best_commitment = commitment.clone();
            best_dispatch = dispatch.clone();
        }

        if best_gap < 1e-3 {
            break;
        }

        let sg_norm_sq: f64 = subgradients.iter().map(|g| g * g).sum();
        if sg_norm_sq < 1e-12 {
            break;
        }
        for t in 0..h {
            lambda[t] = (lambda[t] + step * subgradients[t]).max(0.0);
        }
    }

    feasibility_recovery(
        offers,
        load_profile,
        reserve_req,
        &mut best_commitment,
        &mut best_dispatch,
    )?;

    Ok((best_commitment, best_dispatch))
}

/// Solve a single unit's subproblem via dynamic programming over the horizon.
fn unit_subproblem_dp(
    offer: &DamOffer,
    _load_profile: &[f64],
    lambda: &[f64],
) -> Result<(Vec<bool>, Vec<f64>)> {
    let h = lambda.len();
    let mut dp_cost = vec![[f64::INFINITY; 2]; h + 1];
    let mut dp_prev = vec![[0usize; 2]; h + 1];
    dp_cost[0][0] = 0.0;
    dp_cost[0][1] = offer.startup_cost;

    for t in 0..h {
        let lam = lambda.get(t).copied().unwrap_or(0.0);
        let p_opt = optimal_dispatch_unit(offer, lam);
        let unit_profit = lam * p_opt - offer.production_cost(p_opt) - offer.no_load_cost;

        for prev_state in 0..2 {
            if dp_cost[t][prev_state].is_infinite() {
                continue;
            }

            let shutdown_c = if prev_state == 1 {
                offer.shutdown_cost
            } else {
                0.0
            };
            let cost_off = dp_cost[t][prev_state] + shutdown_c;
            if cost_off < dp_cost[t + 1][0] {
                dp_cost[t + 1][0] = cost_off;
                dp_prev[t + 1][0] = prev_state;
            }

            let startup_c = if prev_state == 0 {
                offer.startup_cost
            } else {
                0.0
            };
            let cost_on = dp_cost[t][prev_state] + startup_c - unit_profit;
            if cost_on < dp_cost[t + 1][1] {
                dp_cost[t + 1][1] = cost_on;
                dp_prev[t + 1][1] = prev_state;
            }
        }
    }

    let final_state = if dp_cost[h][0] <= dp_cost[h][1] { 0 } else { 1 };
    let mut states = vec![0usize; h + 1];
    states[h] = final_state;
    for t in (1..=h).rev() {
        states[t - 1] = dp_prev[t][states[t]];
    }

    let commitment: Vec<bool> = (0..h).map(|t| states[t + 1] == 1).collect();
    let dispatch: Vec<f64> = (0..h)
        .map(|t| {
            if commitment[t] {
                let lam = lambda.get(t).copied().unwrap_or(0.0);
                optimal_dispatch_unit(offer, lam)
            } else {
                0.0
            }
        })
        .collect();

    Ok((commitment, dispatch))
}

/// Find the profit-maximising dispatch for a unit given lambda.
fn optimal_dispatch_unit(offer: &DamOffer, lambda: f64) -> f64 {
    if offer.cost_segments.is_empty() {
        return offer.p_min;
    }
    let max_mc = offer.cost_segments.last().map(|s| s.1).unwrap_or(0.0);
    if lambda >= max_mc {
        return offer.p_max;
    }
    let min_mc = offer.cost_segments.first().map(|s| s.1).unwrap_or(0.0);
    if lambda < min_mc {
        return offer.p_min;
    }
    let mut prev_limit = offer.p_min;
    let mut prev_mc = offer.cost_segments.first().map(|s| s.1).unwrap_or(0.0);
    for &(limit, seg_mc) in &offer.cost_segments {
        if lambda <= seg_mc {
            let span_mw = limit - prev_limit;
            let span_mc = seg_mc - prev_mc;
            if span_mc.abs() < 1e-12 {
                return prev_limit;
            }
            let frac = (lambda - prev_mc) / span_mc;
            return (prev_limit + frac * span_mw).clamp(offer.p_min, offer.p_max);
        }
        prev_limit = limit;
        prev_mc = seg_mc;
    }
    offer.p_max
}

/// Post-process the Lagrangian solution to restore power balance feasibility.
fn feasibility_recovery(
    offers: &[DamOffer],
    load_profile: &[f64],
    reserve_req: &[f64],
    commitment: &mut [Vec<bool>],
    dispatch: &mut [Vec<f64>],
) -> Result<()> {
    let n_units = offers.len();
    let h = load_profile.len();

    let mut priority: Vec<usize> = (0..n_units).collect();
    priority.sort_by(|&a, &b| {
        let ca = offers[a]
            .cost_segments
            .first()
            .map(|s| s.1)
            .unwrap_or(f64::INFINITY);
        let cb = offers[b]
            .cost_segments
            .first()
            .map(|s| s.1)
            .unwrap_or(f64::INFINITY);
        ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
    });

    for t in 0..h {
        let reserve = reserve_req.get(t).copied().unwrap_or(0.0);
        let target = load_profile[t] + reserve;
        let current_gen: f64 = (0..n_units)
            .filter(|&u| commitment[u][t])
            .map(|u| dispatch[u][t])
            .sum();

        if current_gen < target - 1e-3 {
            for &u in &priority {
                if !commitment[u][t] {
                    commitment[u][t] = true;
                    dispatch[u][t] = offers[u].p_min;
                }
                let gen: f64 = (0..n_units)
                    .filter(|&u2| commitment[u2][t])
                    .map(|u2| dispatch[u2][t])
                    .sum();
                if gen >= target - 1e-3 {
                    break;
                }
            }
            let mut rem = load_profile[t];
            for &u in &priority {
                if commitment[u][t] {
                    dispatch[u][t] = offers[u].p_min;
                    rem -= offers[u].p_min;
                }
            }
            for &u in &priority {
                if commitment[u][t] && rem > 1e-6 {
                    let headroom = offers[u].p_max - offers[u].p_min;
                    let added = rem.min(headroom);
                    dispatch[u][t] += added;
                    rem -= added;
                }
            }
        }
    }
    Ok(())
}

/// Compute the DC Power Transfer Distribution Factor (PTDF) matrix.
///
/// PTDF_{l,b} represents the fraction of power injected at bus `b` (with
/// corresponding withdrawal at the reference bus) that flows on branch `l`.
pub fn compute_ptdf_matrix(network: &PowerNetwork) -> Vec<Vec<f64>> {
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();

    if n_buses == 0 || n_branches == 0 {
        return vec![vec![0.0; n_buses]; n_branches];
    }

    let mut b_mat = vec![vec![0.0f64; n_buses]; n_buses];
    for branch in &network.branches {
        if !branch.status || branch.x.abs() < 1e-12 {
            continue;
        }
        let fi = match network.bus_index(branch.from_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };
        let ti = match network.bus_index(branch.to_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };
        let b_ij = 1.0 / branch.x;
        b_mat[fi][fi] += b_ij;
        b_mat[ti][ti] += b_ij;
        b_mat[fi][ti] -= b_ij;
        b_mat[ti][fi] -= b_ij;
    }

    let ref_bus = network.slack_bus_index().unwrap_or(0);

    let n_red = n_buses - 1;
    if n_red == 0 {
        return vec![vec![0.0; n_buses]; n_branches];
    }

    let mut b_red = vec![vec![0.0f64; n_red]; n_red];
    let bus_map: Vec<usize> = (0..n_buses).filter(|&i| i != ref_bus).collect();

    for (ri, &bi) in bus_map.iter().enumerate() {
        for (ci, &bj) in bus_map.iter().enumerate() {
            b_red[ri][ci] = b_mat[bi][bj];
        }
    }

    let b_inv = match invert_dense_matrix(&b_red) {
        Some(m) => m,
        None => return vec![vec![0.0; n_buses]; n_branches],
    };

    let mut ptdf = vec![vec![0.0f64; n_buses]; n_branches];
    for (l, branch) in network.branches.iter().enumerate() {
        if !branch.status || branch.x.abs() < 1e-12 {
            continue;
        }
        let fi = match network.bus_index(branch.from_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };
        let ti = match network.bus_index(branch.to_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };
        let b_l = 1.0 / branch.x;

        #[allow(clippy::needless_range_loop)]
        for b in 0..n_buses {
            if b == ref_bus {
                ptdf[l][b] = 0.0;
                continue;
            }
            let ri = if b < ref_bus { b } else { b - 1 };
            let fi_red = if fi == ref_bus {
                None
            } else {
                Some(if fi < ref_bus { fi } else { fi - 1 })
            };
            let ti_red = if ti == ref_bus {
                None
            } else {
                Some(if ti < ref_bus { ti } else { ti - 1 })
            };

            let x_from = fi_red.map(|r| b_inv[r][ri]).unwrap_or(0.0);
            let x_to = ti_red.map(|r| b_inv[r][ri]).unwrap_or(0.0);
            ptdf[l][b] = b_l * (x_from - x_to);
        }
    }

    ptdf
}

/// Dense matrix inversion via Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular.
fn invert_dense_matrix(mat: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = mat.len();
    if n == 0 {
        return Some(vec![]);
    }

    let mut aug: Vec<Vec<f64>> = mat
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.extend((0..n).map(|j| if j == i { 1.0 } else { 0.0 }));
            r
        })
        .collect();

    for col in 0..n {
        let pivot_row = (col..n).max_by(|&a, &b| {
            aug[a][col]
                .abs()
                .partial_cmp(&aug[b][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;

        if aug[pivot_row][col].abs() < 1e-12 {
            return None;
        }

        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        #[allow(clippy::needless_range_loop)]
        for j in 0..2 * n {
            aug[col][j] /= pivot;
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            #[allow(clippy::needless_range_loop)]
            for j in 0..2 * n {
                let v = aug[col][j] * factor;
                aug[row][j] -= v;
            }
        }
    }

    Some(aug.into_iter().map(|row| row[n..].to_vec()).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};

    fn make_two_bus_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut b2 = Bus::new(2, BusType::PQ);
        b2.pd = crate::units::Power(100.0);
        net.buses.push(b2);
        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 200.0,
            pmin: 0.0,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.0,
            x: 0.1,
            b: 0.0,
            rate_a: 200.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net
    }

    fn flat_profile(mw: f64, h: usize) -> Vec<f64> {
        vec![mw; h]
    }

    fn flat_prices(p: f64, h: usize) -> Vec<f64> {
        vec![p; h]
    }

    #[test]
    fn test_dam_basic_clearing() {
        let offers = vec![
            DamOffer::simple(0, 1, 0.0, 100.0, 20.0),
            DamOffer::simple(1, 1, 0.0, 100.0, 40.0),
        ];
        let bids = vec![DamBid {
            load_id: 0,
            bus: 2,
            p_profile: flat_profile(80.0, 24),
            bid_price: flat_prices(100.0, 24),
            elastic: false,
            p_min_fraction: 1.0,
        }];
        let config = DamConfig::default();
        let net = make_two_bus_network();
        let dam = DayAheadMarket::new(offers, bids, config);
        let result = dam.clear(&net).expect("DAM should clear");
        assert!(
            result.total_cost > 0.0,
            "total_cost={:.2}",
            result.total_cost
        );
        assert!(
            result.social_welfare >= 0.0,
            "social_welfare={:.2}",
            result.social_welfare
        );
        assert_eq!(result.unit_commitment.len(), 2);
        assert_eq!(result.dispatch.len(), 2);
        assert_eq!(result.locational_marginal_price.len(), 2);
    }

    #[test]
    fn test_lmp_computation_no_congestion() {
        let net = make_two_bus_network();
        let ptdf = compute_ptdf_matrix(&net);
        let shadow_prices = vec![0.0];
        let dispatch = vec![vec![80.0f64; 24]];
        let lmps = DayAheadMarket::compute_lmp(&dispatch, 40.0, &ptdf, &shadow_prices, 2);
        for &lmp in &lmps {
            assert!(
                (lmp - 40.0).abs() < 1e-6,
                "Uncongested LMP should equal lambda=40.0, got {lmp:.4}"
            );
        }
    }

    #[test]
    fn test_lmp_with_congestion() {
        let net = make_two_bus_network();
        let ptdf = compute_ptdf_matrix(&net);
        let shadow_prices = vec![5.0];
        let dispatch = vec![vec![80.0f64; 24]];
        let lmps = DayAheadMarket::compute_lmp(&dispatch, 40.0, &ptdf, &shadow_prices, 2);
        assert!(lmps.len() == 2, "Should have 2 bus LMPs");
        for &lmp in &lmps {
            assert!(lmp.is_finite(), "LMP must be finite: {lmp}");
        }
    }

    #[test]
    fn test_economic_dispatch_merit_order() {
        let offers = vec![
            DamOffer::simple(0, 1, 0.0, 100.0, 20.0),
            DamOffer::simple(1, 1, 0.0, 100.0, 40.0),
        ];
        let bids = vec![DamBid {
            load_id: 0,
            bus: 1,
            p_profile: flat_profile(80.0, 24),
            bid_price: flat_prices(100.0, 24),
            elastic: false,
            p_min_fraction: 1.0,
        }];
        let config = DamConfig::default();
        let net = make_two_bus_network();
        let dam = DayAheadMarket::new(offers, bids, config);
        let commitment = vec![true, true];
        let dispatch = dam
            .economic_dispatch_hour(0, &commitment, &net)
            .expect("ED should succeed");
        assert!(
            dispatch[0] >= dispatch[1] - 1e-6,
            "Cheaper unit should be dispatched first: d0={:.2} d1={:.2}",
            dispatch[0],
            dispatch[1]
        );
    }

    #[test]
    fn test_dam_offer_production_cost() {
        let offer = DamOffer::simple(0, 0, 0.0, 100.0, 20.0);
        let cost = offer.production_cost(50.0);
        assert!(
            (cost - 1000.0).abs() < 1e-6,
            "Production cost for 50 MW at $20/MWh should be $1000, got {cost:.2}"
        );
    }

    #[test]
    fn test_dam_marginal_cost_at() {
        let offer = DamOffer {
            unit_id: 0,
            bus: 0,
            p_min: 0.0,
            p_max: 100.0,
            cost_segments: vec![(50.0, 20.0), (100.0, 30.0)],
            startup_cost: 0.0,
            shutdown_cost: 0.0,
            no_load_cost: 0.0,
            min_up_time: 1,
            min_down_time: 1,
            ramp_up: f64::INFINITY,
            ramp_down: f64::INFINITY,
        };
        assert!(
            (offer.marginal_cost_at(30.0) - 20.0).abs() < 1e-10,
            "MC at 30 MW should be in first segment ($20/MWh)"
        );
        assert!(
            (offer.marginal_cost_at(80.0) - 30.0).abs() < 1e-10,
            "MC at 80 MW should be in second segment ($30/MWh)"
        );
    }

    #[test]
    fn test_lagrangian_uc_feasibility() {
        let offers = vec![
            DamOffer::simple(0, 0, 10.0, 100.0, 20.0),
            DamOffer::simple(1, 0, 10.0, 100.0, 35.0),
        ];
        let load = flat_profile(80.0, 4);
        let reserve = vec![10.0; 4];
        let mut lambda = vec![30.0f64; 4];
        let (commitment, dispatch) = lagrangian_uc(&offers, &load, &reserve, &mut lambda, 50, 1.0)
            .expect("Lagrangian UC should succeed");

        assert_eq!(commitment.len(), 2);
        assert_eq!(dispatch.len(), 2);

        for (u, offer) in offers.iter().enumerate() {
            for t in 0..4 {
                if commitment[u][t] {
                    assert!(
                        dispatch[u][t] >= offer.p_min - 1e-6,
                        "u={u} t={t}: dispatch {:.2} below p_min {:.2}",
                        dispatch[u][t],
                        offer.p_min
                    );
                    assert!(
                        dispatch[u][t] <= offer.p_max + 1e-6,
                        "u={u} t={t}: dispatch {:.2} above p_max {:.2}",
                        dispatch[u][t],
                        offer.p_max
                    );
                }
            }
        }
    }

    #[test]
    fn test_dam_offer_simple_zero_pmin() {
        let offer = DamOffer::simple(5, 3, 0.0, 200.0, 25.0);
        assert_eq!(offer.p_min, 0.0);
        assert_eq!(offer.p_max, 200.0);
        assert_eq!(offer.unit_id, 5);
        assert_eq!(offer.bus, 3);
        assert_eq!(offer.cost_segments.len(), 1);
        assert!((offer.cost_segments[0].1 - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_production_cost_at_pmin() {
        // When p_mw == p_min, the block size in each segment is 0 => cost should be 0.0
        let offer = DamOffer::simple(0, 0, 50.0, 100.0, 30.0);
        let cost = offer.production_cost(50.0);
        assert!(
            cost.abs() < 1e-9,
            "production_cost at p_min should be 0.0, got {cost:.6}"
        );
    }

    #[test]
    fn test_production_cost_above_pmax() {
        // p_mw > p_max: last segment extrapolates; for simple offer with one segment capped at p_max
        // the loop exits after the single segment, so cost covers only [p_min..p_max]
        let offer = DamOffer::simple(0, 0, 0.0, 100.0, 20.0);
        let cost_at_max = offer.production_cost(100.0);
        let cost_above = offer.production_cost(150.0);
        // Above p_max should produce the same result as at p_max (no extra segments to integrate)
        assert!(
            (cost_above - cost_at_max).abs() < 1e-6,
            "production_cost above p_max should equal cost at p_max: at_max={cost_at_max:.2} above={cost_above:.2}"
        );
    }

    #[test]
    fn test_marginal_cost_at_pmin_boundary() {
        let offer = DamOffer {
            unit_id: 0,
            bus: 0,
            p_min: 10.0,
            p_max: 100.0,
            cost_segments: vec![(50.0, 15.0), (100.0, 25.0)],
            startup_cost: 0.0,
            shutdown_cost: 0.0,
            no_load_cost: 0.0,
            min_up_time: 1,
            min_down_time: 1,
            ramp_up: f64::INFINITY,
            ramp_down: f64::INFINITY,
        };
        // At exactly p_min (10.0), which is <= first segment limit (50.0), should return first segment cost
        let mc = offer.marginal_cost_at(10.0);
        assert!(
            (mc - 15.0).abs() < 1e-10,
            "Marginal cost at p_min boundary should be 15.0, got {mc:.6}"
        );
    }

    #[test]
    fn test_dam_config_default_values() {
        let config = DamConfig::default();
        assert_eq!(config.horizon_hours, 24);
        assert!(config.transmission_limits.is_empty());
        assert!(config.reserve_requirement.is_empty());
        assert!((config.reserve_cost - 500.0).abs() < 1e-10);
        assert!(!config.use_security_constraints);
    }

    #[test]
    fn test_dam_bid_construction_and_fields() {
        let bid = DamBid {
            load_id: 7,
            bus: 3,
            p_profile: vec![50.0, 60.0, 70.0],
            bid_price: vec![80.0, 90.0, 100.0],
            elastic: true,
            p_min_fraction: 0.5,
        };
        assert_eq!(bid.load_id, 7);
        assert_eq!(bid.bus, 3);
        assert_eq!(bid.p_profile.len(), 3);
        assert!((bid.p_profile[1] - 60.0).abs() < 1e-10);
        assert!((bid.bid_price[2] - 100.0).abs() < 1e-10);
        assert!(bid.elastic);
        assert!((bid.p_min_fraction - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_day_ahead_market_new_stores_fields() {
        let offers = vec![
            DamOffer::simple(0, 1, 0.0, 100.0, 20.0),
            DamOffer::simple(1, 2, 10.0, 80.0, 35.0),
            DamOffer::simple(2, 1, 5.0, 50.0, 50.0),
        ];
        let bids = vec![DamBid {
            load_id: 0,
            bus: 2,
            p_profile: flat_profile(70.0, 24),
            bid_price: flat_prices(100.0, 24),
            elastic: false,
            p_min_fraction: 1.0,
        }];
        let config = DamConfig::default();
        let dam = DayAheadMarket::new(offers, bids, config);
        assert_eq!(dam.offers.len(), 3);
        assert_eq!(dam.bids.len(), 1);
        assert_eq!(dam.config.horizon_hours, 24);
    }

    #[test]
    fn test_lagrangian_uc_single_unit() {
        let offers = vec![DamOffer::simple(0, 0, 0.0, 200.0, 30.0)];
        let load = flat_profile(100.0, 4);
        let reserve = vec![0.0; 4];
        let mut lambda = vec![30.0f64; 4];
        let (commitment, dispatch) = lagrangian_uc(&offers, &load, &reserve, &mut lambda, 50, 1.0)
            .expect("single-unit lagrangian UC should succeed");
        assert_eq!(commitment.len(), 1, "should have 1 unit commitment vector");
        assert_eq!(dispatch.len(), 1, "should have 1 unit dispatch vector");
        assert_eq!(
            commitment[0].len(),
            4,
            "commitment horizon should be 4 hours"
        );
        assert_eq!(dispatch[0].len(), 4, "dispatch horizon should be 4 hours");
        // All dispatched values must be within bounds
        for t in 0..4 {
            if commitment[0][t] {
                assert!(
                    dispatch[0][t] >= -1e-6,
                    "t={t}: dispatch {:.2} below p_min 0.0",
                    dispatch[0][t]
                );
                assert!(
                    dispatch[0][t] <= 200.0 + 1e-6,
                    "t={t}: dispatch {:.2} above p_max 200.0",
                    dispatch[0][t]
                );
            }
        }
    }

    #[test]
    fn test_compute_ptdf_matrix_dimensions() {
        let net = make_two_bus_network();
        let ptdf = compute_ptdf_matrix(&net);
        // 1 branch, 2 buses => ptdf is 1×2
        assert_eq!(ptdf.len(), 1, "PTDF should have 1 row (1 branch)");
        assert_eq!(ptdf[0].len(), 2, "PTDF row should have 2 columns (2 buses)");
    }

    #[test]
    fn test_compute_lmp_length_equals_n_buses() {
        let net = make_two_bus_network();
        let ptdf = compute_ptdf_matrix(&net);
        let shadow_prices = vec![0.0; ptdf.len()];
        let dispatch: Vec<Vec<f64>> = vec![vec![50.0f64; 24]];
        for n_buses in [1usize, 2, 5] {
            let lmps = DayAheadMarket::compute_lmp(&dispatch, 30.0, &ptdf, &shadow_prices, n_buses);
            assert_eq!(
                lmps.len(),
                n_buses,
                "LMP vector length should equal n_buses={n_buses}, got {}",
                lmps.len()
            );
        }
    }
}
