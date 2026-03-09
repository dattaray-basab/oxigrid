//! Flexible Ramping Product (FRP) Optimization.
//!
//! The FRP is a market product procured by grid operators to manage net-load
//! variability from renewable integration.  It reserves *headroom* (upward)
//! and *footroom* (downward) in generators so that the system can ramp quickly
//! if the net load forecast is wrong.
//!
//! # Algorithm (merit-order dispatch)
//!
//! For each hour `h`:
//! 1. Compute **available FRP up** for each generator:
//!    `avail_up = min(P_max − P_current, ramp_up_rate)`
//! 2. Compute **available FRP down** for each generator:
//!    `avail_down = min(P_current − P_min, ramp_down_rate)`
//! 3. Sort committed generators by energy cost (merit order, cheapest first).
//! 4. Assign FRP in merit order until the hourly requirement is met or
//!    all headroom/footroom is exhausted.
//! 5. Record unserved \[MW\] if the requirement cannot be fully met.
//!
//! # References
//! CAISO (2014) *Flexible Ramping Product Technical Analysis*.
//! Gu, Y. & Xie, L. (2014) "Stochastic Look-Ahead Economic Dispatch
//! With Variable Generation Resources", *IEEE Trans. Power Syst.*

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the FRP optimizer.
#[derive(Debug, Error)]
pub enum FrpError {
    /// Hour dimension mismatch between config and generator schedule.
    #[error("schedule length {got} does not match n_hours {expected}")]
    ScheduleLengthMismatch { got: usize, expected: usize },
    /// Invalid configuration.
    #[error("invalid FRP config: {0}")]
    InvalidConfig(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the Flexible Ramping Product optimizer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrpConfig {
    /// Number of dispatch hours.
    pub n_hours: usize,
    /// Number of generators.
    pub n_generators: usize,
    /// Required upward ramping capability \[MW\] per hour.
    pub ramping_requirement_mw_per_h: Vec<f64>,
    /// Required downward ramping capability \[MW\] per hour.
    pub down_ramp_requirement_mw_per_h: Vec<f64>,
    /// Payment rate for FRP capability \[USD/MW\].
    pub frp_price_usd_per_mw: f64,
    /// Spinning reserve price \[USD/MW\].
    pub reserve_price_usd_per_mw: f64,
}

// ─── Generator ────────────────────────────────────────────────────────────────

/// A generator participating in the FRP market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RampingGenerator {
    /// Generator identifier.
    pub id: usize,
    /// Minimum stable output \[MW\].
    pub p_min_mw: f64,
    /// Maximum rated output \[MW\].
    pub p_max_mw: f64,
    /// Maximum upward ramp capability \[MW/h\].
    pub ramp_up_mw_per_h: f64,
    /// Maximum downward ramp capability \[MW/h\].
    pub ramp_down_mw_per_h: f64,
    /// Marginal energy cost \[USD/MWh\].
    pub cost_per_mwh: f64,
    /// Hourly commitment schedule (`true` = online).
    pub commitment: Vec<bool>,
    /// Pre-determined energy dispatch \[MW\] per hour.
    pub dispatch_mw: Vec<f64>,
}

// ─── Result ───────────────────────────────────────────────────────────────────

/// Result of the FRP optimization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrpResult {
    /// Upward FRP allocated per generator per hour \[MW\]:
    /// `upward_frp_mw[hour][generator_index]`.
    pub upward_frp_mw: Vec<Vec<f64>>,
    /// Downward FRP allocated per generator per hour \[MW\]:
    /// `downward_frp_mw[hour][generator_index]`.
    pub downward_frp_mw: Vec<Vec<f64>>,
    /// Whether the hourly up-ramp requirement is met.
    pub frp_requirement_met: Vec<bool>,
    /// Total FRP procurement cost \[USD\].
    pub frp_cost_usd: f64,
    /// Unmet upward ramping requirement \[MW\] per hour.
    pub unserved_frp_mw: Vec<f64>,
    /// Hours where the upward FRP constraint is exactly tight (binding).
    pub binding_hours: Vec<usize>,
}

// ─── Optimizer ────────────────────────────────────────────────────────────────

/// Flexible Ramping Product optimizer.
///
/// # Example
/// ```rust
/// use oxigrid::optimize::dispatch::ramp_product::{
///     FrpConfig, FrpOptimizer, RampingGenerator,
/// };
///
/// let config = FrpConfig {
///     n_hours: 2,
///     n_generators: 1,
///     ramping_requirement_mw_per_h: vec![50.0, 50.0],
///     down_ramp_requirement_mw_per_h: vec![30.0, 30.0],
///     frp_price_usd_per_mw: 5.0,
///     reserve_price_usd_per_mw: 3.0,
/// };
/// let mut opt = FrpOptimizer::new(config);
/// opt.add_generator(RampingGenerator {
///     id: 0, p_min_mw: 0.0, p_max_mw: 200.0,
///     ramp_up_mw_per_h: 100.0, ramp_down_mw_per_h: 80.0,
///     cost_per_mwh: 30.0,
///     commitment: vec![true, true],
///     dispatch_mw: vec![100.0, 110.0],
/// });
/// let result = opt.optimize().unwrap();
/// assert!(result.frp_requirement_met[0]);
/// ```
pub struct FrpOptimizer {
    config: FrpConfig,
    generators: Vec<RampingGenerator>,
}

impl FrpOptimizer {
    /// Create a new FRP optimizer with the given configuration.
    pub fn new(config: FrpConfig) -> Self {
        Self {
            config,
            generators: Vec::new(),
        }
    }

    /// Register a generator for FRP market participation.
    pub fn add_generator(&mut self, gen: RampingGenerator) {
        self.generators.push(gen);
    }

    /// Run the merit-order FRP allocation.
    pub fn optimize(&self) -> Result<FrpResult, FrpError> {
        let n_h = self.config.n_hours;

        // Validate
        if self.config.ramping_requirement_mw_per_h.len() != n_h {
            return Err(FrpError::InvalidConfig(
                "ramping_requirement_mw_per_h length != n_hours".into(),
            ));
        }
        if self.config.down_ramp_requirement_mw_per_h.len() != n_h {
            return Err(FrpError::InvalidConfig(
                "down_ramp_requirement_mw_per_h length != n_hours".into(),
            ));
        }
        for gen in &self.generators {
            if gen.commitment.len() != n_h {
                return Err(FrpError::ScheduleLengthMismatch {
                    got: gen.commitment.len(),
                    expected: n_h,
                });
            }
            if gen.dispatch_mw.len() != n_h {
                return Err(FrpError::ScheduleLengthMismatch {
                    got: gen.dispatch_mw.len(),
                    expected: n_h,
                });
            }
        }

        let n_gen = self.generators.len();
        let mut upward_frp_mw: Vec<Vec<f64>> = vec![vec![0.0; n_gen]; n_h];
        let mut downward_frp_mw: Vec<Vec<f64>> = vec![vec![0.0; n_gen]; n_h];
        let mut frp_requirement_met: Vec<bool> = vec![false; n_h];
        let mut unserved_frp_mw: Vec<f64> = vec![0.0; n_h];
        let mut binding_hours: Vec<usize> = Vec::new();
        let mut total_frp_cost: f64 = 0.0;

        // Merit-order index: sort generators by cost (cheapest first)
        let mut merit_order: Vec<usize> = (0..n_gen).collect();
        merit_order.sort_by(|&a, &b| {
            self.generators[a]
                .cost_per_mwh
                .partial_cmp(&self.generators[b].cost_per_mwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for h in 0..n_h {
            let req_up = self.config.ramping_requirement_mw_per_h[h];
            let req_dn = self.config.down_ramp_requirement_mw_per_h[h];

            let mut remaining_up = req_up;
            let mut remaining_dn = req_dn;

            for &gi in &merit_order {
                let gen = &self.generators[gi];
                if !gen.commitment[h] {
                    continue; // offline generators cannot provide FRP
                }

                let dispatch = gen.dispatch_mw[h];

                // Upward FRP = min(headroom, ramp_up_rate)
                let headroom = (gen.p_max_mw - dispatch).max(0.0);
                let avail_up = headroom.min(gen.ramp_up_mw_per_h);
                let alloc_up = avail_up.min(remaining_up);
                upward_frp_mw[h][gi] = alloc_up;
                remaining_up -= alloc_up;
                total_frp_cost += alloc_up * self.config.frp_price_usd_per_mw;

                // Downward FRP = min(footroom, ramp_down_rate)
                let footroom = (dispatch - gen.p_min_mw).max(0.0);
                let avail_dn = footroom.min(gen.ramp_down_mw_per_h);
                let alloc_dn = avail_dn.min(remaining_dn);
                downward_frp_mw[h][gi] = alloc_dn;
                remaining_dn -= alloc_dn;
            }

            unserved_frp_mw[h] = remaining_up.max(0.0);
            frp_requirement_met[h] = remaining_up <= 1e-6; // effectively zero

            let served_up = req_up - remaining_up.max(0.0);
            // Binding: served within 1 MW of requirement
            if (served_up - req_up).abs() <= 1.0 && req_up > 0.0 {
                binding_hours.push(h);
            }
        }

        Ok(FrpResult {
            upward_frp_mw,
            downward_frp_mw,
            frp_requirement_met,
            frp_cost_usd: total_frp_cost,
            unserved_frp_mw,
            binding_hours,
        })
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn single_gen_config(n_hours: usize, req_up: f64) -> FrpConfig {
        FrpConfig {
            n_hours,
            n_generators: 1,
            ramping_requirement_mw_per_h: vec![req_up; n_hours],
            down_ramp_requirement_mw_per_h: vec![req_up * 0.5; n_hours],
            frp_price_usd_per_mw: 5.0,
            reserve_price_usd_per_mw: 3.0,
        }
    }

    fn gen_online(n_h: usize, pmax: f64, ramp: f64, dispatch: f64, cost: f64) -> RampingGenerator {
        RampingGenerator {
            id: 0,
            p_min_mw: 0.0,
            p_max_mw: pmax,
            ramp_up_mw_per_h: ramp,
            ramp_down_mw_per_h: ramp,
            cost_per_mwh: cost,
            commitment: vec![true; n_h],
            dispatch_mw: vec![dispatch; n_h],
        }
    }

    // ── Test 1 ─────────────────────────────────────────────────────────────────
    /// Sufficient ramp capacity → all hours met.
    #[test]
    fn test_sufficient_ramp_capacity_all_met() {
        let mut opt = FrpOptimizer::new(single_gen_config(3, 50.0));
        // P_max=200, dispatch=100 → headroom=100, ramp=120 → avail=100 > req=50
        opt.add_generator(gen_online(3, 200.0, 120.0, 100.0, 30.0));
        let res = opt.optimize().expect("optimize failed");
        for h in 0..3 {
            assert!(
                res.frp_requirement_met[h],
                "hour {h} requirement must be met"
            );
            assert_eq!(res.unserved_frp_mw[h], 0.0, "no unserved FRP at hour {h}");
        }
    }

    // ── Test 2 ─────────────────────────────────────────────────────────────────
    /// Generator at max output (no headroom) → unserved FRP = requirement.
    #[test]
    fn test_insufficient_capacity_unserved() {
        let mut opt = FrpOptimizer::new(single_gen_config(2, 50.0));
        // dispatch == P_max → headroom = 0
        opt.add_generator(gen_online(2, 100.0, 100.0, 100.0, 30.0));
        let res = opt.optimize().expect("optimize failed");
        for h in 0..2 {
            assert!(
                !res.frp_requirement_met[h],
                "requirement must be unmet when no headroom"
            );
            assert!(
                (res.unserved_frp_mw[h] - 50.0).abs() < 1e-9,
                "unserved must equal requirement when headroom=0"
            );
        }
    }

    // ── Test 3 ─────────────────────────────────────────────────────────────────
    /// Merit order: cheapest generator provides FRP first.
    #[test]
    fn test_merit_order_cheapest_first() {
        let n_h = 1;
        let config = FrpConfig {
            n_hours: n_h,
            n_generators: 2,
            ramping_requirement_mw_per_h: vec![30.0],
            down_ramp_requirement_mw_per_h: vec![20.0],
            frp_price_usd_per_mw: 10.0,
            reserve_price_usd_per_mw: 5.0,
        };
        let mut opt = FrpOptimizer::new(config);
        // Generator 0: expensive, plenty of headroom
        opt.add_generator(RampingGenerator {
            id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            ramp_up_mw_per_h: 100.0,
            ramp_down_mw_per_h: 80.0,
            cost_per_mwh: 80.0,
            commitment: vec![true],
            dispatch_mw: vec![100.0],
        });
        // Generator 1: cheap, plenty of headroom
        opt.add_generator(RampingGenerator {
            id: 1,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            ramp_up_mw_per_h: 100.0,
            ramp_down_mw_per_h: 80.0,
            cost_per_mwh: 20.0,
            commitment: vec![true],
            dispatch_mw: vec![100.0],
        });

        let res = opt.optimize().expect("optimize failed");
        // Cheap generator (index 1 in the array, first in merit order) should
        // provide the full 30 MW; expensive (index 0) should provide nothing.
        assert!(
            res.upward_frp_mw[0][1] >= 30.0 - 1e-9,
            "cheap generator (index 1) should provide 30 MW FRP"
        );
        assert!(
            res.upward_frp_mw[0][0] < 1e-9,
            "expensive generator should not be dispatched for FRP"
        );
    }

    // ── Test 4 ─────────────────────────────────────────────────────────────────
    /// Ramp rate constrains available FRP even when headroom is large.
    #[test]
    fn test_ramp_rate_constraint_limits_frp() {
        let mut opt = FrpOptimizer::new(single_gen_config(1, 100.0));
        // P_max=200, dispatch=0 → headroom=200, but ramp_up=40 → avail_up=40 < req=100
        opt.add_generator(gen_online(1, 200.0, 40.0, 0.0, 30.0));
        let res = opt.optimize().expect("optimize failed");
        assert!(
            !res.frp_requirement_met[0],
            "ramp-limited: requirement of 100 MW should not be met with ramp=40"
        );
        assert!(
            (res.unserved_frp_mw[0] - 60.0).abs() < 1e-9,
            "unserved = 100 - 40 = 60"
        );
        assert!(
            (res.upward_frp_mw[0][0] - 40.0).abs() < 1e-9,
            "allocated FRP = ramp limit = 40 MW"
        );
    }

    // ── Test 5 ─────────────────────────────────────────────────────────────────
    /// Binding hours correctly identified (hours where req is exactly met).
    #[test]
    fn test_binding_hours_identified() {
        let n_h = 3;
        let config = FrpConfig {
            n_hours: n_h,
            n_generators: 1,
            ramping_requirement_mw_per_h: vec![50.0, 80.0, 50.0],
            down_ramp_requirement_mw_per_h: vec![20.0; n_h],
            frp_price_usd_per_mw: 5.0,
            reserve_price_usd_per_mw: 3.0,
        };
        let mut opt = FrpOptimizer::new(config);
        // headroom = 200 - 100 = 100; ramp = 50 → avail_up = 50
        // Hours 0,2: req=50, served=50 → binding
        // Hour 1: req=80, served=50 → NOT binding (unserved)
        opt.add_generator(gen_online(n_h, 200.0, 50.0, 100.0, 30.0));
        let res = opt.optimize().expect("optimize failed");
        assert!(
            res.binding_hours.contains(&0),
            "hour 0 (req=50, avail=50) should be binding"
        );
        assert!(
            res.binding_hours.contains(&2),
            "hour 2 (req=50, avail=50) should be binding"
        );
    }

    // ── Test 6: cost is non-negative ──────────────────────────────────────────
    #[test]
    fn test_frp_cost_non_negative() {
        let mut opt = FrpOptimizer::new(single_gen_config(4, 20.0));
        opt.add_generator(gen_online(4, 150.0, 80.0, 80.0, 40.0));
        let res = opt.optimize().expect("optimize failed");
        assert!(res.frp_cost_usd >= 0.0);
    }
}
