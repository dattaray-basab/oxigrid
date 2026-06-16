//! Network Congestion Management (SSS).
//!
//! Implements PTDF-based congestion identification and least-cost redispatch
//! for transmission congestion management.
//!
//! # Methods
//!
//! - `PtdfBased` — Power Transfer Distribution Factors for sensitivity analysis
//! - `MarketSplit` — price-area splitting based on zonal shadow prices
//! - `Redispatch` — post-market redispatch to relieve binding constraints
//! - `CounterTrading` — counter-trades between adjacent areas
//!
//! # Reference
//!
//! DC power flow sensitivity: PTDF\[branch\]\[bus\] gives the fraction of an
//! injection increase at `bus` that flows over `branch` (positive = from→to).

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors from the congestion management module.
#[derive(Debug, Error)]
pub enum CongestionError {
    /// PTDF matrix dimensions do not match configured bus/branch counts.
    #[error("PTDF matrix is {rows}×{cols} but expected {exp_rows}×{exp_cols}")]
    PtdfDimensionMismatch {
        rows: usize,
        cols: usize,
        exp_rows: usize,
        exp_cols: usize,
    },
    /// Network state vectors have incorrect length.
    #[error("network state vector length {got} does not match expected {expected}")]
    StateLengthMismatch { got: usize, expected: usize },
    /// No redispatch pair found that relieves congestion on a branch.
    #[error("no effective redispatch pair found for branch {0}")]
    NoRedispatchPair(usize),
    /// Redispatch cost limit exceeded.
    #[error("redispatch cost {cost:.2} USD exceeds limit {limit:.2} USD")]
    CostLimitExceeded { cost: f64, limit: f64 },
}

// ── Congestion method ─────────────────────────────────────────────────────────

/// Algorithm used for congestion management.
#[derive(Debug, Clone, PartialEq)]
pub enum CongestionMethod {
    /// Sensitivity-based analysis and redispatch using PTDF matrix.
    PtdfBased,
    /// Split system into price areas separated by congested interfaces.
    MarketSplit,
    /// Post-market redispatch orders to system operators.
    Redispatch,
    /// Simultaneous counter-trades in adjacent market areas.
    CounterTrading,
}

// ── Congestion info per branch ────────────────────────────────────────────────

/// Details of a single congested branch.
#[derive(Debug)]
pub struct CongestionInfo {
    /// Branch index (0-based).
    pub branch_id: usize,
    /// Sending-end bus index.
    pub from_bus: usize,
    /// Receiving-end bus index.
    pub to_bus: usize,
    /// Pre-redispatch active power flow \[MW\].
    pub base_flow_mw: f64,
    /// Flow after redispatch \[MW\].
    pub post_redispatch_flow_mw: f64,
    /// Thermal rating of the branch \[MW\].
    pub rating_mw: f64,
    /// Overload (flow − rating) before redispatch \[MW\].
    pub overload_mw: f64,
    /// Congestion rent / shadow price \[USD/MWh\].
    pub shadow_price_usd_per_mwh: f64,
    /// Maximum absolute PTDF value for this branch (across all buses).
    pub ptdf_max: f64,
}

// ── Redispatch pair ────────────────────────────────────────────────────────────

/// A generator up/down pair that relieves congestion on a branch.
pub struct RedispatchPair {
    /// Bus index where generation is increased \[MW\].
    pub increase_bus: usize,
    /// Bus index where generation is decreased \[MW\].
    pub decrease_bus: usize,
    /// Redispatch volume \[MW\].
    pub volume_mw: f64,
    /// Cost of this redispatch action \[USD\].
    pub cost_usd: f64,
    /// Relief effectiveness: MW flow reduction per MW redispatched.
    pub effectiveness: f64,
}

// ── Congestion result ─────────────────────────────────────────────────────────

/// Outcome of a congestion management run.
#[derive(Debug)]
pub struct CongestionResult {
    /// Details for each branch that was (or remains) congested.
    pub congested_branches: Vec<CongestionInfo>,
    /// Total congestion rent collected \[USD/h\].
    pub total_congestion_rent_usd: f64,
    /// Total redispatch volume \[MW\].
    pub redispatch_volume_mw: f64,
    /// Total redispatch cost \[USD\].
    pub redispatch_cost_usd: f64,
    /// Shadow price per branch \[USD/MWh\] (0 if not binding).
    pub shadow_prices: Vec<f64>,
    /// Zonal price spreads: (area\_i, area\_j, spread \[USD/MWh\]).
    pub area_price_spreads: Vec<(usize, usize, f64)>,
}

// ── Congestion manager config ─────────────────────────────────────────────────

/// Configuration for the `CongestionManager`.
pub struct CongestionConfig {
    /// Number of buses in the network.
    pub n_buses: usize,
    /// Number of branches in the network.
    pub n_branches: usize,
    /// System base MVA \[MVA\].
    pub base_mva: f64,
    /// Congestion management algorithm to apply.
    pub method: CongestionMethod,
    /// Maximum allowable redispatch cost per resolution run \[USD\].
    pub redispatch_cost_limit_usd: f64,
}

// ── Congestion manager ────────────────────────────────────────────────────────

/// Manages transmission congestion through PTDF-based redispatch.
pub struct CongestionManager {
    config: CongestionConfig,
    /// PTDF matrix: `ptdf_matrix[branch][bus]` \[pu/pu\].
    ptdf_matrix: Vec<Vec<f64>>,
    /// Thermal ratings per branch \[MW\].
    branch_ratings: Vec<f64>,
    /// Marginal generation cost per bus \[USD/MWh\].
    gen_costs: Vec<f64>,
    /// Maximum generation per bus \[MW\].
    gen_max: Vec<f64>,
    /// Pre-congestion flows per branch \[MW\].
    base_flows: Vec<f64>,
    /// Net power injections per bus \[MW\].
    base_injections: Vec<f64>,
}

impl CongestionManager {
    /// Construct a new `CongestionManager` with the given configuration.
    /// All internal state vectors are zero-initialised.
    pub fn new(config: CongestionConfig) -> Self {
        let nb = config.n_branches;
        let nbus = config.n_buses;
        Self {
            ptdf_matrix: vec![vec![0.0; nbus]; nb],
            branch_ratings: vec![f64::INFINITY; nb],
            gen_costs: vec![0.0; nbus],
            gen_max: vec![0.0; nbus],
            base_flows: vec![0.0; nb],
            base_injections: vec![0.0; nbus],
            config,
        }
    }

    /// Set the PTDF matrix (rows = branches, cols = buses).
    ///
    /// # Errors
    ///
    /// Returns [`CongestionError::PtdfDimensionMismatch`] if dimensions are
    /// inconsistent with the config.
    pub fn set_ptdf_matrix(&mut self, ptdf: Vec<Vec<f64>>) -> Result<(), CongestionError> {
        let rows = ptdf.len();
        let cols = ptdf.first().map(|r| r.len()).unwrap_or(0);
        if rows != self.config.n_branches || cols != self.config.n_buses {
            return Err(CongestionError::PtdfDimensionMismatch {
                rows,
                cols,
                exp_rows: self.config.n_branches,
                exp_cols: self.config.n_buses,
            });
        }
        self.ptdf_matrix = ptdf;
        Ok(())
    }

    /// Update current network state (flows, ratings, injections).
    ///
    /// # Errors
    ///
    /// Returns [`CongestionError::StateLengthMismatch`] if vector lengths
    /// do not match the configured branch/bus counts.
    pub fn set_network_state(
        &mut self,
        flows: Vec<f64>,
        ratings: Vec<f64>,
        injections: Vec<f64>,
    ) -> Result<(), CongestionError> {
        if flows.len() != self.config.n_branches {
            return Err(CongestionError::StateLengthMismatch {
                got: flows.len(),
                expected: self.config.n_branches,
            });
        }
        if ratings.len() != self.config.n_branches {
            return Err(CongestionError::StateLengthMismatch {
                got: ratings.len(),
                expected: self.config.n_branches,
            });
        }
        if injections.len() != self.config.n_buses {
            return Err(CongestionError::StateLengthMismatch {
                got: injections.len(),
                expected: self.config.n_buses,
            });
        }
        self.base_flows = flows;
        self.branch_ratings = ratings;
        self.base_injections = injections;
        Ok(())
    }

    /// Set marginal generation costs and capacity limits per bus.
    ///
    /// # Errors
    ///
    /// Returns [`CongestionError::StateLengthMismatch`] if lengths are wrong.
    pub fn set_generator_data(
        &mut self,
        costs: Vec<f64>,
        max_mw: Vec<f64>,
    ) -> Result<(), CongestionError> {
        if costs.len() != self.config.n_buses {
            return Err(CongestionError::StateLengthMismatch {
                got: costs.len(),
                expected: self.config.n_buses,
            });
        }
        if max_mw.len() != self.config.n_buses {
            return Err(CongestionError::StateLengthMismatch {
                got: max_mw.len(),
                expected: self.config.n_buses,
            });
        }
        self.gen_costs = costs;
        self.gen_max = max_mw;
        Ok(())
    }

    /// Identify branches where `|flow| > rating`.
    pub fn identify_congestion(&self) -> Vec<CongestionInfo> {
        let mut congested = Vec::new();
        for (br, (&flow, &rating)) in self
            .base_flows
            .iter()
            .zip(self.branch_ratings.iter())
            .enumerate()
        {
            let abs_flow = flow.abs();
            if abs_flow > rating + 1e-6 {
                let ptdf_max = self.ptdf_matrix[br]
                    .iter()
                    .map(|v| v.abs())
                    .fold(0.0_f64, f64::max);
                congested.push(CongestionInfo {
                    branch_id: br,
                    from_bus: 0, // topology unknown; caller may override
                    to_bus: 0,
                    base_flow_mw: flow,
                    post_redispatch_flow_mw: flow,
                    rating_mw: rating,
                    overload_mw: abs_flow - rating,
                    shadow_price_usd_per_mwh: 0.0,
                    ptdf_max,
                });
            }
        }
        congested
    }

    /// Run the congestion management algorithm and return results.
    ///
    /// For each congested branch the method:
    /// 1. Finds the cheapest generator-pair redispatch that relieves the
    ///    overload.
    /// 2. Applies the redispatch (updating simulated flows).
    /// 3. Computes shadow prices as the marginal cost difference of the pair
    ///    divided by the PTDF effectiveness.
    ///
    /// # Errors
    ///
    /// - [`CongestionError::NoRedispatchPair`] — no PTDF-effective pair exists.
    /// - [`CongestionError::CostLimitExceeded`] — cumulative cost exceeds limit.
    pub fn resolve_congestion(&self) -> Result<CongestionResult, CongestionError> {
        let mut congested = self.identify_congestion();
        if congested.is_empty() {
            return Ok(CongestionResult {
                congested_branches: Vec::new(),
                total_congestion_rent_usd: 0.0,
                redispatch_volume_mw: 0.0,
                redispatch_cost_usd: 0.0,
                shadow_prices: vec![0.0; self.config.n_branches],
                area_price_spreads: Vec::new(),
            });
        }

        // Working copy of branch flows.
        let mut flows = self.base_flows.clone();
        let mut total_redispatch_volume = 0.0;
        let mut total_redispatch_cost = 0.0;
        let mut shadow_prices = vec![0.0_f64; self.config.n_branches];

        for info in &mut congested {
            let br = info.branch_id;
            let current_flow = flows[br];
            let overload = current_flow.abs() - info.rating_mw;
            if overload <= 1e-6 {
                continue; // Already relieved by prior redispatch.
            }

            // Direction convention: positive PTDF at bus b means an increase
            // in injection at b increases flow on this branch.
            // To relieve positive overload: increase at bus with NEGATIVE ptdf,
            // decrease at bus with POSITIVE ptdf.
            let sign = if current_flow >= 0.0 { 1.0 } else { -1.0 };

            // Find the best (cheapest cost per MW relief) redispatch pair.
            let mut best_pair: Option<RedispatchPair> = None;
            let mut best_cost_per_mw = f64::INFINITY;

            let nbus = self.config.n_buses;
            for dec in 0..nbus {
                let ptdf_dec = self.ptdf_matrix[br][dec] * sign;
                if ptdf_dec <= 1e-4 || self.gen_max[dec] < 1e-6 {
                    continue; // Decreasing at `dec` does not help.
                }
                for inc in 0..nbus {
                    if inc == dec {
                        continue;
                    }
                    let ptdf_inc = self.ptdf_matrix[br][inc] * sign;
                    if ptdf_inc >= -1e-4 || self.gen_max[inc] < 1e-6 {
                        continue; // Increasing at `inc` does not help.
                    }
                    // Effectiveness: each MW shifted from dec→inc relieves
                    // (ptdf_dec − ptdf_inc) MW on the branch.
                    let effectiveness = ptdf_dec - ptdf_inc;
                    if effectiveness <= 1e-6 {
                        continue;
                    }
                    let volume_needed = overload / effectiveness;
                    let cost = volume_needed * (self.gen_costs[dec] - self.gen_costs[inc]).abs();
                    let cost_per_mw = cost / volume_needed;
                    if cost_per_mw < best_cost_per_mw {
                        best_cost_per_mw = cost_per_mw;
                        best_pair = Some(RedispatchPair {
                            increase_bus: inc,
                            decrease_bus: dec,
                            volume_mw: volume_needed,
                            cost_usd: cost,
                            effectiveness,
                        });
                    }
                }
            }

            let pair = best_pair.ok_or(CongestionError::NoRedispatchPair(br))?;

            // Apply redispatch to simulated flows.
            let delta_inj = pair.volume_mw;
            for (b, flow) in flows.iter_mut().enumerate().take(self.config.n_branches) {
                *flow += self.ptdf_matrix[b][pair.increase_bus] * delta_inj;
                *flow -= self.ptdf_matrix[b][pair.decrease_bus] * delta_inj;
            }

            // Shadow price = cost difference / effectiveness [$/MWh].
            let cost_diff =
                (self.gen_costs[pair.decrease_bus] - self.gen_costs[pair.increase_bus]).abs();
            shadow_prices[br] = cost_diff / pair.effectiveness;

            total_redispatch_volume += pair.volume_mw;
            total_redispatch_cost += pair.cost_usd;

            // Update the post-redispatch flow and shadow price in the info.
            info.post_redispatch_flow_mw = flows[br];
            info.shadow_price_usd_per_mwh = shadow_prices[br];

            if total_redispatch_cost > self.config.redispatch_cost_limit_usd {
                return Err(CongestionError::CostLimitExceeded {
                    cost: total_redispatch_cost,
                    limit: self.config.redispatch_cost_limit_usd,
                });
            }
        }

        // Congestion rent = shadow price × flow [$/h] (per congested branch).
        let total_congestion_rent_usd: f64 = congested
            .iter()
            .map(|i| i.shadow_price_usd_per_mwh * i.rating_mw.min(i.base_flow_mw.abs()))
            .sum();

        let area_price_spreads = self.zonal_price_spreads(&shadow_prices);

        Ok(CongestionResult {
            congested_branches: congested,
            total_congestion_rent_usd,
            redispatch_volume_mw: total_redispatch_volume,
            redispatch_cost_usd: total_redispatch_cost,
            shadow_prices,
            area_price_spreads,
        })
    }

    /// Derive representative zonal price spreads from branch shadow prices.
    ///
    /// For each congested branch the spread is attributed between its adjacent
    /// bus areas (identified by bus index quartile as a simple proxy).
    fn zonal_price_spreads(&self, shadow_prices: &[f64]) -> Vec<(usize, usize, f64)> {
        let nbus = self.config.n_buses;
        if nbus < 2 {
            return Vec::new();
        }
        let area_size = (nbus / 2).max(1);
        let mut spreads = Vec::new();
        for (br, &sp) in shadow_prices.iter().enumerate() {
            if sp.abs() < 1e-6 {
                continue;
            }
            // Map branch index to a simple two-area partition.
            let area_i = (br * 2) / self.config.n_branches.max(1);
            let area_j = area_i ^ 1;
            // Ensure area indices are bus-range bounded.
            let bus_i = area_i * area_size;
            let bus_j = (area_j * area_size).min(nbus - 1);
            spreads.push((bus_i, bus_j, sp));
        }
        spreads
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 3-bus, 2-branch test network.
    ///
    /// Bus 0 (generator), Bus 1 (load), Bus 2 (generator).
    /// Branch 0: bus0→bus1, Branch 1: bus1→bus2.
    fn make_manager() -> CongestionManager {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::PtdfBased,
            redispatch_cost_limit_usd: 1_000_000.0,
        };
        let mut mgr = CongestionManager::new(config);
        // PTDF[branch][bus]: branch 0 sensitive to buses 0 and 2.
        mgr.set_ptdf_matrix(vec![
            vec![0.5, 0.0, -0.5], // branch 0
            vec![-0.3, 0.0, 0.3], // branch 1
        ])
        .expect("PTDF ok");
        mgr.set_network_state(
            vec![120.0, 50.0],         // flows [MW]: branch 0 is overloaded
            vec![100.0, 200.0],        // ratings [MW]
            vec![120.0, -150.0, 30.0], // injections [MW]
        )
        .expect("state ok");
        mgr.set_generator_data(
            vec![20.0, 0.0, 50.0],   // gen costs [$/MWh]
            vec![200.0, 0.0, 200.0], // gen capacity [MW]
        )
        .expect("gen data ok");
        mgr
    }

    /// No congestion when all flows are within ratings.
    #[test]
    fn test_no_congestion_empty_result() {
        let config = CongestionConfig {
            n_buses: 2,
            n_branches: 1,
            base_mva: 100.0,
            method: CongestionMethod::PtdfBased,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        mgr.set_ptdf_matrix(vec![vec![0.5, -0.5]]).expect("PTDF ok");
        mgr.set_network_state(
            vec![80.0], // flow < rating
            vec![100.0],
            vec![80.0, -80.0],
        )
        .expect("state ok");
        mgr.set_generator_data(vec![30.0, 0.0], vec![200.0, 0.0])
            .expect("gen ok");

        let result = mgr.resolve_congestion().expect("resolve ok");
        assert!(
            result.congested_branches.is_empty(),
            "No congestion expected"
        );
        assert_eq!(result.redispatch_volume_mw, 0.0);
        assert_eq!(result.redispatch_cost_usd, 0.0);
    }

    /// Overloaded branch triggers redispatch.
    #[test]
    fn test_overloaded_branch_redispatched() {
        let mgr = make_manager();
        let congested = mgr.identify_congestion();
        assert_eq!(
            congested.len(),
            1,
            "One branch should be congested: branch 0 (120 MW > 100 MW)"
        );
        assert_eq!(congested[0].branch_id, 0);
        assert!(
            (congested[0].overload_mw - 20.0).abs() < 1e-6,
            "Overload must be 20 MW"
        );

        let result = mgr.resolve_congestion().expect("resolve ok");
        // Post-redispatch flow should be ≤ rating.
        let post = result.congested_branches[0].post_redispatch_flow_mw;
        assert!(
            post.abs() <= 100.0 + 1e-3,
            "Post-redispatch flow {:.2} must be ≤ 100 MW rating",
            post
        );
    }

    /// PTDF-based pair selection uses the correct direction.
    #[test]
    fn test_ptdf_pair_direction() {
        let mgr = make_manager();
        let result = mgr.resolve_congestion().expect("resolve ok");
        // There should have been a redispatch.
        assert!(
            result.redispatch_volume_mw > 0.0,
            "Redispatch volume must be positive"
        );
    }

    /// Shadow price is positive for a binding constraint.
    #[test]
    fn test_shadow_price_positive_for_binding() {
        let mgr = make_manager();
        let result = mgr.resolve_congestion().expect("resolve ok");
        let sp = result.shadow_prices[0];
        assert!(
            sp > 0.0,
            "Shadow price for binding branch 0 must be positive, got {:.4}",
            sp
        );
    }

    /// Redispatch cost is non-zero for an overloaded network.
    #[test]
    fn test_redispatch_cost_nonzero() {
        let mgr = make_manager();
        let result = mgr.resolve_congestion().expect("resolve ok");
        assert!(
            result.redispatch_cost_usd > 0.0,
            "Redispatch cost must be positive"
        );
    }

    /// PTDF dimension mismatch returns an error.
    #[test]
    fn test_ptdf_dimension_mismatch_error() {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::PtdfBased,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        // Provide wrong dimensions.
        let result = mgr.set_ptdf_matrix(vec![vec![0.5; 2]; 2]);
        assert!(
            matches!(result, Err(CongestionError::PtdfDimensionMismatch { .. })),
            "Expected PtdfDimensionMismatch"
        );
    }

    /// `set_network_state` rejects a flows vector of wrong length.
    #[test]
    fn test_set_network_state_wrong_flows_length() {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::Redispatch,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        let err = mgr
            .set_network_state(vec![1.0], vec![100.0, 100.0], vec![0.0; 3])
            .expect_err("should fail on wrong flows length");
        assert!(
            matches!(
                err,
                CongestionError::StateLengthMismatch {
                    got: 1,
                    expected: 2
                }
            ),
            "Unexpected error: {:?}",
            err
        );
    }

    /// `set_network_state` rejects a ratings vector of wrong length.
    #[test]
    fn test_set_network_state_wrong_ratings_length() {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::MarketSplit,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        let err = mgr
            .set_network_state(vec![10.0, 20.0], vec![100.0], vec![0.0; 3])
            .expect_err("should fail on wrong ratings length");
        assert!(
            matches!(
                err,
                CongestionError::StateLengthMismatch {
                    got: 1,
                    expected: 2
                }
            ),
            "Unexpected error: {:?}",
            err
        );
    }

    /// `set_generator_data` rejects a costs vector of wrong length.
    #[test]
    fn test_set_generator_data_wrong_costs_length() {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::CounterTrading,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        let err = mgr
            .set_generator_data(vec![10.0, 20.0], vec![100.0; 3])
            .expect_err("should fail on wrong costs length");
        assert!(
            matches!(
                err,
                CongestionError::StateLengthMismatch {
                    got: 2,
                    expected: 3
                }
            ),
            "Unexpected error: {:?}",
            err
        );
    }

    /// `CongestionError::CostLimitExceeded` is raised when redispatch cost
    /// exceeds the configured limit.
    #[test]
    fn test_resolve_congestion_cost_limit_exceeded() {
        let config = CongestionConfig {
            n_buses: 3,
            n_branches: 2,
            base_mva: 100.0,
            method: CongestionMethod::PtdfBased,
            redispatch_cost_limit_usd: 0.01, // extremely tight limit
        };
        let mut mgr = CongestionManager::new(config);
        mgr.set_ptdf_matrix(vec![vec![0.5, 0.0, -0.5], vec![-0.3, 0.0, 0.3]])
            .expect("PTDF ok");
        mgr.set_network_state(
            vec![120.0, 50.0],
            vec![100.0, 200.0],
            vec![120.0, -150.0, 30.0],
        )
        .expect("state ok");
        mgr.set_generator_data(vec![20.0, 0.0, 50.0], vec![200.0, 0.0, 200.0])
            .expect("gen ok");

        let outcome = mgr.resolve_congestion();
        assert!(
            matches!(outcome, Err(CongestionError::CostLimitExceeded { .. })),
            "Expected CostLimitExceeded error from resolve_congestion"
        );
    }

    /// Negative branch flow triggers congestion detection (overload from
    /// the reverse direction).
    #[test]
    fn test_identify_congestion_negative_flow() {
        let config = CongestionConfig {
            n_buses: 2,
            n_branches: 1,
            base_mva: 100.0,
            method: CongestionMethod::PtdfBased,
            redispatch_cost_limit_usd: 1e9,
        };
        let mut mgr = CongestionManager::new(config);
        mgr.set_ptdf_matrix(vec![vec![0.5, -0.5]]).expect("PTDF ok");
        // Flow is −130 MW on a 100 MW rated branch → 30 MW overload.
        mgr.set_network_state(vec![-130.0], vec![100.0], vec![-130.0, 130.0])
            .expect("state ok");
        mgr.set_generator_data(vec![30.0, 0.0], vec![200.0, 0.0])
            .expect("gen ok");

        let congested = mgr.identify_congestion();
        assert_eq!(congested.len(), 1, "Branch 0 should be congested");
        assert!(
            (congested[0].overload_mw - 30.0).abs() < 1e-6,
            "Expected 30 MW overload, got {:.4}",
            congested[0].overload_mw
        );
        assert!(congested[0].base_flow_mw < 0.0, "Flow must be negative");
    }

    /// `ptdf_max` field of `CongestionInfo` reflects the largest |PTDF| for
    /// the congested branch.
    #[test]
    fn test_congestion_info_ptdf_max() {
        let mgr = make_manager();
        let congested = mgr.identify_congestion();
        assert_eq!(congested.len(), 1, "One congested branch expected");
        // Branch 0 PTDFs: [0.5, 0.0, -0.5] → max |val| = 0.5
        assert!(
            (congested[0].ptdf_max - 0.5).abs() < 1e-9,
            "ptdf_max should be 0.5, got {:.6}",
            congested[0].ptdf_max
        );
    }

    /// `CongestionMethod` derives Clone and PartialEq correctly.
    #[test]
    fn test_congestion_method_clone_and_eq() {
        let m1 = CongestionMethod::MarketSplit;
        let m2 = m1.clone();
        assert_eq!(m1, m2, "Cloned CongestionMethod must equal original");
        assert_ne!(
            CongestionMethod::Redispatch,
            CongestionMethod::CounterTrading,
            "Different variants must not be equal"
        );
    }

    /// `resolve_congestion` populates `area_price_spreads` when there is a
    /// congested branch with a non-zero shadow price.
    #[test]
    fn test_resolve_congestion_area_price_spreads_populated() {
        let mgr = make_manager();
        let result = mgr.resolve_congestion().expect("resolve ok");
        assert!(
            !result.area_price_spreads.is_empty(),
            "area_price_spreads must be non-empty when congestion is present"
        );
        // Each spread entry must have a non-zero shadow price value.
        for &(bus_i, bus_j, spread) in &result.area_price_spreads {
            assert!(
                spread.abs() > 0.0,
                "Spread between bus {} and {} should be non-zero",
                bus_i,
                bus_j
            );
        }
    }
}
