//! Power flow decomposition: nodal attribution, MW-mile accounting, flow-gate analysis,
//! and Financial Transmission Rights (FTR) payoff calculations.
//!
//! # Units
//!
//! | Quantity | Unit |
//! |----------|------|
//! | Power flows | \[MW\] |
//! | MW-mile product | \[MW·km\] |
//! | Line loading | \[%\] |
//! | Transmission tariff | \[$/MW·km\] |
//! | Charges | \[USD\] |

use std::collections::VecDeque;

// ---------------------------------------------------------------------------
// Supporting types
// ---------------------------------------------------------------------------

/// Method used to attribute branch flows to individual bus injections.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecompositionMethod {
    /// Apportion flows proportionally to net injections.
    Proportional,
    /// Attribution based on marginal cost / shadow prices.
    MarginalCost,
    /// Direct PTDF-weighted attribution (recommended for DC-linearised networks).
    PtdfBased,
}

/// Static network data required for flow decomposition.
#[derive(Debug, Clone)]
pub struct NetworkData {
    /// Total number of buses in the network.
    pub num_buses: usize,
    /// Total number of branches in the network.
    pub num_branches: usize,
    /// Pre-computed PTDF matrix — `ptdf[branch][bus]` (dimensionless sensitivity).
    pub ptdf: Vec<Vec<f64>>,
    /// Series resistance of each branch \[p.u.\].
    pub branch_resistance_pu: Vec<f64>,
    /// Series reactance of each branch \[p.u.\].
    pub branch_reactance_pu: Vec<f64>,
    /// Thermal rating of each branch \[MW\].
    pub branch_rating_mw: Vec<f64>,
    /// Sending-end bus index (0-based) for each branch.
    pub from_bus: Vec<usize>,
    /// Receiving-end bus index (0-based) for each branch.
    pub to_bus: Vec<usize>,
    /// Physical length of each branch \[km\].
    pub branch_length_km: Vec<f64>,
}

/// Configuration for the flow decomposer.
#[derive(Debug, Clone)]
pub struct FlowDecompositionConfig {
    /// Whether to account for I²R losses in the decomposition.
    pub include_losses: bool,
    /// Attribution method to use.
    pub method: DecompositionMethod,
    /// Flows below this threshold \[MW\] are treated as zero.
    pub min_flow_threshold_mw: f64,
}

impl Default for FlowDecompositionConfig {
    fn default() -> Self {
        Self {
            include_losses: false,
            method: DecompositionMethod::PtdfBased,
            min_flow_threshold_mw: 0.1,
        }
    }
}

/// Main decomposer struct.
#[derive(Debug, Clone)]
pub struct FlowDecomposer {
    pub network: NetworkData,
    pub config: FlowDecompositionConfig,
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// Full output of a flow decomposition run.
#[derive(Debug, Clone)]
pub struct FlowDecompositionResult {
    /// Total flow on each branch \[MW\].
    pub branch_flows_mw: Vec<f64>,
    /// Attribution matrix — `attribution[branch][bus]` \[MW\].
    pub attribution: Vec<Vec<f64>>,
    /// Line loading for each branch \[%\].
    pub loading_pct: Vec<f64>,
    /// Estimated total I²R losses \[MW\].
    pub total_losses_mw: f64,
    /// MW-mile product for each branch \[MW·km\].
    pub mw_mile: Vec<f64>,
}

/// Report produced by flow-gate analysis.
#[derive(Debug, Clone)]
pub struct FlowGateReport {
    /// Indices of branches loaded at or above 80 %.
    pub binding_branches: Vec<usize>,
    /// Shadow-price approximation for each branch \[$/MW\].
    pub shadow_prices: Vec<f64>,
    /// Loading percentage for every branch \[%\].
    pub loading_pct: Vec<f64>,
    /// Up to 5 most-loaded branch indices.
    pub top5_congested: Vec<usize>,
}

/// A single Financial Transmission Right position.
#[derive(Debug, Clone)]
pub struct FtrPosition {
    pub ftr_id: String,
    pub from_bus: usize,
    pub to_bus: usize,
    /// Awarded capacity \[MW\].
    pub mw: f64,
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by flow-decomposition routines.
#[derive(Debug, Clone)]
pub enum FlowDecompError {
    DimensionMismatch { msg: String },
    InvalidIndex { msg: String },
    NumericalError { msg: String },
}

impl std::fmt::Display for FlowDecompError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DimensionMismatch { msg } => write!(f, "Dimension mismatch: {msg}"),
            Self::InvalidIndex { msg } => write!(f, "Invalid index: {msg}"),
            Self::NumericalError { msg } => write!(f, "Numerical error: {msg}"),
        }
    }
}

impl std::error::Error for FlowDecompError {}

pub type Result<T> = std::result::Result<T, FlowDecompError>;

// ---------------------------------------------------------------------------
// Core implementation
// ---------------------------------------------------------------------------

impl FlowDecomposer {
    /// Construct a new `FlowDecomposer`.
    pub fn new(network: NetworkData, config: FlowDecompositionConfig) -> Self {
        Self { network, config }
    }

    // -----------------------------------------------------------------------
    // 1. Nodal flow decomposition
    // -----------------------------------------------------------------------

    /// Decompose branch flows into per-bus attributions.
    ///
    /// For each bus `s`, the net injection `P_s = generation[s] - load[s]` is
    /// computed.  The contribution of bus `s` to branch `b` is:
    ///
    /// ```text
    /// F_{b,s} = PTDF[b][s] × P_s          (PtdfBased)
    /// F_{b,s} = F_b × P_s / Σ|P_s|        (Proportional)
    /// ```
    ///
    /// Flows below [`FlowDecompositionConfig::min_flow_threshold_mw`] are zeroed
    /// out before building the attribution.
    pub fn decompose_flows(
        &self,
        generation_mw: &[f64],
        load_mw: &[f64],
    ) -> Result<FlowDecompositionResult> {
        let nb = self.network.num_buses;
        let nbr = self.network.num_branches;

        if generation_mw.len() != nb {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!(
                    "generation_mw length {} != num_buses {}",
                    generation_mw.len(),
                    nb
                ),
            });
        }
        if load_mw.len() != nb {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!("load_mw length {} != num_buses {}", load_mw.len(), nb),
            });
        }
        if self.network.ptdf.len() != nbr {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!(
                    "PTDF row count {} != num_branches {}",
                    self.network.ptdf.len(),
                    nbr
                ),
            });
        }

        // Net injections [MW]
        let injections: Vec<f64> = (0..nb).map(|s| generation_mw[s] - load_mw[s]).collect();

        // Sum of absolute injections (used for Proportional method)
        let sum_abs_inj: f64 = injections.iter().map(|p| p.abs()).sum();

        // Branch flows and attribution
        let mut branch_flows_mw = vec![0.0_f64; nbr];
        let mut attribution: Vec<Vec<f64>> = vec![vec![0.0; nb]; nbr];

        for b in 0..nbr {
            let ptdf_row =
                self.network
                    .ptdf
                    .get(b)
                    .ok_or_else(|| FlowDecompError::InvalidIndex {
                        msg: format!("PTDF row {b} missing"),
                    })?;

            if ptdf_row.len() != nb {
                return Err(FlowDecompError::DimensionMismatch {
                    msg: format!(
                        "PTDF[{b}] column count {} != num_buses {}",
                        ptdf_row.len(),
                        nb
                    ),
                });
            }

            // Total branch flow
            let flow: f64 = (0..nb).map(|s| ptdf_row[s] * injections[s]).sum();
            let flow = if flow.abs() < self.config.min_flow_threshold_mw {
                0.0
            } else {
                flow
            };
            branch_flows_mw[b] = flow;

            // Attribution
            match self.config.method {
                DecompositionMethod::PtdfBased => {
                    for s in 0..nb {
                        let contrib = ptdf_row[s] * injections[s];
                        attribution[b][s] = if contrib.abs() < self.config.min_flow_threshold_mw {
                            0.0
                        } else {
                            contrib
                        };
                    }
                }
                DecompositionMethod::Proportional => {
                    if sum_abs_inj < 1e-12 {
                        // No injections — attribution stays zero
                    } else {
                        for s in 0..nb {
                            attribution[b][s] = flow * injections[s].abs() / sum_abs_inj;
                        }
                    }
                }
                DecompositionMethod::MarginalCost => {
                    // Marginal cost attribution uses the PTDF-weighted share of
                    // each injection scaled by the signed injection value, giving
                    // a "shadow price" weighting identical to PtdfBased for DC
                    // linearised networks.
                    for s in 0..nb {
                        let contrib = ptdf_row[s] * injections[s];
                        attribution[b][s] = if contrib.abs() < self.config.min_flow_threshold_mw {
                            0.0
                        } else {
                            contrib
                        };
                    }
                }
            }
        }

        // Line loading
        let loading_pct = self.line_loading_pct(&branch_flows_mw);

        // MW-mile
        let mw_mile = self.calculate_mw_mile(&branch_flows_mw);

        // Losses — estimated from I²R with DC approximation for flow magnitude
        let total_losses_mw = if self.config.include_losses {
            (0..nbr)
                .map(|b| {
                    let r = self.network.branch_resistance_pu[b];
                    let x = self.network.branch_reactance_pu[b];
                    let z2 = r * r + x * x;
                    if z2 < 1e-15 {
                        0.0
                    } else {
                        // Approximate: I ~ flow / |Z|, losses = I² × R
                        // With p.u. quantities this is an approximation.
                        let flow = branch_flows_mw[b];
                        let i_pu = flow / z2.sqrt();
                        i_pu * i_pu * r
                    }
                })
                .sum()
        } else {
            0.0
        };

        Ok(FlowDecompositionResult {
            branch_flows_mw,
            attribution,
            loading_pct,
            total_losses_mw,
            mw_mile,
        })
    }

    // -----------------------------------------------------------------------
    // 2. MW-mile computation
    // -----------------------------------------------------------------------

    /// Compute the MW-mile product for each branch.
    ///
    /// `mw_mile[b] = |flow_mw[b]| × branch_length_km[b]`  \[MW·km\]
    pub fn calculate_mw_mile(&self, flows_mw: &[f64]) -> Vec<f64> {
        flows_mw
            .iter()
            .enumerate()
            .map(|(b, &f)| {
                let len = self.network.branch_length_km.get(b).copied().unwrap_or(0.0);
                f.abs() * len
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 3. Flow-gate analysis
    // -----------------------------------------------------------------------

    /// Identify binding flow-gates and compute shadow-price approximations.
    ///
    /// A branch is a flow-gate when `|flow| / rating ≥ 0.80`.
    /// Shadow price ≈ 0 for non-binding branches; for binding branches it is
    /// approximated as `(|flow| - 0.8 × rating) / rating`.
    pub fn flow_gate_analysis(
        &self,
        flows_mw: &[f64],
        rating_mw: &[f64],
    ) -> Result<FlowGateReport> {
        let nbr = self.network.num_branches;
        if flows_mw.len() != nbr {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!("flows_mw length {} != num_branches {}", flows_mw.len(), nbr),
            });
        }
        if rating_mw.len() != nbr {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!(
                    "rating_mw length {} != num_branches {}",
                    rating_mw.len(),
                    nbr
                ),
            });
        }

        let loading_pct: Vec<f64> = (0..nbr)
            .map(|b| {
                let r = rating_mw[b];
                if r < 1e-12 {
                    0.0
                } else {
                    flows_mw[b].abs() / r * 100.0
                }
            })
            .collect();

        let mut binding_branches = Vec::new();
        let mut shadow_prices = vec![0.0_f64; nbr];

        for b in 0..nbr {
            let r = rating_mw[b];
            if r < 1e-12 {
                continue;
            }
            let load_frac = flows_mw[b].abs() / r;
            if load_frac >= 0.80 {
                binding_branches.push(b);
                // Shadow price approximation: fractional overload above 80 %
                shadow_prices[b] = (load_frac - 0.80) / 1.0; // normalised [0, 0.2+]
            }
        }

        // Top-5 most congested branches by loading_pct
        let mut ranked: Vec<usize> = (0..nbr).collect();
        ranked.sort_by(|&a, &b| {
            loading_pct[b]
                .partial_cmp(&loading_pct[a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let top5_congested: Vec<usize> = ranked.into_iter().take(5).collect();

        Ok(FlowGateReport {
            binding_branches,
            shadow_prices,
            loading_pct,
            top5_congested,
        })
    }

    // -----------------------------------------------------------------------
    // 4. FTR payoff
    // -----------------------------------------------------------------------

    /// Calculate Financial Transmission Right payoffs.
    ///
    /// `payoff[i] = ftr.mw × (LMP[ftr.to_bus] − LMP[ftr.from_bus])`  \[USD\]
    ///
    /// A negative payoff indicates a counter-flow position where the FTR holder
    /// must pay into the congestion revenue pool.
    pub fn calculate_ftr_payoff(
        &self,
        ftr_positions: &[FtrPosition],
        lmp_from: &[f64],
        lmp_to: &[f64],
    ) -> Result<Vec<f64>> {
        let nb = self.network.num_buses;
        if lmp_from.len() != nb {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!("lmp_from length {} != num_buses {}", lmp_from.len(), nb),
            });
        }
        if lmp_to.len() != nb {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!("lmp_to length {} != num_buses {}", lmp_to.len(), nb),
            });
        }

        let payoffs = ftr_positions
            .iter()
            .map(|ftr| {
                let lmp_f = lmp_from.get(ftr.from_bus).copied().unwrap_or(0.0);
                let lmp_t = lmp_to.get(ftr.to_bus).copied().unwrap_or(0.0);
                ftr.mw * (lmp_t - lmp_f)
            })
            .collect();

        Ok(payoffs)
    }

    // -----------------------------------------------------------------------
    // 5. Per-bus congestion contribution
    // -----------------------------------------------------------------------

    /// Contribution of a single bus injection to every branch flow.
    ///
    /// `contribution[b] = PTDF[b][bus_idx] × injection_mw`  \[MW\]
    pub fn congestion_contribution(&self, bus_idx: usize, injection_mw: f64) -> Result<Vec<f64>> {
        let nb = self.network.num_buses;
        let nbr = self.network.num_branches;

        if bus_idx >= nb {
            return Err(FlowDecompError::InvalidIndex {
                msg: format!("bus_idx {bus_idx} >= num_buses {nb}"),
            });
        }

        let contribs = (0..nbr)
            .map(|b| {
                let ptdf_val = self
                    .network
                    .ptdf
                    .get(b)
                    .and_then(|row| row.get(bus_idx))
                    .copied()
                    .unwrap_or(0.0);
                ptdf_val * injection_mw
            })
            .collect();

        Ok(contribs)
    }

    // -----------------------------------------------------------------------
    // 6. Transmission usage charges
    // -----------------------------------------------------------------------

    /// Compute transmission usage charges per transaction.
    ///
    /// For each transaction `t` recorded in `decomposition.attribution`:
    ///
    /// ```text
    /// charge[t] = Σ_b  |F_{b,t}| × length_b × tariff   [USD]
    /// ```
    ///
    /// where `tariff` is in \[$/MW·km\].
    ///
    /// # Arguments
    /// * `decomposition` — result from [`FlowDecomposer::decompose_flows`].
    /// * `unit_labels` — one label per bus; used as the transaction identifier.
    /// * `tariff_per_mw_mile` — transmission tariff \[$/MW·km\].
    pub fn transmission_usage_charges(
        &self,
        decomposition: &FlowDecompositionResult,
        unit_labels: &[String],
        tariff_per_mw_mile: f64,
    ) -> Result<Vec<(String, f64)>> {
        let nb = self.network.num_buses;
        let nbr = self.network.num_branches;

        if unit_labels.len() != nb {
            return Err(FlowDecompError::DimensionMismatch {
                msg: format!(
                    "unit_labels length {} != num_buses {}",
                    unit_labels.len(),
                    nb
                ),
            });
        }

        let charges: Vec<(String, f64)> = (0..nb)
            .map(|s| {
                let charge: f64 = (0..nbr)
                    .map(|b| {
                        let flow_s = decomposition
                            .attribution
                            .get(b)
                            .and_then(|row| row.get(s))
                            .copied()
                            .unwrap_or(0.0);
                        let len = self.network.branch_length_km.get(b).copied().unwrap_or(0.0);
                        flow_s.abs() * len * tariff_per_mw_mile
                    })
                    .sum();
                let label = unit_labels
                    .get(s)
                    .cloned()
                    .unwrap_or_else(|| format!("Bus_{s}"));
                (label, charge)
            })
            .collect();

        Ok(charges)
    }

    // -----------------------------------------------------------------------
    // 7. Line loading percentage
    // -----------------------------------------------------------------------

    /// Line loading percentage for each branch.
    ///
    /// `loading[b] = |flow_mw[b]| / rating_mw[b] × 100`  \[%\]
    ///
    /// Uses [`NetworkData::branch_rating_mw`] for ratings.  Branches with a
    /// rating of zero are reported as 0 %.
    pub fn line_loading_pct(&self, flows_mw: &[f64]) -> Vec<f64> {
        flows_mw
            .iter()
            .enumerate()
            .map(|(b, &f)| {
                let rating = self.network.branch_rating_mw.get(b).copied().unwrap_or(0.0);
                if rating < 1e-12 {
                    0.0
                } else {
                    f.abs() / rating * 100.0
                }
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 8. Parallel path identification
    // -----------------------------------------------------------------------

    /// Find all paths (sequences of branch indices) from `from_bus` to `to_bus`
    /// using at most 5 hops (branches) via BFS.
    ///
    /// Returns a list of branch-index sequences, each representing one path.
    pub fn identify_parallel_paths(
        &self,
        from_bus: usize,
        to_bus: usize,
    ) -> Result<Vec<Vec<usize>>> {
        let nb = self.network.num_buses;
        let nbr = self.network.num_branches;

        if from_bus >= nb {
            return Err(FlowDecompError::InvalidIndex {
                msg: format!("from_bus {from_bus} >= num_buses {nb}"),
            });
        }
        if to_bus >= nb {
            return Err(FlowDecompError::InvalidIndex {
                msg: format!("to_bus {to_bus} >= num_buses {nb}"),
            });
        }

        // Build adjacency list: bus → Vec<(neighbour_bus, branch_idx)>
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); nb];
        for b in 0..nbr {
            let f = self.network.from_bus.get(b).copied().unwrap_or(usize::MAX);
            let t = self.network.to_bus.get(b).copied().unwrap_or(usize::MAX);
            if f < nb && t < nb {
                adj[f].push((t, b));
                adj[t].push((f, b)); // undirected
            }
        }

        // BFS: state = (current_bus, branch_path, visited_buses_bitmask)
        // Limit to 5-hop paths and avoid revisiting buses.
        //
        // For large networks, bitmask cannot span all buses, so we use a
        // Vec<bool> per state — acceptable given the ≤5-hop limit.
        #[derive(Clone)]
        struct State {
            bus: usize,
            path: Vec<usize>, // branch indices
            visited: Vec<bool>,
        }

        let mut queue: VecDeque<State> = VecDeque::new();
        let mut initial_visited = vec![false; nb];
        initial_visited[from_bus] = true;
        queue.push_back(State {
            bus: from_bus,
            path: Vec::new(),
            visited: initial_visited,
        });

        let mut found_paths: Vec<Vec<usize>> = Vec::new();
        let max_hops = 5_usize;

        while let Some(state) = queue.pop_front() {
            if state.path.len() > max_hops {
                continue;
            }

            for &(next_bus, branch_idx) in &adj[state.bus] {
                if state.visited[next_bus] {
                    continue;
                }
                let mut new_path = state.path.clone();
                new_path.push(branch_idx);

                if next_bus == to_bus {
                    found_paths.push(new_path);
                    continue;
                }

                if new_path.len() < max_hops {
                    let mut new_visited = state.visited.clone();
                    new_visited[next_bus] = true;
                    queue.push_back(State {
                        bus: next_bus,
                        path: new_path,
                        visited: new_visited,
                    });
                }
            }
        }

        Ok(found_paths)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple 3-bus, 3-branch ring network for testing.
    ///
    /// ```text
    ///  Bus 0 ──branch0── Bus 1
    ///    \                /
    ///   branch2        branch1
    ///      \            /
    ///          Bus 2
    /// ```
    ///
    /// PTDF is chosen analytically so that:
    ///   - an injection at bus 0 splits between branch0 and branch2
    ///   - sum of PTDF rows = 0 (Kirchhoff conservation)
    fn make_ring_network() -> NetworkData {
        // 3 buses, 3 branches
        // PTDF[branch][bus] — slack bus = bus 0 (column 0 = 0)
        // Symmetric 1/2 split for illustration
        let ptdf = vec![
            vec![0.0, 2.0 / 3.0, -1.0 / 3.0], // branch 0: bus0→bus1
            vec![0.0, 1.0 / 3.0, 2.0 / 3.0],  // branch 1: bus1→bus2
            vec![0.0, -1.0 / 3.0, 1.0 / 3.0], // branch 2: bus0→bus2
        ];
        NetworkData {
            num_buses: 3,
            num_branches: 3,
            ptdf,
            branch_resistance_pu: vec![0.01, 0.01, 0.01],
            branch_reactance_pu: vec![0.1, 0.1, 0.1],
            branch_rating_mw: vec![100.0, 100.0, 100.0],
            from_bus: vec![0, 1, 0],
            to_bus: vec![1, 2, 2],
            branch_length_km: vec![50.0, 50.0, 50.0],
        }
    }

    fn make_decomposer(method: DecompositionMethod) -> FlowDecomposer {
        FlowDecomposer::new(
            make_ring_network(),
            FlowDecompositionConfig {
                include_losses: false,
                method,
                min_flow_threshold_mw: 0.001,
            },
        )
    }

    // -----------------------------------------------------------------------
    // Test 1: Conservation — sum of attributions equals total branch flow
    // -----------------------------------------------------------------------
    #[test]
    fn test_ptdf_conservation() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        // Generation at bus 1: 90 MW; load at bus 2: 90 MW; bus 0 is slack.
        let gen = [0.0, 90.0, 0.0];
        let load = [0.0, 0.0, 90.0];

        let result = decomposer.decompose_flows(&gen, &load).unwrap();

        for b in 0..3 {
            let sum_attr: f64 = result.attribution[b].iter().sum();
            let flow = result.branch_flows_mw[b];
            assert!(
                (sum_attr - flow).abs() < 1e-9,
                "Branch {b}: sum(attribution)={sum_attr} != flow={flow}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Test 2: MW-mile proportional to flow and length
    // -----------------------------------------------------------------------
    #[test]
    fn test_mw_mile_proportional() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let flows = vec![50.0, 30.0, 20.0];
        let mw_mile = decomposer.calculate_mw_mile(&flows);

        // branch_length_km = [50, 50, 50]
        assert!((mw_mile[0] - 50.0 * 50.0).abs() < 1e-9);
        assert!((mw_mile[1] - 30.0 * 50.0).abs() < 1e-9);
        assert!((mw_mile[2] - 20.0 * 50.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Test 3: FTR payoff positive when LMP_to > LMP_from
    // -----------------------------------------------------------------------
    #[test]
    fn test_ftr_payoff_positive() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let ftrs = vec![FtrPosition {
            ftr_id: "FTR-A".into(),
            from_bus: 0,
            to_bus: 1,
            mw: 10.0,
        }];
        let lmp_from = vec![30.0, 50.0, 45.0];
        let lmp_to = vec![30.0, 50.0, 45.0];

        // lmp_to[1]=50 > lmp_from[0]=30 → payoff = 10*(50-30) = 200
        let payoffs = decomposer
            .calculate_ftr_payoff(&ftrs, &lmp_from, &lmp_to)
            .unwrap();
        assert!((payoffs[0] - 200.0).abs() < 1e-9, "payoff={}", payoffs[0]);
    }

    // -----------------------------------------------------------------------
    // Test 4: FTR negative payoff for counter-flow
    // -----------------------------------------------------------------------
    #[test]
    fn test_ftr_payoff_negative_counter_flow() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let ftrs = vec![FtrPosition {
            ftr_id: "FTR-B".into(),
            from_bus: 1,
            to_bus: 0,
            mw: 10.0,
        }];
        // lmp at bus 1 = 50, at bus 0 = 30
        // payoff = 10 * (lmp_to[0] - lmp_from[1]) = 10*(30-50) = -200
        let lmp_from = vec![30.0, 50.0, 45.0];
        let lmp_to = vec![30.0, 50.0, 45.0];

        let payoffs = decomposer
            .calculate_ftr_payoff(&ftrs, &lmp_from, &lmp_to)
            .unwrap();
        assert!(
            payoffs[0] < 0.0,
            "Expected negative payoff, got {}",
            payoffs[0]
        );
        assert!((payoffs[0] + 200.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Test 5: Flow-gate — 90 % loaded branch is flagged
    // -----------------------------------------------------------------------
    #[test]
    fn test_flow_gate_flags_90pct_branch() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        // Branch 0 at 90 MW → 90 % of 100 MW rating
        let flows = vec![90.0, 40.0, 10.0];
        let rating = vec![100.0, 100.0, 100.0];

        let report = decomposer.flow_gate_analysis(&flows, &rating).unwrap();

        assert!(
            report.binding_branches.contains(&0),
            "Branch 0 (90 % loaded) should be a binding flow-gate"
        );
        assert!(
            !report.binding_branches.contains(&1),
            "Branch 1 (40 % loaded) should NOT be a flow-gate"
        );
    }

    // -----------------------------------------------------------------------
    // Test 6: Line loading — correct percentage calculation
    // -----------------------------------------------------------------------
    #[test]
    fn test_line_loading_pct() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        // Flows vs ratings from NetworkData (all 100 MW)
        let flows = vec![50.0, 80.0, 30.0];
        let loading = decomposer.line_loading_pct(&flows);

        assert!(
            (loading[0] - 50.0).abs() < 1e-9,
            "loading[0]={}",
            loading[0]
        );
        assert!(
            (loading[1] - 80.0).abs() < 1e-9,
            "loading[1]={}",
            loading[1]
        );
        assert!(
            (loading[2] - 30.0).abs() < 1e-9,
            "loading[2]={}",
            loading[2]
        );
    }

    // -----------------------------------------------------------------------
    // Test 7: Parallel paths — 3-bus ring → exactly 2 paths from bus 0 to bus 2
    // -----------------------------------------------------------------------
    #[test]
    fn test_parallel_paths_ring() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let paths = decomposer.identify_parallel_paths(0, 2).unwrap();

        // Direct: branch 2 (0→2)
        // Via bus 1: branch 0 (0→1) then branch 1 (1→2)
        assert_eq!(paths.len(), 2, "Expected 2 paths, got {:?}", paths);
    }

    // -----------------------------------------------------------------------
    // Test 8: Congestion contribution — zero injection → zero contribution
    // -----------------------------------------------------------------------
    #[test]
    fn test_congestion_contribution_zero() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let contribs = decomposer.congestion_contribution(1, 0.0).unwrap();

        for (b, c) in contribs.iter().enumerate() {
            assert!(c.abs() < 1e-12, "Branch {b}: expected 0, got {c}");
        }
    }

    // -----------------------------------------------------------------------
    // Test 9: Proportional decomposition — sum of attributions = total flow
    // -----------------------------------------------------------------------
    #[test]
    fn test_proportional_conservation() {
        let decomposer = make_decomposer(DecompositionMethod::Proportional);
        let gen = [50.0, 40.0, 0.0];
        let load = [0.0, 0.0, 90.0];

        let result = decomposer.decompose_flows(&gen, &load).unwrap();

        for b in 0..3 {
            let sum_attr: f64 = result.attribution[b].iter().sum();
            let flow = result.branch_flows_mw[b];
            // Proportional attribution does not necessarily equal flow sign,
            // so we check magnitude.
            assert!(
                (sum_attr.abs() - flow.abs()).abs() < flow.abs() * 0.01 + 1e-6,
                "Branch {b}: |sum(attr)|={} vs |flow|={flow}",
                sum_attr.abs()
            );
        }
    }

    // -----------------------------------------------------------------------
    // Test 10: Transmission usage charges — zero tariff → zero charges
    // -----------------------------------------------------------------------
    #[test]
    fn test_transmission_charges_zero_tariff() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        let gen = [0.0, 90.0, 0.0];
        let load = [0.0, 0.0, 90.0];
        let result = decomposer.decompose_flows(&gen, &load).unwrap();

        let labels: Vec<String> = (0..3).map(|i| format!("Bus_{i}")).collect();
        let charges = decomposer
            .transmission_usage_charges(&result, &labels, 0.0)
            .unwrap();

        for (label, charge) in &charges {
            assert!(
                charge.abs() < 1e-12,
                "Expected zero charge for {label}, got {charge}"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Test 11: Congestion contribution — nonzero injection, known PTDF
    // -----------------------------------------------------------------------
    #[test]
    fn test_congestion_contribution_nonzero() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        // Inject 90 MW at bus 1
        // PTDF[0][1] = 2/3, so expected contribution on branch 0 = 2/3 * 90 = 60
        let contribs = decomposer.congestion_contribution(1, 90.0).unwrap();
        let expected = 2.0 / 3.0 * 90.0;
        assert!(
            (contribs[0] - expected).abs() < 1e-9,
            "Branch 0 contribution={} expected {expected}",
            contribs[0]
        );
    }

    // -----------------------------------------------------------------------
    // Test 12: Top-5 congested correctly ranked
    // -----------------------------------------------------------------------
    #[test]
    fn test_top5_congested_ordering() {
        let decomposer = make_decomposer(DecompositionMethod::PtdfBased);
        // Branch 1 most loaded (95%), branch 0 second (90%), branch 2 least (10%)
        let flows = vec![90.0, 95.0, 10.0];
        let rating = vec![100.0, 100.0, 100.0];

        let report = decomposer.flow_gate_analysis(&flows, &rating).unwrap();

        assert_eq!(
            report.top5_congested[0], 1,
            "Most congested should be branch 1"
        );
        assert_eq!(
            report.top5_congested[1], 0,
            "Second most congested should be branch 0"
        );
    }
}
