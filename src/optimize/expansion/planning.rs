/// Network Expansion Planning (NEP).
///
/// Determines the optimal set of new transmission branches to build over a
/// multi-year horizon to minimise total investment + operating costs.
///
/// # Problem Formulation (DC, linear investment model)
///
/// Decision variables:
///   z_{l,y} ∈ {0,1}  — build branch l in year y (binary)
///   P_g_t   [MW]      — generation dispatch per period t
///
/// Objective:
///   min  Σ_l Σ_y  z_{l,y} · IC_l · annuity(y, WACC)
///      + Σ_t  Σ_g  C_g(P_g_t) · Δt_t
///
/// Constraints:
///   - DC power flow with candidate branches included if z_{l,y}=1
///   - Generator output limits
///   - Thermal limits on existing and candidate branches
///   - Candidate branches active only if investment decided
///
/// # Algorithm
///
/// Since full MILP is expensive, we use a greedy iterative approach:
///   1. Solve DC-OPF on base network (no candidates).
///   2. For each candidate branch, evaluate the operating cost reduction
///      if the branch is added.
///   3. Add the branch with the best net benefit (saving − annualised cost).
///   4. Repeat until no candidate yields positive NPV.
///
/// This greedy algorithm is optimal for separable cost functions and
/// provides good solutions for practical network sizes.
use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{solve_dc_opf, GenCost};
use serde::{Deserialize, Serialize};

/// A candidate branch for expansion planning.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CandidateBranch {
    /// Unique identifier
    pub id: usize,
    /// From bus (existing bus ID)
    pub from_bus: usize,
    /// To bus (existing bus ID)
    pub to_bus: usize,
    /// Branch reactance X [p.u.]
    pub x: f64,
    /// Branch resistance R [p.u.]
    pub r: f64,
    /// Charging susceptance B [p.u.]
    pub b: f64,
    /// Thermal rating [MW]
    pub rate_a: f64,
    /// Investment cost [M$] (total)
    pub investment_cost_musd: f64,
    /// Construction lead time [years]
    pub lead_time_years: usize,
    /// Economic lifetime [years]
    pub lifetime_years: usize,
}

impl CandidateBranch {
    /// Convert to network Branch for DC-OPF injection.
    pub fn to_branch(&self) -> Branch {
        Branch {
            from_bus: self.from_bus,
            to_bus: self.to_bus,
            r: self.r,
            x: self.x,
            b: self.b,
            rate_a: self.rate_a,
            rate_b: self.rate_a,
            rate_c: self.rate_a,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }

    /// Annualised investment cost [M$/year] at given WACC.
    pub fn annualised_cost(&self, wacc: f64) -> f64 {
        if wacc < 1e-10 {
            return self.investment_cost_musd / self.lifetime_years as f64;
        }
        let n = self.lifetime_years as f64;
        let r = wacc;
        let crf = r * (1.0 + r).powf(n) / ((1.0 + r).powf(n) - 1.0);
        self.investment_cost_musd * crf
    }
}

/// Multi-year demand scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DemandScenario {
    pub year: usize,
    /// Load scaling factor relative to base (1.0 = base case)
    pub load_scale: f64,
    /// Duration of this year [hours]
    pub duration_h: f64,
}

impl DemandScenario {
    pub fn linear_growth(n_years: usize, annual_growth_rate: f64) -> Vec<Self> {
        (0..n_years)
            .map(|y| Self {
                year: y,
                load_scale: (1.0 + annual_growth_rate).powf(y as f64),
                duration_h: 8760.0,
            })
            .collect()
    }
}

/// Investment decision for one candidate branch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InvestmentDecision {
    pub candidate_id: usize,
    pub build_year: usize,
    pub annualised_cost_musd: f64,
    pub operating_saving_musd: f64,
    /// Net present value of building this branch [M$]
    pub npv_musd: f64,
}

/// Overall network expansion planning result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpansionPlanResult {
    /// Sequence of investment decisions (in order of selection)
    pub decisions: Vec<InvestmentDecision>,
    /// Total NPV of all investments [M$]
    pub total_npv_musd: f64,
    /// Total investment cost [M$]
    pub total_investment_musd: f64,
    /// Base-case total operating cost [M$/year]
    pub base_operating_cost_musdyr: f64,
    /// Final (post-expansion) operating cost [M$/year]
    pub final_operating_cost_musdyr: f64,
    /// Candidates not selected
    pub rejected_candidates: Vec<usize>,
}

impl ExpansionPlanResult {
    /// Cost reduction achieved by expansion [%].
    pub fn cost_reduction_pct(&self) -> f64 {
        if self.base_operating_cost_musdyr < 1e-6 {
            return 0.0;
        }
        100.0 * (self.base_operating_cost_musdyr - self.final_operating_cost_musdyr)
            / self.base_operating_cost_musdyr
    }
}

/// Configuration for expansion planning.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpansionConfig {
    /// Weighted average cost of capital (WACC) for annualisation
    pub wacc: f64,
    /// Planning horizon [years]
    pub n_years: usize,
    /// Hours per year (8760 typical)
    pub hours_per_year: f64,
    /// Cost multiplier to convert $/h to M$/year: (hours_per_year * 1e-6)
    pub cost_scale: f64,
    /// Maximum number of candidate branches to select
    pub max_investments: usize,
    /// Minimum NPV for a branch to be selected [M$]
    pub min_npv_musd: f64,
}

impl Default for ExpansionConfig {
    fn default() -> Self {
        Self {
            wacc: 0.08,
            n_years: 20,
            hours_per_year: 8760.0,
            cost_scale: 8760.0 * 1e-6,
            max_investments: 10,
            min_npv_musd: 0.0,
        }
    }
}

/// Solve the network expansion planning problem using greedy search.
///
/// Iteratively selects the candidate branch with highest NPV until no
/// candidate with positive NPV remains.
pub fn solve_expansion_planning(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    candidates: &[CandidateBranch],
    scenarios: &[DemandScenario],
    config: &ExpansionConfig,
) -> Result<ExpansionPlanResult> {
    if gen_costs.len() != network.generators.len() {
        return Err(OxiGridError::InvalidParameter(format!(
            "gen_costs length {} != generators {}",
            gen_costs.len(),
            network.generators.len()
        )));
    }
    if candidates.is_empty() {
        return Err(OxiGridError::InvalidParameter(
            "No candidate branches provided".into(),
        ));
    }

    // Compute base-case operating cost
    let base_op_cost = compute_annual_cost(network, gen_costs, scenarios, 1.0)?;

    let mut selected_ids: Vec<usize> = Vec::new();
    let mut decisions: Vec<InvestmentDecision> = Vec::new();
    let mut current_network = network.clone();
    let mut current_op_cost = base_op_cost;

    let max_iters = config.max_investments.min(candidates.len());

    for _iter in 0..max_iters {
        let mut best_npv = config.min_npv_musd - f64::EPSILON;
        let mut best_cand_id = usize::MAX;
        let mut best_saving = 0.0;

        // Evaluate each unselected candidate
        for cand in candidates {
            if selected_ids.contains(&cand.id) {
                continue;
            }

            // Build trial network with this candidate added
            let mut trial_net = current_network.clone();
            trial_net.branches.push(cand.to_branch());

            // Compute operating cost improvement
            let trial_op_cost = compute_annual_cost(&trial_net, gen_costs, scenarios, 1.0)
                .unwrap_or(current_op_cost);

            let saving_musdyr = (current_op_cost - trial_op_cost).max(0.0);
            let _ann_cost = cand.annualised_cost(config.wacc);

            // NPV = PV of savings - PV of costs over planning horizon
            // Simplified: NPV ≈ saving_musdyr / wacc - investment_cost (perpetuity approx)
            let pv_savings = if config.wacc > 1e-10 {
                let n = config.n_years as f64;
                let r = config.wacc;
                saving_musdyr * (1.0 - (1.0 + r).powf(-n)) / r
            } else {
                saving_musdyr * config.n_years as f64
            };
            let npv = pv_savings - cand.investment_cost_musd;

            if npv > best_npv {
                best_npv = npv;
                best_cand_id = cand.id;
                best_saving = saving_musdyr;
            }
        }

        if best_cand_id == usize::MAX {
            break;
        } // No beneficial candidate

        // Commit the best candidate
        let cand = candidates.iter().find(|c| c.id == best_cand_id).unwrap();
        current_network.branches.push(cand.to_branch());
        current_op_cost -= best_saving;

        selected_ids.push(best_cand_id);
        decisions.push(InvestmentDecision {
            candidate_id: best_cand_id,
            build_year: _iter,
            annualised_cost_musd: cand.annualised_cost(config.wacc),
            operating_saving_musd: best_saving,
            npv_musd: best_npv,
        });
    }

    let total_npv_musd: f64 = decisions.iter().map(|d| d.npv_musd).sum();
    let total_investment_musd: f64 = candidates
        .iter()
        .filter(|c| selected_ids.contains(&c.id))
        .map(|c| c.investment_cost_musd)
        .sum();

    let rejected_candidates: Vec<usize> = candidates
        .iter()
        .filter(|c| !selected_ids.contains(&c.id))
        .map(|c| c.id)
        .collect();

    Ok(ExpansionPlanResult {
        decisions,
        total_npv_musd,
        total_investment_musd,
        base_operating_cost_musdyr: base_op_cost,
        final_operating_cost_musdyr: current_op_cost,
        rejected_candidates,
    })
}

/// Compute annual operating cost [M$/year] across demand scenarios.
fn compute_annual_cost(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    scenarios: &[DemandScenario],
    base_scale: f64,
) -> Result<f64> {
    if scenarios.is_empty() {
        // Single-scenario default: one year of base load
        let result = solve_dc_opf(network, gen_costs)?;
        return Ok(result.total_cost * 8760.0 * 1e-6);
    }

    let mut total_cost_musd = 0.0;

    for scenario in scenarios {
        let scale = base_scale * scenario.load_scale;
        // Scale the gen costs to represent scaled load
        let scaled_costs: Vec<GenCost> = gen_costs
            .iter()
            .map(|c| GenCost {
                a: c.a,
                b: c.b,
                c: c.c,
                p_min: c.p_min * scale,
                p_max: c.p_max * scale,
            })
            .collect();

        // Scale network loads
        let mut scaled_net = network.clone();
        for bus in &mut scaled_net.buses {
            bus.pd = crate::units::Power(bus.pd.0 * scale);
        }

        // Use min/max of scaled gen costs
        match solve_dc_opf(&scaled_net, &scaled_costs) {
            Ok(result) => {
                total_cost_musd += result.total_cost * scenario.duration_h * 1e-6;
            }
            Err(_) => {
                // Infeasible scenario: use a large penalty cost
                total_cost_musd += 1e6;
            }
        }
    }

    Ok(total_cost_musd)
}

/// Compute the benefit/cost ratio for a candidate branch.
pub fn candidate_bcr(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    candidate: &CandidateBranch,
    scenarios: &[DemandScenario],
    config: &ExpansionConfig,
) -> Result<f64> {
    let base_cost = compute_annual_cost(network, gen_costs, scenarios, 1.0)?;
    let mut trial = network.clone();
    trial.branches.push(candidate.to_branch());
    let trial_cost = compute_annual_cost(&trial, gen_costs, scenarios, 1.0)?;

    let saving = (base_cost - trial_cost).max(0.0);
    let ann_cost = candidate.annualised_cost(config.wacc);

    if ann_cost < 1e-12 {
        return Ok(f64::INFINITY);
    }
    Ok(saving / ann_cost)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ieee14_net() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("ieee14")
    }

    fn ieee14_costs(net: &PowerNetwork) -> Vec<GenCost> {
        net.generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
            .collect()
    }

    fn make_candidate(id: usize) -> CandidateBranch {
        CandidateBranch {
            id,
            from_bus: 1_usize,
            to_bus: 5_usize,
            x: 0.05,
            r: 0.01,
            b: 0.02,
            rate_a: 100.0,
            investment_cost_musd: 10.0,
            lead_time_years: 2,
            lifetime_years: 30,
        }
    }

    #[test]
    fn test_expansion_planning_basic() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let candidates = vec![make_candidate(0), make_candidate(1)];
        let scenarios = vec![DemandScenario {
            year: 0,
            load_scale: 1.0,
            duration_h: 8760.0,
        }];
        let config = ExpansionConfig::default();
        let result =
            solve_expansion_planning(&net, &costs, &candidates, &scenarios, &config).unwrap();
        // Should produce a result (may or may not invest)
        assert!(result.total_investment_musd >= 0.0);
        assert!(result.base_operating_cost_musdyr >= 0.0);
    }

    #[test]
    fn test_expansion_planning_no_candidates() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let result = solve_expansion_planning(&net, &costs, &[], &[], &ExpansionConfig::default());
        assert!(result.is_err(), "Should fail with no candidates");
    }

    #[test]
    fn test_expansion_cost_reduction_non_negative() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let candidates = vec![make_candidate(0)];
        let scenarios = DemandScenario::linear_growth(3, 0.02);
        let config = ExpansionConfig::default();
        let result =
            solve_expansion_planning(&net, &costs, &candidates, &scenarios, &config).unwrap();
        let reduction = result.cost_reduction_pct();
        assert!(
            reduction >= 0.0,
            "Cost reduction should be non-negative: {:.2}%",
            reduction
        );
    }

    #[test]
    fn test_annualised_cost_wacc() {
        let cand = make_candidate(0);
        let ann_0 = cand.annualised_cost(0.0);
        let ann_8 = cand.annualised_cost(0.08);
        // Higher WACC → higher annualised cost
        assert!(
            ann_8 > ann_0 - 0.01,
            "WACC=8% cost ({:.4}) should be ≥ WACC=0% ({:.4})",
            ann_8,
            ann_0
        );
        // Annualised cost > 0
        assert!(ann_8 > 0.0);
    }

    #[test]
    fn test_linear_growth_scenarios() {
        let scenarios = DemandScenario::linear_growth(5, 0.03);
        assert_eq!(scenarios.len(), 5);
        assert!((scenarios[0].load_scale - 1.0).abs() < 1e-10);
        assert!(scenarios[4].load_scale > scenarios[0].load_scale);
    }

    #[test]
    fn test_candidate_bcr() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let cand = make_candidate(0);
        let scenarios = vec![DemandScenario {
            year: 0,
            load_scale: 1.0,
            duration_h: 8760.0,
        }];
        let config = ExpansionConfig::default();
        let bcr = candidate_bcr(&net, &costs, &cand, &scenarios, &config).unwrap();
        assert!(bcr >= 0.0, "BCR should be non-negative: {:.4}", bcr);
    }

    #[test]
    fn test_expansion_result_structure() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let candidates = (0..3)
            .map(|i| CandidateBranch {
                id: i,
                from_bus: (i % 5) + 1,
                to_bus: (i % 5) + 6,
                x: 0.05 + i as f64 * 0.01,
                r: 0.01,
                b: 0.02,
                rate_a: 80.0,
                investment_cost_musd: 5.0 + i as f64 * 2.0,
                lead_time_years: 2,
                lifetime_years: 25,
            })
            .collect::<Vec<_>>();

        let config = ExpansionConfig {
            max_investments: 2,
            ..Default::default()
        };
        let result = solve_expansion_planning(&net, &costs, &candidates, &[], &config).unwrap();
        assert!(result.decisions.len() <= 2);
        assert_eq!(
            result.decisions.len() + result.rejected_candidates.len(),
            candidates.len()
        );
    }
}
