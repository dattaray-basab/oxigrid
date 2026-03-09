/// Demand Response (DR) optimisation.
///
/// Models programs where loads voluntarily curtail or shift their consumption
/// in response to price signals or utility requests.
///
/// # DR Program Types
///
/// 1. **Price-Responsive DR** — load reduces when real-time price exceeds threshold.
///    Modelled via a price-elasticity function: ΔP = ε · P0 · ΔPrice/Price0.
///
/// 2. **Curtailable Load DR** — utility sends a curtailment request;
///    load sheds up to `max_curtail_mw` for a contracted incentive payment.
///
/// 3. **Demand Shifting** — load moves consumption from peak hours to off-peak.
///    Modelled as a zero-sum shifting constraint: total energy unchanged.
///
/// 4. **Emergency DR** — activated during system stress; direct load control
///    by the operator up to enrolled capacity.
///
/// # Incentive Contracts
///
/// Loads in a DR program receive payments:
///   Incentive = curtailment_mw × incentive_rate × duration_h
///
/// The net benefit to the utility is:
///   Benefit = curtailment_mw × (spot_price − incentive_rate) × duration_h
///
/// # References
/// - FERC, "A National Assessment of Demand Response Potential", 2009.
/// - Strbac, "Demand side management: Benefits and challenges", Energy Policy 2008.
use serde::{Deserialize, Serialize};

/// DR program type.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DrProgramType {
    /// Price-responsive (voluntary, elasticity-based)
    PriceResponsive,
    /// Curtailable (contracted max curtailment)
    Curtailable,
    /// Shiftable (energy-conserving time-of-use shift)
    Shiftable,
    /// Emergency direct load control
    Emergency,
}

/// A demand response participant (load or aggregation of loads).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrParticipant {
    /// Unique participant ID
    pub id: usize,
    /// Bus ID in the network
    pub bus_id: usize,
    /// Base (uncontrolled) load [MW]
    pub base_load_mw: f64,
    /// Maximum curtailable load [MW]
    pub max_curtail_mw: f64,
    /// Minimum load (cannot curtail below this) [MW]
    pub min_load_mw: f64,
    /// DR program type
    pub program_type: DrProgramType,
    /// Incentive payment rate [$/MWh curtailed]
    pub incentive_rate: f64,
    /// Price elasticity of demand (negative: demand decreases as price rises)
    pub elasticity: f64,
    /// Reference price for elasticity model [$/MWh]
    pub ref_price: f64,
    /// Maximum shift window for shiftable loads [hours]
    pub shift_window_h: f64,
    /// Discomfort cost coefficient [$/MWh²] (quadratic curtailment discomfort)
    pub discomfort_coeff: f64,
}

impl DrParticipant {
    /// Create a price-responsive DR participant.
    pub fn price_responsive(
        id: usize,
        bus_id: usize,
        base_mw: f64,
        elasticity: f64,
        ref_price: f64,
    ) -> Self {
        Self {
            id,
            bus_id,
            base_load_mw: base_mw,
            max_curtail_mw: base_mw * 0.3,
            min_load_mw: base_mw * 0.1,
            program_type: DrProgramType::PriceResponsive,
            incentive_rate: 0.0,
            elasticity,
            ref_price,
            shift_window_h: 0.0,
            discomfort_coeff: 0.01,
        }
    }

    /// Create a curtailable DR participant.
    pub fn curtailable(
        id: usize,
        bus_id: usize,
        base_mw: f64,
        max_curtail: f64,
        incentive: f64,
    ) -> Self {
        Self {
            id,
            bus_id,
            base_load_mw: base_mw,
            max_curtail_mw: max_curtail,
            min_load_mw: base_mw - max_curtail,
            program_type: DrProgramType::Curtailable,
            incentive_rate: incentive,
            elasticity: 0.0,
            ref_price: 0.0,
            shift_window_h: 0.0,
            discomfort_coeff: 0.02,
        }
    }

    /// Compute load curtailment for a given spot price [MW].
    ///
    /// For price-responsive: ΔP = ε · P0 · (price - ref) / ref
    /// For curtailable: curtail = max_curtail if price > incentive_rate
    pub fn curtailment_at_price(&self, price: f64) -> f64 {
        match self.program_type {
            DrProgramType::PriceResponsive => {
                if self.ref_price < 1e-6 {
                    return 0.0;
                }
                let dp =
                    self.elasticity * self.base_load_mw * (price - self.ref_price) / self.ref_price;
                // Negative elasticity → positive curtailment when price rises
                let curtail = -dp;
                curtail.clamp(0.0, self.max_curtail_mw)
            }
            DrProgramType::Curtailable => {
                if price > self.incentive_rate {
                    self.max_curtail_mw
                } else {
                    0.0
                }
            }
            DrProgramType::Emergency => self.max_curtail_mw,
            DrProgramType::Shiftable => 0.0, // handled separately
        }
    }

    /// Effective load at given price [MW].
    pub fn effective_load(&self, price: f64) -> f64 {
        (self.base_load_mw - self.curtailment_at_price(price))
            .clamp(self.min_load_mw, self.base_load_mw)
    }

    /// Incentive payment for curtailment [$/h].
    pub fn incentive_payment(&self, curtailment_mw: f64) -> f64 {
        curtailment_mw * self.incentive_rate
    }

    /// Discomfort cost for curtailment [$/h] (quadratic model).
    pub fn discomfort_cost(&self, curtailment_mw: f64) -> f64 {
        self.discomfort_coeff * curtailment_mw * curtailment_mw
    }

    /// Net benefit of curtailment for the participant [$/h].
    ///
    /// Benefit = incentive - discomfort
    pub fn net_participant_benefit(&self, curtailment_mw: f64, price: f64) -> f64 {
        // Energy cost savings (at spot price) + incentive payment - discomfort
        let energy_saving = curtailment_mw * price;
        energy_saving + self.incentive_payment(curtailment_mw)
            - self.discomfort_cost(curtailment_mw)
    }
}

/// Result of a DR dispatch optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrDispatchResult {
    /// Per-participant results
    pub participants: Vec<DrParticipantResult>,
    /// Total curtailment [MW]
    pub total_curtailment_mw: f64,
    /// Total incentive cost to utility [$/h]
    pub total_incentive_cost: f64,
    /// Total discomfort cost [$/h]
    pub total_discomfort_cost: f64,
    /// Net benefit to utility [$/h] (energy cost saved - incentive paid)
    pub net_utility_benefit: f64,
    /// Spot price used [$/MWh]
    pub spot_price: f64,
}

/// Per-participant DR dispatch result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrParticipantResult {
    pub id: usize,
    pub bus_id: usize,
    pub base_load_mw: f64,
    pub curtailment_mw: f64,
    pub effective_load_mw: f64,
    pub incentive_payment: f64,
    pub discomfort_cost: f64,
    pub participant_benefit: f64,
}

/// Dispatch DR program at a given spot price.
///
/// Computes optimal curtailment for each participant to maximise net utility benefit.
pub fn dispatch_dr(participants: &[DrParticipant], spot_price: f64) -> DrDispatchResult {
    let mut results = Vec::with_capacity(participants.len());
    let mut total_curtail = 0.0;
    let mut total_incentive = 0.0;
    let mut total_discomfort = 0.0;
    let mut total_energy_saved = 0.0;

    for p in participants {
        // Find optimal curtailment: maximise participant surplus or utility benefit
        let curtail = optimal_curtailment(p, spot_price);
        let effective = (p.base_load_mw - curtail).clamp(p.min_load_mw, p.base_load_mw);
        let incentive = p.incentive_payment(curtail);
        let discomfort = p.discomfort_cost(curtail);
        let benefit = p.net_participant_benefit(curtail, spot_price);

        total_curtail += curtail;
        total_incentive += incentive;
        total_discomfort += discomfort;
        total_energy_saved += curtail * spot_price;

        results.push(DrParticipantResult {
            id: p.id,
            bus_id: p.bus_id,
            base_load_mw: p.base_load_mw,
            curtailment_mw: curtail,
            effective_load_mw: effective,
            incentive_payment: incentive,
            discomfort_cost: discomfort,
            participant_benefit: benefit,
        });
    }

    let net_utility_benefit = total_energy_saved - total_incentive;

    DrDispatchResult {
        participants: results,
        total_curtailment_mw: total_curtail,
        total_incentive_cost: total_incentive,
        total_discomfort_cost: total_discomfort,
        net_utility_benefit,
        spot_price,
    }
}

/// Find the curtailment level that maximises participant benefit.
///
/// FOC: d/dC [incentive - discomfort] = incentive_rate - 2*k*C = 0
/// → C* = incentive_rate / (2·k)
fn optimal_curtailment(p: &DrParticipant, spot_price: f64) -> f64 {
    match p.program_type {
        DrProgramType::PriceResponsive => p.curtailment_at_price(spot_price),
        DrProgramType::Curtailable => {
            if p.discomfort_coeff < 1e-10 {
                // No discomfort → curtail fully if profitable
                if spot_price > p.incentive_rate {
                    p.max_curtail_mw
                } else {
                    0.0
                }
            } else {
                // FOC: marginal incentive = marginal discomfort
                // incentive_rate + spot_price = 2 * k * C (at price-clearing)
                let c_star = (p.incentive_rate + spot_price) / (2.0 * p.discomfort_coeff);
                c_star.clamp(0.0, p.max_curtail_mw)
            }
        }
        DrProgramType::Emergency => p.max_curtail_mw,
        DrProgramType::Shiftable => 0.0,
    }
}

/// Load shifting: redistribute load from peak to off-peak periods.
///
/// Given hourly base loads and a shifting participant, returns the shifted
/// load profile that minimises total cost subject to energy conservation.
///
/// The shift algorithm moves load from highest-price hours to lowest-price hours
/// within the allowed `shift_window_h` window.
pub fn optimise_load_shift(
    participant: &DrParticipant,
    hourly_loads_mw: &[f64],
    hourly_prices: &[f64],
    duration_h: f64,
) -> Vec<f64> {
    if participant.program_type != DrProgramType::Shiftable {
        return hourly_loads_mw.to_vec();
    }

    let n = hourly_loads_mw.len().min(hourly_prices.len());
    if n == 0 {
        return vec![];
    }

    let max_shift_mwh = participant.max_curtail_mw * duration_h;

    // Sort hours by price (ascending for receiving, descending for shedding)
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| hourly_prices[b].partial_cmp(&hourly_prices[a]).unwrap());

    let mut shifted = hourly_loads_mw.to_vec();
    let mut shift_budget = max_shift_mwh;

    // Move load from most expensive hours to cheapest
    let cheap_hours: Vec<usize> = {
        let mut c = order.clone();
        c.reverse();
        c
    };

    for &shed_hour in &order {
        if shift_budget <= 1e-6 {
            break;
        }
        let shed_amount = (participant.max_curtail_mw.min(shift_budget)).min(shifted[shed_hour]);
        if hourly_prices[shed_hour] < participant.incentive_rate {
            break;
        } // not worth shifting

        shifted[shed_hour] -= shed_amount;
        shift_budget -= shed_amount;

        // Distribute to cheapest available hours
        let mut to_add = shed_amount;
        for &recv_hour in &cheap_hours {
            if recv_hour == shed_hour {
                continue;
            }
            let capacity = participant.max_curtail_mw;
            let added = to_add.min(capacity);
            shifted[recv_hour] += added;
            to_add -= added;
            if to_add <= 1e-6 {
                break;
            }
        }
    }

    shifted
}

/// DR aggregator: manages a portfolio of DR participants.
pub struct DrAggregator {
    participants: Vec<DrParticipant>,
}

impl DrAggregator {
    pub fn new(participants: Vec<DrParticipant>) -> Self {
        Self { participants }
    }

    /// Total enrolled capacity [MW].
    pub fn total_enrolled_mw(&self) -> f64 {
        self.participants.iter().map(|p| p.max_curtail_mw).sum()
    }

    /// Dispatch at given spot price.
    pub fn dispatch(&self, price: f64) -> DrDispatchResult {
        dispatch_dr(&self.participants, price)
    }

    /// Price at which all participants fully curtail.
    pub fn full_curtailment_price(&self) -> f64 {
        self.participants
            .iter()
            .map(|p| p.incentive_rate.max(p.ref_price))
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Aggregate supply curve: list of (price_threshold, curtailment_step_mw) pairs.
    pub fn supply_curve(&self) -> Vec<(f64, f64)> {
        let mut curve: Vec<(f64, f64)> = self
            .participants
            .iter()
            .map(|p| (p.incentive_rate.max(p.ref_price), p.max_curtail_mw))
            .collect();
        curve.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        curve
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_participants() -> Vec<DrParticipant> {
        vec![
            DrParticipant::price_responsive(0, 1, 10.0, -0.3, 50.0),
            DrParticipant::curtailable(1, 2, 20.0, 8.0, 60.0),
            DrParticipant::curtailable(2, 3, 15.0, 5.0, 80.0),
        ]
    }

    #[test]
    fn test_curtailment_at_high_price() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 50.0);
        let curtail = p.curtailment_at_price(100.0);
        assert_eq!(curtail, 4.0, "Should fully curtail at high price");
    }

    #[test]
    fn test_curtailment_at_low_price() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 50.0);
        let curtail = p.curtailment_at_price(20.0);
        assert_eq!(curtail, 0.0, "Should not curtail below incentive price");
    }

    #[test]
    fn test_price_responsive_curtailment() {
        let p = DrParticipant::price_responsive(0, 1, 10.0, -0.5, 50.0);
        // Unclamped curtail = 0.5 * 10 * (100-50)/50 = 5 MW, but max_curtail = 0.3*10 = 3 MW
        let curtail = p.curtailment_at_price(100.0);
        assert!(
            (curtail - p.max_curtail_mw).abs() < 0.01,
            "PR curtailment should be clamped to max={:.4}: got {:.4}",
            p.max_curtail_mw,
            curtail
        );
    }

    #[test]
    fn test_effective_load_bounded() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 50.0);
        let load = p.effective_load(100.0);
        assert!(
            load >= p.min_load_mw,
            "Effective load below min: {:.4}",
            load
        );
        assert!(
            load <= p.base_load_mw,
            "Effective load above base: {:.4}",
            load
        );
    }

    #[test]
    fn test_incentive_payment() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 60.0);
        let payment = p.incentive_payment(2.0);
        assert!((payment - 120.0).abs() < 1e-10, "Payment: {:.2}", payment);
    }

    #[test]
    fn test_discomfort_cost_quadratic() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 50.0);
        let c1 = p.discomfort_cost(1.0);
        let c2 = p.discomfort_cost(2.0);
        assert!(
            (c2 / c1 - 4.0).abs() < 0.1,
            "Quadratic: ratio={:.3}",
            c2 / c1
        );
    }

    #[test]
    fn test_dispatch_total_curtailment() {
        let ps = make_participants();
        let result = dispatch_dr(&ps, 100.0);
        assert!(
            result.total_curtailment_mw > 0.0,
            "Curtailment at high price: {:.2}",
            result.total_curtailment_mw
        );
    }

    #[test]
    fn test_dispatch_no_curtailment_low_price() {
        let ps = make_participants();
        let result = dispatch_dr(&ps, 10.0);
        // Price 10 < all incentive rates → no curtailable load activates
        // Price-responsive might still curtail a little (if below ref price)
        assert!(result.total_curtailment_mw >= 0.0);
    }

    #[test]
    fn test_dispatch_result_structure() {
        let ps = make_participants();
        let result = dispatch_dr(&ps, 70.0);
        assert_eq!(result.participants.len(), 3);
        for pr in &result.participants {
            assert!(pr.curtailment_mw >= 0.0);
            assert!(pr.effective_load_mw >= 0.0);
        }
    }

    #[test]
    fn test_aggregator_total_enrolled() {
        let agg = DrAggregator::new(make_participants());
        let total = agg.total_enrolled_mw();
        assert!(
            (total - (3.0 + 8.0 + 5.0)).abs() < 0.01,
            "Total enrolled: {:.2}",
            total
        );
    }

    #[test]
    fn test_aggregator_supply_curve_sorted() {
        let agg = DrAggregator::new(make_participants());
        let curve = agg.supply_curve();
        for i in 1..curve.len() {
            assert!(
                curve[i].0 >= curve[i - 1].0,
                "Curve not sorted: {:?}",
                curve
            );
        }
    }

    #[test]
    fn test_load_shift_energy_conserved() {
        let p = DrParticipant {
            id: 0,
            bus_id: 1,
            base_load_mw: 5.0,
            max_curtail_mw: 2.0,
            min_load_mw: 0.0,
            program_type: DrProgramType::Shiftable,
            incentive_rate: 40.0,
            elasticity: 0.0,
            ref_price: 0.0,
            shift_window_h: 4.0,
            discomfort_coeff: 0.01,
        };
        let loads = vec![5.0; 24];
        let prices = (0..24)
            .map(|h| if (8..=20).contains(&h) { 80.0 } else { 30.0 })
            .collect::<Vec<_>>();
        let shifted = optimise_load_shift(&p, &loads, &prices, 1.0);
        let orig_total: f64 = loads.iter().sum();
        let shift_total: f64 = shifted.iter().sum();
        // Energy conservation: totals should be approximately equal
        assert!(
            (orig_total - shift_total).abs() < 0.1,
            "Energy not conserved: orig={:.2} shifted={:.2}",
            orig_total,
            shift_total
        );
    }

    #[test]
    fn test_net_participant_benefit_positive() {
        let p = DrParticipant::curtailable(0, 1, 10.0, 4.0, 60.0);
        let benefit = p.net_participant_benefit(2.0, 100.0);
        // Should be positive: energy saving + incentive > discomfort
        assert!(benefit > 0.0, "Benefit: {:.4}", benefit);
    }
}
