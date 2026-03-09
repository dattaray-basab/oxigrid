//! Real-Time Market (RTM) / Balancing Market.
//!
//! Clears 5-minute balancing intervals, settling imbalances between day-ahead
//! schedule and real-time conditions.
//!
//! # References
//! - PJM "Manual 11: Energy & Ancillary Services Market Operations", rev. 2023
//! - CAISO "Real-Time Market", 2023
use crate::error::Result;

/// Real-time market configuration.
#[derive(Debug, Clone)]
pub struct RtmConfig {
    /// Dispatch interval in minutes (default 5)
    pub interval_minutes: usize,
    /// Imbalance price multiplier over DA price for violations (default 3.0)
    pub imbalance_price_factor: f64,
    /// Whether Automatic Generation Control (AGC) is active
    pub agc_enabled: bool,
}

impl Default for RtmConfig {
    fn default() -> Self {
        Self {
            interval_minutes: 5,
            imbalance_price_factor: 3.0,
            agc_enabled: true,
        }
    }
}

/// Real-time offer from a generator — incremental/decremental capability above DA.
#[derive(Debug, Clone)]
pub struct RtmOffer {
    /// Unit identifier (must match DA schedule)
    pub unit_id: usize,
    /// MW available *above* DA schedule (upward regulation headroom)
    pub p_available: f64,
    /// MW available *below* DA schedule (downward regulation headroom)
    pub p_curtailable: f64,
    /// Bid price for upward adjustment \[$/MWh\]
    pub bid_up: f64,
    /// Bid price for downward adjustment \[$/MWh\]
    pub bid_down: f64,
}

/// Real-time market clearing result.
#[derive(Debug, Clone)]
pub struct RtmResult {
    /// Adjustment per unit \[MW\]: positive = upward, negative = downward
    pub adjustments: Vec<f64>,
    /// Real-time clearing price \[$/MWh\]
    pub clearing_price: f64,
    /// System imbalance before clearing \[MW\]: positive = shortage
    pub total_imbalance: f64,
    /// Residual imbalance after clearing \[MW\]
    pub settled_imbalance: f64,
}

/// Real-time balancing market.
pub struct RealTimeMarket {
    /// Day-ahead schedule \[MW\] per unit
    pub da_schedule: Vec<f64>,
    /// Day-ahead prices \[$/MWh\] per bus
    pub da_prices: Vec<f64>,
    /// RTM configuration
    pub config: RtmConfig,
}

impl RealTimeMarket {
    /// Create a new real-time market.
    pub fn new(da_schedule: Vec<f64>, da_prices: Vec<f64>, config: RtmConfig) -> Self {
        Self {
            da_schedule,
            da_prices,
            config,
        }
    }

    /// Clear the real-time balancing market.
    ///
    /// # Algorithm
    /// 1. Compute system imbalance: RT_demand + (DA_renewable - RT_renewable) - Σ DA_schedule.
    /// 2. If shortage (imbalance > 0): merit-order dispatch of upward offers.
    /// 3. If surplus (imbalance < 0): merit-order dispatch of downward offers (cheapest curtailment).
    /// 4. Clearing price = price of marginal adjustment offer, or
    ///    imbalance_factor * DA_price if imbalance remains unresolved.
    pub fn clear(
        &self,
        rt_offers: &[RtmOffer],
        rt_demand: f64,
        renewable_actual: f64,
    ) -> Result<RtmResult> {
        let da_total_gen: f64 = self.da_schedule.iter().sum();

        // System imbalance: positive = shortage (RT needs more than DA scheduled)
        // renewable_actual is an additional injection not in the DA schedule.
        let total_imbalance = rt_demand - da_total_gen - renewable_actual;
        let imbalance = total_imbalance;

        let mut adjustments = vec![0.0f64; self.da_schedule.len()];
        let mut clearing_price = self.da_prices.first().copied().unwrap_or(30.0);
        let mut residual = imbalance;

        if imbalance > 1e-6 {
            // Shortage: activate upward offers in merit order
            let mut sorted_up: Vec<&RtmOffer> = rt_offers.iter().collect();
            sorted_up.sort_by(|a, b| {
                a.bid_up
                    .partial_cmp(&b.bid_up)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            for offer in sorted_up {
                if residual <= 1e-6 {
                    break;
                }
                let activated = offer.p_available.min(residual);
                if activated > 0.0 {
                    let idx = offer.unit_id.min(self.da_schedule.len().saturating_sub(1));
                    adjustments[idx] += activated;
                    residual -= activated;
                    clearing_price = offer.bid_up;
                }
            }

            if residual > 1e-6 {
                let da_price = self.da_prices.first().copied().unwrap_or(30.0);
                clearing_price = da_price * self.config.imbalance_price_factor;
            }
        } else if imbalance < -1e-6 {
            // Surplus: activate downward offers in merit order (cheapest curtailment)
            let mut sorted_down: Vec<&RtmOffer> = rt_offers.iter().collect();
            sorted_down.sort_by(|a, b| {
                a.bid_down
                    .partial_cmp(&b.bid_down)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let mut surplus = -imbalance;
            for offer in sorted_down {
                if surplus <= 1e-6 {
                    break;
                }
                let curtailed = offer.p_curtailable.min(surplus);
                if curtailed > 0.0 {
                    let idx = offer.unit_id.min(self.da_schedule.len().saturating_sub(1));
                    adjustments[idx] -= curtailed;
                    surplus -= curtailed;
                    clearing_price = offer.bid_down;
                }
            }
            residual = -surplus.max(0.0);
        }

        Ok(RtmResult {
            adjustments,
            clearing_price,
            total_imbalance,
            settled_imbalance: residual,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rtm_upward_balancing() {
        let da_schedule = vec![80.0, 20.0];
        let da_prices = vec![30.0, 35.0];
        let config = RtmConfig::default();
        let rtm = RealTimeMarket::new(da_schedule, da_prices, config);

        let rt_offers = vec![
            RtmOffer {
                unit_id: 0,
                p_available: 20.0,
                p_curtailable: 10.0,
                bid_up: 45.0,
                bid_down: 15.0,
            },
            RtmOffer {
                unit_id: 1,
                p_available: 30.0,
                p_curtailable: 5.0,
                bid_up: 55.0,
                bid_down: 20.0,
            },
        ];

        let result = rtm.clear(&rt_offers, 120.0, 0.0).expect("RTM should clear");

        assert!(
            result.total_imbalance > 0.0,
            "Should have upward imbalance: {:.2}",
            result.total_imbalance
        );
        assert!(
            result.clearing_price > 0.0,
            "Clearing price should be positive: {:.2}",
            result.clearing_price
        );
    }

    #[test]
    fn test_rtm_downward_balancing() {
        let da_schedule = vec![100.0, 50.0];
        let da_prices = vec![30.0, 35.0];
        let config = RtmConfig::default();
        let rtm = RealTimeMarket::new(da_schedule, da_prices, config);

        let rt_offers = vec![RtmOffer {
            unit_id: 0,
            p_available: 10.0,
            p_curtailable: 40.0,
            bid_up: 45.0,
            bid_down: 10.0,
        }];

        // RT demand 80 MW, DA 150 MW scheduled → surplus
        let result = rtm.clear(&rt_offers, 80.0, 0.0).expect("RTM should clear");
        assert!(result.total_imbalance < 0.0, "Should have surplus");
    }

    #[test]
    fn test_rtm_no_imbalance() {
        let da_schedule = vec![100.0];
        let da_prices = vec![30.0];
        let config = RtmConfig::default();
        let rtm = RealTimeMarket::new(da_schedule, da_prices, config);
        let result = rtm.clear(&[], 100.0, 0.0).expect("RTM should clear");
        assert!(result.total_imbalance.abs() < 1e-6);
    }
}
