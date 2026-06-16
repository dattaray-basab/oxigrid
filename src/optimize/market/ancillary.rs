//! Ancillary Services Market: spinning/non-spinning reserve, regulation, voltage support.
//!
//! # References
//! - NERC, "BAL-003: Frequency Response and Frequency Bias Setting", 2022
//! - FERC Order 755, "Frequency Regulation Compensation", 2011
use crate::error::{OxiGridError, Result};

/// Ancillary service type offered by a generator.
#[derive(Debug, Clone)]
pub enum AncillaryService {
    /// Spinning reserve: online unit with headroom, deployable immediately
    SpinningReserve {
        /// Capacity available \[MW\]
        capacity_mw: f64,
        /// Offer price \[$/MWh\]
        bid_mwh: f64,
    },
    /// Non-spinning reserve: offline unit startable within 10 minutes
    NonSpinningReserve {
        /// Capacity available \[MW\]
        capacity_mw: f64,
        /// Offer price \[$/MWh\]
        bid_mwh: f64,
    },
    /// Regulation up: fast-response upward automatic generation control
    RegulationUp {
        /// Capacity available \[MW\]
        capacity_mw: f64,
        /// Offer price \[$/MWh\]
        bid_mwh: f64,
    },
    /// Regulation down: fast-response downward automatic generation control
    RegulationDown {
        /// Capacity available \[MW\]
        capacity_mw: f64,
        /// Offer price \[$/MWh\]
        bid_mwh: f64,
    },
    /// Voltage support: reactive power compensation
    Voltage {
        /// Reactive power capacity \[MVAr\]
        reactive_mvar: f64,
        /// Offer price \[$/MVAr-hr\]
        bid_mvar: f64,
    },
}

impl AncillaryService {
    /// Return capacity in MW (or MVAr for voltage) and bid price.
    pub fn capacity_and_price(&self) -> (f64, f64) {
        match self {
            AncillaryService::SpinningReserve {
                capacity_mw,
                bid_mwh,
            } => (*capacity_mw, *bid_mwh),
            AncillaryService::NonSpinningReserve {
                capacity_mw,
                bid_mwh,
            } => (*capacity_mw, *bid_mwh),
            AncillaryService::RegulationUp {
                capacity_mw,
                bid_mwh,
            } => (*capacity_mw, *bid_mwh),
            AncillaryService::RegulationDown {
                capacity_mw,
                bid_mwh,
            } => (*capacity_mw, *bid_mwh),
            AncillaryService::Voltage {
                reactive_mvar,
                bid_mvar,
            } => (*reactive_mvar, *bid_mvar),
        }
    }

    /// Return the service type tag string.
    pub fn service_type(&self) -> &'static str {
        match self {
            AncillaryService::SpinningReserve { .. } => "spinning",
            AncillaryService::NonSpinningReserve { .. } => "non_spinning",
            AncillaryService::RegulationUp { .. } => "regulation_up",
            AncillaryService::RegulationDown { .. } => "regulation_down",
            AncillaryService::Voltage { .. } => "voltage",
        }
    }
}

/// Ancillary service offer from a generation unit.
#[derive(Debug, Clone)]
pub struct AncillaryOffer {
    /// Unit identifier
    pub unit_id: usize,
    /// Service type and capacity offered
    pub service: AncillaryService,
    /// Availability price \[$/MW-hr\] to be on standby regardless of activation
    pub availability_price: f64,
}

/// Ancillary services clearing result.
#[derive(Debug, Clone)]
pub struct AncillaryResult {
    /// Cleared offers: (unit_id, cleared_capacity_mw)
    pub cleared_offers: Vec<(usize, f64)>,
    /// Uniform clearing price \[$/MW-hr\]
    pub clearing_price: f64,
    /// Whether the requirement was fully met
    pub requirement_met: bool,
    /// MW shortage (0 if requirement_met)
    pub shortage_mw: f64,
}

/// Clear an ancillary services market via merit-order (cheapest-first) clearing.
///
/// # Arguments
/// - `offers`          — ancillary service offers from generators
/// - `requirement_mw`  — total capacity requirement \[MW\]
/// - `service_type`    — filter string: "spinning", "non_spinning", "regulation_up",
///   "regulation_down", "voltage", or "all"
///
/// # Returns
/// [`AncillaryResult`] with cleared offers, clearing price, and residual shortage.
pub fn clear_ancillary_market(
    offers: &[AncillaryOffer],
    requirement_mw: f64,
    service_type: &str,
) -> Result<AncillaryResult> {
    if requirement_mw < 0.0 {
        return Err(OxiGridError::InvalidParameter(
            "Ancillary service requirement cannot be negative".to_string(),
        ));
    }

    // Filter offers by service type
    let mut eligible: Vec<&AncillaryOffer> = offers
        .iter()
        .filter(|o| service_type == "all" || o.service.service_type() == service_type)
        .collect();

    // Sort by bid price ascending (merit order)
    eligible.sort_by(|a, b| {
        let (_, pa) = a.service.capacity_and_price();
        let (_, pb) = b.service.capacity_and_price();
        pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut cleared_offers: Vec<(usize, f64)> = Vec::new();
    let mut remaining = requirement_mw;
    let mut clearing_price = 0.0;

    for offer in &eligible {
        if remaining <= 1e-9 {
            break;
        }
        let (cap, price) = offer.service.capacity_and_price();
        let cleared_cap = cap.min(remaining);
        if cleared_cap > 0.0 {
            cleared_offers.push((offer.unit_id, cleared_cap));
            remaining -= cleared_cap;
            clearing_price = price;
        }
    }

    let shortage = remaining.max(0.0);
    let requirement_met = shortage < 1e-6;

    Ok(AncillaryResult {
        cleared_offers,
        clearing_price,
        requirement_met,
        shortage_mw: shortage,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ancillary_merit_order() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::SpinningReserve {
                    capacity_mw: 30.0,
                    bid_mwh: 5.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::SpinningReserve {
                    capacity_mw: 40.0,
                    bid_mwh: 8.0,
                },
                availability_price: 2.0,
            },
        ];

        let result = clear_ancillary_market(&offers, 50.0, "spinning")
            .expect("Ancillary market should clear");

        assert!(
            result.requirement_met,
            "50 MW should be met with 70 MW available"
        );
        assert_eq!(result.shortage_mw, 0.0);

        let first_unit = result
            .cleared_offers
            .first()
            .expect("At least one cleared offer");
        assert_eq!(first_unit.0, 0, "Cheapest unit (id=0) should clear first");
    }

    #[test]
    fn test_ancillary_negative_requirement_error() {
        let offers: Vec<AncillaryOffer> = vec![];
        let result = clear_ancillary_market(&offers, -10.0, "spinning");
        assert!(result.is_err());
    }

    #[test]
    fn test_ancillary_shortage() {
        let offers = vec![AncillaryOffer {
            unit_id: 0,
            service: AncillaryService::SpinningReserve {
                capacity_mw: 20.0,
                bid_mwh: 5.0,
            },
            availability_price: 1.0,
        }];
        let result = clear_ancillary_market(&offers, 50.0, "spinning").expect("Should not error");
        assert!(!result.requirement_met);
        assert!((result.shortage_mw - 30.0).abs() < 1e-6);
    }

    #[test]
    fn test_regulation_up_clearing() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::RegulationUp {
                    capacity_mw: 20.0,
                    bid_mwh: 12.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::RegulationUp {
                    capacity_mw: 30.0,
                    bid_mwh: 8.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 2,
                service: AncillaryService::RegulationUp {
                    capacity_mw: 10.0,
                    bid_mwh: 15.0,
                },
                availability_price: 1.0,
            },
        ];

        let result = clear_ancillary_market(&offers, 40.0, "regulation_up")
            .expect("Regulation up market should clear");

        assert!(result.requirement_met, "40 MW should be met");
        assert!(
            (result.clearing_price - 12.0).abs() < 1e-9,
            "Marginal price should be 12.0"
        );
        let first = result
            .cleared_offers
            .first()
            .expect("At least one cleared offer");
        assert_eq!(
            first.0, 1,
            "Cheapest unit (bid 8.0, unit_id 1) should clear first"
        );
    }

    #[test]
    fn test_non_spinning_reserve_procurement() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::NonSpinningReserve {
                    capacity_mw: 50.0,
                    bid_mwh: 6.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::NonSpinningReserve {
                    capacity_mw: 30.0,
                    bid_mwh: 4.0,
                },
                availability_price: 1.0,
            },
        ];

        let result = clear_ancillary_market(&offers, 60.0, "non_spinning")
            .expect("Non-spinning market should clear");

        assert!(result.requirement_met, "60 MW should be met");
        let first = result
            .cleared_offers
            .first()
            .expect("At least one cleared offer");
        assert_eq!(
            first.0, 1,
            "Cheapest unit (bid 4.0, unit_id 1) should clear first"
        );
        assert!(
            (result.clearing_price - 6.0).abs() < 1e-9,
            "Marginal price should be 6.0"
        );
    }

    #[test]
    fn test_regulation_down_price_is_marginal() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::RegulationDown {
                    capacity_mw: 25.0,
                    bid_mwh: 3.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::RegulationDown {
                    capacity_mw: 25.0,
                    bid_mwh: 7.0,
                },
                availability_price: 1.0,
            },
        ];

        let result = clear_ancillary_market(&offers, 30.0, "regulation_down")
            .expect("Regulation down market should clear");

        assert!(result.requirement_met, "30 MW should be met");
        assert!(
            (result.clearing_price - 7.0).abs() < 1e-9,
            "Marginal price should be 7.0"
        );
        let first = result
            .cleared_offers
            .first()
            .expect("At least one cleared offer");
        assert_eq!(
            first.0, 0,
            "Cheapest unit (bid 3.0, unit_id 0) should clear first"
        );
    }

    #[test]
    fn test_voltage_support_clearing() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::Voltage {
                    reactive_mvar: 100.0,
                    bid_mvar: 2.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::Voltage {
                    reactive_mvar: 50.0,
                    bid_mvar: 1.5,
                },
                availability_price: 1.0,
            },
        ];

        let result = clear_ancillary_market(&offers, 80.0, "voltage")
            .expect("Voltage support market should clear");

        assert!(result.requirement_met, "80 MVAr should be met");
        assert!(
            (result.clearing_price - 2.0).abs() < 1e-9,
            "Marginal price should be 2.0"
        );
        let first = result
            .cleared_offers
            .first()
            .expect("At least one cleared offer");
        assert_eq!(
            first.0, 1,
            "Cheapest unit (bid 1.5, unit_id 1) should clear first"
        );
    }

    #[test]
    fn test_all_service_types_filter() {
        let offers = vec![
            AncillaryOffer {
                unit_id: 0,
                service: AncillaryService::SpinningReserve {
                    capacity_mw: 30.0,
                    bid_mwh: 5.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 1,
                service: AncillaryService::NonSpinningReserve {
                    capacity_mw: 20.0,
                    bid_mwh: 3.0,
                },
                availability_price: 1.0,
            },
            AncillaryOffer {
                unit_id: 2,
                service: AncillaryService::RegulationUp {
                    capacity_mw: 15.0,
                    bid_mwh: 7.0,
                },
                availability_price: 1.0,
            },
        ];

        let result_all =
            clear_ancillary_market(&offers, 40.0, "all").expect("All-service market should clear");
        assert!(
            result_all.requirement_met,
            "40 MW should be met from 65 MW total"
        );

        let result_spinning = clear_ancillary_market(&offers, 30.0, "spinning")
            .expect("Spinning-only market should clear");
        assert_eq!(
            result_spinning.cleared_offers.len(),
            1,
            "Only one spinning unit should clear"
        );
        assert_eq!(
            result_spinning.cleared_offers[0].0, 0,
            "The spinning unit (unit_id 0) should be cleared"
        );
    }

    #[test]
    fn test_zero_requirement_always_met() {
        let offers = vec![AncillaryOffer {
            unit_id: 0,
            service: AncillaryService::SpinningReserve {
                capacity_mw: 50.0,
                bid_mwh: 5.0,
            },
            availability_price: 1.0,
        }];

        let result = clear_ancillary_market(&offers, 0.0, "spinning")
            .expect("Zero requirement should not error");

        assert!(
            result.requirement_met,
            "Zero requirement should always be met"
        );
        assert!(
            (result.shortage_mw - 0.0).abs() < 1e-9,
            "Shortage should be zero"
        );
        assert!(
            result.cleared_offers.is_empty(),
            "No offers should be cleared for zero requirement"
        );
    }
}
