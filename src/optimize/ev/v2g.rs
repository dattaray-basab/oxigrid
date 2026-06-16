/// V2G aggregator — manages Vehicle-to-Grid dispatch across multiple EV fleets.
///
/// Handles frequency regulation, peak shaving events, energy arbitrage,
/// and spinning reserve services.  Computes fleet-level V2G capacity and
/// flexibility envelopes for aggregator bidding.
use crate::error::OxiGridError;
use crate::optimize::ev::charging::SmartCharger;
use crate::optimize::ev::fleet::{EvFleet, FleetAlgorithm, FleetCharger, FleetScheduleResult};
use serde::{Deserialize, Serialize};

// ──────────────────────────────────────────────────────────────────────────────
// Grid service definitions
// ──────────────────────────────────────────────────────────────────────────────

/// Grid ancillary service the V2G aggregator participates in.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GridService {
    /// Frequency regulation: provide symmetric capacity around baseline.
    FrequencyRegulation {
        /// Contracted capacity \[MW\].
        capacity_mw: f64,
        /// Payment for availability [$/MWh of capacity].
        price_mwh: f64,
    },
    /// Peak-shaving event: inject power above a threshold.
    PeakShaving {
        /// Grid load threshold above which V2G is activated \[MW\].
        threshold_mw: f64,
        /// Revenue per event-MWh [$/MWh].
        price_event: f64,
    },
    /// Energy arbitrage: buy cheap, sell expensive.
    EnergyArbitrage {
        /// Buy (charge) price per slot [$/kWh].
        price_buy: Vec<f64>,
        /// Sell (V2G) price per slot [$/kWh].
        price_sell: Vec<f64>,
    },
    /// Spinning reserve: standby capacity (not continuously cycling).
    SpinningReserve {
        /// Contracted reserve capacity \[MW\].
        capacity_mw: f64,
        /// Availability payment [$/MWh of reserve].
        price_mwh: f64,
    },
}

// ──────────────────────────────────────────────────────────────────────────────
// V2G result
// ──────────────────────────────────────────────────────────────────────────────

/// Outcome of a V2G aggregator optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gResult {
    /// Per-fleet schedules.
    pub fleet_schedules: Vec<FleetScheduleResult>,
    /// Total V2G power delivered to grid per slot \[MW\] (positive = export to grid).
    pub total_v2g_mw: Vec<f64>,
    /// Total revenue from the grid service [$].
    pub grid_service_revenue: f64,
    /// Total battery degradation cost across all fleets [$].
    pub total_battery_degradation: f64,
    /// Net benefit = revenue - degradation [$].
    pub net_benefit: f64,
    /// Number of vehicles that actively participate in V2G at any slot.
    pub participating_vehicles: usize,
}

// ──────────────────────────────────────────────────────────────────────────────
// V2G aggregator
// ──────────────────────────────────────────────────────────────────────────────

/// Aggregates V2G capacity from multiple EV fleets for grid service provision.
pub struct V2gAggregator {
    /// Managed EV fleets.
    pub fleets: Vec<EvFleet>,
    /// The grid service being targeted.
    pub grid_service: GridService,
}

impl V2gAggregator {
    /// Create a new `V2gAggregator`.
    pub fn new(fleets: Vec<EvFleet>, service: GridService) -> Self {
        Self {
            fleets,
            grid_service: service,
        }
    }

    /// Run V2G optimisation for all fleets.
    ///
    /// # Arguments
    /// - `dt_hours` — time slot duration \[h\]
    /// - `n_slots`  — number of time slots in the horizon
    ///
    /// The function selects an appropriate per-fleet algorithm based on the
    /// grid service type, then computes aggregate V2G and service revenue.
    pub fn optimize(&self, dt_hours: f64, n_slots: usize) -> Result<V2gResult, OxiGridError> {
        // Build price profiles per fleet based on service type
        let (buy_prices, sell_prices) = self.build_price_profiles(n_slots, dt_hours);

        let mut fleet_schedules: Vec<FleetScheduleResult> = Vec::with_capacity(self.fleets.len());
        let mut total_v2g_mw = vec![0.0_f64; n_slots];
        let mut total_degradation = 0.0_f64;
        let mut participating = 0usize;

        for fleet in &self.fleets {
            // Use V2G-optimized algorithm for all fleet members, using sell price profile
            let charger =
                SmartCharger::new(dt_hours, sell_prices.clone(), self.fleet_capacity_kw(fleet));
            let fc = FleetCharger::new(charger, FleetAlgorithm::V2gOptimized);
            let fleet_result = fc.schedule_fleet(fleet)?;

            // Accumulate V2G output (negative power = discharge to grid → positive V2G export)
            for (vi, sched) in fleet_result.schedules.iter().enumerate() {
                let _ = vi; // suppress unused warning
                let mut this_vehicle_v2g = false;
                for (k, &p) in sched.power_kw.iter().enumerate() {
                    if p < 0.0 {
                        let t_hours = sched.time_slots[k];
                        let slot = ((t_hours / dt_hours).floor() as usize).min(n_slots - 1);
                        total_v2g_mw[slot] += p.abs() / 1000.0; // kW → MW
                        this_vehicle_v2g = true;
                    }
                }
                if this_vehicle_v2g {
                    participating += 1;
                }
                total_degradation += sched.degradation_cost;
            }

            fleet_schedules.push(fleet_result);
        }

        // Compute grid service revenue
        let grid_revenue =
            self.compute_service_revenue(&total_v2g_mw, &buy_prices, &sell_prices, dt_hours);

        Ok(V2gResult {
            fleet_schedules,
            total_v2g_mw,
            grid_service_revenue: grid_revenue,
            total_battery_degradation: total_degradation,
            net_benefit: grid_revenue - total_degradation,
            participating_vehicles: participating,
        })
    }

    /// Compute available V2G capacity \[kW\] at slot `t` across all fleets.
    ///
    /// Capacity is the sum of `max_discharge_kw` for all vehicles that are
    /// plugged in during slot `t` (i.e., arrival ≤ t*dt < departure).
    pub fn available_capacity(&self, t: usize) -> f64 {
        // Use a representative dt of 0.25h to convert slot to time
        let dt = 0.25_f64;
        let t_hours = t as f64 * dt;
        self.fleets
            .iter()
            .flat_map(|f| f.sessions.iter())
            .filter(|s| s.arrival_time <= t_hours && t_hours < s.departure_time)
            .map(|s| s.max_discharge_kw)
            .sum()
    }

    /// SoC-weighted V2G availability \[kW\] at slot `t`.
    ///
    /// Each vehicle's contribution is weighted by `(soc_arrival - soc_target) * soc_weight`
    /// to reflect how much energy is available above the minimum required SoC.
    ///
    /// # Arguments
    /// - `t`          — time slot index (with dt=0.25h)
    /// - `soc_weight` — scale factor applied to SoC headroom (0–1)
    pub fn weighted_availability(&self, t: usize, soc_weight: f64) -> f64 {
        let dt = 0.25_f64;
        let t_hours = t as f64 * dt;
        self.fleets
            .iter()
            .flat_map(|f| f.sessions.iter())
            .filter(|s| s.arrival_time <= t_hours && t_hours < s.departure_time)
            .map(|s| {
                let headroom = (s.soc_arrival - s.soc_target).max(0.0);
                let weight = (headroom * soc_weight).clamp(0.0, 1.0);
                s.max_discharge_kw * weight
            })
            .sum()
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ──────────────────────────────────────────────────────────────────────────

    /// Build buy/sell price profiles from the grid service specification.
    fn build_price_profiles(&self, n_slots: usize, _dt_hours: f64) -> (Vec<f64>, Vec<f64>) {
        match &self.grid_service {
            GridService::FrequencyRegulation {
                capacity_mw: _,
                price_mwh,
            } => {
                let price_kwh = price_mwh / 1000.0;
                let buy = vec![price_kwh * 0.5; n_slots]; // charge at half reg price
                let sell = vec![price_kwh; n_slots];
                (buy, sell)
            }
            GridService::PeakShaving {
                threshold_mw: _,
                price_event,
            } => {
                let price_kwh = price_event / 1000.0;
                let buy = vec![0.05; n_slots]; // cheap off-peak charging
                let sell = vec![price_kwh; n_slots];
                (buy, sell)
            }
            GridService::EnergyArbitrage {
                price_buy,
                price_sell,
            } => {
                let buy = if price_buy.len() >= n_slots {
                    price_buy[..n_slots].to_vec()
                } else {
                    let mut v = price_buy.clone();
                    v.resize(n_slots, *price_buy.last().unwrap_or(&0.05));
                    v
                };
                let sell = if price_sell.len() >= n_slots {
                    price_sell[..n_slots].to_vec()
                } else {
                    let mut v = price_sell.clone();
                    v.resize(n_slots, *price_sell.last().unwrap_or(&0.15));
                    v
                };
                (buy, sell)
            }
            GridService::SpinningReserve {
                capacity_mw: _,
                price_mwh,
            } => {
                let price_kwh = price_mwh / 1000.0;
                // Reserve: vehicles are "on standby", minimal cycling
                let buy = vec![price_kwh * 0.3; n_slots];
                let sell = vec![price_kwh; n_slots];
                (buy, sell)
            }
        }
    }

    /// Compute aggregate grid service revenue from V2G power delivered.
    fn compute_service_revenue(
        &self,
        v2g_mw: &[f64],
        _buy_prices: &[f64],
        sell_prices: &[f64],
        dt_hours: f64,
    ) -> f64 {
        match &self.grid_service {
            GridService::FrequencyRegulation { price_mwh, .. }
            | GridService::SpinningReserve { price_mwh, .. } => {
                // Revenue = capacity * price * dt for each slot
                v2g_mw.iter().map(|&p| p * price_mwh * dt_hours).sum()
            }
            GridService::PeakShaving { price_event, .. } => v2g_mw
                .iter()
                .map(|&p| p * price_event / 1000.0 * dt_hours)
                .sum(),
            GridService::EnergyArbitrage { .. } => {
                // Revenue from V2G sales using sell price profile
                v2g_mw
                    .iter()
                    .zip(sell_prices.iter())
                    .map(|(&p_mw, &price)| p_mw * 1000.0 * price * dt_hours) // MW→kW, $/kWh
                    .sum()
            }
        }
    }

    /// Maximum charger power limit for a fleet \[kW\].
    fn fleet_capacity_kw(&self, fleet: &EvFleet) -> f64 {
        fleet
            .sessions
            .iter()
            .map(|s| s.max_charge_kw.max(s.max_discharge_kw))
            .fold(0.0_f64, f64::max)
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Flexibility envelope
// ──────────────────────────────────────────────────────────────────────────────

/// Flexibility envelope for aggregator bidding.
///
/// Characterises the aggregate charging flexibility of a fleet over time:
/// how much power the fleet can shift up (charge more) or down (discharge)
/// relative to the uncontrolled baseline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlexibilityEnvelope {
    /// Start time of each slot [hours from day start].
    pub time_slots: Vec<f64>,
    /// Maximum aggregate charge power at each slot \[kW\] (upper bound).
    pub max_charge_kw: Vec<f64>,
    /// Maximum aggregate V2G discharge power at each slot \[kW\] (≥ 0, lower bound).
    pub max_discharge_kw: Vec<f64>,
    /// Uncontrolled (dumb) charging baseline at each slot \[kW\].
    pub baseline_kw: Vec<f64>,
}

/// Compute the flexibility envelope for a fleet over `n_slots` time steps.
///
/// For each slot, sums over all vehicles that are plugged in:
/// - `max_charge_kw\[t\]`    = Σ min(max_charge_kw, soc-headroom-limited rate)
/// - `max_discharge_kw\[t\]` = Σ min(max_discharge_kw, soc-above-target rate)
/// - `baseline_kw\[t\]`      = uncontrolled charging power
pub fn compute_flexibility_envelope(
    fleet: &EvFleet,
    dt_hours: f64,
    n_slots: usize,
) -> FlexibilityEnvelope {
    let mut max_charge = vec![0.0_f64; n_slots];
    let mut max_discharge = vec![0.0_f64; n_slots];
    let mut baseline = vec![0.0_f64; n_slots];

    for session in &fleet.sessions {
        // Track SoC for baseline simulation
        let mut soc_baseline = session.soc_arrival;

        for t in 0..n_slots {
            let t_hours = t as f64 * dt_hours;
            if t_hours < session.arrival_time || t_hours >= session.departure_time {
                continue;
            }

            // Upper bound: max charge rate limited by SoC headroom to 1.0
            let soc_headroom_c = (1.0 - soc_baseline).max(0.0);
            let p_chg_max = if soc_headroom_c > 1e-9 {
                session
                    .max_charge_kw
                    .min(soc_headroom_c * session.battery_kwh / (session.eta_charge * dt_hours))
            } else {
                0.0
            };
            max_charge[t] += p_chg_max;

            // Lower bound: max discharge rate limited to energy above soc_target
            let soc_above_target = (soc_baseline - session.soc_target).max(0.0);
            let p_dis_max = if session.max_discharge_kw > 1e-9 {
                session
                    .max_discharge_kw
                    .min(soc_above_target * session.battery_kwh * session.eta_discharge / dt_hours)
            } else {
                0.0
            };
            max_discharge[t] += p_dis_max;

            // Baseline: uncontrolled charging until target
            let p_base = if soc_baseline < session.soc_target - 1e-9 {
                let soc_need = (session.soc_target - soc_baseline).max(0.0);
                let p_limit = soc_need * session.battery_kwh / (session.eta_charge * dt_hours);
                session.max_charge_kw.min(p_limit)
            } else {
                0.0
            };
            baseline[t] += p_base;

            // Advance SoC for baseline simulation
            let delta = session.eta_charge * p_base * dt_hours / session.battery_kwh;
            soc_baseline = (soc_baseline + delta).clamp(0.0, 1.0);
        }
    }

    let time_slots: Vec<f64> = (0..n_slots).map(|t| t as f64 * dt_hours).collect();

    FlexibilityEnvelope {
        time_slots,
        max_charge_kw: max_charge,
        max_discharge_kw: max_discharge,
        baseline_kw: baseline,
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Unit tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::ev::charging::EvSession;

    fn make_session(id: usize, arrival: f64, departure: f64) -> EvSession {
        EvSession {
            vehicle_id: id,
            arrival_time: arrival,
            departure_time: departure,
            soc_arrival: 0.5,
            soc_target: 0.8,
            battery_kwh: 60.0,
            max_charge_kw: 11.0,
            max_discharge_kw: 7.4,
            eta_charge: 0.92,
            eta_discharge: 0.92,
            degradation_cost: 0.05,
        }
    }

    fn make_fleet(id: usize) -> EvFleet {
        EvFleet {
            fleet_id: id,
            bus: id,
            sessions: vec![
                make_session(0, 17.0, 31.0),
                make_session(1, 18.0, 31.0),
                make_session(2, 19.0, 31.0),
            ],
            transformer_limit_kw: 100.0,
            charger_slots: 5,
        }
    }

    #[test]
    fn test_v2g_aggregator_capacity() {
        let fleet1 = make_fleet(0);
        let fleet2 = make_fleet(1);
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![fleet1, fleet2], service);

        // At slot 72 (= 18.0h with dt=0.25h), all sessions with arrival<=18h should be counted
        let cap = agg.available_capacity(72);
        assert!(
            cap > 0.0,
            "Aggregator capacity should be > 0: {:.2} kW",
            cap
        );
    }

    #[test]
    fn test_v2g_aggregator_optimize() {
        let fleet1 = make_fleet(0);
        let service = GridService::EnergyArbitrage {
            price_buy: vec![0.05; 96],
            price_sell: vec![0.25; 96],
        };
        let agg = V2gAggregator::new(vec![fleet1], service);
        let result = agg.optimize(0.25, 96).expect("V2G optimize");
        assert_eq!(result.fleet_schedules.len(), 1);
    }

    #[test]
    fn test_flexibility_envelope_bounds() {
        let fleet = make_fleet(0);
        let envelope = compute_flexibility_envelope(&fleet, 0.25, 96);

        for t in 0..96 {
            let charge = envelope.max_charge_kw[t];
            let discharge = envelope.max_discharge_kw[t];
            let base = envelope.baseline_kw[t];

            assert!(
                charge >= -1e-9,
                "Slot {}: max_charge negative: {:.4}",
                t,
                charge
            );
            assert!(
                discharge >= -1e-9,
                "Slot {}: max_discharge negative: {:.4}",
                t,
                discharge
            );
            assert!(
                base <= charge + 1e-6,
                "Slot {}: baseline {:.4} > max_charge {:.4}",
                t,
                base,
                charge
            );
        }
    }

    #[test]
    fn test_weighted_availability_decreases_with_low_weight() {
        let fleet = make_fleet(0);
        let service = GridService::SpinningReserve {
            capacity_mw: 0.05,
            price_mwh: 60.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        let full = agg.available_capacity(72);
        let weighted = agg.weighted_availability(72, 1.0);
        // Weighted availability ≤ full (since soc headroom ≤ 1)
        assert!(
            weighted <= full + 1e-6,
            "weighted {:.2} > full {:.2}",
            weighted,
            full
        );
    }

    #[test]
    fn test_peak_shaving_grid_service() {
        let fleet = make_fleet(0);
        let service = GridService::PeakShaving {
            threshold_mw: 5.0,
            price_event: 200.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        let result = agg.optimize(0.25, 96).expect("peak shaving V2G");
        // net_benefit can be positive or negative depending on degradation, just check no panic
        let _ = result.net_benefit;
    }

    #[test]
    fn test_available_capacity_at_slot_zero() {
        let fleet = make_fleet(0);
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        // slot 0 → time 0.0h, all sessions arrive at 17.0h or later — none are plugged in
        let cap = agg.available_capacity(0);
        assert_eq!(cap, 0.0, "no vehicles plugged in at slot 0 (0.0h)");
    }

    #[test]
    fn test_available_capacity_past_departure() {
        let fleet = make_fleet(0);
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        // all sessions depart at 31.0h; slot 200 → time 50.0h → none plugged in
        let cap = agg.available_capacity(200);
        assert_eq!(
            cap, 0.0,
            "no vehicles plugged in at slot 200 (50.0h), past departure"
        );
    }

    #[test]
    fn test_available_capacity_middle_slot() {
        let fleet = make_fleet(0);
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        // slot 76 → time 19.0h; all 3 sessions (arrivals 17, 18, 19h) are plugged in
        // each has max_discharge_kw = 7.4, so total = 3 × 7.4 = 22.2 kW
        let cap = agg.available_capacity(76);
        let expected = 3.0 * 7.4_f64;
        assert!(
            (cap - expected).abs() < 1e-9,
            "expected {:.1} kW at slot 76 (all 3 vehicles), got {:.6}",
            expected,
            cap
        );
    }

    #[test]
    fn test_weighted_availability_zero_weight() {
        let fleet = make_fleet(0);
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        // At slot 76, 3 vehicles are plugged in, but soc_weight=0.0 zeroes every contribution
        let wa = agg.weighted_availability(76, 0.0);
        assert_eq!(
            wa, 0.0,
            "weighted_availability with soc_weight=0.0 must return 0.0"
        );
    }

    #[test]
    fn test_weighted_availability_soc_below_target() {
        let fleet = make_fleet(0);
        let service = GridService::SpinningReserve {
            capacity_mw: 0.05,
            price_mwh: 60.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        // Sessions: soc_arrival=0.5, soc_target=0.8 → headroom = 0.5 - 0.8 = -0.3 → clamped to 0.0
        // Therefore weighted_availability must return 0.0 regardless of soc_weight
        let wa = agg.weighted_availability(76, 1.0);
        assert_eq!(
            wa, 0.0,
            "weighted_availability must be 0.0 when soc_arrival < soc_target (no SoC headroom)"
        );
    }

    #[test]
    fn test_optimize_minimal_one_slot() {
        // Sessions run 17h–31h with dt=0.25h, so need at least 125 slots to cover departure.
        // Use 128 slots (32h total) to ensure all sessions are within range.
        let fleet = make_fleet(0);
        let service = GridService::SpinningReserve {
            capacity_mw: 0.05,
            price_mwh: 60.0,
        };
        let agg = V2gAggregator::new(vec![fleet], service);
        let n_slots = 128usize;
        let result = agg
            .optimize(0.25, n_slots)
            .expect("optimize with 128 slots should succeed");
        assert_eq!(
            result.total_v2g_mw.len(),
            n_slots,
            "total_v2g_mw must have exactly {} entries",
            n_slots
        );
        assert!(
            result.grid_service_revenue >= 0.0,
            "grid_service_revenue must be non-negative, got {}",
            result.grid_service_revenue
        );
        assert!(
            result.total_battery_degradation >= 0.0,
            "total_battery_degradation must be non-negative, got {}",
            result.total_battery_degradation
        );
    }

    #[test]
    fn test_optimize_empty_fleet_list() {
        let service = GridService::FrequencyRegulation {
            capacity_mw: 0.1,
            price_mwh: 80.0,
        };
        let agg = V2gAggregator::new(vec![], service);
        let result = agg
            .optimize(0.25, 96)
            .expect("optimize with empty fleet list should succeed");
        assert!(
            result.fleet_schedules.is_empty(),
            "fleet_schedules must be empty when no fleets provided"
        );
        assert_eq!(
            result.total_v2g_mw.len(),
            96,
            "total_v2g_mw must have 96 entries matching n_slots"
        );
        assert_eq!(
            result.participating_vehicles, 0,
            "participating_vehicles must be 0 for empty fleet"
        );
        assert!(
            result.total_v2g_mw.iter().all(|&v| v == 0.0),
            "expected all zeros in empty fleet"
        );
        assert_eq!(
            result.grid_service_revenue, 0.0,
            "grid_service_revenue must be 0.0 for empty fleet"
        );
    }
}
