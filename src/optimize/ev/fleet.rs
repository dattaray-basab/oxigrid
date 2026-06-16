/// EV fleet aggregation and coordinated charging at a single location.
///
/// Supports valley filling (congestion-pricing coordination), peak shaving,
/// TOU optimised, V2G-enabled, and uncontrolled fleet strategies.
use crate::error::OxiGridError;
use crate::optimize::ev::charging::{ChargingSchedule, EvSession, SmartCharger};
use serde::{Deserialize, Serialize};

/// An EV fleet at one grid location (parking lot, fleet depot, etc.).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvFleet {
    /// Unique fleet identifier.
    pub fleet_id: usize,
    /// Network bus this fleet is connected to.
    pub bus: usize,
    /// Individual EV sessions currently plugged in.
    pub sessions: Vec<EvSession>,
    /// Maximum allowable aggregate power at the building transformer \[kW\].
    pub transformer_limit_kw: f64,
    /// Number of simultaneous charging spots (charger slots).
    pub charger_slots: usize,
}

/// Aggregated result of scheduling an entire EV fleet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetScheduleResult {
    /// Per-vehicle optimal schedules.
    pub schedules: Vec<ChargingSchedule>,
    /// Total fleet power draw per time slot \[kW\].
    pub aggregate_power: Vec<f64>,
    /// Peak fleet power \[kW\].
    pub peak_power_kw: f64,
    /// Total energy delivered to all vehicles \[kWh\].
    pub total_energy_kwh: f64,
    /// Total electricity cost [$].
    pub total_cost: f64,
    /// Total V2G revenue [$].
    pub v2g_revenue: f64,
    /// Peak reduction achieved vs. uncontrolled baseline [%].
    /// Positive = controlled is better (lower peak).
    pub peak_reduction_vs_uncontrolled: f64,
}

/// Fleet charging algorithm selection.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum FleetAlgorithm {
    /// Each vehicle charges immediately at max rate on arrival.
    Uncontrolled,
    /// Each vehicle independently minimises its own TOU cost.
    TouOptimized,
    /// Each vehicle uses V2G bi-directional DP optimisation.
    V2gOptimized,
    /// Iterative congestion pricing to flatten the aggregate load profile.
    ValleyFilling,
    /// Defer low-priority charging slots to stay below a transformer limit.
    PeakShaving { limit_kw: f64 },
}

/// Coordinates fleet charging using a chosen algorithm.
pub struct FleetCharger {
    /// The per-vehicle smart charger (holds prices and dt).
    pub charger: SmartCharger,
    /// Algorithm used to coordinate the fleet.
    pub algorithm: FleetAlgorithm,
}

impl FleetCharger {
    /// Construct a new `FleetCharger`.
    pub fn new(charger: SmartCharger, algorithm: FleetAlgorithm) -> Self {
        Self { charger, algorithm }
    }

    /// Schedule the entire fleet using the configured algorithm.
    pub fn schedule_fleet(&self, fleet: &EvFleet) -> Result<FleetScheduleResult, OxiGridError> {
        // Validate charger slot capacity
        let n_sessions = fleet.sessions.len();
        if n_sessions == 0 {
            return Ok(FleetScheduleResult {
                schedules: vec![],
                aggregate_power: vec![0.0; self.charger.n_slots()],
                peak_power_kw: 0.0,
                total_energy_kwh: 0.0,
                total_cost: 0.0,
                v2g_revenue: 0.0,
                peak_reduction_vs_uncontrolled: 0.0,
            });
        }

        let uncontrolled_peak = self.compute_uncontrolled_peak(fleet)?;

        let mut result = match self.algorithm {
            FleetAlgorithm::Uncontrolled => self.schedule_all(fleet, |c, s| c.uncontrolled(s))?,
            FleetAlgorithm::TouOptimized => self.schedule_all(fleet, |c, s| c.tou_optimized(s))?,
            FleetAlgorithm::V2gOptimized => self.schedule_all(fleet, |c, s| c.v2g_optimized(s))?,
            FleetAlgorithm::ValleyFilling => self.valley_filling(fleet)?,
            FleetAlgorithm::PeakShaving { limit_kw } => self.peak_shaving(fleet, limit_kw)?,
        };

        let peak = result
            .aggregate_power
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        let reduction = if uncontrolled_peak > 1e-9 {
            (uncontrolled_peak - peak) / uncontrolled_peak * 100.0
        } else {
            0.0
        };
        result.peak_reduction_vs_uncontrolled = reduction;
        Ok(result)
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ──────────────────────────────────────────────────────────────────────────

    /// Apply a per-vehicle scheduling closure to all sessions and aggregate.
    fn schedule_all<F>(
        &self,
        fleet: &EvFleet,
        mut f: F,
    ) -> Result<FleetScheduleResult, OxiGridError>
    where
        F: FnMut(&SmartCharger, &EvSession) -> Result<ChargingSchedule, OxiGridError>,
    {
        let n_slots = self.charger.n_slots();
        let mut aggregate = vec![0.0_f64; n_slots];
        let mut schedules = Vec::with_capacity(fleet.sessions.len());
        let mut total_energy = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut v2g_rev = 0.0_f64;

        for session in &fleet.sessions {
            let sched = f(&self.charger, session)?;
            // Add power into aggregate timeline
            let dt = self.charger.dt_hours;
            for (k, &p) in sched.power_kw.iter().enumerate() {
                let t_hours = sched.time_slots[k];
                let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                aggregate[slot] += p;
            }
            // Accumulate totals
            total_energy += sched
                .power_kw
                .iter()
                .filter(|&&p| p > 0.0)
                .map(|&p| p * dt)
                .sum::<f64>();
            total_cost += sched.energy_cost;
            v2g_rev += sched.v2g_revenue;
            schedules.push(sched);
        }

        let peak = aggregate.iter().cloned().fold(0.0_f64, f64::max);

        Ok(FleetScheduleResult {
            schedules,
            aggregate_power: aggregate,
            peak_power_kw: peak,
            total_energy_kwh: total_energy,
            total_cost,
            v2g_revenue: v2g_rev,
            peak_reduction_vs_uncontrolled: 0.0, // filled in by caller
        })
    }

    /// Compute uncontrolled (dumb) peak for baseline comparison.
    fn compute_uncontrolled_peak(&self, fleet: &EvFleet) -> Result<f64, OxiGridError> {
        let baseline = self.schedule_all(fleet, |c, s| c.uncontrolled(s))?;
        Ok(baseline.peak_power_kw)
    }

    /// **Valley filling** via iterative congestion pricing.
    ///
    /// Algorithm:
    /// 1. Start with uncontrolled schedule to initialise aggregate load.
    /// 2. Sort vehicles by flexibility (ascending = least flexible first, so
    ///    inflexible vehicles are scheduled first and anchor the profile).
    /// 3. For each vehicle, compute an effective price = grid_price + k_cong * fleet_load\[t\]
    ///    and re-schedule that vehicle alone.
    /// 4. Repeat until aggregate load change is below tolerance or max iterations.
    fn valley_filling(&self, fleet: &EvFleet) -> Result<FleetScheduleResult, OxiGridError> {
        const MAX_ITER: usize = 15;
        const TOL: f64 = 0.5; // kW convergence tolerance
                              // Congestion coefficient: high enough to spread load but not over-penalise.
                              // At 110 kW peak with 10 EVs the effective cost premium becomes ~0.22 $/kWh
                              // which dominates the price differential and forces spreading.
        const K_CONG: f64 = 0.002;

        let n_slots = self.charger.n_slots();
        let dt = self.charger.dt_hours;
        let n_sessions = fleet.sessions.len();

        // Initialise with uncontrolled schedules
        let mut schedules: Vec<ChargingSchedule> = fleet
            .sessions
            .iter()
            .map(|s| self.charger.uncontrolled(s))
            .collect::<Result<Vec<_>, _>>()?;

        // Sort session indices by flexibility (ascending: least flexible first).
        // Least-flexible vehicles anchor the profile; more-flexible ones adapt around them.
        let mut order: Vec<usize> = (0..n_sessions).collect();
        order.sort_by(|&a, &b| {
            let fa = fleet.sessions[a].window_hours() - fleet.sessions[a].min_charge_hours();
            let fb = fleet.sessions[b].window_hours() - fleet.sessions[b].min_charge_hours();
            fa.partial_cmp(&fb).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Maintain a live aggregate that is updated after each individual rescheduling.
        // This ensures the congestion signal seen by each vehicle reflects the current
        // state of all other vehicles' schedules (Gauss-Seidel style update).
        let mut aggregate = vec![0.0_f64; n_slots];
        for sched in &schedules {
            for (k, &p) in sched.power_kw.iter().enumerate() {
                let t_hours = sched.time_slots[k];
                let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                aggregate[slot] += p;
            }
        }

        for _iter in 0..MAX_ITER {
            let mut max_change = 0.0_f64;

            for &vi in &order {
                let session = &fleet.sessions[vi];

                // Step 1: remove this vehicle's current contribution from the live aggregate.
                for (k, &p) in schedules[vi].power_kw.iter().enumerate() {
                    let t_hours = schedules[vi].time_slots[k];
                    let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                    aggregate[slot] -= p;
                    aggregate[slot] = aggregate[slot].max(0.0); // numerical guard
                }

                // Step 2: build effective price = grid_price + K_CONG * aggregate_without_vi[t].
                let eff_price: Vec<f64> = self
                    .charger
                    .price_profile
                    .iter()
                    .enumerate()
                    .map(|(i, &gp)| gp + K_CONG * aggregate[i].max(0.0))
                    .collect();

                // Step 3: reschedule this vehicle using effective price.
                let new_charger = SmartCharger::new(dt, eff_price, self.charger.grid_capacity_kw);
                let new_sched = new_charger.tou_optimized(session)?;

                // Step 4: add new contribution back into live aggregate.
                for (k, &p) in new_sched.power_kw.iter().enumerate() {
                    let t_hours = new_sched.time_slots[k];
                    let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                    aggregate[slot] += p;
                }

                // Track convergence: max absolute power change for this vehicle.
                let n_common = new_sched.power_kw.len().min(schedules[vi].power_kw.len());
                for k in 0..n_common {
                    let change = (new_sched.power_kw[k] - schedules[vi].power_kw[k]).abs();
                    if change > max_change {
                        max_change = change;
                    }
                }

                schedules[vi] = new_sched;
            }

            if max_change < TOL {
                break;
            }
        }

        // Aggregate final result
        let mut aggregate = vec![0.0_f64; n_slots];
        let mut total_energy = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut v2g_rev = 0.0_f64;

        for sched in &schedules {
            for (k, &p) in sched.power_kw.iter().enumerate() {
                let t_hours = sched.time_slots[k];
                let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                aggregate[slot] += p;
            }
            total_energy += sched
                .power_kw
                .iter()
                .filter(|&&p| p > 0.0)
                .map(|&p| p * dt)
                .sum::<f64>();
            total_cost += sched.energy_cost;
            v2g_rev += sched.v2g_revenue;
        }

        let peak = aggregate.iter().cloned().fold(0.0_f64, f64::max);

        Ok(FleetScheduleResult {
            schedules,
            aggregate_power: aggregate,
            peak_power_kw: peak,
            total_energy_kwh: total_energy,
            total_cost,
            v2g_revenue: v2g_rev,
            peak_reduction_vs_uncontrolled: 0.0,
        })
    }

    /// **Peak shaving** — iteratively defer charging slots that push aggregate above `limit_kw`.
    ///
    /// Algorithm:
    /// 1. Start with TOU-optimised schedules.
    /// 2. Find slots where aggregate > limit_kw.
    /// 3. For each over-limit slot, identify the vehicle contributing the most,
    ///    defer that slot to the next cheapest feasible slot.
    /// 4. Repeat until aggregate is within limit or no deferral is possible.
    fn peak_shaving(
        &self,
        fleet: &EvFleet,
        limit_kw: f64,
    ) -> Result<FleetScheduleResult, OxiGridError> {
        const MAX_PASS: usize = 200;

        let n_slots = self.charger.n_slots();
        let dt = self.charger.dt_hours;

        // Start from TOU-optimised schedules
        let mut schedules: Vec<ChargingSchedule> = fleet
            .sessions
            .iter()
            .map(|s| self.charger.tou_optimized(s))
            .collect::<Result<Vec<_>, _>>()?;

        // Build mutable aggregate
        let mut aggregate = vec![0.0_f64; n_slots];
        for sched in &schedules {
            for (k, &p) in sched.power_kw.iter().enumerate() {
                let t_hours = sched.time_slots[k];
                let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                aggregate[slot] += p;
            }
        }

        for _pass in 0..MAX_PASS {
            // Find the slot with maximum excess power
            let (peak_slot, peak_val) = aggregate
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, &v)| (i, v))
                .unwrap_or((0, 0.0));

            if peak_val <= limit_kw + 1e-6 {
                break; // within limit
            }

            // Find the vehicle with the largest charging power at peak_slot
            let mut best_vi = None;
            let mut best_p = 0.0_f64;
            let mut best_k = 0usize;

            for (vi, sched) in schedules.iter().enumerate() {
                for (k, &p) in sched.power_kw.iter().enumerate() {
                    if p <= 0.0 {
                        continue;
                    }
                    let t_hours = sched.time_slots[k];
                    let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                    if slot == peak_slot && p > best_p {
                        best_p = p;
                        best_vi = Some(vi);
                        best_k = k;
                    }
                }
            }

            let vi = match best_vi {
                Some(v) => v,
                None => break, // no vehicle at this slot
            };

            // Find a cheaper off-peak slot within the vehicle's window to transfer to
            let session = &fleet.sessions[vi];
            let session_slots = self.charger.session_slots(session);

            // Find the cheapest slot in window that is not over limit
            let target_slot_opt = session_slots
                .iter()
                .filter(|&&s| {
                    s != peak_slot
                        && aggregate[s] + best_p <= limit_kw + 1e-6
                        // Only slots not yet fully loaded
                        && schedules[vi].power_kw.get(
                            session_slots.iter().position(|&ss| ss == s).unwrap_or(usize::MAX)
                        ).copied().unwrap_or(0.0)
                            < session.max_charge_kw - 1e-6
                })
                .min_by(|&&a, &&b| {
                    let pa = self
                        .charger
                        .price_profile
                        .get(a)
                        .copied()
                        .unwrap_or(f64::INFINITY);
                    let pb = self
                        .charger
                        .price_profile
                        .get(b)
                        .copied()
                        .unwrap_or(f64::INFINITY);
                    pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
                })
                .copied();

            let target_slot = match target_slot_opt {
                Some(s) => s,
                None => break, // can't move power
            };

            let target_k = match session_slots.iter().position(|&s| s == target_slot) {
                Some(p) => p,
                None => break,
            };

            // Transfer power: reduce at peak_slot, add at target_slot
            let excess = (peak_val - limit_kw).min(best_p);
            let move_p = excess.min(best_p).min(
                session.max_charge_kw
                    - schedules[vi].power_kw.get(target_k).copied().unwrap_or(0.0),
            );

            if move_p <= 1e-9 {
                break;
            }

            // Ensure bounds on both power vectors
            if best_k < schedules[vi].power_kw.len() {
                schedules[vi].power_kw[best_k] -= move_p;
                schedules[vi].power_kw[best_k] = schedules[vi].power_kw[best_k].max(0.0);
            }
            if target_k < schedules[vi].power_kw.len() {
                schedules[vi].power_kw[target_k] += move_p;
            }

            // Update aggregate
            aggregate[peak_slot] -= move_p;
            aggregate[target_slot] += move_p;

            // Recompute SoC trajectory for this vehicle (best-effort, ignore error)
            let soc_traj = self.charger.simulate_soc(
                &schedules[vi].power_kw,
                session.soc_arrival,
                session.battery_kwh,
                session.eta_charge,
                session.eta_discharge,
            );
            schedules[vi].soc_trajectory = soc_traj;
        }

        // Re-aggregate for final result
        let mut aggregate = vec![0.0_f64; n_slots];
        let mut total_energy = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut v2g_rev = 0.0_f64;

        for (vi, sched) in schedules.iter_mut().enumerate() {
            let session = &fleet.sessions[vi];
            // Recompute energy cost/revenue
            let session_slots = self.charger.session_slots(session);
            let (ec, vr, dc) = self.charger.compute_metrics(
                &sched.power_kw,
                &session_slots,
                session.degradation_cost,
                dt,
            );
            sched.energy_cost = ec;
            sched.v2g_revenue = vr;
            sched.degradation_cost = dc;
            sched.net_cost = ec - vr + dc;

            for (k, &p) in sched.power_kw.iter().enumerate() {
                let t_hours = sched.time_slots[k];
                let slot = ((t_hours / dt).floor() as usize).min(n_slots - 1);
                aggregate[slot] += p;
            }
            total_energy += sched
                .power_kw
                .iter()
                .filter(|&&p| p > 0.0)
                .map(|&p| p * dt)
                .sum::<f64>();
            total_cost += sched.energy_cost;
            v2g_rev += sched.v2g_revenue;
        }

        let peak = aggregate.iter().cloned().fold(0.0_f64, f64::max);

        Ok(FleetScheduleResult {
            schedules,
            aggregate_power: aggregate,
            peak_power_kw: peak,
            total_energy_kwh: total_energy,
            total_cost,
            v2g_revenue: v2g_rev,
            peak_reduction_vs_uncontrolled: 0.0,
        })
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Unit tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::ev::charging::SmartCharger;

    fn make_charger() -> SmartCharger {
        let prices: Vec<f64> = (0..96)
            .map(|i| if !(32..76).contains(&i) { 0.05 } else { 0.25 })
            .collect();
        SmartCharger::new(0.25, prices, 22.0)
    }

    fn make_session(id: usize, arrival: f64, departure: f64, soc_arr: f64) -> EvSession {
        EvSession {
            vehicle_id: id,
            arrival_time: arrival,
            departure_time: departure,
            soc_arrival: soc_arr,
            soc_target: 0.8,
            battery_kwh: 60.0,
            max_charge_kw: 11.0,
            max_discharge_kw: 7.4,
            eta_charge: 0.92,
            eta_discharge: 0.92,
            degradation_cost: 0.05,
        }
    }

    fn make_fleet(n: usize) -> EvFleet {
        // Stagger arrivals 17:00–20:00, all depart at 07:00 next day
        let sessions = (0..n)
            .map(|i| {
                let arrival = 17.0 + i as f64 * 0.3;
                let soc = 0.2 + (i as f64 * 0.07) % 0.5;
                make_session(i, arrival, 31.0, soc)
            })
            .collect();

        EvFleet {
            fleet_id: 0,
            bus: 1,
            sessions,
            transformer_limit_kw: 80.0,
            charger_slots: n,
        }
    }

    #[test]
    fn test_fleet_valley_filling_reduces_peak() {
        let fleet = make_fleet(10);
        let charger = make_charger();
        let fleet_charger = FleetCharger::new(charger, FleetAlgorithm::ValleyFilling);
        let result = fleet_charger
            .schedule_fleet(&fleet)
            .expect("valley filling");
        assert!(
            result.peak_reduction_vs_uncontrolled > 0.0,
            "Valley filling should reduce peak: {:.2}%",
            result.peak_reduction_vs_uncontrolled
        );
    }

    #[test]
    fn test_fleet_peak_shaving_respects_limit() {
        let limit_kw = 60.0_f64;
        let fleet = make_fleet(10);
        let charger = make_charger();
        let fleet_charger = FleetCharger::new(charger, FleetAlgorithm::PeakShaving { limit_kw });
        let result = fleet_charger.schedule_fleet(&fleet).expect("peak shaving");
        for (t, &p) in result.aggregate_power.iter().enumerate() {
            assert!(
                p <= limit_kw + 1e-3,
                "Slot {}: aggregate {:.2} kW exceeds limit {:.2} kW",
                t,
                p,
                limit_kw
            );
        }
    }

    #[test]
    fn test_fleet_uncontrolled_no_error() {
        let fleet = make_fleet(5);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::Uncontrolled);
        let result = fc.schedule_fleet(&fleet).expect("uncontrolled fleet");
        assert_eq!(result.schedules.len(), 5);
        assert!(result.total_energy_kwh > 0.0);
    }

    #[test]
    fn test_fleet_empty() {
        let fleet = EvFleet {
            fleet_id: 0,
            bus: 0,
            sessions: vec![],
            transformer_limit_kw: 100.0,
            charger_slots: 10,
        };
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::TouOptimized);
        let result = fc.schedule_fleet(&fleet).expect("empty fleet");
        assert_eq!(result.schedules.len(), 0);
        assert_eq!(result.peak_power_kw, 0.0);
    }

    #[test]
    fn test_tou_optimized_basic() {
        let fleet = make_fleet(5);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::TouOptimized);
        let result = fc.schedule_fleet(&fleet).expect("TOU optimized fleet");
        assert_eq!(result.schedules.len(), 5, "should have one schedule per EV");
        assert!(
            result.total_energy_kwh > 0.0,
            "total energy must be positive"
        );
    }

    #[test]
    fn test_v2g_optimized_basic() {
        let fleet = make_fleet(3);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::V2gOptimized);
        let result = fc.schedule_fleet(&fleet).expect("V2G optimized fleet");
        assert_eq!(result.schedules.len(), 3, "should have one schedule per EV");
    }

    #[test]
    fn test_tou_optimized_result_invariants() {
        let fleet = make_fleet(4);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::TouOptimized);
        let result = fc.schedule_fleet(&fleet).expect("TOU invariants");
        assert!(
            result.total_energy_kwh >= 0.0,
            "energy must be non-negative"
        );
        assert!(
            result.peak_power_kw >= 0.0,
            "peak power must be non-negative"
        );
        for (i, &p) in result.aggregate_power.iter().enumerate() {
            assert!(p >= 0.0, "aggregate_power[{}] = {} is negative", i, p);
        }
    }

    #[test]
    fn test_v2g_optimized_result_invariants() {
        let fleet = make_fleet(3);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::V2gOptimized);
        let result = fc.schedule_fleet(&fleet).expect("V2G invariants");
        assert!(
            result.total_energy_kwh >= 0.0,
            "energy must be non-negative"
        );
        assert!(
            result.peak_power_kw >= 0.0,
            "peak power must be non-negative"
        );
        assert!(
            result.peak_reduction_vs_uncontrolled.is_finite(),
            "peak_reduction_vs_uncontrolled must be finite"
        );
        // V2G is bidirectional: the fleet charges during cheap slots and
        // discharges back to the grid (negative aggregate power) during the
        // high-price window, so non-negativity does NOT hold here. The physical
        // invariant is finiteness and a magnitude bounded by the fleet's
        // aggregate charge/discharge capability.
        let charge_cap_kw: f64 = fleet.sessions.iter().map(|s| s.max_charge_kw).sum();
        let discharge_cap_kw: f64 = fleet.sessions.iter().map(|s| s.max_discharge_kw).sum();
        for (i, &p) in result.aggregate_power.iter().enumerate() {
            assert!(p.is_finite(), "aggregate_power[{i}] = {p} must be finite");
            assert!(
                p >= -discharge_cap_kw - 1e-3 && p <= charge_cap_kw + 1e-3,
                "aggregate_power[{i}] = {p:.2} kW outside fleet capability \
                 [{:.2}, {:.2}] kW",
                -discharge_cap_kw,
                charge_cap_kw
            );
        }
        // The V2G optimiser must actually export to the grid during the
        // high-price window — confirm at least one discharge (negative) slot.
        assert!(
            result.aggregate_power.iter().any(|&p| p < 0.0),
            "V2G schedule must discharge to the grid in at least one slot"
        );
    }

    #[test]
    fn test_uncontrolled_aggregate_length() {
        let fleet = make_fleet(5);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::Uncontrolled);
        let result = fc
            .schedule_fleet(&fleet)
            .expect("uncontrolled aggregate length");
        assert_eq!(
            result.aggregate_power.len(),
            96,
            "aggregate_power must have 96 slots"
        );
    }

    #[test]
    fn test_valley_filling_aggregate_length() {
        let fleet = make_fleet(5);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::ValleyFilling);
        let result = fc
            .schedule_fleet(&fleet)
            .expect("valley filling aggregate length");
        assert_eq!(
            result.aggregate_power.len(),
            96,
            "aggregate_power must have 96 slots"
        );
    }

    #[test]
    fn test_peak_shaving_result_nonneg_energy() {
        let fleet = make_fleet(6);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::PeakShaving { limit_kw: 50.0 });
        let result = fc
            .schedule_fleet(&fleet)
            .expect("peak shaving nonneg energy");
        assert!(
            result.total_energy_kwh >= 0.0,
            "total_energy_kwh must be non-negative after peak shaving"
        );
    }

    #[test]
    fn test_single_ev_fleet_tou() {
        let fleet = make_fleet(1);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::TouOptimized);
        let result = fc.schedule_fleet(&fleet).expect("single EV TOU");
        assert_eq!(result.schedules.len(), 1, "exactly one schedule");
        assert!(result.total_energy_kwh > 0.0, "single EV must charge");
    }

    #[test]
    fn test_single_ev_fleet_uncontrolled() {
        let fleet = make_fleet(1);
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::Uncontrolled);
        let result = fc.schedule_fleet(&fleet).expect("single EV uncontrolled");
        let max_charge_kw = 11.0_f64;
        let epsilon = 1e-3;
        assert!(
            result.peak_power_kw <= max_charge_kw + epsilon,
            "peak {:.3} kW should not exceed max charge rate {:.3} kW",
            result.peak_power_kw,
            max_charge_kw
        );
    }

    #[test]
    fn test_all_same_arrival_departure_uncontrolled() {
        let n = 4usize;
        let sessions: Vec<EvSession> = (0..n).map(|i| make_session(i, 17.0, 31.0, 0.3)).collect();
        let fleet = EvFleet {
            fleet_id: 1,
            bus: 2,
            sessions,
            transformer_limit_kw: 200.0,
            charger_slots: n,
        };
        let charger = make_charger();
        let fc = FleetCharger::new(charger, FleetAlgorithm::Uncontrolled);
        let result = fc
            .schedule_fleet(&fleet)
            .expect("same arrival/departure uncontrolled");
        assert_eq!(
            result.schedules.len(),
            n,
            "should have one schedule per EV even with identical windows"
        );
    }
}
