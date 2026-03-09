//! Virtual Power Plant aggregator — aggregates DERs for market participation.
//!
//! A VPP pools heterogeneous Distributed Energy Resources (batteries, EV fleets,
//! demand response, CHP, P2G electrolysers, biogas units, PV+storage, wind+storage)
//! into a single dispatchable entity that can participate in wholesale and ancillary
//! service markets.
//!
//! ## Dispatch algorithm
//! Resources are sorted by variable cost (`cost_mwh`) and dispatched in merit order
//! (cheapest first), subject to individual power/energy limits and availability.
//!
//! ## References
//! - Pudjianto, D. et al., "Virtual power plant and system integration of distributed energy
//!   resources", IET Renewable Power Generation, 2007.
//! - Giuntoli, M. & Poli, D., "Optimized Thermal and Electrical Scheduling of a Large Scale
//!   Virtual Power Plant in the Presence of Energy Storages", IEEE Trans. Smart Grid, 2013.

use crate::error::Result;
use serde::{Deserialize, Serialize};

// ── DER type taxonomy ─────────────────────────────────────────────────────────

/// Classification of a Distributed Energy Resource.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DerType {
    /// Utility-scale or distributed battery energy storage.
    BatteryStorage,
    /// Rooftop / ground-mount PV co-located with battery.
    PvWithStorage,
    /// Wind turbine co-located with battery buffer.
    WindWithStorage,
    /// Aggregated EV charging fleet (vehicle-to-grid capable).
    EvFleet,
    /// Industrial or commercial demand-response contract.
    IndustrialDr,
    /// Combined heat and power / cogeneration plant.
    CombinedHeatPower,
    /// Power-to-gas electrolyser (flexible load / storage).
    ElectrolyzerP2g,
    /// Biogas / biomass generator.
    Biogas,
}

// ── DER state ─────────────────────────────────────────────────────────────────

/// Real-time operational state of a single DER.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DerState {
    /// Current net power output (positive = injection, negative = absorption) \[MW\].
    pub current_power_mw: f64,
    /// State of charge [0, 1] for storage-capable resources; `None` otherwise.
    pub soc: Option<f64>,
    /// Whether the resource is currently able to respond to dispatch commands.
    pub available: bool,
    /// Last time a dispatch command was issued [hours from simulation start].
    pub last_dispatch_time_h: f64,
}

impl DerState {
    /// Construct a default idle state with 50 % SoC.
    pub fn idle_with_soc(soc: f64) -> Self {
        Self {
            current_power_mw: 0.0,
            soc: Some(soc.clamp(0.0, 1.0)),
            available: true,
            last_dispatch_time_h: 0.0,
        }
    }

    /// Construct a default idle state for a non-storage resource.
    pub fn idle_no_storage() -> Self {
        Self {
            current_power_mw: 0.0,
            soc: None,
            available: true,
            last_dispatch_time_h: 0.0,
        }
    }
}

// ── DER resource descriptor ───────────────────────────────────────────────────

/// A single Distributed Energy Resource participating in the VPP.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DerResource {
    /// Unique identifier within the VPP.
    pub resource_id: usize,
    /// Technology class.
    pub resource_type: DerType,
    /// Network bus where this resource is connected.
    pub bus: usize,
    /// Maximum injection power \[MW\] (≥ 0).
    pub p_max_mw: f64,
    /// Minimum injection power \[MW\].
    /// Negative values indicate absorption capability (e.g. charging a battery).
    pub p_min_mw: f64,
    /// Energy storage capacity \[MWh\]; `None` for non-storage assets.
    pub e_max_mwh: Option<f64>,
    /// Real-time operational state.
    pub current_state: DerState,
    /// Fraction of rated capacity that is currently available [0, 1].
    pub availability: f64,
    /// Time from dispatch command to full response \[seconds\].
    pub response_time_s: f64,
    /// Variable operating cost [$/MWh].
    pub cost_mwh: f64,
}

impl DerResource {
    /// Effective maximum injection power, accounting for availability.
    pub fn effective_p_max(&self) -> f64 {
        if !self.current_state.available {
            return 0.0;
        }
        self.p_max_mw * self.availability
    }

    /// Effective minimum injection power, accounting for availability.
    /// Absorption capability is also scaled by availability.
    pub fn effective_p_min(&self) -> f64 {
        if !self.current_state.available {
            return 0.0;
        }
        // p_min_mw may be negative (absorption); scale magnitude by availability.
        self.p_min_mw * self.availability
    }

    /// Available energy for discharge \[MWh\], considering current SoC.
    /// Returns 0.0 for non-storage resources.
    pub fn available_energy_mwh(&self) -> f64 {
        match (self.e_max_mwh, self.current_state.soc) {
            (Some(e_cap), Some(soc)) => e_cap * soc.clamp(0.0, 1.0),
            _ => 0.0,
        }
    }
}

// ── VPP capability envelope ───────────────────────────────────────────────────

/// Aggregated capability envelope of the VPP across a dispatch horizon.
///
/// Each entry corresponds to one time slot of duration `dt_h` hours.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VppEnvelope {
    /// Slot start times [hours from now].
    pub time_slots: Vec<f64>,
    /// Maximum net injection per slot \[MW\].
    pub p_max_mw: Vec<f64>,
    /// Minimum net injection per slot \[MW\] (negative = absorption).
    pub p_min_mw: Vec<f64>,
    /// Available stored energy per slot \[MWh\].
    pub energy_remaining_mwh: Vec<f64>,
    /// Aggregate ramp-up rate [MW/min].
    pub ramp_up_mw_per_min: f64,
    /// Aggregate ramp-down rate [MW/min].
    pub ramp_down_mw_per_min: f64,
}

// ── VPP aggregate metrics ─────────────────────────────────────────────────────

/// Snapshot metrics for the VPP.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VppMetrics {
    /// Installed capacity \[MW\].
    pub total_capacity_mw: f64,
    /// Currently available capacity \[MW\].
    pub available_capacity_mw: f64,
    /// Total stored energy across all storage resources \[MWh\].
    pub storage_energy_mwh: f64,
    /// Capacity-weighted average variable cost [$/MWh].
    pub weighted_avg_cost: f64,
    /// Capacity-weighted average response time \[seconds\].
    pub average_response_time_s: f64,
    /// Number of DER resources in the VPP.
    pub n_resources: usize,
}

// ── VPP dispatch result ───────────────────────────────────────────────────────

/// Result of a single VPP dispatch command.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VppDispatchResult {
    /// Per-resource dispatch: `(resource_id, power_mw)`.
    pub dispatched: Vec<(usize, f64)>,
    /// Total dispatched power \[MW\].
    pub total_power_mw: f64,
    /// Power that could not be dispatched (due to capacity limits) \[MW\].
    pub curtailed_mw: f64,
    /// Total variable cost of the dispatch [$].
    pub total_cost: f64,
    /// Maximum response time of the dispatched resources \[seconds\].
    pub response_time_s: f64,
}

// ── Virtual Power Plant ───────────────────────────────────────────────────────

/// Virtual Power Plant — aggregates DERs for unified market participation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualPowerPlant {
    /// Unique VPP identifier.
    pub vpp_id: usize,
    /// Human-readable name.
    pub name: String,
    /// Collection of managed DER resources.
    pub resources: Vec<DerResource>,
    /// Network bus at which the VPP presents its aggregated schedule.
    pub grid_connection_bus: usize,
    /// Maximum injection / absorption at the grid connection point \[MW\].
    pub grid_connection_limit_mw: f64,
    /// Number of hours in the dispatch / forecast horizon.
    pub forecast_horizon_h: usize,
}

impl VirtualPowerPlant {
    /// Create a new VPP with the given resources.
    ///
    /// # Arguments
    /// - `vpp_id`   — Unique identifier.
    /// - `resources` — DERs managed by this VPP.
    /// - `bus`       — Grid connection bus index.
    /// - `limit_mw`  — Grid connection capacity limit \[MW\].
    pub fn new(vpp_id: usize, resources: Vec<DerResource>, bus: usize, limit_mw: f64) -> Self {
        Self {
            vpp_id,
            name: format!("VPP-{vpp_id}"),
            resources,
            grid_connection_bus: bus,
            grid_connection_limit_mw: limit_mw,
            forecast_horizon_h: 24,
        }
    }

    /// Compute the VPP capability envelope over `n_slots` time steps of `dt_h` hours each.
    ///
    /// The envelope aggregates individual resource limits.  For each time slot the
    /// available-energy vector decreases by the energy dispatched in the previous slot
    /// (pessimistic assumption: resources operate at their maximum rate continuously).
    pub fn compute_envelope(&self, n_slots: usize, dt_h: f64) -> VppEnvelope {
        let mut p_max_slots = Vec::with_capacity(n_slots);
        let mut p_min_slots = Vec::with_capacity(n_slots);
        let mut energy_slots = Vec::with_capacity(n_slots);
        let mut time_slots = Vec::with_capacity(n_slots);

        // Running storage energy [MWh] — decremented each slot assuming worst-case dispatch.
        let mut storage_remaining: Vec<f64> = self
            .resources
            .iter()
            .map(|r| r.available_energy_mwh())
            .collect();

        for slot in 0..n_slots {
            time_slots.push(slot as f64 * dt_h);

            let mut p_max_agg = 0.0_f64;
            let mut p_min_agg = 0.0_f64;
            let mut energy_agg = 0.0_f64;

            for (idx, res) in self.resources.iter().enumerate() {
                if !res.current_state.available {
                    continue;
                }

                let eff_pmax = res.effective_p_max();
                let eff_pmin = res.effective_p_min();

                // For storage resources, limit by remaining energy.
                let pmax_slot = if res.e_max_mwh.is_some() {
                    let max_by_energy = if dt_h > 1e-12 {
                        storage_remaining[idx] / dt_h
                    } else {
                        eff_pmax
                    };
                    eff_pmax.min(max_by_energy).max(0.0)
                } else {
                    eff_pmax
                };

                p_max_agg += pmax_slot;
                p_min_agg += eff_pmin;
                energy_agg += storage_remaining[idx];

                // Pessimistic: assume full discharge in each slot.
                if res.e_max_mwh.is_some() {
                    let discharged = (pmax_slot * dt_h).min(storage_remaining[idx]);
                    storage_remaining[idx] = (storage_remaining[idx] - discharged).max(0.0);
                }
            }

            // Clamp to grid connection limit.
            let p_max_clamp = p_max_agg.min(self.grid_connection_limit_mw);
            let p_min_clamp = p_min_agg.max(-self.grid_connection_limit_mw);

            p_max_slots.push(p_max_clamp);
            p_min_slots.push(p_min_clamp);
            energy_slots.push(energy_agg);
        }

        // Aggregate ramp rates: sum of individual resource ramp capabilities.
        // We approximate each resource as capable of P_max / 5 min (i.e. 5-min ramp to full).
        let total_p_max: f64 = self.resources.iter().map(|r| r.effective_p_max()).sum();
        let ramp_mw_per_min = total_p_max / 5.0; // 5-minute ramp assumption

        VppEnvelope {
            time_slots,
            p_max_mw: p_max_slots,
            p_min_mw: p_min_slots,
            energy_remaining_mwh: energy_slots,
            ramp_up_mw_per_min: ramp_mw_per_min,
            ramp_down_mw_per_min: ramp_mw_per_min,
        }
    }

    /// Dispatch the VPP to a power setpoint for a given time slot.
    ///
    /// Resources are sorted by variable cost (merit order: cheapest first for positive
    /// power, most expensive absorbed first for negative power).  Individual resource
    /// limits and availability are respected.
    ///
    /// # Arguments
    /// - `target_mw` — Desired aggregate power \[MW\] (positive = injection).
    /// - `slot`       — Time slot index (used for logging only; does not affect limits).
    pub fn dispatch(&mut self, target_mw: f64, slot: usize) -> Result<VppDispatchResult> {
        let _ = slot; // slot reserved for future time-varying limit look-up

        // Clamp target to grid connection limit.
        let target_clamped = target_mw.clamp(
            -self.grid_connection_limit_mw,
            self.grid_connection_limit_mw,
        );

        // Sort resource indices by cost (ascending for generation, descending for absorption).
        let mut indices: Vec<usize> = (0..self.resources.len()).collect();
        if target_clamped >= 0.0 {
            // Discharge / generation: cheapest first.
            indices.sort_by(|&a, &b| {
                self.resources[a]
                    .cost_mwh
                    .partial_cmp(&self.resources[b].cost_mwh)
                    .unwrap_or(core::cmp::Ordering::Equal)
            });
        } else {
            // Charging / absorption: most expensive to absorb last (effectively cheapest to charge).
            indices.sort_by(|&a, &b| {
                self.resources[b]
                    .cost_mwh
                    .partial_cmp(&self.resources[a].cost_mwh)
                    .unwrap_or(core::cmp::Ordering::Equal)
            });
        }

        let mut remaining = target_clamped;
        let mut dispatched = Vec::with_capacity(self.resources.len());
        let mut total_cost = 0.0_f64;
        let mut max_response_s = 0.0_f64;

        for &idx in &indices {
            if remaining.abs() < 1e-9 {
                break;
            }

            let res = &self.resources[idx];
            if !res.current_state.available {
                dispatched.push((res.resource_id, 0.0));
                continue;
            }

            let p_max = res.effective_p_max();
            let p_min = res.effective_p_min();

            let p_dispatch = if remaining > 0.0 {
                // Provide positive power (generation / discharge).
                remaining.min(p_max).max(0.0)
            } else {
                // Absorb negative power (charging / demand response).
                remaining.max(p_min).min(0.0)
            };

            // Energy feasibility check for storage resources.
            let p_feasible =
                if let (Some(e_cap), Some(soc)) = (res.e_max_mwh, res.current_state.soc) {
                    if p_dispatch > 0.0 {
                        // Discharging: limit by stored energy (assume 1-hour slot for cap check).
                        let max_by_soc = e_cap * soc.clamp(0.0, 1.0);
                        p_dispatch.min(max_by_soc)
                    } else {
                        // Charging: limit by available headroom.
                        let max_absorb = e_cap * (1.0 - soc.clamp(0.0, 1.0));
                        p_dispatch.max(-max_absorb)
                    }
                } else {
                    p_dispatch
                };

            if p_feasible.abs() > 1e-9 {
                remaining -= p_feasible;
                total_cost += p_feasible.abs() * res.cost_mwh;
                max_response_s = max_response_s.max(res.response_time_s);
                dispatched.push((res.resource_id, p_feasible));
                // Update resource state.
                self.resources[idx].current_state.current_power_mw = p_feasible;
            } else {
                dispatched.push((res.resource_id, 0.0));
            }
        }

        let total_power_mw = target_clamped - remaining;
        let curtailed_mw = (target_clamped - total_power_mw).abs();

        Ok(VppDispatchResult {
            dispatched,
            total_power_mw,
            curtailed_mw,
            total_cost,
            response_time_s: max_response_s,
        })
    }

    /// Forecast VPP aggregate output for the next `n_slots` time steps.
    ///
    /// Uses a simple price-responsive heuristic:
    /// - When the forecast price exceeds the weighted average cost, the VPP generates
    ///   at maximum capacity (up to its energy limit).
    /// - When the price is below average cost, the VPP absorbs (charges) at minimum.
    ///
    /// # Arguments
    /// - `price_forecast` — Expected market prices per slot [$/MWh].
    /// - `n_slots`         — Number of slots to forecast.
    /// - `dt_h`            — Slot duration \[hours\].
    pub fn forecast_output(&self, price_forecast: &[f64], n_slots: usize, dt_h: f64) -> Vec<f64> {
        let envelope = self.compute_envelope(n_slots, dt_h);
        let metrics = self.metrics();
        let avg_cost = metrics.weighted_avg_cost;

        let n = n_slots
            .min(price_forecast.len())
            .min(envelope.p_max_mw.len());
        let mut output = Vec::with_capacity(n);

        for (i, &price) in price_forecast.iter().enumerate().take(n) {
            let p_max = envelope.p_max_mw[i];
            let p_min = envelope.p_min_mw[i];

            let p = if price > avg_cost {
                // Profitable to generate: inject at max.
                p_max
            } else if price < avg_cost * 0.5 {
                // Very low price: charge storage.
                p_min
            } else {
                // Near cost: idle or minimal output.
                0.0_f64.clamp(p_min, p_max)
            };

            output.push(p);
        }

        output
    }

    /// Compute current aggregate metrics for the VPP.
    pub fn metrics(&self) -> VppMetrics {
        let n = self.resources.len();
        if n == 0 {
            return VppMetrics {
                total_capacity_mw: 0.0,
                available_capacity_mw: 0.0,
                storage_energy_mwh: 0.0,
                weighted_avg_cost: 0.0,
                average_response_time_s: 0.0,
                n_resources: 0,
            };
        }

        let total_capacity_mw: f64 = self.resources.iter().map(|r| r.p_max_mw).sum();
        let available_capacity_mw: f64 = self.resources.iter().map(|r| r.effective_p_max()).sum();
        let storage_energy_mwh: f64 = self
            .resources
            .iter()
            .map(|r| r.available_energy_mwh())
            .sum();

        // Capacity-weighted average cost.
        let weight_sum: f64 = self.resources.iter().map(|r| r.effective_p_max()).sum();
        let weighted_avg_cost = if weight_sum > 1e-12 {
            self.resources
                .iter()
                .map(|r| r.effective_p_max() * r.cost_mwh)
                .sum::<f64>()
                / weight_sum
        } else {
            0.0
        };

        let average_response_time_s = if weight_sum > 1e-12 {
            self.resources
                .iter()
                .map(|r| r.effective_p_max() * r.response_time_s)
                .sum::<f64>()
                / weight_sum
        } else {
            0.0
        };

        VppMetrics {
            total_capacity_mw,
            available_capacity_mw,
            storage_energy_mwh,
            weighted_avg_cost,
            average_response_time_s,
            n_resources: n,
        }
    }
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_battery_resource(
        id: usize,
        p_max: f64,
        e_max: f64,
        soc: f64,
        cost: f64,
    ) -> DerResource {
        DerResource {
            resource_id: id,
            resource_type: DerType::BatteryStorage,
            bus: id,
            p_max_mw: p_max,
            p_min_mw: -p_max,
            e_max_mwh: Some(e_max),
            current_state: DerState::idle_with_soc(soc),
            availability: 1.0,
            response_time_s: 30.0,
            cost_mwh: cost,
        }
    }

    fn make_dr_resource(id: usize, p_max: f64, cost: f64) -> DerResource {
        DerResource {
            resource_id: id,
            resource_type: DerType::IndustrialDr,
            bus: id,
            p_max_mw: p_max,
            p_min_mw: 0.0,
            e_max_mwh: None,
            current_state: DerState::idle_no_storage(),
            availability: 1.0,
            response_time_s: 120.0,
            cost_mwh: cost,
        }
    }

    fn make_vpp_two_batteries() -> VirtualPowerPlant {
        let resources = vec![
            make_battery_resource(0, 5.0, 20.0, 0.8, 10.0),
            make_battery_resource(1, 3.0, 12.0, 0.6, 20.0),
        ];
        VirtualPowerPlant::new(0, resources, 0, 10.0)
    }

    #[test]
    fn test_envelope_aggregates_resource_p_max() {
        let vpp = make_vpp_two_batteries();
        let env = vpp.compute_envelope(3, 1.0);
        // Slot 0: p_max = min(5+3, limit=10) = 8.0 MW (first slot, full SoC headroom)
        assert_eq!(env.p_max_mw.len(), 3);
        assert!(env.p_max_mw[0] > 0.0, "p_max should be positive");
        assert!(env.p_max_mw[0] <= 10.0, "p_max must not exceed grid limit");
    }

    #[test]
    fn test_envelope_p_min_negative() {
        let vpp = make_vpp_two_batteries();
        let env = vpp.compute_envelope(2, 1.0);
        // Resources have p_min_mw = -p_max, so aggregate p_min should be negative.
        assert!(
            env.p_min_mw[0] < 0.0,
            "aggregate p_min should be negative for storage"
        );
    }

    #[test]
    fn test_envelope_slot_count() {
        let vpp = make_vpp_two_batteries();
        let env = vpp.compute_envelope(5, 0.5);
        assert_eq!(env.time_slots.len(), 5);
        assert_eq!(env.p_max_mw.len(), 5);
        assert_eq!(env.p_min_mw.len(), 5);
        assert_eq!(env.energy_remaining_mwh.len(), 5);
    }

    #[test]
    fn test_dispatch_merit_order_cheapest_first() {
        let mut vpp = VirtualPowerPlant::new(
            0,
            vec![
                make_battery_resource(0, 5.0, 20.0, 0.9, 30.0), // expensive
                make_battery_resource(1, 5.0, 20.0, 0.9, 10.0), // cheap
            ],
            0,
            20.0,
        );
        let result = vpp.dispatch(4.0, 0).expect("dispatch ok");
        // Cheap resource (id=1) should be dispatched first/more.
        let power_cheap = result
            .dispatched
            .iter()
            .find(|(id, _)| *id == 1)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        let power_expensive = result
            .dispatched
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        // With target=4.0 and cheap unit at 5 MW max, cheap should cover all 4 MW.
        assert!(
            power_cheap >= power_expensive,
            "cheap resource ({power_cheap:.2} MW) should be dispatched before expensive ({power_expensive:.2} MW)"
        );
    }

    #[test]
    fn test_dispatch_respects_individual_limits() {
        let mut vpp = VirtualPowerPlant::new(
            0,
            vec![
                make_battery_resource(0, 2.0, 8.0, 0.9, 10.0),
                make_battery_resource(1, 3.0, 12.0, 0.9, 15.0),
            ],
            0,
            10.0,
        );
        // Request more than each individual unit can provide alone, but within total.
        let result = vpp.dispatch(4.0, 0).expect("dispatch ok");
        for (id, p) in &result.dispatched {
            let res = vpp.resources.iter().find(|r| r.resource_id == *id);
            if let Some(r) = res {
                assert!(
                    *p <= r.p_max_mw + 1e-9,
                    "resource {id} dispatched {p:.3} > p_max {:.3}",
                    r.p_max_mw
                );
            }
        }
    }

    #[test]
    fn test_dispatch_grid_limit_respected() {
        let mut vpp = VirtualPowerPlant::new(
            0,
            vec![
                make_battery_resource(0, 10.0, 50.0, 0.9, 10.0),
                make_battery_resource(1, 10.0, 50.0, 0.9, 15.0),
            ],
            0,
            5.0, // grid limit = 5 MW
        );
        let result = vpp.dispatch(20.0, 0).expect("dispatch ok");
        assert!(
            result.total_power_mw <= 5.0 + 1e-9,
            "total power {} must not exceed grid limit 5 MW",
            result.total_power_mw
        );
    }

    #[test]
    fn test_dispatch_unavailable_resource_skipped() {
        let mut vpp = VirtualPowerPlant::new(
            0,
            vec![{
                let mut r = make_battery_resource(0, 5.0, 20.0, 0.9, 10.0);
                r.current_state.available = false;
                r
            }],
            0,
            10.0,
        );
        let result = vpp.dispatch(3.0, 0).expect("dispatch ok");
        assert_eq!(
            result.total_power_mw, 0.0,
            "unavailable resource should provide 0 power"
        );
    }

    #[test]
    fn test_metrics_n_resources() {
        let vpp = make_vpp_two_batteries();
        let m = vpp.metrics();
        assert_eq!(m.n_resources, 2);
    }

    #[test]
    fn test_metrics_storage_energy() {
        let vpp = make_vpp_two_batteries();
        let m = vpp.metrics();
        // Resource 0: 20 MWh * 0.8 = 16 MWh; Resource 1: 12 MWh * 0.6 = 7.2 MWh
        let expected = 20.0 * 0.8 + 12.0 * 0.6;
        assert!(
            (m.storage_energy_mwh - expected).abs() < 1e-9,
            "storage energy {:.4} != expected {:.4}",
            m.storage_energy_mwh,
            expected
        );
    }

    #[test]
    fn test_metrics_weighted_avg_cost() {
        let resources = vec![
            make_battery_resource(0, 4.0, 16.0, 0.8, 10.0),
            make_battery_resource(1, 4.0, 16.0, 0.8, 30.0),
        ];
        let vpp = VirtualPowerPlant::new(0, resources, 0, 20.0);
        let m = vpp.metrics();
        // Equal capacity → average cost = (10+30)/2 = 20
        assert!(
            (m.weighted_avg_cost - 20.0).abs() < 1e-9,
            "weighted_avg_cost should be 20.0, got {:.4}",
            m.weighted_avg_cost
        );
    }

    #[test]
    fn test_forecast_output_high_price_generates() {
        let vpp = make_vpp_two_batteries();
        let prices = vec![200.0; 4]; // very high price → should generate
        let output = vpp.forecast_output(&prices, 4, 1.0);
        for p in &output {
            assert!(*p >= 0.0, "should not absorb at very high price");
        }
    }

    #[test]
    fn test_forecast_output_low_price_absorbs() {
        let vpp = make_vpp_two_batteries();
        let prices = vec![1.0; 4]; // very low price → should charge
        let output = vpp.forecast_output(&prices, 4, 1.0);
        for p in &output {
            assert!(
                *p <= 0.0 + 1e-9,
                "should charge at very low price, got {p:.4}"
            );
        }
    }

    #[test]
    fn test_der_availability_partial() {
        let mut res = make_battery_resource(0, 10.0, 40.0, 0.8, 15.0);
        res.availability = 0.5;
        assert!(
            (res.effective_p_max() - 5.0).abs() < 1e-9,
            "effective p_max should be 5.0 with 50% availability"
        );
    }

    #[test]
    fn test_der_unavailable_zero_power() {
        let mut res = make_dr_resource(0, 10.0, 15.0);
        res.current_state.available = false;
        assert_eq!(res.effective_p_max(), 0.0);
    }
}
