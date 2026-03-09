//! Storage fleet coordination — joint dispatch of multiple battery units.
//!
//! This module coordinates a fleet of co-located or distributed battery storage
//! units to deliver a combined power setpoint while managing state-of-charge (SoC)
//! balance, degradation costs, and multi-period optimality.
//!
//! ## Coordination algorithms
//!
//! | Algorithm | Description |
//! |-----------|-------------|
//! | `MeritOrder` | Dispatch cheapest unit (lowest degradation cost) first |
//! | `EqualSoc` | Preferentially dispatch units with highest SoC (discharge) or lowest SoC (charge) |
//! | `ProRata` | Each unit contributes proportionally to its usable capacity |
//! | `PriorityBased` | User-assigned priority ranking |
//! | `OptimalDp` | Dynamic programming minimisation of cost over a multi-period horizon |
//!
//! ## References
//! - Megel, O. et al., "Distributed Real-Time Control of D-STATCOM and Battery Energy
//!   Storage Units", IEEE Trans. Smart Grid, 2014.
//! - Zheng, Y. et al., "Optimal Operation of Battery Energy Storage System considering
//!   Degradation Cost", IEEE Trans. Sustain. Energy, 2015.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ── Fleet storage unit ────────────────────────────────────────────────────────

/// A single battery storage unit within the fleet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetStorageUnit {
    /// Unique unit identifier.
    pub unit_id: usize,
    /// Network bus where the unit is connected.
    pub bus: usize,
    /// Usable energy capacity \[MWh\].
    pub e_max_mwh: f64,
    /// Maximum charge or discharge power \[MW\].
    pub p_max_mw: f64,
    /// Round-trip efficiency (0, 1].
    pub eta_roundtrip: f64,
    /// Current state of charge [0, 1].
    pub soc_current: f64,
    /// Minimum allowable SoC [0, 1].
    pub soc_min: f64,
    /// Maximum allowable SoC [0, 1].
    pub soc_max: f64,
    /// Dispatch priority (higher = dispatched first for `PriorityBased` algorithm).
    pub priority: usize,
    /// Variable degradation cost [$/MWh of throughput].
    pub degradation_cost: f64,
}

impl FleetStorageUnit {
    /// Maximum discharge power available given current SoC and slot duration.
    ///
    /// # Arguments
    /// - `dt_h` — Slot duration \[hours\].  Used to convert stored energy to power.
    pub fn max_discharge_mw(&self, dt_h: f64) -> f64 {
        let usable = (self.soc_current - self.soc_min) * self.e_max_mwh;
        let eta_d = self.eta_roundtrip.sqrt().max(1e-12); // one-way discharge efficiency
        let p_by_energy = if dt_h > 1e-12 {
            usable * eta_d / dt_h
        } else {
            self.p_max_mw
        };
        p_by_energy.min(self.p_max_mw).max(0.0)
    }

    /// Maximum charge power available given current SoC and slot duration.
    pub fn max_charge_mw(&self, dt_h: f64) -> f64 {
        let headroom = (self.soc_max - self.soc_current) * self.e_max_mwh;
        let eta_c = self.eta_roundtrip.sqrt().max(1e-12); // one-way charge efficiency
        let p_by_energy = if dt_h > 1e-12 {
            headroom / (eta_c * dt_h)
        } else {
            self.p_max_mw
        };
        p_by_energy.min(self.p_max_mw).max(0.0)
    }

    /// Update SoC after a dispatch action.
    ///
    /// # Arguments
    /// - `power_mw` — Net power \[MW\] (positive = discharge, negative = charge).
    /// - `dt_h`      — Duration \[hours\].
    pub fn update_soc(&mut self, power_mw: f64, dt_h: f64) {
        let eta_c = self.eta_roundtrip.sqrt().max(1e-12);
        let eta_d = self.eta_roundtrip.sqrt().max(1e-12);
        let e_cap = self.e_max_mwh.max(1e-12);

        let delta_soc = if power_mw >= 0.0 {
            // Discharging: SoC decreases.
            -(power_mw * dt_h) / (eta_d * e_cap)
        } else {
            // Charging: SoC increases.
            (-power_mw * eta_c * dt_h) / e_cap
        };

        self.soc_current = (self.soc_current + delta_soc).clamp(self.soc_min, self.soc_max);
    }

    /// Usable energy available for discharge \[MWh\].
    pub fn available_discharge_energy_mwh(&self) -> f64 {
        (self.soc_current - self.soc_min) * self.e_max_mwh
    }

    /// Usable energy headroom for charging \[MWh\].
    pub fn available_charge_headroom_mwh(&self) -> f64 {
        (self.soc_max - self.soc_current) * self.e_max_mwh
    }
}

// ── Coordination algorithm ────────────────────────────────────────────────────

/// Algorithm used to distribute the power setpoint across fleet units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoordinationAlgorithm {
    /// Dispatch cheapest unit (lowest degradation cost) first.
    MeritOrder,
    /// Equalise SoC: discharge highest-SoC units first; charge lowest-SoC units first.
    EqualSoc,
    /// Proportional: each unit contributes proportionally to its usable capacity.
    ProRata,
    /// User-defined priority ranking (higher `priority` field = dispatched first).
    PriorityBased,
    /// Multi-period dynamic programming for horizon-optimal dispatch.
    OptimalDp,
}

// ── Fleet dispatch result ─────────────────────────────────────────────────────

/// Dispatch decision for the entire storage fleet at a single time slot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetDispatch {
    /// Per-unit dispatch: `(unit_id, power_mw)` (positive = discharge).
    pub unit_dispatches: Vec<(usize, f64)>,
    /// Aggregate fleet power \[MW\].
    pub total_power_mw: f64,
    /// Total variable cost for this dispatch [$].
    pub total_cost: f64,
    /// Per-unit SoC after dispatch `[0, 1]`.
    pub soc_after: Vec<f64>,
    /// Standard deviation of SoC across units (SoC imbalance indicator).
    pub soc_imbalance: f64,
}

// ── Storage fleet coordinator ─────────────────────────────────────────────────

/// Coordinates multiple battery storage units to deliver a joint power setpoint.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StorageFleetCoordinator {
    /// Managed storage units.
    pub storage_units: Vec<FleetStorageUnit>,
    /// Selected dispatch algorithm.
    pub coordination_algorithm: CoordinationAlgorithm,
}

impl StorageFleetCoordinator {
    /// Create a new fleet coordinator.
    pub fn new(units: Vec<FleetStorageUnit>, algorithm: CoordinationAlgorithm) -> Self {
        Self {
            storage_units: units,
            coordination_algorithm: algorithm,
        }
    }

    /// Dispatch the fleet to meet a power setpoint.
    ///
    /// # Arguments
    /// - `target_mw` — Desired aggregate power \[MW\] (positive = discharge, negative = charge).
    /// - `dt_h`       — Slot duration \[hours\].
    pub fn dispatch(&mut self, target_mw: f64, dt_h: f64) -> Result<FleetDispatch> {
        if dt_h <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_h must be positive".to_string(),
            ));
        }
        if self.storage_units.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "fleet has no storage units".to_string(),
            ));
        }

        let dispatch = match self.coordination_algorithm {
            CoordinationAlgorithm::EqualSoc => self.dispatch_equal_soc(target_mw, dt_h),
            CoordinationAlgorithm::MeritOrder => self.dispatch_merit_order(target_mw, dt_h),
            CoordinationAlgorithm::ProRata => self.dispatch_pro_rata(target_mw, dt_h),
            CoordinationAlgorithm::PriorityBased => self.dispatch_priority(target_mw, dt_h),
            CoordinationAlgorithm::OptimalDp => {
                // Single-period DP degenerates to merit order.
                self.dispatch_merit_order(target_mw, dt_h)
            }
        };

        Ok(dispatch)
    }

    /// Equal SoC algorithm.
    ///
    /// Allocates the fleet setpoint so that all units converge towards the same
    /// post-dispatch SoC, minimising the spread (standard deviation).
    ///
    /// The algorithm computes the target aggregate SoC after dispatch, then assigns
    /// each unit power proportional to its SoC deviation from the target — units
    /// with higher SoC are discharged more during generation dispatch, and units with
    /// lower SoC are charged more during absorption dispatch.  Physical power limits
    /// are respected; any residual is distributed proportionally to available capacity.
    pub fn dispatch_equal_soc(&mut self, target_mw: f64, dt_h: f64) -> FleetDispatch {
        let n = self.storage_units.len();
        if n == 0 {
            return FleetDispatch {
                unit_dispatches: Vec::new(),
                total_power_mw: 0.0,
                total_cost: 0.0,
                soc_after: Vec::new(),
                soc_imbalance: 0.0,
            };
        }

        // Total energy capacity and current stored energy.
        let total_e_cap: f64 = self.storage_units.iter().map(|u| u.e_max_mwh).sum();
        let total_soe: f64 = self
            .storage_units
            .iter()
            .map(|u| u.soc_current * u.e_max_mwh)
            .sum();

        // Energy delta required to meet the target over dt_h hours.
        // Positive target_mw = discharge → negative delta (SoC decreases).
        let eta_avg = self
            .storage_units
            .iter()
            .map(|u| u.eta_roundtrip)
            .sum::<f64>()
            / n as f64;
        let eta = eta_avg.sqrt().max(1e-12);
        let delta_e = if target_mw >= 0.0 {
            -(target_mw * dt_h / eta) // discharge: energy leaves storage
        } else {
            -target_mw * dt_h * eta // charge: energy enters storage
        };

        // Target aggregate SoE after dispatch.
        let target_soe = (total_soe + delta_e).clamp(
            self.storage_units
                .iter()
                .map(|u| u.soc_min * u.e_max_mwh)
                .sum::<f64>(),
            self.storage_units
                .iter()
                .map(|u| u.soc_max * u.e_max_mwh)
                .sum::<f64>(),
        );
        let target_soc_fleet = if total_e_cap > 1e-12 {
            target_soe / total_e_cap
        } else {
            0.5
        };

        // Ideal power for each unit: drive it to target_soc_fleet.
        // power_i = (soc_i - target_soc_fleet) * e_max_i / dt_h * eta (discharge)
        // Use available capacity as proportional weight to target converging SoC.
        let avail: Vec<f64> = self
            .storage_units
            .iter()
            .map(|u| {
                if target_mw >= 0.0 {
                    u.max_discharge_mw(dt_h)
                } else {
                    u.max_charge_mw(dt_h)
                }
            })
            .collect();
        let avail_sum: f64 = avail.iter().sum();

        // Weight proportional to how far each unit's SoC is from the target
        // in the dispatch direction.
        let weights: Vec<f64> = self
            .storage_units
            .iter()
            .enumerate()
            .map(|(i, u)| {
                let dev = u.soc_current - target_soc_fleet;
                let w = if target_mw >= 0.0 {
                    // Discharge: prefer units with SoC above target.
                    dev
                } else {
                    // Charge: prefer units with SoC below target.
                    -dev
                };
                // Ensure non-negative and cap by available capacity.
                w.max(0.0).min(avail[i])
            })
            .collect();
        let weight_sum: f64 = weights.iter().sum();

        // Choose effective weights: SoC-deviation-based if meaningful, else proportional
        // to available capacity.
        let eff_weights: Vec<f64> = if weight_sum > 1e-12 {
            weights
        } else {
            avail.clone()
        };
        let eff_sum: f64 = eff_weights.iter().sum();

        // First-pass allocation.
        let mut powers = vec![0.0_f64; n];
        let mut remaining = target_mw;

        if eff_sum > 1e-12 {
            for i in 0..n {
                let share = eff_weights[i] / eff_sum;
                let p_raw = target_mw * share;
                let p_clipped = if target_mw >= 0.0 {
                    p_raw.clamp(0.0, avail[i])
                } else {
                    p_raw.clamp(-avail[i], 0.0)
                };
                powers[i] = p_clipped;
                remaining -= p_clipped;
            }
        }

        // Distribute residual proportionally to remaining capacity.
        if remaining.abs() > 1e-9 && avail_sum > 1e-9 {
            let residual_avail: Vec<f64> = (0..n)
                .map(|i| (avail[i] - powers[i].abs()).max(0.0))
                .collect();
            let res_sum: f64 = residual_avail.iter().sum();
            if res_sum > 1e-9 {
                for i in 0..n {
                    let extra = remaining * (residual_avail[i] / res_sum);
                    let extra_clipped = if remaining > 0.0 {
                        extra.clamp(0.0, residual_avail[i])
                    } else {
                        extra.clamp(-residual_avail[i], 0.0)
                    };
                    powers[i] += extra_clipped;
                }
            }
        }

        // Apply dispatches and update SoC.
        let mut unit_dispatches = Vec::with_capacity(n);
        let mut total_cost = 0.0_f64;
        for (i, unit) in self.storage_units.iter_mut().enumerate() {
            unit.update_soc(powers[i], dt_h);
            total_cost += powers[i].abs() * unit.degradation_cost;
            unit_dispatches.push((unit.unit_id, powers[i]));
        }

        let total_power_mw: f64 = powers.iter().sum();
        let soc_after: Vec<f64> = self.storage_units.iter().map(|u| u.soc_current).collect();
        let soc_imbalance = std_dev(&soc_after);

        FleetDispatch {
            unit_dispatches,
            total_power_mw,
            total_cost,
            soc_after,
            soc_imbalance,
        }
    }

    /// Merit order algorithm: dispatch cheapest unit (lowest degradation cost) first.
    pub fn dispatch_merit_order(&mut self, target_mw: f64, dt_h: f64) -> FleetDispatch {
        let n = self.storage_units.len();
        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| {
            self.storage_units[a]
                .degradation_cost
                .partial_cmp(&self.storage_units[b].degradation_cost)
                .unwrap_or(core::cmp::Ordering::Equal)
        });
        self.greedy_dispatch_ordered(target_mw, dt_h, &indices)
    }

    /// Proportional (pro-rata) algorithm.
    ///
    /// Each unit is assigned a share of the total setpoint proportional to its
    /// usable capacity in the requested direction.
    pub fn dispatch_pro_rata(&mut self, target_mw: f64, dt_h: f64) -> FleetDispatch {
        let n = self.storage_units.len();

        // Compute each unit's available power in the requested direction.
        let avail: Vec<f64> = self
            .storage_units
            .iter()
            .map(|u| {
                if target_mw >= 0.0 {
                    u.max_discharge_mw(dt_h)
                } else {
                    u.max_charge_mw(dt_h)
                }
            })
            .collect();

        let total_avail: f64 = avail.iter().sum();
        let mut unit_dispatches = Vec::with_capacity(n);
        let mut total_cost = 0.0_f64;

        let powers: Vec<f64> = if total_avail > 1e-9 {
            avail
                .iter()
                .map(|&a| {
                    let share = a / total_avail;
                    let p_raw = target_mw * share;
                    if target_mw >= 0.0 {
                        p_raw.clamp(0.0, a)
                    } else {
                        p_raw.clamp(-a, 0.0)
                    }
                })
                .collect()
        } else {
            vec![0.0; n]
        };

        for (idx, (&power, unit)) in powers.iter().zip(self.storage_units.iter_mut()).enumerate() {
            let _ = idx;
            unit_dispatches.push((unit.unit_id, power));
            total_cost += power.abs() * unit.degradation_cost;
            unit.update_soc(power, dt_h);
        }

        let total_power_mw = unit_dispatches.iter().map(|(_, p)| p).sum();
        let soc_after: Vec<f64> = self.storage_units.iter().map(|u| u.soc_current).collect();
        let soc_imbalance = std_dev(&soc_after);

        FleetDispatch {
            unit_dispatches,
            total_power_mw,
            total_cost,
            soc_after,
            soc_imbalance,
        }
    }

    /// Priority-based algorithm: dispatch in descending priority order.
    fn dispatch_priority(&mut self, target_mw: f64, dt_h: f64) -> FleetDispatch {
        let n = self.storage_units.len();
        let mut indices: Vec<usize> = (0..n).collect();
        // Higher priority first.
        indices.sort_by(|&a, &b| {
            self.storage_units[b]
                .priority
                .cmp(&self.storage_units[a].priority)
        });
        self.greedy_dispatch_ordered(target_mw, dt_h, &indices)
    }

    /// Greedy dispatch following a pre-sorted index order.
    fn greedy_dispatch_ordered(
        &mut self,
        target_mw: f64,
        dt_h: f64,
        order: &[usize],
    ) -> FleetDispatch {
        let n = self.storage_units.len();
        let mut powers = vec![0.0_f64; n];
        let mut remaining = target_mw;
        let mut total_cost = 0.0_f64;

        for &idx in order {
            if remaining.abs() < 1e-9 {
                break;
            }
            let unit = &self.storage_units[idx];
            let p = if remaining > 0.0 {
                let max_d = unit.max_discharge_mw(dt_h);
                remaining.min(max_d)
            } else {
                let max_c = unit.max_charge_mw(dt_h);
                remaining.max(-max_c)
            };
            powers[idx] = p;
            remaining -= p;
            total_cost += p.abs() * unit.degradation_cost;
        }

        // Apply dispatches and update SoC.
        let mut unit_dispatches = Vec::with_capacity(n);
        for (idx, unit) in self.storage_units.iter_mut().enumerate() {
            unit.update_soc(powers[idx], dt_h);
            unit_dispatches.push((unit.unit_id, powers[idx]));
        }

        let total_power_mw: f64 = powers.iter().sum();
        let soc_after: Vec<f64> = self.storage_units.iter().map(|u| u.soc_current).collect();
        let soc_imbalance = std_dev(&soc_after);

        FleetDispatch {
            unit_dispatches,
            total_power_mw,
            total_cost,
            soc_after,
            soc_imbalance,
        }
    }

    /// Multi-period dynamic programming dispatch optimisation.
    ///
    /// Minimises total cost over the provided `power_profile` horizon.
    ///
    /// The DP formulation:
    /// - State: discretised SoC vector (simplified to scalar aggregate SoE).
    /// - Transitions: feasible dispatch from the current aggregate SoE.
    /// - Cost: degradation cost of each unit's throughput plus opportunity cost
    ///   of stored energy (price signal).
    ///
    /// For computational tractability with large fleets this implementation uses
    /// a forward-pass cost-to-go approximation: at each step, units are dispatched
    /// by a combination of merit order and proportional allocation weighted by
    /// the remaining price opportunity.
    pub fn optimize_dispatch(
        &self,
        power_profile: &[f64],
        prices: &[f64],
        dt_h: f64,
    ) -> Result<Vec<FleetDispatch>> {
        if power_profile.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "power_profile must not be empty".to_string(),
            ));
        }
        if dt_h <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_h must be positive".to_string(),
            ));
        }

        let n_slots = power_profile.len();
        let n_prices = prices.len();

        // Work with a mutable clone to simulate forward pass.
        let mut sim = self.clone();
        let mut results = Vec::with_capacity(n_slots);

        for (slot, &target) in power_profile.iter().enumerate() {
            let price = if slot < n_prices { prices[slot] } else { 0.0 };

            // Future value: sum of positive price opportunities weighted by remaining slots.
            // High future price → save discharge capacity.
            let remaining_slots = (n_slots - slot) as f64;
            let future_avg_price = if slot + 1 < n_prices {
                prices[slot + 1..n_prices.min(n_slots)].iter().sum::<f64>()
                    / remaining_slots.max(1.0)
            } else {
                0.0
            };

            // Adjust effective capacity based on future opportunity.
            // If future price > current price, save capacity (scale down discharge).
            let scale = if target > 0.0 && future_avg_price > price * 1.2 {
                // Future looks more profitable: only dispatch 70% of available.
                0.70_f64
            } else {
                1.0_f64
            };

            // Apply scaled target via merit order.
            let scaled_target = target * scale;
            let n = sim.storage_units.len();
            let mut indices: Vec<usize> = (0..n).collect();
            indices.sort_by(|&a, &b| {
                sim.storage_units[a]
                    .degradation_cost
                    .partial_cmp(&sim.storage_units[b].degradation_cost)
                    .unwrap_or(core::cmp::Ordering::Equal)
            });

            let dispatch = sim.greedy_dispatch_ordered(scaled_target, dt_h, &indices);
            results.push(dispatch);
        }

        Ok(results)
    }

    /// Total stored energy across all units \[MWh\].
    pub fn total_soe_mwh(&self) -> f64 {
        self.storage_units
            .iter()
            .map(|u| u.soc_current * u.e_max_mwh)
            .sum()
    }

    /// Maximum discharge power available from the fleet \[MW\].
    pub fn total_available_power_mw(&self) -> f64 {
        // Use 1-hour slot for power limit estimation.
        self.storage_units
            .iter()
            .map(|u| u.max_discharge_mw(1.0))
            .sum()
    }

    /// SoC standard deviation across units — measure of imbalance.
    pub fn soc_imbalance(&self) -> f64 {
        let socs: Vec<f64> = self.storage_units.iter().map(|u| u.soc_current).collect();
        std_dev(&socs)
    }
}

// ── Statistics helper ─────────────────────────────────────────────────────────

/// Compute the population standard deviation of a slice.
fn std_dev(values: &[f64]) -> f64 {
    let n = values.len();
    if n < 2 {
        return 0.0;
    }
    let mean = values.iter().sum::<f64>() / n as f64;
    let variance = values.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    variance.sqrt()
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_unit(id: usize, e_max: f64, p_max: f64, soc: f64, deg_cost: f64) -> FleetStorageUnit {
        FleetStorageUnit {
            unit_id: id,
            bus: id,
            e_max_mwh: e_max,
            p_max_mw: p_max,
            eta_roundtrip: 0.9,
            soc_current: soc,
            soc_min: 0.1,
            soc_max: 0.9,
            priority: id,
            degradation_cost: deg_cost,
        }
    }

    fn make_fleet_two(algorithm: CoordinationAlgorithm) -> StorageFleetCoordinator {
        StorageFleetCoordinator::new(
            vec![
                make_unit(0, 10.0, 5.0, 0.7, 5.0),
                make_unit(1, 10.0, 5.0, 0.5, 10.0),
            ],
            algorithm,
        )
    }

    #[test]
    fn test_fleet_total_soe() {
        let fleet = make_fleet_two(CoordinationAlgorithm::MeritOrder);
        let expected = 0.7 * 10.0 + 0.5 * 10.0; // 7 + 5 = 12 MWh
        assert!(
            (fleet.total_soe_mwh() - expected).abs() < 1e-9,
            "total SoE {:.4} != {:.4}",
            fleet.total_soe_mwh(),
            expected
        );
    }

    #[test]
    fn test_fleet_soc_imbalance_nonzero() {
        let fleet = make_fleet_two(CoordinationAlgorithm::EqualSoc);
        // Units have different SoC (0.7 vs 0.5), so imbalance > 0.
        assert!(
            fleet.soc_imbalance() > 1e-9,
            "initial SoC imbalance should be non-zero"
        );
    }

    #[test]
    fn test_fleet_merit_order_cheapest_first() {
        let mut fleet = make_fleet_two(CoordinationAlgorithm::MeritOrder);
        // Unit 0 has degradation_cost=5 (cheaper), unit 1 has 10 (expensive).
        let dispatch = fleet.dispatch(3.0, 1.0).expect("dispatch ok");
        let p0 = dispatch
            .unit_dispatches
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        let p1 = dispatch
            .unit_dispatches
            .iter()
            .find(|(id, _)| *id == 1)
            .map(|(_, p)| *p)
            .unwrap_or(0.0);
        // Cheap unit 0 should carry more or all of the load.
        assert!(
            p0 >= p1 - 1e-9,
            "merit order: cheap unit should dispatch first. p0={p0:.3} p1={p1:.3}"
        );
    }

    #[test]
    fn test_fleet_equal_soc_reduces_imbalance() {
        let mut fleet = make_fleet_two(CoordinationAlgorithm::EqualSoc);
        let imbalance_before = fleet.soc_imbalance();
        fleet.dispatch(4.0, 1.0).expect("dispatch ok");
        let imbalance_after = fleet.soc_imbalance();
        assert!(
            imbalance_after <= imbalance_before + 1e-9,
            "EqualSoc should reduce or maintain SoC imbalance: before={imbalance_before:.4} after={imbalance_after:.4}"
        );
    }

    #[test]
    fn test_fleet_pro_rata_proportional() {
        let e_max = 10.0;
        let p_max = 5.0;
        let soc = 0.8;
        let mut fleet = StorageFleetCoordinator::new(
            vec![
                make_unit(0, e_max, p_max, soc, 5.0),
                make_unit(1, e_max, p_max, soc, 10.0),
            ],
            CoordinationAlgorithm::ProRata,
        );
        let dispatch = fleet.dispatch(4.0, 1.0).expect("dispatch ok");
        // Both units have identical available capacity → each should get ~2 MW.
        let p0 = dispatch
            .unit_dispatches
            .iter()
            .find(|(id, _)| *id == 0)
            .map(|(_, p)| *p)
            .unwrap_or(-1.0);
        let p1 = dispatch
            .unit_dispatches
            .iter()
            .find(|(id, _)| *id == 1)
            .map(|(_, p)| *p)
            .unwrap_or(-1.0);
        assert!(
            (p0 - p1).abs() < 0.1,
            "pro-rata: both units should get equal share. p0={p0:.3} p1={p1:.3}"
        );
    }

    #[test]
    fn test_fleet_total_power_meets_target() {
        let mut fleet = make_fleet_two(CoordinationAlgorithm::MeritOrder);
        let target = 6.0;
        let dispatch = fleet.dispatch(target, 1.0).expect("dispatch ok");
        assert!(
            (dispatch.total_power_mw - target).abs() < 1e-9,
            "total power {:.4} should equal target {target}",
            dispatch.total_power_mw
        );
    }

    #[test]
    fn test_fleet_soc_updated_after_dispatch() {
        let mut fleet = make_fleet_two(CoordinationAlgorithm::MeritOrder);
        let soc_before: Vec<f64> = fleet.storage_units.iter().map(|u| u.soc_current).collect();
        fleet.dispatch(3.0, 1.0).expect("dispatch ok");
        let soc_after: Vec<f64> = fleet.storage_units.iter().map(|u| u.soc_current).collect();
        // At least one unit should have decreased SoC.
        let any_decreased = soc_before
            .iter()
            .zip(soc_after.iter())
            .any(|(b, a)| *b > *a + 1e-12);
        assert!(
            any_decreased,
            "SoC should decrease after discharge dispatch"
        );
    }

    #[test]
    fn test_fleet_dp_optimize_returns_all_slots() {
        let fleet = make_fleet_two(CoordinationAlgorithm::OptimalDp);
        let profile = vec![3.0, 2.0, 4.0, 1.0];
        let prices = vec![50.0, 60.0, 40.0, 70.0];
        let results = fleet
            .optimize_dispatch(&profile, &prices, 1.0)
            .expect("optimize ok");
        assert_eq!(results.len(), profile.len());
    }

    #[test]
    fn test_fleet_empty_returns_error() {
        let mut fleet = StorageFleetCoordinator::new(vec![], CoordinationAlgorithm::MeritOrder);
        let result = fleet.dispatch(1.0, 1.0);
        assert!(result.is_err(), "empty fleet should return error");
    }

    #[test]
    fn test_fleet_charge_direction() {
        let mut fleet = make_fleet_two(CoordinationAlgorithm::EqualSoc);
        let dispatch = fleet.dispatch(-3.0, 1.0).expect("dispatch ok");
        // Total power should be negative (charging).
        assert!(
            dispatch.total_power_mw < 0.0,
            "charging dispatch should have negative total power"
        );
    }
}
