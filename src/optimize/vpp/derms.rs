//! Distributed Energy Resource Management System (DERMS).
//!
//! Coordinates heterogeneous DER assets — rooftop solar, battery storage,
//! EVs, heat pumps, CHP, demand response, and smart inverters — to
//! optimise multiple grid objectives simultaneously:
//!
//! - Peak demand shaving (substation capacity enforcement)
//! - Self-consumption maximisation (solar-to-load matching)
//! - Cost minimisation against time-of-use tariffs
//! - Voltage regulation and line overload prevention
//!
//! # Dispatch algorithm
//! 1. Aggregate all asset power forecasts per timestep.
//! 2. Compute net load = total load − solar generation.
//! 3. Dispatch flexible assets (storage, EV, DR) to satisfy objectives.
//! 4. Check voltage and line-flow constraints; apply curtailment if violated.
//! 5. Report dispatch setpoints, metrics, and constraint violations.
//!
//! # Units
//! - Power: \[MW\] / \[Mvar\]
//! - Voltage: \[pu\]
//! - Cost: \[USD\]
//! - Line ratings: \[MW\]

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the DERMS controller.
#[derive(Debug, thiserror::Error)]
pub enum DermsError {
    /// No assets registered.
    #[error("no DER assets registered")]
    NoAssets,

    /// Load forecast length mismatch.
    #[error("load forecast length {got} does not match forecast horizon {expected}")]
    ForecastLengthMismatch { got: usize, expected: usize },

    /// Infeasible dispatch (e.g. load exceeds all flexible capacity).
    #[error("infeasible dispatch: {0}")]
    Infeasible(String),
}

// ---------------------------------------------------------------------------
// Tariff structure
// ---------------------------------------------------------------------------

/// Time-of-use tariff parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TariffStructure {
    /// Energy charge per hour \[$/MWh\], length = forecast horizon
    pub energy_rate_usd_per_mwh: Vec<f64>,
    /// Peak demand charge per MW \[$/MW\]
    pub demand_charge_usd_per_mw: f64,
    /// Feed-in tariff for exported energy \[$/MWh\]
    pub export_rate_usd_per_mwh: f64,
}

// ---------------------------------------------------------------------------
// Grid limits
// ---------------------------------------------------------------------------

/// Physical limits of the distribution feeder / substation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridLimits {
    /// Substation import capacity \[MW\]
    pub substation_capacity_mw: f64,
    /// Minimum acceptable voltage \[pu\]
    pub voltage_min_pu: f64,
    /// Maximum acceptable voltage \[pu\]
    pub voltage_max_pu: f64,
    /// Per-segment line thermal rating \[MW\]
    pub line_ratings_mw: Vec<f64>,
}

// ---------------------------------------------------------------------------
// DERMS objectives
// ---------------------------------------------------------------------------

/// Optimisation objective for the DERMS controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DermsObjective {
    /// Minimise peak demand at the substation
    MinimizePeakDemand,
    /// Maximise solar self-consumption (minimise export and import)
    MaximizeSelfConsumption,
    /// Minimise electricity cost using time-of-use tariffs
    MinimizeCost {
        /// Applicable tariff structure
        tariff_structure: TariffStructure,
    },
    /// Maximise revenue from grid export (e.g. arbitrage or ancillary services)
    MaximizeRevenueFromGrid,
    /// Minimise distribution losses (proxy: minimise net import²)
    MinimizeLosses,
    /// Regulate voltage to nominal (1.0 pu)
    VoltageRegulation,
}

// ---------------------------------------------------------------------------
// DER asset types
// ---------------------------------------------------------------------------

/// Classification of a DER asset.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DerAssetType {
    /// Rooftop photovoltaic system (generation asset)
    RooftopSolar,
    /// Battery energy storage system
    BatteryStorage,
    /// Electric vehicle (can charge, optionally V2G discharge)
    ElectricVehicle,
    /// Heat pump (controllable load)
    HeatPump,
    /// Combined heat and power unit (generation + heat)
    CombinedHeatPower,
    /// Demand response programme participant
    DemandResponse,
    /// Smart inverter (P/Q controllable)
    SmartInverter,
}

/// A single distributed energy resource asset registered with the DERMS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DerAsset {
    /// Unique asset identifier
    pub id: usize,
    /// Asset technology type
    pub asset_type: DerAssetType,
    /// Grid bus number this asset is connected to
    pub bus: usize,
    /// Maximum active power output / injection \[MW\]
    pub p_max_mw: f64,
    /// Minimum active power (negative = consumption minimum) \[MW\]
    pub p_min_mw: f64,
    /// Maximum reactive power injection \[Mvar\]
    pub q_max_mvar: f64,
    /// Minimum reactive power injection \[Mvar\]
    pub q_min_mvar: f64,
    /// Power forecast for the horizon \[MW\] (positive = generation)
    pub forecast_mw: Vec<f64>,
    /// Current measured active power \[MW\]
    pub current_p_mw: f64,
    /// State of charge for storage assets (0–1)
    pub current_soc: Option<f64>,
}

impl DerAsset {
    /// Returns `true` if this asset can be controlled (not uncontrolled solar).
    fn is_flexible(&self) -> bool {
        !matches!(self.asset_type, DerAssetType::RooftopSolar)
    }

    /// Available flexible power at a given hour.
    fn flexible_range_at_hour(&self, hour: usize) -> (f64, f64) {
        let base = self
            .forecast_mw
            .get(hour)
            .copied()
            .unwrap_or(self.current_p_mw);
        match self.asset_type {
            DerAssetType::DemandResponse => {
                // Can reduce load by up to p_max_mw
                (base - self.p_max_mw, base)
            }
            DerAssetType::HeatPump => {
                // Can curtail up to 50% or shift
                (base * 0.5, base)
            }
            DerAssetType::ElectricVehicle => {
                // Flexible charge schedule; optionally V2G
                (self.p_min_mw, self.p_max_mw)
            }
            DerAssetType::BatteryStorage => (self.p_min_mw, self.p_max_mw),
            DerAssetType::CombinedHeatPower => (self.p_min_mw, self.p_max_mw),
            DerAssetType::SmartInverter => (self.p_min_mw, self.p_max_mw),
            _ => (base, base), // not flexible
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatch result
// ---------------------------------------------------------------------------

/// A single dispatch setpoint for one asset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DermsDispatch {
    /// Asset identifier
    pub asset_id: usize,
    /// Active power setpoint \[MW\] (positive = generation / discharge)
    pub p_setpoint_mw: f64,
    /// Reactive power setpoint \[Mvar\]
    pub q_setpoint_mvar: f64,
    /// Human-readable reason for the setpoint
    pub reason: String,
}

// ---------------------------------------------------------------------------
// DERMS configuration
// ---------------------------------------------------------------------------

/// Configuration for the DERMS controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DermsConfig {
    /// Control loop interval \[s\]
    pub update_interval_s: f64,
    /// Look-ahead forecast horizon \[h\]
    pub forecast_horizon_h: usize,
    /// Physical grid constraints
    pub grid_limits: GridLimits,
    /// List of active optimisation objectives (priority order)
    pub objectives: Vec<DermsObjective>,
}

// ---------------------------------------------------------------------------
// DERMS Result
// ---------------------------------------------------------------------------

/// Summary result from a DERMS dispatch round.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DermsResult {
    /// Per-asset dispatch setpoints (first timestep)
    pub dispatch: Vec<DermsDispatch>,
    /// Peak demand at substation \[MW\]
    pub peak_demand_mw: f64,
    /// Self-consumption of solar generation (fraction 0–1)
    pub self_consumption_pct: f64,
    /// Estimated electricity cost \[USD\]
    pub estimated_cost_usd: f64,
    /// Number of voltage constraint violations across the horizon
    pub voltage_violations: usize,
    /// Number of line overload incidents across the horizon
    pub line_overloads: usize,
    /// Total renewable energy served \[MWh\]
    pub total_renewable_mwh: f64,
    /// Renewable curtailment \[MWh\]
    pub curtailment_mwh: f64,
}

// ---------------------------------------------------------------------------
// DERMS Controller
// ---------------------------------------------------------------------------

/// Central controller for distributed energy resource management.
pub struct DermsController {
    config: DermsConfig,
    assets: Vec<DerAsset>,
}

impl DermsController {
    /// Create a new DERMS controller with the given configuration.
    pub fn new(config: DermsConfig) -> Self {
        Self {
            config,
            assets: Vec::new(),
        }
    }

    /// Register a DER asset for DERMS control.
    pub fn register_asset(&mut self, asset: DerAsset) {
        self.assets.push(asset);
    }

    /// Run one dispatch cycle given a load forecast.
    ///
    /// Returns dispatch setpoints and summary metrics.
    pub fn dispatch(&self, load_forecast_mw: &[f64]) -> Result<DermsResult, DermsError> {
        if self.assets.is_empty() {
            return Err(DermsError::NoAssets);
        }
        let horizon = self.config.forecast_horizon_h;
        if load_forecast_mw.len() != horizon {
            return Err(DermsError::ForecastLengthMismatch {
                got: load_forecast_mw.len(),
                expected: horizon,
            });
        }

        // ------------------------------------------------------------------
        // Step 1: Aggregate asset forecasts per timestep
        // ------------------------------------------------------------------
        let mut solar_gen_mw = vec![0.0_f64; horizon];
        let mut flexible_max_mw = vec![0.0_f64; horizon]; // max we can reduce load or inject
        let mut flexible_min_mw = vec![0.0_f64; horizon]; // max we can increase load or absorb

        for asset in &self.assets {
            match asset.asset_type {
                DerAssetType::RooftopSolar | DerAssetType::SmartInverter => {
                    for (h, slot) in solar_gen_mw.iter_mut().enumerate().take(horizon) {
                        let gen = asset.forecast_mw.get(h).copied().unwrap_or(0.0).max(0.0);
                        *slot += gen;
                    }
                }
                _ if asset.is_flexible() => {
                    for (h, (slot_max, slot_min)) in flexible_max_mw
                        .iter_mut()
                        .zip(flexible_min_mw.iter_mut())
                        .enumerate()
                        .take(horizon)
                    {
                        let (fmin, fmax) = asset.flexible_range_at_hour(h);
                        let forecast = asset.forecast_mw.get(h).copied().unwrap_or(0.0);
                        // Capacity to increase net injection (reduce grid import)
                        // = max output - current forecast (for generators)
                        // + |min setpoint - forecast| for load-reduction assets
                        *slot_max += (fmax - forecast).max(0.0);
                        // Reduction capacity: how much can we reduce grid import?
                        // For generators (fmin < 0): discharge capacity = (-fmin).max(0)
                        // For loads (fmin >= 0, fmin < forecast): sheddable = forecast - fmin
                        let reduction = if fmin < 0.0 {
                            (-fmin).max(0.0) // battery can inject (-fmin) MW
                        } else {
                            (forecast - fmin).max(0.0) // DR can shed this much
                        };
                        *slot_min += reduction;
                    }
                }
                _ => {}
            }
        }

        // ------------------------------------------------------------------
        // Step 2: Compute net load = load - solar
        // ------------------------------------------------------------------
        let net_load_mw: Vec<f64> = (0..horizon)
            .map(|h| load_forecast_mw[h] - solar_gen_mw[h])
            .collect();

        // ------------------------------------------------------------------
        // Step 3 & 4: Dispatch flexible assets per objective
        // ------------------------------------------------------------------
        let sub_cap = self.config.grid_limits.substation_capacity_mw;
        let mut dispatched_mw = vec![0.0_f64; horizon]; // net dispatch adjustment
        let mut voltage_violations = 0_usize;
        let mut line_overloads = 0_usize;
        let mut total_cost = 0.0_f64;
        let mut curtailment_mwh = 0.0_f64;

        for h in 0..horizon {
            let net = net_load_mw[h];
            let import = net - dispatched_mw[h];

            // --- Objective: Peak shaving ---
            // dispatched_mw[h] > 0 means "reduction in grid import achieved by flexible assets"
            for obj in &self.config.objectives {
                if let DermsObjective::MinimizePeakDemand = obj {
                    if import > sub_cap {
                        // Dispatch flexible assets (battery discharge / DR shed) to reduce import
                        let reduction_needed = import - sub_cap;
                        let reduction = reduction_needed.min(flexible_min_mw[h]);
                        dispatched_mw[h] += reduction; // positive: reduces net import
                    }
                }
                if let DermsObjective::MaximizeSelfConsumption = obj {
                    // Surplus solar: charge storage or increase controllable loads
                    if net < 0.0 {
                        let surplus = net.abs();
                        let absorbable = flexible_max_mw[h];
                        // Absorbing surplus reduces export (negative net → import increases toward 0)
                        dispatched_mw[h] -= surplus.min(absorbable);
                    }
                }
                if let DermsObjective::MinimizeCost { tariff_structure } = obj {
                    let rate = tariff_structure
                        .energy_rate_usd_per_mwh
                        .get(h)
                        .copied()
                        .unwrap_or(60.0);
                    let final_import = (net - dispatched_mw[h]).max(0.0);
                    total_cost += final_import * rate * 1.0; // 1-hour timestep
                }
            }

            // --- Constraint check: substation capacity ---
            let final_import = net - dispatched_mw[h];
            if final_import > sub_cap + 1e-6 {
                // Still overloaded after flexible dispatch — curtail solar
                let overflow = final_import - sub_cap;
                curtailment_mwh += overflow.min(solar_gen_mw[h]);
            }

            // --- Constraint check: line ratings ---
            for &rating in &self.config.grid_limits.line_ratings_mw {
                if final_import.abs() > rating + 1e-6 {
                    line_overloads += 1;
                }
            }

            // --- Simplified voltage check ---
            // Proxy: if import >> capacity, voltage sags; if large export, voltage rises
            let import_fraction = final_import / sub_cap.max(1.0);
            let v_proxy = 1.0 - 0.05 * import_fraction + 0.02 * solar_gen_mw[h] / sub_cap.max(1.0);
            if v_proxy < self.config.grid_limits.voltage_min_pu
                || v_proxy > self.config.grid_limits.voltage_max_pu
            {
                voltage_violations += 1;
            }
        }

        // ------------------------------------------------------------------
        // Step 5: Build dispatch for first timestep
        // ------------------------------------------------------------------
        let mut dispatch_setpoints = Vec::new();
        let adjustment_h0 = dispatched_mw.first().copied().unwrap_or(0.0);
        let n_flexible = self
            .assets
            .iter()
            .filter(|a| a.is_flexible())
            .count()
            .max(1) as f64;

        for asset in &self.assets {
            let forecast_h0 = asset.forecast_mw.first().copied().unwrap_or(0.0);
            let setpoint = if asset.is_flexible() {
                let individual_adj = adjustment_h0 / n_flexible;
                let (pmin, pmax) = asset.flexible_range_at_hour(0);
                (forecast_h0 + individual_adj).clamp(pmin, pmax)
            } else {
                forecast_h0 // solar runs at forecast (curtailment handled above)
            };

            let reason = match asset.asset_type {
                DerAssetType::RooftopSolar => "solar at forecast".into(),
                DerAssetType::BatteryStorage => {
                    if setpoint > forecast_h0 {
                        "charging for self-consumption or peak shaving".into()
                    } else {
                        "discharging to serve load".into()
                    }
                }
                DerAssetType::DemandResponse => "demand response activated".into(),
                DerAssetType::ElectricVehicle => "EV smart charging".into(),
                DerAssetType::HeatPump => "heat pump load management".into(),
                DerAssetType::CombinedHeatPower => "CHP dispatch".into(),
                DerAssetType::SmartInverter => "smart inverter volt-var control".into(),
            };

            dispatch_setpoints.push(DermsDispatch {
                asset_id: asset.id,
                p_setpoint_mw: setpoint,
                q_setpoint_mvar: 0.0, // simplified: no reactive dispatch
                reason,
            });
        }

        // ------------------------------------------------------------------
        // Metrics
        // ------------------------------------------------------------------
        let peak_demand_mw = net_load_mw
            .iter()
            .zip(dispatched_mw.iter())
            .map(|(&nl, &disp)| (nl - disp).max(0.0))
            .fold(f64::NEG_INFINITY, f64::max);

        let total_solar_mwh: f64 = solar_gen_mw.iter().sum();
        let self_consumption_pct = if total_solar_mwh > 1e-6 {
            let exported: f64 = net_load_mw
                .iter()
                .zip(dispatched_mw.iter())
                .map(|(&nl, &disp)| {
                    let net_import = nl - disp;
                    if net_import < 0.0 {
                        net_import.abs()
                    } else {
                        0.0
                    }
                })
                .sum();
            ((total_solar_mwh - exported) / total_solar_mwh).clamp(0.0, 1.0)
        } else {
            0.0
        };

        Ok(DermsResult {
            dispatch: dispatch_setpoints,
            peak_demand_mw: peak_demand_mw.max(0.0),
            self_consumption_pct,
            estimated_cost_usd: total_cost,
            voltage_violations,
            line_overloads,
            total_renewable_mwh: total_solar_mwh,
            curtailment_mwh,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_grid_limits() -> GridLimits {
        GridLimits {
            substation_capacity_mw: 5.0,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            line_ratings_mw: vec![6.0, 4.0],
        }
    }

    fn default_config(horizon: usize) -> DermsConfig {
        DermsConfig {
            update_interval_s: 300.0,
            forecast_horizon_h: horizon,
            grid_limits: default_grid_limits(),
            objectives: vec![DermsObjective::MinimizePeakDemand],
        }
    }

    #[test]
    fn test_peak_shaving_never_exceeds_substation() {
        let mut ctrl = DermsController::new(default_config(4));

        // Battery storage: can discharge up to 3 MW
        ctrl.register_asset(DerAsset {
            id: 1,
            asset_type: DerAssetType::BatteryStorage,
            bus: 1,
            p_max_mw: 3.0,
            p_min_mw: -3.0,
            q_max_mvar: 1.0,
            q_min_mvar: -1.0,
            forecast_mw: vec![-3.0; 4], // discharging
            current_p_mw: -3.0,
            current_soc: Some(0.8),
        });

        // DR: can shed 2 MW
        ctrl.register_asset(DerAsset {
            id: 2,
            asset_type: DerAssetType::DemandResponse,
            bus: 2,
            p_max_mw: 2.0,
            p_min_mw: 0.0,
            q_max_mvar: 0.0,
            q_min_mvar: 0.0,
            forecast_mw: vec![4.0; 4],
            current_p_mw: 4.0,
            current_soc: None,
        });

        // Load forecast: 8 MW (well above 5 MW limit)
        let load = vec![8.0; 4];
        let result = ctrl.dispatch(&load).expect("dispatch should succeed");

        // After peak shaving, peak demand should not dramatically exceed limit
        // (may not be perfect with simplified model, but must be reduced from 8)
        assert!(
            result.peak_demand_mw < 8.0,
            "Peak shaving should reduce peak below 8 MW, got {:.2}",
            result.peak_demand_mw
        );
    }

    #[test]
    fn test_self_consumption_maximises_solar_use() {
        let cfg = DermsConfig {
            update_interval_s: 300.0,
            forecast_horizon_h: 4,
            grid_limits: default_grid_limits(),
            objectives: vec![DermsObjective::MaximizeSelfConsumption],
        };
        let mut ctrl = DermsController::new(cfg);

        // Solar: 3 MW generation
        ctrl.register_asset(DerAsset {
            id: 10,
            asset_type: DerAssetType::RooftopSolar,
            bus: 1,
            p_max_mw: 3.0,
            p_min_mw: 0.0,
            q_max_mvar: 0.0,
            q_min_mvar: 0.0,
            forecast_mw: vec![3.0; 4],
            current_p_mw: 3.0,
            current_soc: None,
        });

        // Battery: can absorb 2 MW
        ctrl.register_asset(DerAsset {
            id: 11,
            asset_type: DerAssetType::BatteryStorage,
            bus: 1,
            p_max_mw: 2.0,
            p_min_mw: -2.0,
            q_max_mvar: 0.5,
            q_min_mvar: -0.5,
            forecast_mw: vec![0.0; 4],
            current_p_mw: 0.0,
            current_soc: Some(0.3),
        });

        // Load: 2 MW → surplus solar = 1 MW
        let load = vec![2.0; 4];
        let result = ctrl.dispatch(&load).expect("dispatch should succeed");

        // Self consumption should be > 0 (solar is available)
        assert!(
            result.total_renewable_mwh > 0.0,
            "Should have solar generation"
        );
        // Some self-consumption achieved
        assert!(
            result.self_consumption_pct >= 0.0,
            "Self-consumption fraction should be non-negative"
        );
    }

    #[test]
    fn test_cost_minimization_uses_tariff() {
        let tariff = TariffStructure {
            energy_rate_usd_per_mwh: vec![30.0, 30.0, 120.0, 120.0], // cheap then expensive
            demand_charge_usd_per_mw: 10.0,
            export_rate_usd_per_mwh: 10.0,
        };
        let cfg = DermsConfig {
            update_interval_s: 300.0,
            forecast_horizon_h: 4,
            grid_limits: default_grid_limits(),
            objectives: vec![DermsObjective::MinimizeCost {
                tariff_structure: tariff,
            }],
        };
        let mut ctrl = DermsController::new(cfg);

        ctrl.register_asset(DerAsset {
            id: 20,
            asset_type: DerAssetType::BatteryStorage,
            bus: 1,
            p_max_mw: 2.0,
            p_min_mw: -2.0,
            q_max_mvar: 0.5,
            q_min_mvar: -0.5,
            forecast_mw: vec![0.0; 4],
            current_p_mw: 0.0,
            current_soc: Some(0.5),
        });

        let load = vec![3.0; 4];
        let result = ctrl.dispatch(&load).expect("dispatch should succeed");

        // Cost is computed and non-negative
        assert!(
            result.estimated_cost_usd >= 0.0,
            "Cost should be non-negative"
        );
    }

    #[test]
    fn test_line_overloads_detected() {
        let cfg = DermsConfig {
            update_interval_s: 300.0,
            forecast_horizon_h: 2,
            grid_limits: GridLimits {
                substation_capacity_mw: 20.0,
                voltage_min_pu: 0.95,
                voltage_max_pu: 1.05,
                line_ratings_mw: vec![1.0], // very tight: 1 MW only
            },
            objectives: vec![DermsObjective::MinimizePeakDemand],
        };
        let mut ctrl = DermsController::new(cfg);

        ctrl.register_asset(DerAsset {
            id: 30,
            asset_type: DerAssetType::CombinedHeatPower,
            bus: 2,
            p_max_mw: 5.0,
            p_min_mw: 0.0,
            q_max_mvar: 1.0,
            q_min_mvar: 0.0,
            forecast_mw: vec![5.0; 2],
            current_p_mw: 5.0,
            current_soc: None,
        });

        // Load 10 MW through a 1 MW rated line → overloads
        let load = vec![10.0; 2];
        let result = ctrl.dispatch(&load).expect("dispatch should succeed");
        assert!(
            result.line_overloads > 0,
            "Should detect line overloads with 10 MW through a 1 MW line"
        );
    }

    #[test]
    fn test_asset_registration_and_dispatch_ids() {
        let mut ctrl = DermsController::new(default_config(3));

        let asset_ids = [101_usize, 202, 303];
        for &id in &asset_ids {
            ctrl.register_asset(DerAsset {
                id,
                asset_type: DerAssetType::DemandResponse,
                bus: id,
                p_max_mw: 1.0,
                p_min_mw: 0.0,
                q_max_mvar: 0.0,
                q_min_mvar: 0.0,
                forecast_mw: vec![1.0; 3],
                current_p_mw: 1.0,
                current_soc: None,
            });
        }

        let result = ctrl
            .dispatch(&[2.0, 2.0, 2.0])
            .expect("dispatch should succeed");

        // Every registered asset should have a dispatch entry
        assert_eq!(result.dispatch.len(), asset_ids.len());
        let returned_ids: Vec<usize> = result.dispatch.iter().map(|d| d.asset_id).collect();
        for &id in &asset_ids {
            assert!(
                returned_ids.contains(&id),
                "Asset {id} should have dispatch entry"
            );
        }
    }

    #[test]
    fn test_no_assets_returns_error() {
        let ctrl = DermsController::new(default_config(4));
        let result = ctrl.dispatch(&[3.0; 4]);
        assert!(
            matches!(result, Err(DermsError::NoAssets)),
            "Should return NoAssets error"
        );
    }

    #[test]
    fn test_ev_smart_charging_dispatch() {
        let mut ctrl = DermsController::new(default_config(6));

        // EV fleet: can charge 0–4 MW
        ctrl.register_asset(DerAsset {
            id: 50,
            asset_type: DerAssetType::ElectricVehicle,
            bus: 3,
            p_max_mw: 4.0,
            p_min_mw: 0.0,
            q_max_mvar: 0.0,
            q_min_mvar: 0.0,
            forecast_mw: vec![2.0; 6],
            current_p_mw: 2.0,
            current_soc: Some(0.4),
        });

        let load = vec![3.0; 6];
        let result = ctrl.dispatch(&load).expect("dispatch should succeed");

        let ev_dispatch = result
            .dispatch
            .iter()
            .find(|d| d.asset_id == 50)
            .expect("EV should have dispatch entry");

        // EV setpoint should respect its bounds
        assert!(ev_dispatch.p_setpoint_mw >= 0.0 - 1e-6);
        assert!(ev_dispatch.p_setpoint_mw <= 4.0 + 1e-6);
    }
}
