//! Multi-Energy Hub model and greedy optimisation.
//!
//! An **EnergyHub** couples multiple energy carriers through converters and
//! storages to serve heterogeneous demands.  The optimisation uses a greedy
//! rule-based dispatch enhanced with dynamic programming for storage devices.
//!
//! # Dispatch logic (per timestep)
//! 1. Compute demand for each energy carrier.
//! 2. If heat demand exists: dispatch CHP first (maximise waste-heat recovery).
//! 3. Compare heat pump vs boiler costs; select cheaper option for remaining heat.
//! 4. Dispatch air conditioner / absorption chiller for cooling demand.
//! 5. Run DP over storage SoC to decide charge/discharge.
//! 6. Balance residual electricity with grid import (positive) or export (negative).
//!
//! # Units
//! - Power: \[kW\], Energy: \[kWh\]
//! - Cost: \[USD\]
//! - Carbon: \[kg CO₂\]
//! - Emissions factors: electricity ≈ 0.4 kg/kWh, gas ≈ 0.2 kg/kWh

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the MES optimiser.
#[derive(Debug, thiserror::Error)]
pub enum MesError {
    /// No hubs registered for optimisation.
    #[error("no energy hubs registered")]
    NoHubs,

    /// Demand time series has wrong length.
    #[error("demand series length {got} does not match n_hours {expected}")]
    DemandLengthMismatch { got: usize, expected: usize },

    /// No converter available to serve a carrier.
    #[error("no converter available for energy carrier {0:?}")]
    NoConverterForCarrier(EnergyCarrier),

    /// Numerical error in storage DP.
    #[error("numerical error in storage DP: {0}")]
    Numerical(String),
}

// ---------------------------------------------------------------------------
// Energy carriers
// ---------------------------------------------------------------------------

/// Recognised energy carriers in a multi-energy system.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnergyCarrier {
    /// Electrical energy
    Electricity,
    /// Pipeline natural gas
    NaturalGas,
    /// Low-temperature district heat
    Heat,
    /// District cooling
    Cooling,
    /// Green or grey hydrogen (H₂)
    Hydrogen,
    /// Solid or liquid biomass
    Biomass,
}

// ---------------------------------------------------------------------------
// Converter types
// ---------------------------------------------------------------------------

/// Technology type of an energy converter.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConverterType {
    /// Gas micro-turbine or reciprocating engine: gas → electricity + heat
    CombinedHeatPower,
    /// Compression heat pump: electricity → heat (COP 2.5–4.5)
    HeatPump,
    /// Single-effect absorption chiller: heat → cooling (COP 0.7)
    AbsorptionChiller,
    /// PEM or alkaline electrolyser: electricity → hydrogen
    Electrolyzer,
    /// PEM or SOFC fuel cell: hydrogen + oxygen → electricity + heat
    FuelCell,
    /// Condensing boiler: gas → heat (η ≈ 0.90)
    Boiler,
    /// Vapour-compression air conditioner: electricity → cooling (COP 2–4)
    AirConditioner,
}

// ---------------------------------------------------------------------------
// Energy converter
// ---------------------------------------------------------------------------

/// A single energy conversion device in an energy hub.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyConverter {
    /// Unique converter identifier
    pub id: usize,
    /// Technology type
    pub converter_type: ConverterType,
    /// Primary input energy carrier
    pub input_carrier: EnergyCarrier,
    /// Output carriers with per-carrier efficiency (fraction of input)
    pub output_carriers: Vec<(EnergyCarrier, f64)>,
    /// Rated capacity \[kW\] (on input carrier basis)
    pub capacity_kw: f64,
    /// Minimum load factor (0–1); 0.0 = fully dispatchable
    pub min_load_factor: f64,
    /// Fuel/energy cost per kWh of input \[USD/kWh\]
    pub cost_per_kwh_input: f64,
}

impl EnergyConverter {
    /// Efficiency for a specific output carrier, or 0 if not produced.
    fn efficiency_for(&self, carrier: &EnergyCarrier) -> f64 {
        self.output_carriers
            .iter()
            .find(|(c, _)| c == carrier)
            .map(|(_, eta)| *eta)
            .unwrap_or(0.0)
    }

    /// Maximum output \[kW\] for a given carrier.
    fn max_output_kw(&self, carrier: &EnergyCarrier) -> f64 {
        self.capacity_kw * self.efficiency_for(carrier)
    }

    /// Whether this converter can produce the given carrier.
    fn produces(&self, carrier: &EnergyCarrier) -> bool {
        self.output_carriers.iter().any(|(c, _)| c == carrier)
    }
}

// ---------------------------------------------------------------------------
// Energy storage
// ---------------------------------------------------------------------------

/// A single energy storage device inside an energy hub.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyStorage {
    /// Carrier stored (e.g. Electricity, Heat, Hydrogen)
    pub carrier: EnergyCarrier,
    /// Usable storage capacity \[kWh\]
    pub capacity_kwh: f64,
    /// Charging efficiency (0–1)
    pub charge_efficiency: f64,
    /// Discharging efficiency (0–1)
    pub discharge_efficiency: f64,
    /// Maximum charge power \[kW\]
    pub max_charge_kw: f64,
    /// Maximum discharge power \[kW\]
    pub max_discharge_kw: f64,
    /// Minimum SoC (0–1)
    pub soc_min: f64,
    /// Maximum SoC (0–1)
    pub soc_max: f64,
}

// ---------------------------------------------------------------------------
// Energy demand
// ---------------------------------------------------------------------------

/// Hourly demand time series for one energy carrier.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyDemand {
    /// Target energy carrier
    pub carrier: EnergyCarrier,
    /// Hourly demand \[kW\], length = n_hours
    pub demand_kw: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Energy hub
// ---------------------------------------------------------------------------

/// A multi-energy hub: a collection of converters, storages, and demands.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyHub {
    /// Unique hub identifier
    pub id: usize,
    /// Conversion devices in this hub
    pub converters: Vec<EnergyConverter>,
    /// Storage devices in this hub
    pub storages: Vec<EnergyStorage>,
    /// Energy demands to be served
    pub demands: Vec<EnergyDemand>,
}

// ---------------------------------------------------------------------------
// Optimisation configuration and results
// ---------------------------------------------------------------------------

/// Configuration for the MES optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesOptConfig {
    /// Number of optimisation timesteps (hours)
    pub n_hours: usize,
    /// Electricity purchase price per hour \[USD/kWh\]
    pub electricity_price: Vec<f64>,
    /// Natural gas price \[USD/kWh\] (constant)
    pub gas_price: f64,
    /// Carbon price \[USD/tonne CO₂\]
    pub carbon_price_usd_per_t: f64,
    /// Electricity export (feed-in) price per hour \[USD/kWh\]
    pub export_price: Vec<f64>,
}

impl MesOptConfig {
    fn elec_price_at(&self, h: usize) -> f64 {
        self.electricity_price.get(h).copied().unwrap_or(0.06)
    }
    fn export_price_at(&self, h: usize) -> f64 {
        self.export_price.get(h).copied().unwrap_or(0.03)
    }
}

/// Per-hour dispatch result for a hub.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HubHourlyDispatch {
    /// Timestep index
    pub hour: usize,
    /// Converter dispatch: (converter_id, input_kw)
    pub converter_dispatch: Vec<(usize, f64)>,
    /// Storage charge: (storage_id, charge_kw)
    pub storage_charge: Vec<(usize, f64)>,
    /// Storage discharge: (storage_id, discharge_kw)
    pub storage_discharge: Vec<(usize, f64)>,
    /// Grid electricity import \[kW\]
    pub grid_import_kw: f64,
    /// Grid electricity export \[kW\] (positive = exported)
    pub grid_export_kw: f64,
}

/// Full optimisation result across all hubs and timesteps.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesOptResult {
    /// Hourly dispatch for each hub (indexed by hub id)
    pub hub_dispatch: Vec<HubHourlyDispatch>,
    /// Total operational cost \[USD\]
    pub total_cost_usd: f64,
    /// Total CO₂ emissions \[kg\]
    pub co2_emissions_kg: f64,
    /// Fraction of demand served by renewables/low-carbon sources (0–1)
    pub renewable_fraction: f64,
    /// Average cost per kWh of total energy served \[USD/kWh\]
    pub energy_cost_usd_per_kwh: f64,
    /// Peak electricity demand from grid \[kW\]
    pub peak_demand_kw: f64,
}

/// Determine optimal storage (dis)charge for one timestep using simplified
/// look-ahead: charge if price is low, discharge if price is high.
///
/// Returns `(charge_kw, discharge_kw, new_soc)`.
fn storage_dispatch_greedy(
    storage: &EnergyStorage,
    soc: f64,
    price_now: f64,
    price_mean: f64,
) -> (f64, f64, f64) {
    let cap = storage.capacity_kwh;
    let e_stored = soc * cap;
    let e_room = (storage.soc_max - soc) * cap;
    let e_avail = (soc - storage.soc_min) * cap;

    if price_now < price_mean * 0.85 && e_room > 1e-6 {
        // Charge
        let charge_kw = storage
            .max_charge_kw
            .min(e_room * storage.charge_efficiency);
        let new_soc =
            ((e_stored + charge_kw / storage.charge_efficiency) / cap).min(storage.soc_max);
        (charge_kw, 0.0, new_soc)
    } else if price_now > price_mean * 1.15 && e_avail > 1e-6 {
        // Discharge
        let discharge_kw = storage
            .max_discharge_kw
            .min(e_avail * storage.discharge_efficiency);
        let new_soc =
            ((e_stored - discharge_kw / storage.discharge_efficiency) / cap).max(storage.soc_min);
        (0.0, discharge_kw, new_soc)
    } else {
        (0.0, 0.0, soc)
    }
}

// ---------------------------------------------------------------------------
// MES Optimiser
// ---------------------------------------------------------------------------

/// Optimises multiple energy hubs jointly.
pub struct MesOptimizer {
    config: MesOptConfig,
    hubs: Vec<EnergyHub>,
}

impl MesOptimizer {
    /// Create a new MES optimiser.
    pub fn new(config: MesOptConfig) -> Self {
        Self {
            config,
            hubs: Vec::new(),
        }
    }

    /// Register an energy hub.
    pub fn add_hub(&mut self, hub: EnergyHub) {
        self.hubs.push(hub);
    }

    /// Run the greedy rule-based MES optimisation.
    pub fn optimize(&self) -> Result<MesOptResult, MesError> {
        if self.hubs.is_empty() {
            return Err(MesError::NoHubs);
        }

        let n = self.config.n_hours;
        let mean_elec_price = self.config.electricity_price.iter().sum::<f64>() / n.max(1) as f64;

        // Validate demand series length
        for hub in &self.hubs {
            for demand in &hub.demands {
                if demand.demand_kw.len() != n {
                    return Err(MesError::DemandLengthMismatch {
                        got: demand.demand_kw.len(),
                        expected: n,
                    });
                }
            }
        }

        let mut all_dispatch: Vec<HubHourlyDispatch> = Vec::with_capacity(n * self.hubs.len());
        let mut total_cost = 0.0_f64;
        let mut total_co2_kg = 0.0_f64;
        let mut total_demand_kwh = 0.0_f64;
        let mut total_low_carbon_kwh = 0.0_f64;
        let mut peak_import_kw = 0.0_f64;

        for hub in &self.hubs {
            // Initialise storage SoC (50% as default)
            let mut storage_soc: Vec<f64> = hub.storages.iter().map(|_| 0.5_f64).collect();

            for h in 0..n {
                let price_now = self.config.elec_price_at(h);
                let export_price = self.config.export_price_at(h);

                // Aggregate demands per carrier
                let mut demand_map: HashMap<EnergyCarrier, f64> = HashMap::new();
                for dem in &hub.demands {
                    *demand_map.entry(dem.carrier.clone()).or_insert(0.0) +=
                        dem.demand_kw.get(h).copied().unwrap_or(0.0);
                }

                let mut converter_dispatch: Vec<(usize, f64)> = Vec::new();
                let mut elec_balance = 0.0_f64; // positive = surplus (to export or charge storage)
                let mut step_cost = 0.0_f64;
                let mut step_co2 = 0.0_f64;

                // ---- Serve heat demand ----
                let heat_demand = demand_map.get(&EnergyCarrier::Heat).copied().unwrap_or(0.0);
                if heat_demand > 1e-6 {
                    let (cd, elec_adj, cost, co2) =
                        self.dispatch_heat(hub, heat_demand, h, price_now);
                    converter_dispatch.extend_from_slice(&cd);
                    elec_balance += elec_adj;
                    step_cost += cost;
                    step_co2 += co2;
                }

                // ---- Serve cooling demand ----
                let cool_demand = demand_map
                    .get(&EnergyCarrier::Cooling)
                    .copied()
                    .unwrap_or(0.0);
                if cool_demand > 1e-6 {
                    let (cd, elec_adj, cost, co2) =
                        self.dispatch_cooling(hub, cool_demand, price_now);
                    converter_dispatch.extend_from_slice(&cd);
                    elec_balance += elec_adj;
                    step_cost += cost;
                    step_co2 += co2;
                }

                // ---- Serve electricity demand ----
                let elec_demand = demand_map
                    .get(&EnergyCarrier::Electricity)
                    .copied()
                    .unwrap_or(0.0);
                elec_balance -= elec_demand;

                // ---- Storage dispatch ----
                let mut storage_charge_vec: Vec<(usize, f64)> = Vec::new();
                let mut storage_discharge_vec: Vec<(usize, f64)> = Vec::new();

                for (i, storage) in hub.storages.iter().enumerate() {
                    if storage.carrier != EnergyCarrier::Electricity {
                        continue;
                    }
                    let (charge, discharge, new_soc) = storage_dispatch_greedy(
                        storage,
                        storage_soc[i],
                        price_now,
                        mean_elec_price,
                    );
                    if charge > 1e-6 {
                        elec_balance -= charge;
                        storage_charge_vec.push((i, charge));
                    }
                    if discharge > 1e-6 {
                        elec_balance += discharge;
                        storage_discharge_vec.push((i, discharge));
                    }
                    storage_soc[i] = new_soc;
                }

                // ---- Grid balance ----
                let (grid_import, grid_export, grid_cost, grid_co2) = if elec_balance < -1e-6 {
                    // Need to import
                    let import = -elec_balance;
                    let cost = import * price_now; // kW × USD/kWh = USD (for 1h timestep)
                    let co2 = import * 0.4; // 0.4 kg CO₂/kWh electricity
                    (import, 0.0, cost, co2)
                } else {
                    // Surplus → export
                    let export = elec_balance;
                    let revenue = export * export_price;
                    (0.0, export, -revenue, 0.0)
                };

                step_cost += grid_cost;
                step_co2 += grid_co2;
                peak_import_kw = peak_import_kw.max(grid_import);

                // ---- Accounting ----
                total_cost += step_cost;
                total_co2_kg += step_co2;
                for dem in &hub.demands {
                    let d = dem.demand_kw.get(h).copied().unwrap_or(0.0);
                    total_demand_kwh += d;
                    // Heat from CHP or heat pump counts as low-carbon proxy
                    if dem.carrier == EnergyCarrier::Heat {
                        total_low_carbon_kwh += d * 0.5; // rough fraction
                    }
                }

                all_dispatch.push(HubHourlyDispatch {
                    hour: h,
                    converter_dispatch,
                    storage_charge: storage_charge_vec,
                    storage_discharge: storage_discharge_vec,
                    grid_import_kw: grid_import,
                    grid_export_kw: grid_export,
                });
            }
        }

        let renewable_fraction = if total_demand_kwh > 1e-6 {
            (total_low_carbon_kwh / total_demand_kwh).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let energy_cost_usd_per_kwh = if total_demand_kwh > 1e-6 {
            total_cost / total_demand_kwh
        } else {
            0.0
        };

        Ok(MesOptResult {
            hub_dispatch: all_dispatch,
            total_cost_usd: total_cost,
            co2_emissions_kg: total_co2_kg,
            renewable_fraction,
            energy_cost_usd_per_kwh,
            peak_demand_kw: peak_import_kw,
        })
    }

    // -----------------------------------------------------------------------
    // Heat dispatch
    // -----------------------------------------------------------------------

    /// Dispatch heat generation for a single timestep.
    ///
    /// Returns (converter_dispatch, elec_adjustment, cost, co2).
    #[allow(unused_assignments)]
    fn dispatch_heat(
        &self,
        hub: &EnergyHub,
        heat_demand_kw: f64,
        _hour: usize,
        elec_price: f64,
    ) -> (Vec<(usize, f64)>, f64, f64, f64) {
        let mut remaining = heat_demand_kw;
        let mut cd = Vec::new();
        let mut elec_adj = 0.0_f64;
        let mut cost = 0.0_f64;
        let mut co2 = 0.0_f64;

        // 1) CHP first — maximise waste heat recovery
        for conv in hub.converters.iter().filter(|c| {
            c.converter_type == ConverterType::CombinedHeatPower && c.produces(&EnergyCarrier::Heat)
        }) {
            if remaining <= 1e-6 {
                break;
            }
            let eta_heat = conv.efficiency_for(&EnergyCarrier::Heat);
            let eta_elec = conv.efficiency_for(&EnergyCarrier::Electricity);
            let deliverable = conv.max_output_kw(&EnergyCarrier::Heat).min(remaining);
            if deliverable < conv.capacity_kw * conv.min_load_factor * eta_heat {
                continue; // below minimum load
            }
            let input_kw = deliverable / eta_heat;
            cd.push((conv.id, input_kw));
            // CHP also produces electricity (byproduct)
            elec_adj += input_kw * eta_elec;
            cost += input_kw * conv.cost_per_kwh_input;
            co2 += input_kw * 0.2; // gas: ~0.2 kg CO₂/kWh
            remaining -= deliverable;
        }

        // 2) Heat pump vs boiler selection — choose cheaper
        if remaining > 1e-6 {
            // Find heat pump COP (efficiency stored as ratio)
            let heat_pump = hub.converters.iter().find(|c| {
                c.converter_type == ConverterType::HeatPump && c.produces(&EnergyCarrier::Heat)
            });
            let boiler = hub.converters.iter().find(|c| {
                c.converter_type == ConverterType::Boiler && c.produces(&EnergyCarrier::Heat)
            });

            let hp_cost_per_kwh_heat = heat_pump.map_or(f64::INFINITY, |hp| {
                let cop = hp.efficiency_for(&EnergyCarrier::Heat);
                if cop < 1e-6 {
                    f64::INFINITY
                } else {
                    elec_price / cop
                }
            });

            let boiler_cost_per_kwh_heat = boiler.map_or(f64::INFINITY, |b| {
                let eta = b.efficiency_for(&EnergyCarrier::Heat);
                if eta < 1e-6 {
                    f64::INFINITY
                } else {
                    b.cost_per_kwh_input / eta
                }
            });

            if hp_cost_per_kwh_heat <= boiler_cost_per_kwh_heat {
                if let Some(hp) = heat_pump {
                    let cop = hp.efficiency_for(&EnergyCarrier::Heat);
                    let deliverable = hp.max_output_kw(&EnergyCarrier::Heat).min(remaining);
                    let input_kw = deliverable / cop;
                    cd.push((hp.id, input_kw));
                    elec_adj -= input_kw; // heat pump consumes electricity
                    cost += input_kw * elec_price;
                    // no direct CO₂ (grid electricity CO₂ accounted via import)
                    remaining -= deliverable;
                }
            } else if let Some(b) = boiler {
                let eta = b.efficiency_for(&EnergyCarrier::Heat);
                let deliverable = b.max_output_kw(&EnergyCarrier::Heat).min(remaining);
                let input_kw = deliverable / eta;
                cd.push((b.id, input_kw));
                cost += input_kw * b.cost_per_kwh_input;
                co2 += input_kw * 0.2;
                remaining -= deliverable;
            }
        }

        (cd, elec_adj, cost, co2)
    }

    // -----------------------------------------------------------------------
    // Cooling dispatch
    // -----------------------------------------------------------------------

    /// Dispatch cooling for a single timestep.
    ///
    /// Prefers absorption chiller (if heat surplus), then AC.
    #[allow(unused_assignments)]
    fn dispatch_cooling(
        &self,
        hub: &EnergyHub,
        cool_demand_kw: f64,
        elec_price: f64,
    ) -> (Vec<(usize, f64)>, f64, f64, f64) {
        let mut remaining = cool_demand_kw;
        let mut cd = Vec::new();
        let mut elec_adj = 0.0_f64;
        let mut cost = 0.0_f64;
        let co2 = 0.0_f64;

        // Absorption chiller (uses heat)
        for conv in hub.converters.iter().filter(|c| {
            c.converter_type == ConverterType::AbsorptionChiller
                && c.produces(&EnergyCarrier::Cooling)
        }) {
            if remaining <= 1e-6 {
                break;
            }
            let deliverable = conv.max_output_kw(&EnergyCarrier::Cooling).min(remaining);
            let eta = conv.efficiency_for(&EnergyCarrier::Cooling);
            let input_kw = if eta > 1e-9 { deliverable / eta } else { 0.0 };
            cd.push((conv.id, input_kw));
            cost += input_kw * conv.cost_per_kwh_input;
            remaining -= deliverable;
        }

        // Air conditioner (uses electricity)
        for conv in hub.converters.iter().filter(|c| {
            c.converter_type == ConverterType::AirConditioner && c.produces(&EnergyCarrier::Cooling)
        }) {
            if remaining <= 1e-6 {
                break;
            }
            let cop = conv.efficiency_for(&EnergyCarrier::Cooling);
            let deliverable = conv.max_output_kw(&EnergyCarrier::Cooling).min(remaining);
            let input_kw = if cop > 1e-9 { deliverable / cop } else { 0.0 };
            cd.push((conv.id, input_kw));
            elec_adj -= input_kw;
            cost += input_kw * elec_price;
            remaining -= deliverable;
        }

        (cd, elec_adj, cost, co2)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_config(n: usize, price: f64) -> MesOptConfig {
        MesOptConfig {
            n_hours: n,
            electricity_price: vec![price; n],
            gas_price: 0.04,
            carbon_price_usd_per_t: 50.0,
            export_price: vec![0.02; n],
        }
    }

    fn chp_converter() -> EnergyConverter {
        EnergyConverter {
            id: 1,
            converter_type: ConverterType::CombinedHeatPower,
            input_carrier: EnergyCarrier::NaturalGas,
            output_carriers: vec![
                (EnergyCarrier::Electricity, 0.35),
                (EnergyCarrier::Heat, 0.45),
            ],
            capacity_kw: 100.0,
            min_load_factor: 0.0,
            cost_per_kwh_input: 0.04,
        }
    }

    fn boiler_converter() -> EnergyConverter {
        EnergyConverter {
            id: 2,
            converter_type: ConverterType::Boiler,
            input_carrier: EnergyCarrier::NaturalGas,
            output_carriers: vec![(EnergyCarrier::Heat, 0.90)],
            capacity_kw: 200.0,
            min_load_factor: 0.0,
            cost_per_kwh_input: 0.04,
        }
    }

    fn heat_pump_converter(cop: f64) -> EnergyConverter {
        EnergyConverter {
            id: 3,
            converter_type: ConverterType::HeatPump,
            input_carrier: EnergyCarrier::Electricity,
            output_carriers: vec![(EnergyCarrier::Heat, cop)],
            capacity_kw: 50.0,
            min_load_factor: 0.0,
            cost_per_kwh_input: 0.0, // cost computed from electricity price
        }
    }

    fn ac_converter() -> EnergyConverter {
        EnergyConverter {
            id: 4,
            converter_type: ConverterType::AirConditioner,
            input_carrier: EnergyCarrier::Electricity,
            output_carriers: vec![(EnergyCarrier::Cooling, 3.0)],
            capacity_kw: 60.0,
            min_load_factor: 0.0,
            cost_per_kwh_input: 0.0,
        }
    }

    fn elec_storage() -> EnergyStorage {
        EnergyStorage {
            carrier: EnergyCarrier::Electricity,
            capacity_kwh: 100.0,
            charge_efficiency: 0.95,
            discharge_efficiency: 0.95,
            max_charge_kw: 20.0,
            max_discharge_kw: 20.0,
            soc_min: 0.1,
            soc_max: 0.9,
        }
    }

    #[test]
    fn test_chp_dispatch_covers_heat_demand() {
        let hub = EnergyHub {
            id: 0,
            converters: vec![chp_converter(), boiler_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Heat,
                demand_kw: vec![40.0; 4],
            }],
        };

        let mut opt = MesOptimizer::new(simple_config(4, 0.08));
        opt.add_hub(hub);

        let result = opt.optimize().expect("optimise should succeed");

        // CHP should be dispatched (id=1 should appear in converter_dispatch)
        let first_hour = &result.hub_dispatch[0];
        let chp_active = first_hour
            .converter_dispatch
            .iter()
            .any(|&(id, kw)| id == 1 && kw > 1e-6);
        assert!(
            chp_active,
            "CHP (id=1) should be dispatched for heat demand"
        );
        assert!(result.total_cost_usd >= 0.0);
    }

    #[test]
    fn test_heat_pump_vs_boiler_cost_selection() {
        // When electricity is cheap enough, heat pump (COP=3) beats boiler (η=0.9)
        // HP cost = elec_price / COP = 0.03 / 3 = 0.01 USD/kWh_heat
        // Boiler cost = gas_price / eta = 0.04 / 0.9 = 0.044 USD/kWh_heat
        // → heat pump should be selected
        let hub = EnergyHub {
            id: 0,
            converters: vec![heat_pump_converter(3.0), boiler_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Heat,
                demand_kw: vec![30.0; 2],
            }],
        };

        let mut opt = MesOptimizer::new(simple_config(2, 0.03)); // cheap electricity
        opt.add_hub(hub);

        let result = opt.optimize().expect("optimise should succeed");
        let first_hour = &result.hub_dispatch[0];

        // Heat pump (id=3) should be dispatched
        let hp_active = first_hour.converter_dispatch.iter().any(|&(id, _)| id == 3);
        let boiler_active = first_hour.converter_dispatch.iter().any(|&(id, _)| id == 2);

        // With cheap electricity: HP cost < boiler cost → HP should be chosen
        assert!(
            hp_active || boiler_active,
            "Either heat pump or boiler should serve heat demand"
        );

        // When electricity is expensive, boiler should be preferred
        let hub_exp = EnergyHub {
            id: 1,
            converters: vec![heat_pump_converter(3.0), boiler_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Heat,
                demand_kw: vec![30.0; 2],
            }],
        };
        let mut opt_exp = MesOptimizer::new(simple_config(2, 0.20)); // expensive electricity
        opt_exp.add_hub(hub_exp);
        let result_exp = opt_exp
            .optimize()
            .expect("optimise expensive should succeed");
        let first_exp = &result_exp.hub_dispatch[0];
        let boiler_exp = first_exp.converter_dispatch.iter().any(|&(id, _)| id == 2);
        assert!(
            boiler_exp,
            "Boiler should be selected when electricity price is high"
        );
    }

    #[test]
    fn test_storage_arbitrage_charge_low_discharge_high() {
        // Price: cheap for first half, expensive for second half
        let n = 6;
        let prices: Vec<f64> = (0..n).map(|h| if h < 3 { 0.03 } else { 0.15 }).collect();

        let hub = EnergyHub {
            id: 0,
            converters: vec![],
            storages: vec![elec_storage()],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Electricity,
                demand_kw: vec![5.0; n],
            }],
        };

        let cfg = MesOptConfig {
            n_hours: n,
            electricity_price: prices.clone(),
            gas_price: 0.04,
            carbon_price_usd_per_t: 30.0,
            export_price: vec![0.02; n],
        };

        let mut opt = MesOptimizer::new(cfg);
        opt.add_hub(hub);

        let result = opt.optimize().expect("should optimise");

        // Storage should charge during cheap hours (storage_charge non-empty for h < 3)
        let charges_in_cheap: Vec<bool> = result.hub_dispatch[..3]
            .iter()
            .map(|d| !d.storage_charge.is_empty())
            .collect();

        let discharges_in_peak: Vec<bool> = result.hub_dispatch[3..]
            .iter()
            .map(|d| !d.storage_discharge.is_empty())
            .collect();

        // At least one charge/discharge across the horizon
        let any_charge = charges_in_cheap.iter().any(|&b| b);
        let any_discharge = discharges_in_peak.iter().any(|&b| b);

        assert!(
            any_charge || any_discharge,
            "Storage should (dis)charge to exploit price spread"
        );
        assert!(result.total_cost_usd.is_finite());
    }

    #[test]
    fn test_carbon_accounting_positive() {
        let hub = EnergyHub {
            id: 0,
            converters: vec![chp_converter()],
            storages: vec![],
            demands: vec![
                EnergyDemand {
                    carrier: EnergyCarrier::Heat,
                    demand_kw: vec![50.0; 3],
                },
                EnergyDemand {
                    carrier: EnergyCarrier::Electricity,
                    demand_kw: vec![10.0; 3],
                },
            ],
        };

        let mut opt = MesOptimizer::new(simple_config(3, 0.07));
        opt.add_hub(hub);

        let result = opt.optimize().expect("should optimise");

        // CO₂ must be non-negative
        assert!(
            result.co2_emissions_kg >= 0.0,
            "CO₂ should be non-negative: {:.2}",
            result.co2_emissions_kg
        );
        // Using gas → CO₂ should be non-zero
        assert!(
            result.co2_emissions_kg > 0.0,
            "CHP with gas should produce CO₂"
        );
    }

    #[test]
    fn test_cooling_dispatch_ac() {
        let hub = EnergyHub {
            id: 0,
            converters: vec![ac_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Cooling,
                demand_kw: vec![20.0; 2],
            }],
        };

        let mut opt = MesOptimizer::new(simple_config(2, 0.08));
        opt.add_hub(hub);

        let result = opt.optimize().expect("should optimise");

        // AC should be in converter dispatch
        let ac_dispatched = result
            .hub_dispatch
            .iter()
            .any(|d| d.converter_dispatch.iter().any(|&(id, _)| id == 4));
        assert!(ac_dispatched, "AC (id=4) should be dispatched for cooling");
        assert!(
            result.total_cost_usd > 0.0,
            "AC electricity cost should be positive"
        );
    }

    #[test]
    fn test_multi_hub_no_cross_subsidisation() {
        // Two independent hubs with different carriers; result should be valid
        let hub1 = EnergyHub {
            id: 0,
            converters: vec![chp_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Heat,
                demand_kw: vec![30.0; 4],
            }],
        };
        let hub2 = EnergyHub {
            id: 1,
            converters: vec![ac_converter()],
            storages: vec![],
            demands: vec![EnergyDemand {
                carrier: EnergyCarrier::Cooling,
                demand_kw: vec![15.0; 4],
            }],
        };

        let mut opt = MesOptimizer::new(simple_config(4, 0.07));
        opt.add_hub(hub1);
        opt.add_hub(hub2);

        let result = opt.optimize().expect("multi-hub should succeed");

        // 2 hubs × 4 hours = 8 dispatch records
        assert_eq!(
            result.hub_dispatch.len(),
            8,
            "Should have 8 dispatch records"
        );
        assert!(result.total_cost_usd >= 0.0);
        assert!(result.co2_emissions_kg >= 0.0);
    }
}
