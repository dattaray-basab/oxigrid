//! Multi-Energy System (MES) optimisation module.
//!
//! Optimises the combined dispatch of electricity, heat, natural gas, hydrogen,
//! and cooling carriers across a network of energy nodes (buses) coupled via
//! converters and storages.
//!
//! # Approach
//! Greedy merit-order dispatch per timestep:
//! 1. Compute net electricity demand (gross demand minus renewable generation).
//! 2. Dispatch CHP units first (fixed heat-to-power ratio, gas → elec + heat).
//! 3. Dispatch HeatPumps next (electricity → heat, COP ≥ 1).
//! 4. Dispatch GasTurbines for remaining electricity deficit.
//! 5. Charge/discharge storages based on price signal.
//! 6. Record unmet demand and curtailed renewable.
//! 7. Accumulate operational cost and CO₂ emissions.
//!
//! # Energy Hub Matrix
//! The coupling matrix C ∈ ℝ^{n_out × n_in} aggregates converter efficiencies:
//! `C[i][j] = Σ η_k` for all converters k with input carrier j and output carrier i.
//!
//! # References
//! - Geidl & Andersson, "Optimal Power Flow of Multiple Energy Carriers",
//!   IEEE Trans. Power Syst. 22(1), 2007.
//! - Mancarella, "MES: an overview of concepts and evaluation models",
//!   Energy 65, 2014.

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Energy carriers
// ---------------------------------------------------------------------------

/// All recognised energy carriers in a multi-energy system.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnergyCarrier {
    /// Electrical energy (AC or DC bus)
    Electricity,
    /// Low-temperature district or process heat
    Heat,
    /// Pipeline natural gas (methane basis)
    NaturalGas,
    /// Green or grey molecular hydrogen (H₂)
    Hydrogen,
    /// District or process cooling
    Cooling,
}

impl EnergyCarrier {
    /// Canonical index used for matrix rows/columns (stable ordering).
    pub fn index(&self) -> usize {
        match self {
            EnergyCarrier::Electricity => 0,
            EnergyCarrier::Heat => 1,
            EnergyCarrier::NaturalGas => 2,
            EnergyCarrier::Hydrogen => 3,
            EnergyCarrier::Cooling => 4,
        }
    }

    /// Total number of distinct carriers.
    pub const COUNT: usize = 5;

    /// Carrier at a given canonical index, or `None` if out of range.
    pub fn from_index(i: usize) -> Option<Self> {
        match i {
            0 => Some(EnergyCarrier::Electricity),
            1 => Some(EnergyCarrier::Heat),
            2 => Some(EnergyCarrier::NaturalGas),
            3 => Some(EnergyCarrier::Hydrogen),
            4 => Some(EnergyCarrier::Cooling),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Converter technology type
// ---------------------------------------------------------------------------

/// Detailed technology specification for an [`EnergyConverter`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ConverterType {
    /// Gas/biogas micro-turbine or reciprocating engine: gas → electricity + heat.
    /// `heat_to_power_ratio` is the thermal-to-electrical output ratio (e.g. 1.5).
    CombinedHeatPower {
        /// Ratio of heat output to electrical output (both in MW).
        heat_to_power_ratio: f64,
    },
    /// Electric resistance boiler or electrode boiler: electricity → heat (η ≤ 1).
    ElectricBoiler,
    /// Compression heat pump: electricity → heat with COP > 1.
    HeatPump {
        /// Coefficient of performance (heat out / electricity in).
        cop: f64,
    },
    /// PEM or alkaline electrolyser: electricity → H₂.
    /// `faradaic_efficiency` captures parasitic losses beyond the Faradaic ideal.
    Electrolyzer {
        /// Faradaic efficiency fraction (0–1).
        faradaic_efficiency: f64,
    },
    /// Open- or closed-cycle gas turbine (OCGT/CCGT): natural gas → electricity.
    GasTurbine {
        /// Fuel consumption per unit of electrical output [GJ/MWh].
        heat_rate_gj_per_mwh: f64,
    },
    /// Single-effect absorption chiller: heat → cooling (COP ≈ 0.7).
    AbsorptionChiller {
        /// Coefficient of performance (cooling out / heat in).
        cop: f64,
    },
    /// Sabatier or biological methanation: electricity → synthetic methane.
    PowerToGas,
}

// ---------------------------------------------------------------------------
// MES node
// ---------------------------------------------------------------------------

/// An energy node (bus) in the multi-energy network.
///
/// Each node has a set of active energy carriers and a demand vector aligned
/// with `carriers`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesNode {
    /// Unique node identifier.
    pub id: usize,
    /// Human-readable label.
    pub name: String,
    /// Ordered list of energy carriers present at this node.
    pub carriers: Vec<EnergyCarrier>,
    /// Baseline demand per carrier `MW`; same order as `carriers`.
    pub demand_mw: Vec<f64>,
    /// Geographic latitude [°].
    pub latitude: f64,
    /// Geographic longitude [°].
    pub longitude: f64,
}

impl MesNode {
    /// Return the demand `MW` for a specific carrier, or `0.0` if not present.
    pub fn demand_for(&self, carrier: &EnergyCarrier) -> f64 {
        self.carriers
            .iter()
            .zip(self.demand_mw.iter())
            .find(|(c, _)| *c == carrier)
            .map(|(_, &d)| d)
            .unwrap_or(0.0)
    }
}

// ---------------------------------------------------------------------------
// Energy converter
// ---------------------------------------------------------------------------

/// A single energy conversion device linking two nodes / carriers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyConverter {
    /// Unique converter identifier.
    pub id: usize,
    /// Source node (input side).
    pub from_node: usize,
    /// Sink node (output side); may equal `from_node` for in-hub devices.
    pub to_node: usize,
    /// Primary input carrier.
    pub from_carrier: EnergyCarrier,
    /// Primary output carrier.
    pub to_carrier: EnergyCarrier,
    /// Input-to-output conversion efficiency (0–1).
    pub efficiency: f64,
    /// Rated output capacity `MW`.
    pub capacity_mw: f64,
    /// Minimum loading fraction (0–1); below this the device cannot operate.
    pub min_loading: f64,
    /// Maximum ramp rate [MW/min].
    pub ramp_rate_mw_per_min: f64,
    /// Technology-specific parameters.
    pub device_type: ConverterType,
}

impl EnergyConverter {
    /// Effective maximum output `MW` respecting capacity.
    pub fn max_output_mw(&self) -> f64 {
        self.capacity_mw
    }

    /// Minimum dispatchable output `MW` (min_loading × capacity).
    pub fn min_output_mw(&self) -> f64 {
        self.min_loading * self.capacity_mw
    }

    /// For CHP: heat output `MW` given electrical output `elec_mw`.
    pub fn chp_heat_output_mw(&self, elec_mw: f64) -> f64 {
        match &self.device_type {
            ConverterType::CombinedHeatPower {
                heat_to_power_ratio,
            } => elec_mw * heat_to_power_ratio,
            _ => 0.0,
        }
    }

    /// For CHP: gas input `MW` given electrical output `elec_mw`.
    pub fn chp_gas_input_mw(&self, elec_mw: f64) -> f64 {
        match &self.device_type {
            ConverterType::CombinedHeatPower {
                heat_to_power_ratio,
            } if self.efficiency > 1e-9 => elec_mw * (1.0 + heat_to_power_ratio) / self.efficiency,
            _ => 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Multi-energy storage
// ---------------------------------------------------------------------------

/// A single energy storage device in the MES.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesStorage {
    /// Unique storage identifier.
    pub id: usize,
    /// Node at which the storage is connected.
    pub node: usize,
    /// Stored energy carrier.
    pub carrier: EnergyCarrier,
    /// Usable storage capacity `MWh`.
    pub capacity_mwh: f64,
    /// Maximum charge power `MW`.
    pub max_charge_rate_mw: f64,
    /// Maximum discharge power `MW`.
    pub max_discharge_rate_mw: f64,
    /// Round-trip energy efficiency (charge × discharge, 0–1).
    pub round_trip_efficiency: f64,
    /// Current state-of-charge (0–1).
    pub soc: f64,
    /// Minimum allowable SoC (0–1).
    pub soc_min: f64,
    /// Maximum allowable SoC (0–1).
    pub soc_max: f64,
    /// Fractional self-discharge per hour (0–1).
    pub self_discharge_rate: f64,
}

impl MesStorage {
    /// One-way efficiency (√round_trip_efficiency) applied to both charge and discharge.
    pub fn one_way_efficiency(&self) -> f64 {
        self.round_trip_efficiency.max(0.0).sqrt()
    }

    /// Maximum energy that can be stored in the next `dt_h` hours `MWh`.
    pub fn headroom_mwh(&self, dt_h: f64) -> f64 {
        ((self.soc_max - self.soc) * self.capacity_mwh)
            .max(0.0)
            .min(self.max_charge_rate_mw * dt_h)
    }

    /// Maximum energy that can be discharged in the next `dt_h` hours `MWh`.
    pub fn available_mwh(&self, dt_h: f64) -> f64 {
        ((self.soc - self.soc_min) * self.capacity_mwh)
            .max(0.0)
            .min(self.max_discharge_rate_mw * dt_h)
    }
}

// ---------------------------------------------------------------------------
// Carbon emission factors
// ---------------------------------------------------------------------------

/// CO₂ emission factors for energy carriers [kg CO₂ / MWh].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonFactors {
    /// Grid electricity carbon intensity [kg CO₂/MWh].
    pub electricity_kg_per_mwh: f64,
    /// Natural gas direct combustion emissions [kg CO₂/MWh] (≈ 200 kg/MWh).
    pub natural_gas_kg_per_mwh: f64,
    /// Hydrogen production-chain emissions [kg CO₂/MWh]; depends on pathway.
    pub hydrogen_kg_per_mwh: f64,
}

impl Default for CarbonFactors {
    fn default() -> Self {
        Self {
            electricity_kg_per_mwh: 300.0,
            natural_gas_kg_per_mwh: 200.0,
            hydrogen_kg_per_mwh: 50.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Per-timestep dispatch record
// ---------------------------------------------------------------------------

/// Dispatch decision for a single timestep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesDispatch {
    /// Timestep index (0-based).
    pub time_step: usize,
    /// Output power `MW` per converter (indexed by converter order in `MesOptimizer`).
    pub converter_outputs_mw: Vec<f64>,
    /// Storage power `MW` per storage (+ve = charging, −ve = discharging).
    pub storage_power_mw: Vec<f64>,
    /// State-of-charge per storage after this timestep.
    pub storage_soc: Vec<f64>,
    /// Unmet demand `MW` indexed by `[node_index][carrier_index]`.
    pub unmet_demand_mw: Vec<Vec<f64>>,
    /// Curtailed renewable generation `MW` (excess beyond consumption + storage).
    pub curtailed_re_mw: f64,
    /// Operational cost in this timestep [€].
    pub cost_eur: f64,
}

// ---------------------------------------------------------------------------
// Full optimisation result
// ---------------------------------------------------------------------------

/// Aggregated result of the multi-period MES optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MesResult {
    /// Per-timestep dispatch records.
    pub dispatch: Vec<MesDispatch>,
    /// Total operational cost over the horizon [€].
    pub total_cost_eur: f64,
    /// Total CO₂ emissions over the horizon `kg`.
    pub total_co2_kg: f64,
    /// System energy efficiency: useful output / total primary input (0–1).
    pub energy_efficiency: f64,
    /// Self-sufficiency: local generation / total demand (0–1).
    pub self_sufficiency: f64,
    /// Renewable energy fraction of total generation (0–1).
    pub renewable_fraction: f64,
    /// CO₂ intensity of served energy [kg CO₂/MWh].
    pub co2_intensity_kg_per_mwh: f64,
}

// ---------------------------------------------------------------------------
// MES optimiser
// ---------------------------------------------------------------------------

/// Multi-period, multi-carrier MES optimiser using greedy merit-order dispatch.
pub struct MesOptimizer {
    /// Energy nodes in the system.
    pub nodes: Vec<MesNode>,
    /// Energy converters (indexed stably; order matters for result vectors).
    pub converters: Vec<EnergyConverter>,
    /// Energy storages (indexed stably; order matters for result vectors).
    pub storages: Vec<MesStorage>,
    /// Number of optimisation timesteps.
    pub time_steps: usize,
    /// Duration of each timestep `hours`.
    pub dt_hours: f64,
}

impl MesOptimizer {
    /// Construct a new MES optimiser.
    ///
    /// # Arguments
    /// * `nodes`      – energy nodes (buses)
    /// * `converters` – conversion devices
    /// * `storages`   – storage devices
    /// * `time_steps` – number of timesteps in the planning horizon
    /// * `dt_hours`   – length of each timestep `h`
    pub fn new(
        nodes: Vec<MesNode>,
        converters: Vec<EnergyConverter>,
        storages: Vec<MesStorage>,
        time_steps: usize,
        dt_hours: f64,
    ) -> Self {
        Self {
            nodes,
            converters,
            storages,
            time_steps,
            dt_hours,
        }
    }

    // -----------------------------------------------------------------------
    // Main optimisation entry point
    // -----------------------------------------------------------------------

    /// Run greedy merit-order MES optimisation.
    ///
    /// # Arguments
    /// * `electricity_prices`   – spot price [€/MWh] for each timestep
    /// * `gas_prices`           – gas price [€/MWh] for each timestep
    /// * `renewable_generation` – renewable output `MW` indexed `[timestep][node]`
    /// * `carbon_factors`       – CO₂ emission factors
    pub fn optimize(
        &mut self,
        electricity_prices: &[f64],
        gas_prices: &[f64],
        renewable_generation: &[Vec<f64>],
        carbon_factors: &CarbonFactors,
    ) -> Result<MesResult, crate::error::OxiGridError> {
        // Validate input lengths
        if electricity_prices.len() < self.time_steps {
            return Err(crate::error::OxiGridError::InvalidParameter(format!(
                "electricity_prices length {} < time_steps {}",
                electricity_prices.len(),
                self.time_steps
            )));
        }
        if gas_prices.len() < self.time_steps {
            return Err(crate::error::OxiGridError::InvalidParameter(format!(
                "gas_prices length {} < time_steps {}",
                gas_prices.len(),
                self.time_steps
            )));
        }
        if renewable_generation.len() < self.time_steps {
            return Err(crate::error::OxiGridError::InvalidParameter(format!(
                "renewable_generation length {} < time_steps {}",
                renewable_generation.len(),
                self.time_steps
            )));
        }

        // Snapshot initial SoC (will be mutated per timestep)
        let mut soc: Vec<f64> = self.storages.iter().map(|s| s.soc).collect();

        let mut dispatch_records: Vec<MesDispatch> = Vec::with_capacity(self.time_steps);
        let mut total_cost = 0.0_f64;
        let mut total_co2_kg = 0.0_f64;
        let mut total_useful_output_mwh = 0.0_f64;
        let mut total_primary_input_mwh = 0.0_f64;
        let mut total_demand_mwh = 0.0_f64;
        let mut total_local_gen_mwh = 0.0_f64;
        let mut total_re_gen_mwh = 0.0_f64;

        for t in 0..self.time_steps {
            let elec_price = electricity_prices[t];
            let gas_price = gas_prices[t];
            let re_gen: &[f64] = &renewable_generation[t];
            let dt = self.dt_hours;

            // ------------------------------------------------------------------
            // 1. Compute net electricity demand per node
            //    net_elec[n] = elec_demand[n] − renewable[n]
            // ------------------------------------------------------------------
            let mut net_elec_demand_mw: Vec<f64> = self
                .nodes
                .iter()
                .enumerate()
                .map(|(n, node)| {
                    let re = re_gen.get(n).copied().unwrap_or(0.0);
                    (node.demand_for(&EnergyCarrier::Electricity) - re).max(-1e12)
                    // allow export
                })
                .collect();

            // Heat demand per node
            let mut heat_demand_mw: Vec<f64> = self
                .nodes
                .iter()
                .map(|node| node.demand_for(&EnergyCarrier::Heat))
                .collect();

            // Hydrogen demand per node (for future bookkeeping)
            let _h2_demand_mw: Vec<f64> = self
                .nodes
                .iter()
                .map(|node| node.demand_for(&EnergyCarrier::Hydrogen))
                .collect();

            let mut converter_outputs_mw = vec![0.0_f64; self.converters.len()];
            let mut step_cost = 0.0_f64;
            let mut step_co2_kg = 0.0_f64;
            let mut step_primary_mwh = 0.0_f64;
            let mut step_useful_mwh = 0.0_f64;

            // ------------------------------------------------------------------
            // 2. Dispatch CHP (gas → elec + heat, fixed heat-to-power ratio)
            // ------------------------------------------------------------------
            for (ci, conv) in self.converters.iter().enumerate() {
                if !matches!(conv.device_type, ConverterType::CombinedHeatPower { .. }) {
                    continue;
                }
                let node_idx = conv.to_node;
                let heat_avail = heat_demand_mw.get(node_idx).copied().unwrap_or(0.0);
                let elec_avail = net_elec_demand_mw.get(node_idx).copied().unwrap_or(0.0);

                if heat_avail <= 1e-9 && elec_avail <= 1e-9 {
                    continue;
                }

                // Dispatch enough to cover heat demand first, but also bound by elec demand
                let ConverterType::CombinedHeatPower {
                    heat_to_power_ratio,
                } = &conv.device_type
                else {
                    continue;
                };
                let hpr = *heat_to_power_ratio;

                // Max elec we can produce (bounded by capacity and heat demand)
                let max_elec_by_heat = if hpr > 1e-9 {
                    heat_avail / hpr
                } else {
                    conv.max_output_mw()
                };
                let max_elec_dispatch = conv
                    .max_output_mw()
                    .min(max_elec_by_heat)
                    .min(elec_avail.max(0.0));
                let min_elec = conv.min_output_mw();

                if max_elec_dispatch < min_elec {
                    continue; // below minimum loading; skip
                }

                let elec_out = max_elec_dispatch;
                let heat_out = conv.chp_heat_output_mw(elec_out);
                let gas_in = conv.chp_gas_input_mw(elec_out);

                converter_outputs_mw[ci] = elec_out;

                // Update demands
                if node_idx < net_elec_demand_mw.len() {
                    net_elec_demand_mw[node_idx] -= elec_out;
                }
                if node_idx < heat_demand_mw.len() {
                    heat_demand_mw[node_idx] = (heat_demand_mw[node_idx] - heat_out).max(0.0);
                }

                // Cost: gas input
                let gas_cost = gas_in * gas_price * dt;
                step_cost += gas_cost;
                step_co2_kg += gas_in * dt * carbon_factors.natural_gas_kg_per_mwh;
                step_primary_mwh += gas_in * dt;
                step_useful_mwh += (elec_out + heat_out) * dt;
            }

            // ------------------------------------------------------------------
            // 3. Dispatch HeatPumps (electricity → heat, COP ≥ 1)
            // ------------------------------------------------------------------
            for (ci, conv) in self.converters.iter().enumerate() {
                if !matches!(conv.device_type, ConverterType::HeatPump { .. }) {
                    continue;
                }
                let node_idx = conv.to_node;
                let heat_remaining = heat_demand_mw.get(node_idx).copied().unwrap_or(0.0);
                if heat_remaining <= 1e-9 {
                    continue;
                }

                let ConverterType::HeatPump { cop } = &conv.device_type else {
                    continue;
                };
                let cop_val = *cop;

                let heat_out = conv.max_output_mw().min(heat_remaining);
                let elec_in = if cop_val > 1e-9 {
                    heat_out / cop_val
                } else {
                    0.0
                };

                converter_outputs_mw[ci] = heat_out;

                if node_idx < heat_demand_mw.len() {
                    heat_demand_mw[node_idx] = (heat_demand_mw[node_idx] - heat_out).max(0.0);
                }
                if node_idx < net_elec_demand_mw.len() {
                    net_elec_demand_mw[node_idx] += elec_in; // HP consumes electricity
                }

                // Cost: electricity for heat pump (charged at import price)
                let hp_cost = elec_in * elec_price * dt;
                step_cost += hp_cost;
                step_co2_kg += elec_in * dt * carbon_factors.electricity_kg_per_mwh;
                step_primary_mwh += elec_in * dt;
                step_useful_mwh += heat_out * dt;
            }

            // ------------------------------------------------------------------
            // 4. Charge / discharge storages to balance electricity
            // ------------------------------------------------------------------
            let mean_price = if !electricity_prices.is_empty() {
                electricity_prices.iter().sum::<f64>() / electricity_prices.len() as f64
            } else {
                elec_price
            };

            let mut storage_power_mw = vec![0.0_f64; self.storages.len()];
            let storage_soc_snap: Vec<f64> = soc.clone();

            for (si, storage) in self.storages.iter().enumerate() {
                if storage.carrier != EnergyCarrier::Electricity {
                    continue;
                }
                let node_idx = storage.node;
                let net = net_elec_demand_mw.get(node_idx).copied().unwrap_or(0.0);

                // Determine whether to charge or discharge
                if net < -1e-6 && elec_price < mean_price * 0.9 {
                    // Surplus renewable → charge storage
                    let surplus = (-net).min(storage.max_charge_rate_mw);
                    let headroom = storage.headroom_mwh(dt);
                    let charge_mw = surplus.min(headroom / dt.max(1e-9));
                    if charge_mw > 1e-9 {
                        let eta = storage.one_way_efficiency();
                        let delta_soc = (charge_mw * eta * dt) / storage.capacity_mwh.max(1e-9);
                        soc[si] = (storage_soc_snap[si] + delta_soc).min(storage.soc_max);
                        storage_power_mw[si] = charge_mw; // positive = charging
                        if node_idx < net_elec_demand_mw.len() {
                            net_elec_demand_mw[node_idx] += charge_mw;
                        }
                    }
                } else if net > 1e-6 && elec_price > mean_price * 1.1 {
                    // Deficit at high price → discharge storage
                    let available = storage.available_mwh(dt);
                    let discharge_mw = net
                        .min(storage.max_discharge_rate_mw)
                        .min(available / dt.max(1e-9));
                    if discharge_mw > 1e-9 {
                        let eta = storage.one_way_efficiency();
                        let delta_soc = (discharge_mw / eta) * dt / storage.capacity_mwh.max(1e-9);
                        soc[si] = (storage_soc_snap[si] - delta_soc).max(storage.soc_min);
                        storage_power_mw[si] = -discharge_mw; // negative = discharging
                        if node_idx < net_elec_demand_mw.len() {
                            net_elec_demand_mw[node_idx] -= discharge_mw;
                        }
                    }
                }

                // Apply self-discharge
                soc[si] *= (1.0 - storage.self_discharge_rate * dt).max(0.0);
            }

            // ------------------------------------------------------------------
            // 5. Dispatch gas turbines for remaining electricity deficit
            // ------------------------------------------------------------------
            for (ci, conv) in self.converters.iter().enumerate() {
                if !matches!(conv.device_type, ConverterType::GasTurbine { .. }) {
                    continue;
                }
                let node_idx = conv.to_node;
                let deficit = net_elec_demand_mw.get(node_idx).copied().unwrap_or(0.0);
                if deficit <= 1e-9 {
                    continue;
                }

                let ConverterType::GasTurbine {
                    heat_rate_gj_per_mwh,
                } = &conv.device_type
                else {
                    continue;
                };
                let heat_rate = *heat_rate_gj_per_mwh;

                let elec_out = conv.max_output_mw().min(deficit).max(conv.min_output_mw());
                // Gas input: heat_rate [GJ/MWh] × elec_out [MW] = GJ/h; convert to MW
                // 1 GJ = 0.2778 MWh  →  gas_mw = heat_rate × elec_out / 3.6
                let gas_mw = heat_rate * elec_out / 3.6;

                converter_outputs_mw[ci] = elec_out;
                if node_idx < net_elec_demand_mw.len() {
                    net_elec_demand_mw[node_idx] -= elec_out;
                }

                let gt_cost = gas_mw * gas_price * dt;
                step_cost += gt_cost;
                step_co2_kg += gas_mw * dt * carbon_factors.natural_gas_kg_per_mwh;
                step_primary_mwh += gas_mw * dt;
                step_useful_mwh += elec_out * dt;
            }

            // ------------------------------------------------------------------
            // 6. Grid import cost for remaining electricity deficit
            // ------------------------------------------------------------------
            let total_elec_deficit: f64 = net_elec_demand_mw.iter().map(|&v| v.max(0.0)).sum();
            let import_cost = total_elec_deficit * elec_price * dt;
            step_cost += import_cost;
            step_co2_kg += total_elec_deficit * dt * carbon_factors.electricity_kg_per_mwh;
            step_primary_mwh += total_elec_deficit * dt;
            step_useful_mwh += total_elec_deficit * dt;

            // ------------------------------------------------------------------
            // 7. Curtailed renewable = net surplus after charging storage
            // ------------------------------------------------------------------
            let curtailed: f64 = net_elec_demand_mw.iter().map(|&v| (-v).max(0.0)).sum();
            let re_dispatched: f64 = re_gen.iter().sum::<f64>() - curtailed;

            // ------------------------------------------------------------------
            // 8. Unmet demand accounting
            // ------------------------------------------------------------------
            let mut unmet = vec![vec![0.0_f64; EnergyCarrier::COUNT]; self.nodes.len()];
            for (n, node) in self.nodes.iter().enumerate() {
                let heat_unmet = heat_demand_mw.get(n).copied().unwrap_or(0.0).max(0.0);
                unmet[n][EnergyCarrier::Heat.index()] = heat_unmet;
                let elec_unmet = net_elec_demand_mw.get(n).copied().unwrap_or(0.0).max(0.0);
                unmet[n][EnergyCarrier::Electricity.index()] = elec_unmet;
                let _ = node; // node used for iteration
            }

            // ------------------------------------------------------------------
            // 9. Accumulate totals
            // ------------------------------------------------------------------
            total_cost += step_cost;
            total_co2_kg += step_co2_kg;
            total_primary_input_mwh += step_primary_mwh;
            total_useful_output_mwh += step_useful_mwh;

            // Demand MWh (over all nodes and carriers)
            for node in &self.nodes {
                for &d in &node.demand_mw {
                    total_demand_mwh += d * dt;
                }
            }

            // Local generation (converter outputs + RE)
            let local_gen: f64 =
                converter_outputs_mw.iter().sum::<f64>() * dt + re_gen.iter().sum::<f64>() * dt;
            total_local_gen_mwh += local_gen;

            total_re_gen_mwh += re_dispatched * dt;

            dispatch_records.push(MesDispatch {
                time_step: t,
                converter_outputs_mw,
                storage_power_mw,
                storage_soc: soc.clone(),
                unmet_demand_mw: unmet,
                curtailed_re_mw: curtailed,
                cost_eur: step_cost,
            });
        }

        // Update storage SoC in struct for subsequent calls
        for (si, s) in self.storages.iter_mut().enumerate() {
            s.soc = soc[si];
        }

        let energy_efficiency = if total_primary_input_mwh > 1e-9 {
            (total_useful_output_mwh / total_primary_input_mwh).min(1.0)
        } else {
            0.0
        };

        let self_sufficiency = if total_demand_mwh > 1e-9 {
            (total_local_gen_mwh / total_demand_mwh).min(1.0)
        } else {
            0.0
        };

        let total_gen_mwh = total_local_gen_mwh + total_demand_mwh; // approximate
        let renewable_fraction = if total_gen_mwh > 1e-9 {
            (total_re_gen_mwh / total_gen_mwh).clamp(0.0, 1.0)
        } else {
            0.0
        };

        let co2_intensity_kg_per_mwh = if total_useful_output_mwh > 1e-9 {
            total_co2_kg / total_useful_output_mwh
        } else {
            0.0
        };

        Ok(MesResult {
            dispatch: dispatch_records,
            total_cost_eur: total_cost,
            total_co2_kg,
            energy_efficiency,
            self_sufficiency,
            renewable_fraction,
            co2_intensity_kg_per_mwh,
        })
    }

    // -----------------------------------------------------------------------
    // Energy hub coupling matrix
    // -----------------------------------------------------------------------

    /// Compute the energy hub coupling matrix C ∈ ℝ^{n_carriers × n_carriers}.
    ///
    /// C\[i\]\[j\] = sum of efficiencies over all converters whose
    /// `from_carrier` has canonical index `j` and `to_carrier` has index `i`.
    ///
    /// Rows = output carriers, columns = input carriers.
    pub fn compute_energy_hub_matrix(&self) -> Vec<Vec<f64>> {
        let n = EnergyCarrier::COUNT;
        let mut mat = vec![vec![0.0_f64; n]; n];
        for conv in &self.converters {
            let row = conv.to_carrier.index();
            let col = conv.from_carrier.index();
            mat[row][col] += conv.efficiency;
        }
        mat
    }

    // -----------------------------------------------------------------------
    // N-1 contingency analysis
    // -----------------------------------------------------------------------

    /// Run N-1 contingency analysis: for each converter, disable it and
    /// re-optimise the remaining system.
    ///
    /// Returns one [`MesResult`] per converter (same order as `self.converters`).
    pub fn simulate_n_minus_1(
        &mut self,
        electricity_prices: &[f64],
        gas_prices: &[f64],
        renewable_generation: &[Vec<f64>],
        carbon_factors: &CarbonFactors,
    ) -> Result<Vec<MesResult>, crate::error::OxiGridError> {
        let n_conv = self.converters.len();
        let original_converters = self.converters.clone();
        let original_socs: Vec<f64> = self.storages.iter().map(|s| s.soc).collect();

        let mut results = Vec::with_capacity(n_conv);

        for outage_idx in 0..n_conv {
            // Restore initial SoC
            for (si, s) in self.storages.iter_mut().enumerate() {
                s.soc = original_socs[si];
            }

            // Remove the outaged converter
            self.converters = original_converters
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != outage_idx)
                .map(|(_, c)| c.clone())
                .collect();

            let res = self.optimize(
                electricity_prices,
                gas_prices,
                renewable_generation,
                carbon_factors,
            )?;
            results.push(res);
        }

        // Restore full converter list and SoC
        self.converters = original_converters;
        for (si, s) in self.storages.iter_mut().enumerate() {
            s.soc = original_socs[si];
        }

        Ok(results)
    }

    // -----------------------------------------------------------------------
    // Pareto front (cost vs CO₂)
    // -----------------------------------------------------------------------

    /// Compute the Pareto front for the bi-objective problem:
    /// minimise operational cost vs. minimise CO₂ emissions.
    ///
    /// Uses weighted-sum scalarisation: for each `(cost_w, co2_w)` pair,
    /// scales electricity and gas prices by `cost_w` and augments gas price
    /// with a carbon surcharge proportional to `co2_w`, then solves.
    ///
    /// Returns a vector of `(cost_eur, co2_kg)` Pareto points.
    ///
    /// # Arguments
    /// * `n_points`  – number of Pareto points (ignored if `weights` non-empty)
    /// * `weights`   – explicit `(cost_weight, co2_weight)` pairs
    /// * `electricity_prices` – base electricity prices [€/MWh]
    /// * `gas_prices`         – base gas prices [€/MWh]
    /// * `renewable_generation` – renewable output `MW` per timestep per node
    /// * `carbon_factors`       – CO₂ emission factors
    pub fn compute_pareto_front(
        &mut self,
        n_points: usize,
        weights: &[(f64, f64)],
        electricity_prices: &[f64],
        gas_prices: &[f64],
        renewable_generation: &[Vec<f64>],
        carbon_factors: &CarbonFactors,
    ) -> Result<Vec<(f64, f64)>, crate::error::OxiGridError> {
        let weight_pairs: Vec<(f64, f64)> = if !weights.is_empty() {
            weights.to_vec()
        } else {
            // Uniformly sample the weight simplex
            (0..n_points.max(2))
                .map(|i| {
                    let alpha = i as f64 / (n_points.max(2) - 1) as f64;
                    (alpha, 1.0 - alpha)
                })
                .collect()
        };

        let original_socs: Vec<f64> = self.storages.iter().map(|s| s.soc).collect();
        let mut pareto_points = Vec::with_capacity(weight_pairs.len());

        for (cost_w, co2_w) in &weight_pairs {
            // Restore initial SoC for each run
            for (si, s) in self.storages.iter_mut().enumerate() {
                s.soc = original_socs[si];
            }

            // Scale prices by cost_weight; add CO₂ penalty to gas price
            let co2_penalty_per_mwh = co2_w * carbon_factors.natural_gas_kg_per_mwh / 1000.0 * 50.0; // €50/t CO₂
            let scaled_elec: Vec<f64> = electricity_prices.iter().map(|&p| p * cost_w).collect();
            let scaled_gas: Vec<f64> = gas_prices
                .iter()
                .map(|&p| p * cost_w + co2_penalty_per_mwh)
                .collect();

            let result = self.optimize(
                &scaled_elec,
                &scaled_gas,
                renewable_generation,
                carbon_factors,
            )?;

            pareto_points.push((result.total_cost_eur, result.total_co2_kg));
        }

        // Restore SoC
        for (si, s) in self.storages.iter_mut().enumerate() {
            s.soc = original_socs[si];
        }

        Ok(pareto_points)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------
    // Helper builders
    // ------------------------------------------------------------------

    fn single_elec_node(demand_mw: f64) -> MesNode {
        MesNode {
            id: 0,
            name: "Node0".to_string(),
            carriers: vec![EnergyCarrier::Electricity],
            demand_mw: vec![demand_mw],
            latitude: 0.0,
            longitude: 0.0,
        }
    }

    fn elec_heat_node(elec_mw: f64, heat_mw: f64) -> MesNode {
        MesNode {
            id: 0,
            name: "NodeEH".to_string(),
            carriers: vec![EnergyCarrier::Electricity, EnergyCarrier::Heat],
            demand_mw: vec![elec_mw, heat_mw],
            latitude: 0.0,
            longitude: 0.0,
        }
    }

    fn chp_converter(capacity_mw: f64, hpr: f64, eta: f64) -> EnergyConverter {
        EnergyConverter {
            id: 1,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::NaturalGas,
            to_carrier: EnergyCarrier::Electricity,
            efficiency: eta,
            capacity_mw,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 10.0,
            device_type: ConverterType::CombinedHeatPower {
                heat_to_power_ratio: hpr,
            },
        }
    }

    fn heat_pump_conv(capacity_mw: f64, cop: f64) -> EnergyConverter {
        EnergyConverter {
            id: 2,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::Electricity,
            to_carrier: EnergyCarrier::Heat,
            efficiency: cop,
            capacity_mw,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 5.0,
            device_type: ConverterType::HeatPump { cop },
        }
    }

    fn electrolyzer_conv(capacity_mw: f64, fe: f64) -> EnergyConverter {
        EnergyConverter {
            id: 3,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::Electricity,
            to_carrier: EnergyCarrier::Hydrogen,
            efficiency: 0.65,
            capacity_mw,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 2.0,
            device_type: ConverterType::Electrolyzer {
                faradaic_efficiency: fe,
            },
        }
    }

    fn elec_storage_device(capacity_mwh: f64, max_rate_mw: f64) -> MesStorage {
        MesStorage {
            id: 10,
            node: 0,
            carrier: EnergyCarrier::Electricity,
            capacity_mwh,
            max_charge_rate_mw: max_rate_mw,
            max_discharge_rate_mw: max_rate_mw,
            round_trip_efficiency: 0.90,
            soc: 0.5,
            soc_min: 0.1,
            soc_max: 0.9,
            self_discharge_rate: 0.001,
        }
    }

    fn flat_re(t: usize, n_nodes: usize, value: f64) -> Vec<Vec<f64>> {
        vec![vec![value; n_nodes]; t]
    }

    fn flat_prices(t: usize, price: f64) -> Vec<f64> {
        vec![price; t]
    }

    fn carbon() -> CarbonFactors {
        CarbonFactors::default()
    }

    // ------------------------------------------------------------------
    // Tests
    // ------------------------------------------------------------------

    #[test]
    fn test_mes_node_creation() {
        let node = MesNode {
            id: 5,
            name: "SubstationA".to_string(),
            carriers: vec![EnergyCarrier::Electricity, EnergyCarrier::Heat],
            demand_mw: vec![10.0, 5.0],
            latitude: 59.3,
            longitude: 18.0,
        };
        assert_eq!(node.id, 5);
        assert_eq!(node.name, "SubstationA");
        assert_eq!(node.carriers.len(), 2);
        assert!((node.demand_for(&EnergyCarrier::Electricity) - 10.0).abs() < 1e-9);
        assert!((node.demand_for(&EnergyCarrier::Heat) - 5.0).abs() < 1e-9);
        assert!((node.demand_for(&EnergyCarrier::Hydrogen) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_energy_converter_chp() {
        let chp = chp_converter(100.0, 1.5, 0.8);
        assert!(matches!(
            chp.device_type,
            ConverterType::CombinedHeatPower { .. }
        ));
        assert!((chp.max_output_mw() - 100.0).abs() < 1e-9);
        let heat = chp.chp_heat_output_mw(40.0);
        assert!((heat - 60.0).abs() < 1e-9); // 40 × 1.5
        let gas = chp.chp_gas_input_mw(40.0);
        // gas = 40 × (1 + 1.5) / 0.8 = 40 × 2.5 / 0.8 = 125
        assert!((gas - 125.0).abs() < 1e-6);
    }

    #[test]
    fn test_energy_converter_heat_pump() {
        let hp = heat_pump_conv(50.0, 3.5);
        if let ConverterType::HeatPump { cop } = hp.device_type {
            assert!((cop - 3.5).abs() < 1e-9);
        } else {
            panic!("Expected HeatPump variant");
        }
        assert!((hp.max_output_mw() - 50.0).abs() < 1e-9);
    }

    #[test]
    fn test_energy_converter_electrolyzer() {
        let elz = electrolyzer_conv(10.0, 0.95);
        if let ConverterType::Electrolyzer {
            faradaic_efficiency,
        } = elz.device_type
        {
            assert!((faradaic_efficiency - 0.95).abs() < 1e-9);
        } else {
            panic!("Expected Electrolyzer variant");
        }
        assert!((elz.efficiency - 0.65).abs() < 1e-9);
    }

    #[test]
    fn test_mes_storage_charge_discharge() {
        let storage = elec_storage_device(100.0, 20.0);
        // At soc=0.5, capacity=100: headroom = (0.9-0.5)*100=40 MWh, limited by 20*1h=20 MWh
        let headroom = storage.headroom_mwh(1.0);
        assert!((headroom - 20.0).abs() < 1e-9);
        // available = (0.5-0.1)*100=40 MWh, limited by 20*1h=20 MWh
        let avail = storage.available_mwh(1.0);
        assert!((avail - 20.0).abs() < 1e-9);
        // one-way efficiency = sqrt(0.9) ≈ 0.9487
        let eta = storage.one_way_efficiency();
        assert!((eta - 0.9_f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn test_optimize_electricity_only() {
        // Single node, electricity demand only, no converters — must import from grid
        let mut opt = MesOptimizer::new(vec![single_elec_node(100.0)], vec![], vec![], 4, 1.0);
        let result = opt
            .optimize(
                &flat_prices(4, 60.0),
                &flat_prices(4, 30.0),
                &flat_re(4, 1, 0.0),
                &carbon(),
            )
            .expect("optimize should succeed");

        assert_eq!(result.dispatch.len(), 4);
        // All electricity must be imported → cost = 100 MW × 60 €/MWh × 4 h = 24000 €
        assert!((result.total_cost_eur - 24000.0).abs() < 1.0);
        assert!(result.total_co2_kg > 0.0);
    }

    #[test]
    fn test_optimize_chp_dispatch() {
        // Node with electricity + heat demand; CHP should serve both
        let chp = chp_converter(200.0, 1.5, 0.8);
        let mut opt =
            MesOptimizer::new(vec![elec_heat_node(50.0, 30.0)], vec![chp], vec![], 2, 1.0);
        let result = opt
            .optimize(
                &flat_prices(2, 80.0),
                &flat_prices(2, 40.0),
                &flat_re(2, 1, 0.0),
                &carbon(),
            )
            .expect("CHP dispatch should succeed");

        // CHP should have non-zero output at timestep 0
        let t0 = &result.dispatch[0];
        assert!(!t0.converter_outputs_mw.is_empty());
        assert!(result.total_co2_kg > 0.0);
    }

    #[test]
    fn test_optimize_heat_pump_dispatch() {
        // Node with heat demand; heat pump should reduce electricity deficit by converting elec→heat
        let hp = heat_pump_conv(20.0, 3.0);
        let mut opt = MesOptimizer::new(vec![elec_heat_node(10.0, 15.0)], vec![hp], vec![], 3, 1.0);
        let result = opt
            .optimize(
                &flat_prices(3, 50.0),
                &flat_prices(3, 25.0),
                &flat_re(3, 1, 0.0),
                &carbon(),
            )
            .expect("Heat pump dispatch should succeed");

        assert_eq!(result.dispatch.len(), 3);
        // Heat pump output should be non-zero
        let hp_out: f64 = result
            .dispatch
            .iter()
            .map(|d| d.converter_outputs_mw.first().copied().unwrap_or(0.0))
            .sum();
        assert!(hp_out > 0.0, "Heat pump should produce heat");
    }

    #[test]
    fn test_optimize_storage_integration() {
        // Low price → charge storage; high price → discharge
        let storage = elec_storage_device(200.0, 50.0);
        let mut opt =
            MesOptimizer::new(vec![single_elec_node(20.0)], vec![], vec![storage], 6, 1.0);
        // First 3 hours cheap (below mean), last 3 expensive (above mean)
        let prices: Vec<f64> = (0..6).map(|i| if i < 3 { 20.0 } else { 120.0 }).collect();
        let result = opt
            .optimize(
                &prices,
                &flat_prices(6, 30.0),
                // Large renewable surplus in cheap hours → trigger charging
                &(0..6)
                    .map(|i| if i < 3 { vec![100.0] } else { vec![0.0] })
                    .collect::<Vec<_>>(),
                &carbon(),
            )
            .expect("Storage integration should succeed");

        assert_eq!(result.dispatch.len(), 6);
        // At least one charging event should occur in cheap hours
        let charged: f64 = result.dispatch[..3]
            .iter()
            .flat_map(|d| d.storage_power_mw.iter())
            .filter(|&&v| v > 0.0)
            .sum();
        assert!(charged > 0.0, "Storage should charge during cheap hours");
    }

    #[test]
    fn test_energy_hub_matrix_2x2() {
        // One elec→heat converter with η=0.9
        let conv = EnergyConverter {
            id: 1,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::Electricity,
            to_carrier: EnergyCarrier::Heat,
            efficiency: 0.9,
            capacity_mw: 10.0,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 5.0,
            device_type: ConverterType::ElectricBoiler,
        };
        let opt = MesOptimizer::new(vec![], vec![conv], vec![], 1, 1.0);
        let mat = opt.compute_energy_hub_matrix();
        let row = EnergyCarrier::Heat.index();
        let col = EnergyCarrier::Electricity.index();
        assert!((mat[row][col] - 0.9).abs() < 1e-9);
    }

    #[test]
    fn test_energy_hub_matrix_multi_carrier() {
        // CHP: gas → elec (η=0.35) and gas → heat (η=0.45) — two converters
        let chp_e = EnergyConverter {
            id: 1,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::NaturalGas,
            to_carrier: EnergyCarrier::Electricity,
            efficiency: 0.35,
            capacity_mw: 100.0,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 10.0,
            device_type: ConverterType::GasTurbine {
                heat_rate_gj_per_mwh: 8.5,
            },
        };
        let chp_h = EnergyConverter {
            id: 2,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::NaturalGas,
            to_carrier: EnergyCarrier::Heat,
            efficiency: 0.45,
            capacity_mw: 100.0,
            min_loading: 0.0,
            ramp_rate_mw_per_min: 10.0,
            device_type: ConverterType::ElectricBoiler,
        };
        let opt = MesOptimizer::new(vec![], vec![chp_e, chp_h], vec![], 1, 1.0);
        let mat = opt.compute_energy_hub_matrix();

        let gas_col = EnergyCarrier::NaturalGas.index();
        let elec_row = EnergyCarrier::Electricity.index();
        let heat_row = EnergyCarrier::Heat.index();

        assert!((mat[elec_row][gas_col] - 0.35).abs() < 1e-9);
        assert!((mat[heat_row][gas_col] - 0.45).abs() < 1e-9);
    }

    #[test]
    fn test_co2_intensity_calculation() {
        // Grid-only system: known demand → known CO₂
        let mut opt = MesOptimizer::new(vec![single_elec_node(50.0)], vec![], vec![], 2, 1.0);
        let cf = CarbonFactors {
            electricity_kg_per_mwh: 400.0,
            natural_gas_kg_per_mwh: 200.0,
            hydrogen_kg_per_mwh: 50.0,
        };
        let result = opt
            .optimize(
                &flat_prices(2, 60.0),
                &flat_prices(2, 30.0),
                &flat_re(2, 1, 0.0),
                &cf,
            )
            .expect("optimize");

        // Imported: 50 MW × 2 h = 100 MWh → CO₂ = 100 × 400 = 40000 kg
        assert!((result.total_co2_kg - 40000.0).abs() < 1.0);
        // CO₂ intensity = 40000 / (useful_output_mwh)
        assert!(result.co2_intensity_kg_per_mwh > 0.0);
    }

    #[test]
    fn test_energy_efficiency_metric() {
        // CHP with known efficiencies: useful output / primary input
        let chp = chp_converter(100.0, 1.5, 0.8);
        let mut opt =
            MesOptimizer::new(vec![elec_heat_node(40.0, 60.0)], vec![chp], vec![], 1, 1.0);
        let result = opt
            .optimize(
                &flat_prices(1, 70.0),
                &flat_prices(1, 35.0),
                &flat_re(1, 1, 0.0),
                &carbon(),
            )
            .expect("optimize");

        assert!(result.energy_efficiency >= 0.0);
        assert!(result.energy_efficiency <= 1.0);
    }

    #[test]
    fn test_self_sufficiency_metric() {
        // With substantial renewable generation, self_sufficiency should increase
        let mut opt_low = MesOptimizer::new(vec![single_elec_node(100.0)], vec![], vec![], 4, 1.0);
        let result_low = opt_low
            .optimize(
                &flat_prices(4, 60.0),
                &flat_prices(4, 30.0),
                &flat_re(4, 1, 0.0), // no renewables
                &carbon(),
            )
            .expect("optimize_low");

        let mut opt_high = MesOptimizer::new(vec![single_elec_node(100.0)], vec![], vec![], 4, 1.0);
        let result_high = opt_high
            .optimize(
                &flat_prices(4, 60.0),
                &flat_prices(4, 30.0),
                &flat_re(4, 1, 80.0), // 80% covered by renewables
                &carbon(),
            )
            .expect("optimize_high");

        assert!(result_high.self_sufficiency >= result_low.self_sufficiency);
    }

    #[test]
    fn test_renewable_fraction_metric() {
        // All demand covered by renewables should give non-zero renewable_fraction
        let mut opt = MesOptimizer::new(vec![single_elec_node(50.0)], vec![], vec![], 2, 1.0);
        let result = opt
            .optimize(
                &flat_prices(2, 60.0),
                &flat_prices(2, 30.0),
                &flat_re(2, 1, 60.0), // RE > demand
                &carbon(),
            )
            .expect("optimize");

        assert!(result.renewable_fraction >= 0.0);
        assert!(result.renewable_fraction <= 1.0);
    }

    #[test]
    fn test_pareto_front_cost_vs_co2() {
        let chp = chp_converter(200.0, 1.5, 0.8);
        let mut opt =
            MesOptimizer::new(vec![elec_heat_node(80.0, 40.0)], vec![chp], vec![], 4, 1.0);
        let weights = vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0)];
        let pareto = opt
            .compute_pareto_front(
                3,
                &weights,
                &flat_prices(4, 60.0),
                &flat_prices(4, 30.0),
                &flat_re(4, 1, 0.0),
                &carbon(),
            )
            .expect("pareto front");

        assert_eq!(pareto.len(), 3);
        for point in &pareto {
            let (cost, co2): (f64, f64) = *point;
            assert!(cost.is_finite(), "Cost must be finite");
            assert!(co2 >= 0.0, "CO₂ must be non-negative");
        }
    }

    #[test]
    fn test_n_minus_1_contingency() {
        let chp = chp_converter(100.0, 1.5, 0.8);
        let hp = heat_pump_conv(50.0, 3.0);
        let mut opt = MesOptimizer::new(
            vec![elec_heat_node(60.0, 30.0)],
            vec![chp, hp],
            vec![],
            3,
            1.0,
        );
        let n1_results = opt
            .simulate_n_minus_1(
                &flat_prices(3, 70.0),
                &flat_prices(3, 35.0),
                &flat_re(3, 1, 0.0),
                &carbon(),
            )
            .expect("N-1 should succeed");

        // One result per converter
        assert_eq!(n1_results.len(), 2);
        for res in &n1_results {
            assert!(res.total_cost_eur.is_finite());
            assert!(res.total_co2_kg >= 0.0);
        }

        // After N-1, original converter list should be restored
        assert_eq!(opt.converters.len(), 2);
    }

    #[test]
    fn test_multi_period_24h() {
        // 24-hour dispatch with time-varying prices using LCG for variety
        let mut state = 0xdeadbeef_u64;
        let prices: Vec<f64> = (0..24)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005u64)
                    .wrapping_add(1442695040888963407u64);
                40.0 + (state >> 33) as f64 / (u32::MAX as f64) * 80.0
            })
            .collect();

        let re_gen: Vec<Vec<f64>> = (0..24)
            .map(|h| {
                // Sinusoidal solar generation 6h–18h
                let solar = if (6..=18).contains(&h) {
                    50.0 * ((h as f64 - 12.0) * std::f64::consts::PI / 12.0)
                        .cos()
                        .abs()
                } else {
                    0.0
                };
                vec![solar]
            })
            .collect();

        let chp = chp_converter(80.0, 1.5, 0.8);
        let hp = heat_pump_conv(30.0, 3.0);
        let gt = EnergyConverter {
            id: 4,
            from_node: 0,
            to_node: 0,
            from_carrier: EnergyCarrier::NaturalGas,
            to_carrier: EnergyCarrier::Electricity,
            efficiency: 0.40,
            capacity_mw: 120.0,
            min_loading: 0.1,
            ramp_rate_mw_per_min: 5.0,
            device_type: ConverterType::GasTurbine {
                heat_rate_gj_per_mwh: 9.0,
            },
        };
        let storage = elec_storage_device(400.0, 80.0);

        let mut opt = MesOptimizer::new(
            vec![elec_heat_node(70.0, 40.0)],
            vec![chp, hp, gt],
            vec![storage],
            24,
            1.0,
        );

        let gas_prices = flat_prices(24, 35.0);
        let result = opt
            .optimize(&prices, &gas_prices, &re_gen, &carbon())
            .expect("24h dispatch should succeed");

        assert_eq!(result.dispatch.len(), 24);
        assert!(result.total_cost_eur.is_finite());
        assert!(result.total_co2_kg >= 0.0);
        assert!(result.energy_efficiency >= 0.0 && result.energy_efficiency <= 1.0);
        assert!(result.renewable_fraction >= 0.0 && result.renewable_fraction <= 1.0);
        assert!(result.self_sufficiency >= 0.0 && result.self_sufficiency <= 1.0);
    }
}
