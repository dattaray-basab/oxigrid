//! Integrated Power-to-Gas (P2G) system dispatch and optimisation.
//!
//! Combines electrolyzer, hydrogen storage tank, and optional fuel cell into a single
//! dispatchable asset connected to the electricity grid. Supports multiple dispatch
//! strategies via a greedy heuristic scheduler.

use super::electrolyzer::Electrolyzer;
use super::fuel_cell::FuelCell;
use super::storage::HydrogenTank;
use crate::error::{OxiGridError, Result};

// ── System model ──────────────────────────────────────────────────────────────

/// Integrated Power-to-Gas system connecting electrolyzer, tank, and optional fuel cell.
#[derive(Debug, Clone)]
pub struct P2gSystem {
    /// Unique system identifier
    pub system_id: usize,
    /// Grid connection bus index
    pub bus: usize,
    /// Electrolyzer for converting electricity to hydrogen
    pub electrolyzer: Electrolyzer,
    /// Hydrogen storage tank
    pub tank: HydrogenTank,
    /// Optional fuel cell for reconverting hydrogen to electricity
    pub fuel_cell: Option<FuelCell>,
    /// Maximum power injection from fuel cell to grid \[MW\]
    pub grid_injection_limit_mw: f64,
}

impl P2gSystem {
    /// Create a new P2G system.
    pub fn new(
        system_id: usize,
        bus: usize,
        electrolyzer: Electrolyzer,
        tank: HydrogenTank,
        fuel_cell: Option<FuelCell>,
        grid_injection_limit_mw: f64,
    ) -> Self {
        Self {
            system_id,
            bus,
            electrolyzer,
            tank,
            fuel_cell,
            grid_injection_limit_mw,
        }
    }

    /// True if a fuel cell is present (bidirectional operation possible).
    pub fn has_fuel_cell(&self) -> bool {
        self.fuel_cell.is_some()
    }
}

// ── Dispatch configuration ────────────────────────────────────────────────────

/// Optimisation mode for the P2G dispatcher.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum P2gOptimizeMode {
    /// Minimise total electricity purchase cost
    CostMinimization,
    /// Minimise carbon footprint of hydrogen production
    CarbonMinimization,
    /// Maximise absorption of excess renewable generation (avoid curtailment)
    SelfConsumption,
    /// Price arbitrage between electrolysis (buy low) and fuel cell (sell high)
    GridBalancing,
}

/// Time-resolved dispatch configuration for the P2G system.
#[derive(Debug, Clone)]
pub struct P2gDispatchConfig {
    /// Time step duration \[hours\]
    pub dt_hours: f64,
    /// Electricity price per time slot [$/MWh]
    pub electricity_price: Vec<f64>,
    /// External hydrogen demand per time slot [kg/h]
    pub h2_demand: Vec<f64>,
    /// Grid carbon intensity per time slot [gCO2/kWh]
    pub grid_carbon_intensity: Vec<f64>,
    /// Minimum fraction of h2_demand that must be met (0–1)
    pub min_h2_supply_fraction: f64,
    /// Dispatch optimisation strategy
    pub optimize_mode: P2gOptimizeMode,
    /// Price threshold below which opportunistic electrolysis is triggered [$/MWh]
    pub low_price_threshold: f64,
    /// Price threshold above which fuel cell dispatch is triggered [$/MWh]
    pub high_price_threshold: f64,
}

impl P2gDispatchConfig {
    /// Validate the configuration for internal consistency.
    pub fn validate(&self, n_slots: usize) -> Result<()> {
        if self.dt_hours <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_hours must be positive".to_string(),
            ));
        }
        if self.electricity_price.len() != n_slots {
            return Err(OxiGridError::InvalidParameter(format!(
                "electricity_price length {} != n_slots {}",
                self.electricity_price.len(),
                n_slots
            )));
        }
        if self.h2_demand.len() != n_slots {
            return Err(OxiGridError::InvalidParameter(format!(
                "h2_demand length {} != n_slots {}",
                self.h2_demand.len(),
                n_slots
            )));
        }
        if !self.grid_carbon_intensity.is_empty() && self.grid_carbon_intensity.len() != n_slots {
            return Err(OxiGridError::InvalidParameter(format!(
                "grid_carbon_intensity length {} != n_slots {}",
                self.grid_carbon_intensity.len(),
                n_slots
            )));
        }
        if !(0.0..=1.0).contains(&self.min_h2_supply_fraction) {
            return Err(OxiGridError::InvalidParameter(
                "min_h2_supply_fraction must be in [0, 1]".to_string(),
            ));
        }
        Ok(())
    }
}

// ── Dispatch result ───────────────────────────────────────────────────────────

/// Per-slot and aggregate results from a P2G dispatch run.
#[derive(Debug, Clone)]
pub struct P2gDispatchResult {
    /// Time index for each slot [hours from start]
    pub time_slots: Vec<f64>,
    /// Electrolyzer power consumption per slot \[MW\]
    pub electrolysis_power_mw: Vec<f64>,
    /// Fuel cell power output to grid per slot \[MW\]
    pub fuel_cell_power_mw: Vec<f64>,
    /// Hydrogen produced by electrolyzer per slot \[kg\]
    pub h2_production_kg: Vec<f64>,
    /// Hydrogen consumed by fuel cell per slot \[kg\]
    pub h2_consumption_kg: Vec<f64>,
    /// Hydrogen delivered to external demand per slot \[kg\]
    pub h2_delivered_kg: Vec<f64>,
    /// Tank state of charge at end of each slot [fraction 0–1]
    pub tank_soc: Vec<f64>,
    /// Total electricity purchase cost [$]
    pub total_electricity_cost: f64,
    /// Total hydrogen revenue (delivered H2 at implicit H2 price) [$]
    pub total_h2_revenue: f64,
    /// Carbon emissions per slot [kg CO2]
    pub carbon_emissions_kg: Vec<f64>,
    /// Fraction of H2 demand that was met (0–1)
    pub h2_demand_met_pct: f64,
}

// ── Dispatcher ────────────────────────────────────────────────────────────────

/// Greedy/heuristic P2G dispatch engine.
pub struct P2gDispatcher {
    /// The P2G system being dispatched (mutable clone used during optimise)
    pub system: P2gSystem,
    /// Dispatch configuration
    pub config: P2gDispatchConfig,
}

impl P2gDispatcher {
    /// Construct a new dispatcher.
    pub fn new(system: P2gSystem, config: P2gDispatchConfig) -> Self {
        Self { system, config }
    }

    /// Run the P2G dispatch optimisation.
    ///
    /// Uses a greedy heuristic strategy that varies by `optimize_mode`:
    /// - If electricity price is low AND tank has room: electrolyze
    /// - If H2 demand exists AND tank has H2: deliver from tank
    /// - If electricity price is high AND tank has H2 AND fuel cell present: generate power
    /// - Otherwise: idle
    pub fn optimize(&self) -> Result<P2gDispatchResult> {
        let n = self.config.electricity_price.len();
        self.config.validate(n)?;

        // Work on a mutable clone of the tank (electrolyzer/FC are stateless wrt. energy)
        let mut tank = self.system.tank.clone();

        let mut time_slots = Vec::with_capacity(n);
        let mut electrolysis_power_mw = Vec::with_capacity(n);
        let mut fuel_cell_power_mw = Vec::with_capacity(n);
        let mut h2_production_kg = Vec::with_capacity(n);
        let mut h2_consumption_kg = Vec::with_capacity(n);
        let mut h2_delivered_kg = Vec::with_capacity(n);
        let mut tank_soc_out = Vec::with_capacity(n);
        let mut carbon_emissions_kg = Vec::with_capacity(n);

        let mut total_electricity_cost = 0.0_f64;
        let mut total_h2_delivered = 0.0_f64;
        let mut total_h2_demanded = 0.0_f64;

        // Determine reference price for thresholding (mean price or supplied thresholds)
        let mean_price = if n > 0 {
            self.config.electricity_price.iter().sum::<f64>() / n as f64
        } else {
            50.0
        };
        let low_thresh = if self.config.low_price_threshold > 0.0 {
            self.config.low_price_threshold
        } else {
            mean_price * 0.7 // default: 70% of mean = "cheap"
        };
        let high_thresh = if self.config.high_price_threshold > 0.0 {
            self.config.high_price_threshold
        } else {
            mean_price * 1.3 // default: 130% of mean = "expensive"
        };

        // Carbon intensity default (0 if not provided)
        let get_carbon = |t: usize| -> f64 {
            if t < self.config.grid_carbon_intensity.len() {
                self.config.grid_carbon_intensity[t]
            } else {
                0.0
            }
        };

        let elz = &self.system.electrolyzer;
        let dt = self.config.dt_hours;

        // Implicit H2 price for revenue calculation (marginal cost of cheapest production)
        let implicit_h2_price = self.marginal_h2_cost();

        for t in 0..n {
            let price = self.config.electricity_price[t];
            let demand_kg_h = self.config.h2_demand[t].max(0.0);
            let demand_kg = demand_kg_h * dt;
            let carbon_intensity_g_kwh = get_carbon(t);

            total_h2_demanded += demand_kg;

            let mut elz_power = 0.0_f64;
            let mut fc_power = 0.0_f64;
            let mut produced_kg = 0.0_f64;
            let mut consumed_fc_kg = 0.0_f64;
            let mut delivered_kg = 0.0_f64;

            // ── Step 1: Determine electrolyzer action ─────────────────────────
            let should_electrolyze = match self.config.optimize_mode {
                P2gOptimizeMode::CostMinimization => price <= low_thresh && !tank.is_full(),
                P2gOptimizeMode::CarbonMinimization => {
                    // Electrolyze when carbon intensity is below average
                    let mean_carbon = self
                        .config
                        .grid_carbon_intensity
                        .iter()
                        .copied()
                        .sum::<f64>()
                        / self.config.grid_carbon_intensity.len().max(1) as f64;
                    carbon_intensity_g_kwh <= mean_carbon * 1.1 && !tank.is_full()
                }
                P2gOptimizeMode::SelfConsumption => !tank.is_full(), // always electrolyze if room
                P2gOptimizeMode::GridBalancing => price <= low_thresh && !tank.is_full(),
            };

            if should_electrolyze {
                // Run electrolyzer at rated power (or reduce if tank nearly full)
                let max_h2_capacity = tank.available_headroom_kg();
                let max_h2_rate = elz.hydrogen_production_kg_per_h(elz.rated_power_mw);
                let max_producible = max_h2_rate * dt;
                if max_producible > 1e-9 && max_h2_capacity > 1e-9 {
                    // Scale power down if needed to avoid overfilling
                    let power_fraction = (max_h2_capacity / max_producible)
                        .min(1.0)
                        .max(elz.min_power_fraction);
                    elz_power = elz.rated_power_mw * power_fraction;
                    let h2_produced_h = elz.hydrogen_production_kg_per_h(elz_power);
                    let h2_produced = h2_produced_h * dt;
                    produced_kg = tank.charge(h2_produced, dt);
                }
            }

            // ── Step 2: Supply H2 demand from tank ───────────────────────────
            if demand_kg > 1e-9 {
                delivered_kg = tank.discharge(demand_kg, dt);
            }

            // ── Step 3: Fuel cell dispatch at high price ──────────────────────
            let should_use_fc = match self.config.optimize_mode {
                P2gOptimizeMode::GridBalancing | P2gOptimizeMode::CostMinimization => {
                    price >= high_thresh
                        && self.system.has_fuel_cell()
                        && !tank.is_empty()
                        && elz_power < 1e-9 // don't simultaneously electrolyze and discharge
                }
                _ => false,
            };

            if should_use_fc {
                if let Some(ref fc) = self.system.fuel_cell {
                    let fc_rated = fc.rated_power_mw;
                    let fc_limit = fc_rated.min(self.system.grid_injection_limit_mw);
                    let h2_needed_kg = fc.hydrogen_consumption_kg_per_h(fc_limit) * dt;
                    let h2_available = tank.available_to_discharge_kg();
                    // Scale down if not enough H2
                    let scale = if h2_needed_kg > 1e-12 {
                        (h2_available / h2_needed_kg).min(1.0)
                    } else {
                        0.0
                    };
                    fc_power = fc_limit * scale;
                    let h2_consumed_h = fc.hydrogen_consumption_kg_per_h(fc_power);
                    let h2_consumed = h2_consumed_h * dt;
                    consumed_fc_kg = tank.discharge(h2_consumed, dt);
                    // Adjust fc_power to actual H2 consumed
                    if h2_consumed_h > 1e-12 {
                        fc_power *= consumed_fc_kg / (h2_consumed.max(1e-12));
                    }
                }
            }

            // ── Carbon emissions from electrolysis ────────────────────────────
            // Emissions [kg CO2] = power [MW] * 1000 [kW/MW] * dt [h] * carbon [gCO2/kWh] / 1000 [g→kg]
            let emissions_kg = elz_power * 1000.0 * dt * carbon_intensity_g_kwh / 1000.0;

            // ── Cost accounting ───────────────────────────────────────────────
            // Electricity purchased for electrolysis [$/MWh * MW * h = $]
            let slot_cost = price * elz_power * dt;
            // Fuel cell revenues reduce cost (grid export at market price)
            let slot_revenue_fc = price * fc_power * dt;
            total_electricity_cost += slot_cost - slot_revenue_fc;
            total_h2_delivered += delivered_kg;

            time_slots.push(t as f64 * dt);
            electrolysis_power_mw.push(elz_power);
            fuel_cell_power_mw.push(fc_power);
            h2_production_kg.push(produced_kg);
            h2_consumption_kg.push(consumed_fc_kg);
            h2_delivered_kg.push(delivered_kg);
            tank_soc_out.push(tank.soc());
            carbon_emissions_kg.push(emissions_kg);
        }

        let h2_demand_met_pct = if total_h2_demanded > 1e-12 {
            (total_h2_delivered / total_h2_demanded).clamp(0.0, 1.0)
        } else {
            1.0
        };

        // Revenue from H2 delivered to external demand
        let total_h2_revenue = total_h2_delivered * implicit_h2_price;

        Ok(P2gDispatchResult {
            time_slots,
            electrolysis_power_mw,
            fuel_cell_power_mw,
            h2_production_kg,
            h2_consumption_kg,
            h2_delivered_kg,
            tank_soc: tank_soc_out,
            total_electricity_cost,
            total_h2_revenue,
            carbon_emissions_kg,
            h2_demand_met_pct,
        })
    }

    /// Compute the weighted average (marginal) cost of hydrogen production [$/kg].
    ///
    /// Weight = H2 production rate at each hour, assuming rated electrolyzer operation.
    /// Falls back to cost at rated power if insufficient data.
    pub fn marginal_h2_cost(&self) -> f64 {
        let elz = &self.system.electrolyzer;
        let mean_price = if self.config.electricity_price.is_empty() {
            50.0
        } else {
            self.config.electricity_price.iter().sum::<f64>()
                / self.config.electricity_price.len() as f64
        };
        // Approximation: use mean price to compute cost
        elz.operating_cost_per_kg(mean_price, elz.rated_power_mw)
    }

    /// Determine which time slots should have electrolysis active (cheapest n_hours slots
    /// where the tank has theoretical room).
    ///
    /// Returns a boolean vector of length `electricity_price.len()`.
    pub fn optimal_electrolysis_schedule(&self, n_hours: usize, tank_soc: f64) -> Vec<bool> {
        let prices = &self.config.electricity_price;
        let n = prices.len();
        if n == 0 || n_hours == 0 {
            return vec![false; n];
        }

        // Effective capacity available (SoC from tank_soc to 1.0)
        let avail_fraction = (1.0 - tank_soc).max(0.0);
        // Number of hours we can actually fill (considering tank capacity and electrolyzer rate)
        let elz = &self.system.electrolyzer;
        let max_prod_kg_h = elz.hydrogen_production_kg_per_h(elz.rated_power_mw);
        let tank_capacity_kg = self.system.tank.capacity_kg;
        let avail_kg = avail_fraction * tank_capacity_kg;
        // How many hours at full rate to fill available space
        let fillable_hours = if max_prod_kg_h > 1e-12 {
            (avail_kg / max_prod_kg_h).ceil() as usize
        } else {
            0
        };
        let effective_hours = n_hours.min(fillable_hours).min(n);

        if effective_hours == 0 {
            return vec![false; n];
        }

        // Sort slot indices by price ascending
        let mut indexed: Vec<(usize, f64)> = prices.iter().copied().enumerate().collect();
        indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut schedule = vec![false; n];
        for (idx, _price) in indexed.iter().take(effective_hours) {
            schedule[*idx] = true;
        }
        schedule
    }
}

// ── Portfolio optimisation ────────────────────────────────────────────────────

/// Optimise a portfolio of P2G systems responding to a shared grid signal.
///
/// Each system responds independently to the same price/carbon signal,
/// but with its own tank state and equipment constraints. Returns one
/// `P2gDispatchResult` per system.
pub fn optimize_p2g_portfolio(
    systems: &mut [P2gDispatcher],
    grid_signal: &[f64],
    dt_hours: f64,
) -> Result<Vec<P2gDispatchResult>> {
    if systems.is_empty() {
        return Err(OxiGridError::InvalidParameter(
            "Portfolio must contain at least one P2G system".to_string(),
        ));
    }
    if grid_signal.is_empty() {
        return Err(OxiGridError::InvalidParameter(
            "Grid signal must not be empty".to_string(),
        ));
    }

    let mut results = Vec::with_capacity(systems.len());

    for dispatcher in systems.iter_mut() {
        // Override price signal and dt with portfolio-level values
        let n = grid_signal.len();
        dispatcher.config.electricity_price = grid_signal.to_vec();
        dispatcher.config.dt_hours = dt_hours;
        // Resize demand to match signal length if needed
        if dispatcher.config.h2_demand.len() != n {
            dispatcher.config.h2_demand = vec![0.0; n];
        }
        if dispatcher.config.grid_carbon_intensity.len() != n {
            dispatcher.config.grid_carbon_intensity = vec![0.0; n];
        }
        let result = dispatcher.optimize()?;
        results.push(result);
    }

    Ok(results)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::hydrogen::electrolyzer::{Electrolyzer, ElectrolyzerType};
    use crate::optimize::hydrogen::fuel_cell::{FuelCell, FuelCellType};
    use crate::optimize::hydrogen::storage::{HydrogenStorageType, HydrogenTank};

    fn make_system() -> P2gSystem {
        let elz = Electrolyzer::new(2.0, ElectrolyzerType::Pem);
        let tank = HydrogenTank::new(500.0, HydrogenStorageType::CompressedGas700bar);
        let fc = Some(FuelCell::new(1.0, FuelCellType::Pemfc));
        P2gSystem::new(0, 1, elz, tank, fc, 1.0)
    }

    fn make_config(n: usize, prices: Vec<f64>) -> P2gDispatchConfig {
        P2gDispatchConfig {
            dt_hours: 1.0,
            electricity_price: prices,
            h2_demand: vec![10.0; n], // 10 kg/h demand every hour
            grid_carbon_intensity: vec![300.0; n],
            min_h2_supply_fraction: 0.5,
            optimize_mode: P2gOptimizeMode::CostMinimization,
            low_price_threshold: 40.0,
            high_price_threshold: 80.0,
        }
    }

    #[test]
    fn test_p2g_dispatch_meets_h2_demand() {
        // Pre-fill tank, then run with demand
        let mut system = make_system();
        // Pre-fill tank to 80%
        system.tank.current_kg = 400.0;

        let n = 6;
        let prices = vec![60.0; n]; // mid-price, no electrolysis trigger (thresh 40)
        let mut config = make_config(n, prices);
        config.h2_demand = vec![20.0; n]; // 20 kg/h demand

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        assert_eq!(result.time_slots.len(), n);
        // Tank had 400 kg, demand is 20 kg/h * 1h = 20 kg per slot → 6*20 = 120 kg total
        // Should meet full demand (400 > 120)
        assert!(
            result.h2_demand_met_pct > 0.99,
            "Demand should be fully met, got {:.4}",
            result.h2_demand_met_pct
        );
    }

    #[test]
    fn test_p2g_electrolyzes_at_low_price() {
        let system = make_system();
        let n = 4;
        // All prices below low threshold (40 $/MWh)
        let prices = vec![25.0; n];
        let config = make_config(n, prices);

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        // With low prices, electrolyzer should be active
        let total_elz_power: f64 = result.electrolysis_power_mw.iter().sum();
        assert!(
            total_elz_power > 0.0,
            "Electrolysis should be triggered at low prices, total power = {total_elz_power:.4} MW"
        );
    }

    #[test]
    fn test_p2g_fuel_cell_at_high_price() {
        let mut system = make_system();
        // Pre-fill tank
        system.tank.current_kg = 400.0;

        let n = 4;
        // All prices above high threshold (80 $/MWh)
        let prices = vec![120.0; n];
        let mut config = make_config(n, prices);
        config.h2_demand = vec![0.0; n]; // no external demand
        config.optimize_mode = P2gOptimizeMode::GridBalancing;

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        let total_fc_power: f64 = result.fuel_cell_power_mw.iter().sum();
        assert!(
            total_fc_power > 0.0,
            "Fuel cell should dispatch at high prices, total = {total_fc_power:.4} MW"
        );
    }

    #[test]
    fn test_p2g_no_fc_no_export() {
        let elz = Electrolyzer::new(1.0, ElectrolyzerType::Alkaline);
        let tank = HydrogenTank::new(200.0, HydrogenStorageType::CompressedGas350bar);
        let system = P2gSystem::new(1, 0, elz, tank, None, 0.5); // no fuel cell

        let n = 6;
        let prices = vec![150.0; n]; // high price
        let mut config = make_config(n, prices);
        config.h2_demand = vec![0.0; n];

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        let total_fc: f64 = result.fuel_cell_power_mw.iter().sum();
        assert_eq!(
            total_fc, 0.0,
            "No fuel cell → no FC export, got {total_fc:.4}"
        );
    }

    #[test]
    fn test_optimal_electrolysis_schedule_selects_cheapest() {
        let system = make_system();
        let n = 8;
        // Prices: alternating high/low
        let prices = vec![100.0, 20.0, 90.0, 15.0, 80.0, 30.0, 70.0, 25.0];
        let config = make_config(n, prices.clone());
        let dispatcher = P2gDispatcher::new(system, config);

        let schedule = dispatcher.optimal_electrolysis_schedule(4, 0.5);
        assert_eq!(schedule.len(), n);

        // Cheapest 4 slots: index 3 (15), 1 (20), 7 (25), 5 (30)
        let mut chosen_prices: Vec<f64> = schedule
            .iter()
            .enumerate()
            .filter_map(|(i, &active)| if active { Some(prices[i]) } else { None })
            .collect();
        chosen_prices.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        assert_eq!(chosen_prices.len(), 4);
        assert!(
            chosen_prices[0] <= 20.0,
            "Cheapest selected slot should be ≤ 20 $/MWh, got {:.2}",
            chosen_prices[0]
        );
    }

    #[test]
    fn test_p2g_tank_soc_tracked() {
        let system = make_system();
        let n = 3;
        let prices = vec![20.0; n]; // low price → electrolyze
        let mut config = make_config(n, prices);
        config.h2_demand = vec![0.0; n];

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        assert_eq!(result.tank_soc.len(), n);
        for &soc in &result.tank_soc {
            assert!(
                (0.0..=1.0).contains(&soc),
                "SoC {soc:.4} out of range [0,1]"
            );
        }
    }

    #[test]
    fn test_p2g_portfolio_multiple_systems() {
        let mut dispatchers = vec![
            P2gDispatcher::new(make_system(), make_config(4, vec![30.0, 80.0, 25.0, 90.0])),
            P2gDispatcher::new(make_system(), make_config(4, vec![30.0, 80.0, 25.0, 90.0])),
        ];

        let signal = vec![30.0, 80.0, 25.0, 90.0];
        let results = optimize_p2g_portfolio(&mut dispatchers, &signal, 1.0).expect("portfolio ok");

        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_p2g_dispatch_result_lengths() {
        let system = make_system();
        let n = 12;
        let prices = vec![50.0; n];
        let config = make_config(n, prices);

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("dispatch should succeed");

        assert_eq!(result.time_slots.len(), n);
        assert_eq!(result.electrolysis_power_mw.len(), n);
        assert_eq!(result.fuel_cell_power_mw.len(), n);
        assert_eq!(result.h2_production_kg.len(), n);
        assert_eq!(result.h2_delivered_kg.len(), n);
        assert_eq!(result.carbon_emissions_kg.len(), n);
    }

    #[test]
    fn test_p2g_marginal_cost_positive() {
        let system = make_system();
        let config = make_config(4, vec![50.0; 4]);
        let dispatcher = P2gDispatcher::new(system, config);
        let cost = dispatcher.marginal_h2_cost();
        assert!(
            cost > 0.0 && cost.is_finite(),
            "Marginal H2 cost should be positive finite, got {cost}"
        );
    }

    #[test]
    fn test_p2g_carbon_minimization_mode() {
        let system = make_system();
        let n = 6;
        // Carbon intensities: alternating high/low
        let mut config = P2gDispatchConfig {
            dt_hours: 1.0,
            electricity_price: vec![50.0; n],
            h2_demand: vec![0.0; n],
            grid_carbon_intensity: vec![500.0, 100.0, 500.0, 100.0, 500.0, 100.0],
            min_h2_supply_fraction: 0.0,
            optimize_mode: P2gOptimizeMode::CarbonMinimization,
            low_price_threshold: 0.0,
            high_price_threshold: 0.0,
        };
        config.electricity_price = vec![50.0; n];

        let dispatcher = P2gDispatcher::new(system, config);
        let result = dispatcher.optimize().expect("carbon mode dispatch ok");

        // Electrolysis should prefer low-carbon slots
        // Low carbon slots: 1, 3, 5 (100 gCO2/kWh)
        let high_carbon_elz: f64 = [0, 2, 4]
            .iter()
            .map(|&i| result.electrolysis_power_mw[i])
            .sum();
        let low_carbon_elz: f64 = [1, 3, 5]
            .iter()
            .map(|&i| result.electrolysis_power_mw[i])
            .sum();
        assert!(
            low_carbon_elz >= high_carbon_elz,
            "More electrolysis in low-carbon slots ({low_carbon_elz:.3}) vs high-carbon ({high_carbon_elz:.3})"
        );
    }
}
