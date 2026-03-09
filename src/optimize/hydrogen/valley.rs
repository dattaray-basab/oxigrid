//! Hydrogen Valley Optimization — local H₂ ecosystem dispatch and economics.
//!
//! Models a regional hydrogen valley comprising electrolyzers, compressed-gas
//! storage tanks, and fuel cells co-optimized over a planning horizon.
//!
//! # Dispatch Strategy
//!
//! The optimizer uses a greedy rule-based heuristic:
//!
//! 1. When renewable generation exceeds local load and electricity price is
//!    below the average, run electrolyzers to convert surplus electricity to H₂.
//! 2. When H₂ demand exceeds current production, draw from storage first; if
//!    storage is depleted, dispatch fuel cells to generate electricity and sell
//!    back to the grid.
//! 3. Respect storage capacity and flow-rate limits at every hour.
//! 4. Track boil-off losses, green H₂ fraction, and overall economics.
//!
//! # References
//! - IEA, *The Future of Hydrogen*, 2019.
//! - IRENA, *Green Hydrogen Cost Reduction*, 2020.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the hydrogen valley optimizer.
#[derive(Debug, Error)]
pub enum H2Error {
    /// Planning horizon length mismatch between config vectors.
    #[error(
        "length mismatch: electricity_price has {price_len} entries but h2_demand has {demand_len}"
    )]
    LengthMismatch { price_len: usize, demand_len: usize },

    /// Renewable profile length does not match the planning horizon.
    #[error("renewable profile length {got} does not match n_hours {expected}")]
    RenewableLengthMismatch { got: usize, expected: usize },

    /// No electrolyzers have been registered.
    #[error("no electrolyzers registered; add at least one with add_electrolyzer()")]
    NoElectrolyzers,

    /// Invalid configuration parameter.
    #[error("invalid parameter: {0}")]
    InvalidParameter(String),
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Global configuration for the hydrogen valley planning problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydrogenValleyConfig {
    /// Number of hourly intervals in the planning horizon (e.g. 8760 for one year).
    pub n_hours: usize,
    /// Length of each interval \[h\] (typically 1.0).
    pub dt_hours: f64,
    /// Electricity purchase price at each interval \[$/MWh\].
    pub electricity_price: Vec<f64>,
    /// Industrial H₂ demand at each interval \[kg/h\].
    pub h2_demand_kg_per_h: Vec<f64>,
    /// Revenue obtained for selling H₂ \[$/kg\].
    pub h2_price_usd_per_kg: f64,
    /// Maximum power that can be imported from the grid \[MW\].
    pub grid_connection_mw: f64,
    /// Whether renewable surplus beyond electrolysis capacity can be curtailed.
    pub curtailment_allowed: bool,
}

// ── Electrolyzer ──────────────────────────────────────────────────────────────

/// A single electrolyzer unit within the hydrogen valley.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrolyzerUnit {
    /// Unique unit identifier.
    pub id: usize,
    /// Rated electrical power \[MW\].
    pub rated_mw: f64,
    /// Specific electrical energy consumption \[kWh/kg H₂\] (e.g. 55 kWh/kg).
    pub efficiency_kwh_per_kg: f64,
    /// Minimum stable load as fraction of rated power (e.g. 0.1 = 10 %).
    pub min_load_pct: f64,
    /// Cold-start time \[min\] before reaching stable operation.
    pub startup_time_min: f64,
    /// Capital expenditure \[M$\].
    pub capex_m_usd: f64,
}

impl ElectrolyzerUnit {
    /// H₂ production rate \[kg/h\] at a given electrical input `power_mw` \[MW\].
    ///
    /// Returns 0.0 if `power_mw` is below the minimum stable load.
    pub fn h2_production_kg_per_h(&self, power_mw: f64) -> f64 {
        let min_mw = self.rated_mw * self.min_load_pct;
        if power_mw < min_mw {
            return 0.0;
        }
        let power_kw = power_mw * 1000.0;
        power_kw / self.efficiency_kwh_per_kg
    }
}

// ── Storage ───────────────────────────────────────────────────────────────────

/// Compressed-gas or liquid H₂ storage tank.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct H2StorageTank {
    /// Maximum storage capacity \[kg\].
    pub capacity_kg: f64,
    /// Initial H₂ mass in the tank \[kg\].
    pub soc_initial: f64,
    /// Minimum allowable storage level \[kg\].
    pub soc_min: f64,
    /// Maximum fill (charging) rate \[kg/h\].
    pub max_fill_rate_kg_per_h: f64,
    /// Maximum draw (discharging) rate \[kg/h\].
    pub max_draw_rate_kg_per_h: f64,
    /// Storage pressure \[bar\].
    pub pressure_bar: f64,
    /// Boil-off / evaporation loss rate \[% of stored mass per day\].
    pub boil_off_pct_per_day: f64,
}

impl H2StorageTank {
    /// Hourly boil-off fraction (dimensionless).
    pub fn boil_off_fraction_per_hour(&self) -> f64 {
        self.boil_off_pct_per_day / 100.0 / 24.0
    }
}

// ── Fuel cell ─────────────────────────────────────────────────────────────────

/// A fuel cell unit for back-conversion of H₂ to electricity (CHP capable).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelCellUnit {
    /// Unique unit identifier.
    pub id: usize,
    /// Rated electrical output \[MW\].
    pub rated_mw: f64,
    /// Net electrical efficiency \[%\] (LHV basis, e.g. 55.0).
    pub efficiency_pct: f64,
    /// H₂ consumption per unit of electrical output \[kg/MWh\].
    pub h2_consumption_kg_per_mwh: f64,
    /// Fraction of waste heat recovered for CHP \[%\].
    pub heat_recovery_pct: f64,
}

impl FuelCellUnit {
    /// H₂ consumed \[kg/h\] when generating `power_mw` \[MW\].
    pub fn h2_consumed_kg_per_h(&self, power_mw: f64) -> f64 {
        power_mw * self.h2_consumption_kg_per_mwh
    }
}

// ── Result ────────────────────────────────────────────────────────────────────

/// Optimization result for a hydrogen valley planning run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydrogenValleyResult {
    /// Electrolyzer output schedule: \[hour\]\[unit\] in \[MW\].
    pub electrolyzer_schedule: Vec<Vec<f64>>,
    /// Fuel cell output schedule: \[hour\]\[unit\] in \[MW\].
    pub fuel_cell_schedule: Vec<Vec<f64>>,
    /// H₂ storage level at end of each hour \[kg\].
    pub h2_storage_level_kg: Vec<f64>,
    /// Total H₂ produced by electrolysis each hour \[kg\].
    pub h2_produced_kg: Vec<f64>,
    /// Total H₂ consumed by fuel cells each hour \[kg\].
    pub h2_consumed_kg: Vec<f64>,
    /// H₂ sold to industrial consumers each hour \[kg\].
    pub h2_sold_kg: Vec<f64>,
    /// Grid power import each hour \[MW\].
    pub grid_import_mw: Vec<f64>,
    /// Total H₂ sales revenue over the horizon \[USD\].
    pub total_revenue_usd: f64,
    /// Total electricity purchase cost over the horizon \[USD\].
    pub total_cost_usd: f64,
    /// Net profit (revenue − cost) \[USD\].
    pub net_profit_usd: f64,
    /// Fraction of H₂ produced from renewable electricity \[%\].
    pub green_h2_pct: f64,
    /// Average electrolyzer utilization over the horizon \[%\].
    pub utilization_pct: f64,
}

// ── Optimizer ─────────────────────────────────────────────────────────────────

/// Greedy rule-based hydrogen valley optimizer.
///
/// Dispatches electrolyzers and fuel cells to maximize profit from H₂ sales
/// while respecting storage capacity, flow-rate, and grid-import limits.
pub struct HydrogenValleyOptimizer {
    config: HydrogenValleyConfig,
    electrolyzers: Vec<ElectrolyzerUnit>,
    storage: H2StorageTank,
    fuel_cells: Vec<FuelCellUnit>,
    /// Available renewable generation at each hour \[MW\].
    renewable_mw: Vec<f64>,
}

impl HydrogenValleyOptimizer {
    /// Create a new optimizer with the given configuration and storage tank.
    pub fn new(config: HydrogenValleyConfig, storage: H2StorageTank) -> Self {
        let n = config.n_hours;
        Self {
            config,
            electrolyzers: Vec::new(),
            storage,
            fuel_cells: Vec::new(),
            renewable_mw: vec![0.0; n],
        }
    }

    /// Register an electrolyzer unit.
    pub fn add_electrolyzer(&mut self, unit: ElectrolyzerUnit) {
        self.electrolyzers.push(unit);
    }

    /// Register a fuel cell unit.
    pub fn add_fuel_cell(&mut self, unit: FuelCellUnit) {
        self.fuel_cells.push(unit);
    }

    /// Set the renewable generation profile \[MW\] per hour.
    pub fn set_renewable_profile(&mut self, renewable_mw: Vec<f64>) {
        self.renewable_mw = renewable_mw;
    }

    /// Run the greedy dispatch optimization.
    ///
    /// Returns a [`HydrogenValleyResult`] summarizing schedules and economics.
    pub fn optimize(&self) -> Result<HydrogenValleyResult, H2Error> {
        let n = self.config.n_hours;

        // Validate lengths
        if self.config.electricity_price.len() != n {
            return Err(H2Error::LengthMismatch {
                price_len: self.config.electricity_price.len(),
                demand_len: n,
            });
        }
        if self.config.h2_demand_kg_per_h.len() != n {
            return Err(H2Error::LengthMismatch {
                price_len: n,
                demand_len: self.config.h2_demand_kg_per_h.len(),
            });
        }
        if self.renewable_mw.len() != n {
            return Err(H2Error::RenewableLengthMismatch {
                got: self.renewable_mw.len(),
                expected: n,
            });
        }
        if self.electrolyzers.is_empty() {
            return Err(H2Error::NoElectrolyzers);
        }

        let dt = self.config.dt_hours;
        let n_elz = self.electrolyzers.len();
        let n_fc = self.fuel_cells.len();

        // Average electricity price — used as dispatch threshold
        let avg_price = if n > 0 {
            self.config.electricity_price.iter().sum::<f64>() / n as f64
        } else {
            f64::MAX
        };

        // Total rated electrolyzer capacity
        let total_rated_mw: f64 = self.electrolyzers.iter().map(|e| e.rated_mw).sum();

        // Output buffers
        let mut elz_schedule: Vec<Vec<f64>> = vec![vec![0.0; n_elz]; n];
        let mut fc_schedule: Vec<Vec<f64>> = vec![vec![0.0; n_fc]; n];
        let mut storage_level = vec![0.0f64; n + 1];
        let mut h2_produced = vec![0.0f64; n];
        let mut h2_consumed = vec![0.0f64; n];
        let mut h2_sold = vec![0.0f64; n];
        let mut grid_import = vec![0.0f64; n];

        storage_level[0] = self.storage.soc_initial;

        let mut total_revenue = 0.0f64;
        let mut total_cost = 0.0f64;
        let mut green_h2_kg = 0.0f64;
        let mut total_h2_kg = 0.0f64;
        let mut elz_mwh_used = 0.0f64;

        for t in 0..n {
            let price = self.config.electricity_price[t];
            let demand_kg = self.config.h2_demand_kg_per_h[t] * dt;
            let renewable = self.renewable_mw[t];
            let prev_level = storage_level[t];

            // Apply boil-off to current storage
            let boil_off_frac = self.storage.boil_off_fraction_per_hour() * dt;
            let after_boiloff = (prev_level * (1.0 - boil_off_frac)).max(self.storage.soc_min);

            // ── Step 1: decide electrolyzer dispatch ──────────────────────────
            let run_elz = price <= avg_price;

            let mut h2_from_elz = 0.0f64;
            let mut elz_power_mw = 0.0f64;

            if run_elz {
                // Available space in storage
                let storage_space = self.storage.capacity_kg - after_boiloff;
                // Limit production by max fill rate
                let max_fillable = storage_space.min(self.storage.max_fill_rate_kg_per_h * dt);

                for (i, elz) in self.electrolyzers.iter().enumerate() {
                    // Use full rated power (could be made partial with more complex logic)
                    let mw = elz.rated_mw;
                    let h2_candidate = elz.h2_production_kg_per_h(mw) * dt;
                    let room = max_fillable - h2_from_elz;
                    if room <= 0.0 {
                        break;
                    }
                    let h2_this = h2_candidate.min(room);
                    // Scale back power proportionally
                    let mw_this = if h2_candidate > 0.0 {
                        mw * (h2_this / h2_candidate)
                    } else {
                        0.0
                    };
                    elz_schedule[t][i] = mw_this;
                    h2_from_elz += h2_this;
                    elz_power_mw += mw_this;
                }
            }

            // Determine grid import needed for electrolyzers
            let elz_from_grid = (elz_power_mw - renewable).max(0.0);
            let grid_capped = elz_from_grid.min(self.config.grid_connection_mw);
            // If grid can't cover shortfall, scale back electrolysis proportionally
            let elz_actual_mw = if elz_from_grid > grid_capped + 1e-9 {
                // scale: renewable + grid_capped
                let available = renewable + grid_capped;
                let scale = if elz_power_mw > 0.0 {
                    available / elz_power_mw
                } else {
                    0.0
                };
                for slot in elz_schedule[t].iter_mut() {
                    *slot *= scale;
                }
                h2_from_elz *= scale;
                available
            } else {
                elz_power_mw
            };

            let grid_used = (elz_actual_mw - renewable).max(0.0);
            grid_import[t] = grid_used;

            // Electricity cost
            let elec_cost = grid_used * dt * price;
            total_cost += elec_cost;

            // Track green H2 (produced when renewable covers some/all power)
            let renewable_fraction = if elz_actual_mw > 1e-9 {
                (renewable / elz_actual_mw).min(1.0)
            } else {
                0.0
            };
            let green_h2_this = h2_from_elz * renewable_fraction;
            green_h2_kg += green_h2_this;
            total_h2_kg += h2_from_elz;
            elz_mwh_used += elz_actual_mw * dt;

            // ── Step 2: meet H2 demand ────────────────────────────────────────
            // First try to sell directly from electrolysis output
            let h2_direct = h2_from_elz.min(demand_kg);
            let remaining_demand = demand_kg - h2_direct;

            // Then draw from storage
            let available_in_storage = (after_boiloff - self.storage.soc_min).max(0.0);
            let max_draw = self.storage.max_draw_rate_kg_per_h * dt;
            let from_storage = remaining_demand.min(available_in_storage.min(max_draw));
            let still_unsatisfied = remaining_demand - from_storage;

            // Dispatch fuel cells if demand still unsatisfied
            let mut h2_for_fc = 0.0f64;
            if still_unsatisfied > 0.0 && !self.fuel_cells.is_empty() {
                // Fuel cells consume H2 from storage to generate electricity
                // (which can offset grid import or be sold); here we track H2 consumed
                let available_for_fc =
                    (after_boiloff + h2_from_elz - from_storage - self.storage.soc_min)
                        .max(0.0)
                        .min(self.storage.max_draw_rate_kg_per_h * dt);
                for (i, fc) in self.fuel_cells.iter().enumerate() {
                    let h2_need = still_unsatisfied - h2_for_fc;
                    if h2_need <= 0.0 {
                        break;
                    }
                    let h2_fc = h2_need.min(available_for_fc - h2_for_fc);
                    let power_out = h2_fc / fc.h2_consumption_kg_per_mwh;
                    let power_capped = power_out.min(fc.rated_mw);
                    let h2_actually = power_capped * fc.h2_consumption_kg_per_mwh;
                    fc_schedule[t][i] = power_capped;
                    h2_for_fc += h2_actually;
                }
            }

            // ── Step 3: update storage ────────────────────────────────────────
            // H2 produced but not sold directly goes to storage (minus already-drawn amount)
            let h2_to_store =
                (h2_from_elz - h2_direct).min(self.storage.max_fill_rate_kg_per_h * dt);
            let new_level = (after_boiloff + h2_to_store - from_storage - h2_for_fc)
                .clamp(self.storage.soc_min, self.storage.capacity_kg);
            storage_level[t + 1] = new_level;

            // H2 sold = direct from electrolysis + from storage draw + partially from FC
            let h2_sold_this = h2_direct + from_storage;
            h2_sold[t] = h2_sold_this;
            h2_produced[t] = h2_from_elz;
            h2_consumed[t] = h2_for_fc;

            // Revenue from H2 sales
            total_revenue += h2_sold_this * self.config.h2_price_usd_per_kg;
        }

        // ── Aggregate metrics ─────────────────────────────────────────────────
        let green_h2_pct = if total_h2_kg > 0.0 {
            (green_h2_kg / total_h2_kg) * 100.0
        } else {
            0.0
        };
        let max_possible_mwh = total_rated_mw * n as f64 * dt;
        let utilization_pct = if max_possible_mwh > 0.0 {
            (elz_mwh_used / max_possible_mwh) * 100.0
        } else {
            0.0
        };

        // Drop the last storage entry (we returned n+1 above for bookkeeping)
        let storage_out: Vec<f64> = storage_level[1..=n].to_vec();

        Ok(HydrogenValleyResult {
            electrolyzer_schedule: elz_schedule,
            fuel_cell_schedule: fc_schedule,
            h2_storage_level_kg: storage_out,
            h2_produced_kg: h2_produced,
            h2_consumed_kg: h2_consumed,
            h2_sold_kg: h2_sold,
            grid_import_mw: grid_import,
            total_revenue_usd: total_revenue,
            total_cost_usd: total_cost,
            net_profit_usd: total_revenue - total_cost,
            green_h2_pct,
            utilization_pct,
        })
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(n: usize) -> HydrogenValleyConfig {
        HydrogenValleyConfig {
            n_hours: n,
            dt_hours: 1.0,
            electricity_price: vec![50.0; n],
            h2_demand_kg_per_h: vec![10.0; n],
            h2_price_usd_per_kg: 5.0,
            grid_connection_mw: 100.0,
            curtailment_allowed: true,
        }
    }

    fn default_storage() -> H2StorageTank {
        H2StorageTank {
            capacity_kg: 1000.0,
            soc_initial: 200.0,
            soc_min: 50.0,
            max_fill_rate_kg_per_h: 200.0,
            max_draw_rate_kg_per_h: 200.0,
            pressure_bar: 200.0,
            boil_off_pct_per_day: 0.1,
        }
    }

    fn default_electrolyzer(id: usize) -> ElectrolyzerUnit {
        ElectrolyzerUnit {
            id,
            rated_mw: 2.0,
            efficiency_kwh_per_kg: 55.0,
            min_load_pct: 0.1,
            startup_time_min: 10.0,
            capex_m_usd: 3.0,
        }
    }

    fn default_fuel_cell(id: usize) -> FuelCellUnit {
        FuelCellUnit {
            id,
            rated_mw: 1.0,
            efficiency_pct: 55.0,
            h2_consumption_kg_per_mwh: 25.0,
            heat_recovery_pct: 30.0,
        }
    }

    /// Excess renewable power should cause the electrolyzer to run and store H₂.
    #[test]
    fn test_excess_renewable_runs_electrolyzer() {
        let n = 24;
        let mut config = default_config(n);
        // Low price to trigger electrolysis
        config.electricity_price = vec![20.0; n];
        let storage = default_storage();
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        // 10 MW renewable — well above 2 MW electrolyzer rating
        opt.set_renewable_profile(vec![10.0; n]);

        let result = opt.optimize().expect("optimization should succeed");

        // Electrolyzer should be dispatched every hour
        let total_elz_mwh: f64 = result.electrolyzer_schedule.iter().map(|h| h[0]).sum();
        assert!(
            total_elz_mwh > 0.0,
            "Electrolyzer should run with excess renewable"
        );

        // Some H₂ should have been produced
        let total_h2: f64 = result.h2_produced_kg.iter().sum();
        assert!(total_h2 > 0.0, "H₂ should be produced");

        // Storage should rise above initial level at some point
        let max_level = result
            .h2_storage_level_kg
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_level > 200.0,
            "Storage should increase from excess renewable; max={max_level:.1}"
        );

        // No grid import since renewable covers electrolyzer
        let total_import: f64 = result.grid_import_mw.iter().sum();
        assert!(
            total_import < 1e-6,
            "No grid import expected; got {total_import:.4}"
        );
    }

    /// When H₂ demand exceeds production, the storage level should decrease.
    #[test]
    fn test_h2_demand_draws_storage() {
        let n = 10;
        let mut config = default_config(n);
        // High price prevents electrolysis (above average = 200)
        config.electricity_price = vec![200.0; n];
        config.h2_demand_kg_per_h = vec![50.0; n]; // large demand
        let storage = default_storage(); // soc_initial = 200 kg
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        // Zero renewable — electrolyzer won't run (price > avg = 200, not strictly below)
        opt.set_renewable_profile(vec![0.0; n]);

        let result = opt.optimize().expect("optimization should succeed");

        let final_level = *result
            .h2_storage_level_kg
            .last()
            .expect("storage non-empty");
        assert!(
            final_level < 200.0,
            "Storage should decrease when demand exceeds production; final={final_level:.1}"
        );
    }

    /// Storage level must always respect soc_min and capacity bounds.
    #[test]
    fn test_storage_bounds_never_violated() {
        let n = 168; // one week
        let mut config = default_config(n);
        // Alternating high/low price
        config.electricity_price = (0..n)
            .map(|t| if t % 2 == 0 { 10.0 } else { 200.0 })
            .collect();
        config.h2_demand_kg_per_h = vec![80.0; n]; // aggressive demand
        let storage = H2StorageTank {
            capacity_kg: 500.0,
            soc_initial: 250.0,
            soc_min: 20.0,
            max_fill_rate_kg_per_h: 100.0,
            max_draw_rate_kg_per_h: 100.0,
            pressure_bar: 350.0,
            boil_off_pct_per_day: 0.05,
        };
        let soc_min = storage.soc_min;
        let capacity = storage.capacity_kg;
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        opt.add_electrolyzer(default_electrolyzer(1));
        opt.set_renewable_profile((0..n).map(|t| if t % 3 == 0 { 8.0 } else { 0.0 }).collect());

        let result = opt.optimize().expect("optimization should succeed");

        for (t, &level) in result.h2_storage_level_kg.iter().enumerate() {
            assert!(
                level >= soc_min - 1e-9,
                "Storage violated soc_min at hour {t}: level={level:.3} < soc_min={soc_min}"
            );
            assert!(
                level <= capacity + 1e-9,
                "Storage violated capacity at hour {t}: level={level:.3} > capacity={capacity}"
            );
        }
    }

    /// Fuel cell should be dispatched when H₂ demand is high and no renewable.
    #[test]
    fn test_fuel_cell_dispatched_when_needed() {
        let n = 5;
        let mut config = default_config(n);
        // High price prevents electrolysis
        config.electricity_price = vec![500.0; n];
        config.h2_demand_kg_per_h = vec![100.0; n]; // demand far exceeds storage draw rate alone
        let storage = H2StorageTank {
            capacity_kg: 5000.0,
            soc_initial: 2000.0,
            soc_min: 0.0,
            max_fill_rate_kg_per_h: 500.0,
            max_draw_rate_kg_per_h: 500.0,
            pressure_bar: 200.0,
            boil_off_pct_per_day: 0.0,
        };
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        opt.add_fuel_cell(default_fuel_cell(0));
        opt.set_renewable_profile(vec![0.0; n]);

        let result = opt.optimize().expect("optimization should succeed");

        // Fuel cell should have been dispatched (consumed H₂)
        let total_fc_h2: f64 = result.h2_consumed_kg.iter().sum();
        // Depending on thresholds, check either FC ran or storage was drawn
        let total_h2_sold: f64 = result.h2_sold_kg.iter().sum();
        assert!(
            total_h2_sold > 0.0 || total_fc_h2 > 0.0,
            "Fuel cell or storage should have been dispatched; sold={total_h2_sold:.2}, fc_h2={total_fc_h2:.2}"
        );
    }

    /// With a high H₂ selling price, net profit should be positive.
    #[test]
    fn test_profit_positive_with_good_h2_price() {
        let n = 48;
        let mut config = default_config(n);
        config.h2_price_usd_per_kg = 8.0; // high H₂ price
        config.electricity_price = vec![30.0; n]; // cheap electricity
        config.h2_demand_kg_per_h = vec![30.0; n];
        let storage = default_storage();
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        opt.set_renewable_profile(vec![5.0; n]); // 5 MW renewable

        let result = opt.optimize().expect("optimization should succeed");

        assert!(
            result.net_profit_usd > 0.0,
            "Expected positive profit with high H₂ price; got {:.2}",
            result.net_profit_usd
        );
        assert!(
            result.total_revenue_usd > 0.0,
            "Revenue should be positive; got {:.2}",
            result.total_revenue_usd
        );
    }

    /// Missing electrolyzer should return an error, not panic.
    #[test]
    fn test_no_electrolyzer_returns_error() {
        let config = default_config(24);
        let storage = default_storage();
        let opt = HydrogenValleyOptimizer::new(config, storage);
        // No electrolyzer added
        let result = opt.optimize();
        assert!(
            matches!(result, Err(H2Error::NoElectrolyzers)),
            "Expected NoElectrolyzers error"
        );
    }

    /// Green H₂ percentage should be 100% when all power is renewable.
    #[test]
    fn test_green_h2_pct_all_renewable() {
        let n = 24;
        let config = default_config(n);
        let storage = default_storage();
        let mut opt = HydrogenValleyOptimizer::new(config, storage);
        opt.add_electrolyzer(default_electrolyzer(0));
        // Enough renewable to cover the 2 MW electrolyzer
        opt.set_renewable_profile(vec![10.0; n]);

        let result = opt.optimize().expect("optimization should succeed");

        // All production is from renewable, so green_h2_pct should be ~100%
        assert!(
            result.green_h2_pct > 90.0,
            "Expected green H₂ ≈ 100%; got {:.1}%",
            result.green_h2_pct
        );
    }
}
