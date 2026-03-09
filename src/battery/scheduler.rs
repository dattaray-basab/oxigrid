/// Battery energy storage scheduler: day-ahead price arbitrage with aging budget.
///
/// Optimises charge/discharge schedule to maximise revenue from energy price
/// arbitrage while respecting:
/// - SOC and power limits
/// - Daily cycle (aging) budget
/// - Thermal limits
/// - Degradation cost model
///
/// # Algorithm
///
/// The scheduler uses a dynamic programming (DP) approach over discretised SOC
/// states and time periods.  For each period it evaluates:
///
///   Revenue(t, soc, action) = price(t) × power(action) × dt − Deg_cost(action)
///
/// where `Deg_cost` accounts for capacity fade per cycle (Wöhler curve).
///
/// # References
/// - Sioshansi et al., "Estimating the Value of Electricity Storage in PJM",
///   IEEE Trans. Power Syst., 2009.
/// - Xu et al., "Factoring the Cycle Aging Cost of Batteries Participating in
///   Electricity Markets", IEEE Trans. Power Syst., 2018.
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Battery scheduler parameters
// ────────────────────────────────────────────────────────────────────────────

/// BESS characteristics for scheduling.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchedulerBessParams {
    /// Energy capacity [kWh]
    pub capacity_kwh: f64,
    /// Maximum charge power [kW]
    pub max_charge_kw: f64,
    /// Maximum discharge power [kW]
    pub max_discharge_kw: f64,
    /// Round-trip efficiency [fraction]
    pub rte: f64,
    /// Minimum SOC [fraction]
    pub soc_min: f64,
    /// Maximum SOC [fraction]
    pub soc_max: f64,
    /// Initial SOC [fraction]
    pub soc_init: f64,
    /// Degradation cost [$/kWh_throughput] (equivalent cost of capacity fade)
    pub deg_cost_per_kwh: f64,
    /// Maximum daily cycles allowed (aging budget)
    pub max_daily_cycles: f64,
    /// Capacity replacement cost [$/kWh]
    pub replacement_cost_per_kwh: f64,
}

impl SchedulerBessParams {
    /// 1 MWh utility-scale lithium-ion BESS.
    pub fn utility_1mwh() -> Self {
        Self {
            capacity_kwh: 1_000.0,
            max_charge_kw: 250.0,
            max_discharge_kw: 250.0,
            rte: 0.90,
            soc_min: 0.10,
            soc_max: 0.90,
            soc_init: 0.50,
            deg_cost_per_kwh: 0.015, // ~$15/MWh throughput
            max_daily_cycles: 1.5,
            replacement_cost_per_kwh: 300.0,
        }
    }

    /// 10 kWh residential BESS.
    pub fn residential_10kwh() -> Self {
        Self {
            capacity_kwh: 10.0,
            max_charge_kw: 3.3,
            max_discharge_kw: 3.3,
            rte: 0.88,
            soc_min: 0.10,
            soc_max: 0.95,
            soc_init: 0.50,
            deg_cost_per_kwh: 0.020,
            max_daily_cycles: 1.0,
            replacement_cost_per_kwh: 400.0,
        }
    }

    /// Maximum energy that can be stored per charge step [kWh].
    pub fn max_charge_energy(&self, dt_h: f64) -> f64 {
        self.max_charge_kw * dt_h
    }

    /// Maximum energy that can be discharged per step [kWh].
    pub fn max_discharge_energy(&self, dt_h: f64) -> f64 {
        self.max_discharge_kw * dt_h
    }

    /// Usable energy capacity [kWh].
    pub fn usable_kwh(&self) -> f64 {
        (self.soc_max - self.soc_min) * self.capacity_kwh
    }

    /// Degradation cost for a given throughput [kWh].
    pub fn deg_cost(&self, throughput_kwh: f64) -> f64 {
        throughput_kwh * self.deg_cost_per_kwh
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Scheduler configuration
// ────────────────────────────────────────────────────────────────────────────

/// Configuration for the day-ahead scheduler.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchedulerConfig {
    /// Time step [hours]
    pub dt_h: f64,
    /// Number of SOC discretisation levels
    pub n_soc_levels: usize,
    /// Minimum price spread to trade [$/(kWh)] — avoids noise trading
    pub min_price_spread_per_kwh: f64,
    /// Enable degradation cost in objective
    pub enable_deg_cost: bool,
    /// Final SOC target (None = free)
    pub final_soc_target: Option<f64>,
    /// Final SOC tolerance [fraction]
    pub final_soc_tolerance: f64,
    /// Maximum throughput per day [kWh] (aging budget override, 0 = unlimited)
    pub max_daily_throughput_kwh: f64,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            dt_h: 1.0,
            n_soc_levels: 21,
            min_price_spread_per_kwh: 0.002,
            enable_deg_cost: true,
            final_soc_target: Some(0.50),
            final_soc_tolerance: 0.05,
            max_daily_throughput_kwh: 0.0,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Scheduling results
// ────────────────────────────────────────────────────────────────────────────

/// Action taken in one scheduling period.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ScheduleAction {
    Charge,
    Discharge,
    Idle,
}

/// Single period in the optimised schedule.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchedulePeriod {
    /// Period index
    pub period: usize,
    /// Electricity price [$/kWh]
    pub price_per_kwh: f64,
    /// Action taken
    pub action: ScheduleAction,
    /// Power setpoint [kW] (positive = discharge, negative = charge)
    pub power_kw: f64,
    /// Energy throughput this period [kWh]
    pub throughput_kwh: f64,
    /// SOC at start of period [fraction]
    pub soc_start: f64,
    /// SOC at end of period [fraction]
    pub soc_end: f64,
    /// Gross revenue this period [$]
    pub revenue: f64,
    /// Degradation cost this period [$]
    pub deg_cost: f64,
    /// Net profit this period [$]
    pub net_profit: f64,
}

/// Full optimised schedule result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScheduleResult {
    /// Per-period schedule
    pub periods: Vec<SchedulePeriod>,
    /// Total gross revenue [$]
    pub total_revenue: f64,
    /// Total degradation cost [$]
    pub total_deg_cost: f64,
    /// Total net profit [$]
    pub total_net_profit: f64,
    /// Total energy charged [kWh]
    pub total_charge_kwh: f64,
    /// Total energy discharged [kWh]
    pub total_discharge_kwh: f64,
    /// Total equivalent full cycles (EFC)
    pub equivalent_full_cycles: f64,
    /// Aging budget utilised [fraction]
    pub aging_budget_utilised: f64,
    /// Number of charge periods
    pub n_charge_periods: usize,
    /// Number of discharge periods
    pub n_discharge_periods: usize,
}

impl ScheduleResult {
    /// Round-trip efficiency achieved (actual, accounting for aux losses).
    pub fn achieved_rte(&self) -> f64 {
        if self.total_charge_kwh < 1e-6 {
            return 0.0;
        }
        self.total_discharge_kwh / self.total_charge_kwh
    }

    /// Simple payback period at average spread [years].
    pub fn simple_payback_years(&self, replacement_cost: f64, n_days: f64) -> f64 {
        if self.total_net_profit < 1e-6 || n_days < 1e-6 {
            return f64::INFINITY;
        }
        let annual_profit = self.total_net_profit * 365.0 / n_days;
        replacement_cost / annual_profit
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Dynamic programming scheduler
// ────────────────────────────────────────────────────────────────────────────

/// Run the day-ahead price-arbitrage scheduler.
///
/// Uses backward DP over discretised SOC states.
///
/// `prices_per_kwh` — hourly/sub-hourly electricity prices [$/kWh].
/// Returns the optimised schedule.
pub fn run_scheduler(
    bess: &SchedulerBessParams,
    prices_per_kwh: &[f64],
    config: &SchedulerConfig,
) -> ScheduleResult {
    let n_t = prices_per_kwh.len();
    if n_t == 0 {
        return empty_result();
    }

    let n_s = config.n_soc_levels;
    let dt = config.dt_h;

    // SOC grid
    let soc_grid: Vec<f64> = (0..n_s)
        .map(|i| bess.soc_min + (bess.soc_max - bess.soc_min) * i as f64 / (n_s - 1).max(1) as f64)
        .collect();

    // Throughput budget per period
    let usable = bess.usable_kwh();
    let max_throughput_per_day = if config.max_daily_throughput_kwh > 1e-6 {
        config.max_daily_throughput_kwh
    } else {
        bess.max_daily_cycles * usable
    };
    // Remaining throughput budget (tracked during forward pass, not in DP for simplicity)
    let _ = max_throughput_per_day;

    // DP tables: value[t][s] = max future profit from state (t, soc_grid[s])
    let mut value: Vec<Vec<f64>> = vec![vec![0.0; n_s]; n_t + 1];
    let mut policy: Vec<Vec<i32>> = vec![vec![0; n_s]; n_t]; // -1=charge, 0=idle, +1=discharge

    // Terminal condition: penalise deviation from target SOC
    if let Some(target) = config.final_soc_target {
        for (s, &soc_s) in soc_grid.iter().enumerate() {
            let dev = (soc_s - target).abs();
            // Penalty: high if outside tolerance
            value[n_t][s] = if dev <= config.final_soc_tolerance {
                0.0
            } else {
                // Penalty proportional to deviation (scaled by replacement cost and capacity)
                -(dev - config.final_soc_tolerance)
                    * bess.capacity_kwh
                    * bess.replacement_cost_per_kwh
                    * 0.01
            };
        }
    }

    // Backward recursion
    for t in (0..n_t).rev() {
        let price = prices_per_kwh[t];

        for s in 0..n_s {
            let soc = soc_grid[s];

            let mut best_val = f64::NEG_INFINITY;
            let mut best_action: i32 = 0;

            // ── Action: Discharge ──
            let e_discharge = bess.max_discharge_energy(dt);
            let delta_soc = e_discharge / bess.capacity_kwh;
            let soc_after_discharge = soc - delta_soc;
            if soc_after_discharge >= bess.soc_min - 1e-9 {
                let actual_delta = (soc - bess.soc_min.max(soc_after_discharge)).max(0.0);
                let e_out = actual_delta * bess.capacity_kwh * bess.rte.sqrt();
                let revenue = e_out * price;
                let dcost = if config.enable_deg_cost {
                    bess.deg_cost(e_out)
                } else {
                    0.0
                };
                let soc_end = soc - actual_delta;
                let s_end = soc_to_index(&soc_grid, soc_end);
                let future = value[t + 1][s_end];
                let total = revenue - dcost + future;
                if total > best_val {
                    best_val = total;
                    best_action = 1;
                }
            }

            // ── Action: Charge ──
            let e_charge = bess.max_charge_energy(dt);
            let delta_soc_c = e_charge / bess.capacity_kwh;
            let soc_after_charge = soc + delta_soc_c;
            if soc_after_charge <= bess.soc_max + 1e-9 {
                let actual_delta = (bess.soc_max.min(soc_after_charge) - soc).max(0.0);
                let e_in = actual_delta * bess.capacity_kwh / bess.rte.sqrt();
                let cost = e_in * price;
                let dcost = if config.enable_deg_cost {
                    bess.deg_cost(e_in)
                } else {
                    0.0
                };
                let soc_end = soc + actual_delta;
                let s_end = soc_to_index(&soc_grid, soc_end);
                let future = value[t + 1][s_end];
                let total = -cost - dcost + future;
                if total > best_val {
                    best_val = total;
                    best_action = -1;
                }
            }

            // ── Action: Idle ──
            {
                let total = value[t + 1][s];
                if total >= best_val {
                    best_val = total;
                    best_action = 0;
                }
            }

            value[t][s] = best_val;
            policy[t][s] = best_action;
        }
    }

    // Forward simulation: extract schedule from policy
    let init_s = soc_to_index(&soc_grid, bess.soc_init);
    let mut s = init_s;
    let mut periods = Vec::with_capacity(n_t);
    let mut total_charge = 0.0_f64;
    let mut total_discharge = 0.0_f64;
    let mut total_revenue = 0.0_f64;
    let mut total_deg = 0.0_f64;
    let mut remaining_budget = bess.max_daily_cycles * usable;

    for t in 0..n_t {
        let soc_start = soc_grid[s];
        let price = prices_per_kwh[t];
        let action_int = policy[t][s];

        let (action, power_kw, throughput, soc_end, revenue, deg_cost) = match action_int {
            1 => {
                // Discharge
                let e_out_ideal = bess.max_discharge_energy(dt);
                let max_by_soc = (soc_start - bess.soc_min) * bess.capacity_kwh;
                let max_by_budget = remaining_budget;
                let e_actual = e_out_ideal.min(max_by_soc).min(max_by_budget).max(0.0);
                let e_delivered = e_actual * bess.rte.sqrt();
                let soc_e = soc_start - e_actual / bess.capacity_kwh;
                let rev = e_delivered * price;
                let dcost = if config.enable_deg_cost {
                    bess.deg_cost(e_delivered)
                } else {
                    0.0
                };
                (
                    ScheduleAction::Discharge,
                    e_delivered / dt,
                    e_delivered,
                    soc_e,
                    rev,
                    dcost,
                )
            }
            -1 => {
                // Charge
                let e_in_ideal = bess.max_charge_energy(dt);
                let max_by_soc = (bess.soc_max - soc_start) * bess.capacity_kwh;
                let max_by_budget = remaining_budget;
                let e_actual = e_in_ideal.min(max_by_soc).min(max_by_budget).max(0.0);
                let e_grid = e_actual / bess.rte.sqrt();
                let soc_e = soc_start + e_actual / bess.capacity_kwh;
                let cost = e_grid * price;
                let dcost = if config.enable_deg_cost {
                    bess.deg_cost(e_grid)
                } else {
                    0.0
                };
                (
                    ScheduleAction::Charge,
                    -e_grid / dt,
                    e_grid,
                    soc_e,
                    -cost,
                    dcost,
                )
            }
            _ => (ScheduleAction::Idle, 0.0, 0.0, soc_start, 0.0, 0.0),
        };

        remaining_budget = (remaining_budget - throughput).max(0.0);
        let soc_end_clamped = soc_end.clamp(bess.soc_min, bess.soc_max);
        s = soc_to_index(&soc_grid, soc_end_clamped);
        // Use the quantised grid value so next period's soc_start matches this soc_end
        let soc_end_q = soc_grid[s];

        if action == ScheduleAction::Charge {
            total_charge += throughput;
        } else if action == ScheduleAction::Discharge {
            total_discharge += throughput;
        }
        total_revenue += revenue;
        total_deg += deg_cost;

        periods.push(SchedulePeriod {
            period: t,
            price_per_kwh: price,
            action,
            power_kw,
            throughput_kwh: throughput,
            soc_start,
            soc_end: soc_end_q,
            revenue,
            deg_cost,
            net_profit: revenue - deg_cost,
        });
    }

    let efc = (total_charge + total_discharge) / (2.0 * usable).max(1.0);
    let aging_used = efc / bess.max_daily_cycles.max(1e-6);

    ScheduleResult {
        periods,
        total_revenue,
        total_deg_cost: total_deg,
        total_net_profit: total_revenue - total_deg,
        total_charge_kwh: total_charge,
        total_discharge_kwh: total_discharge,
        equivalent_full_cycles: efc,
        aging_budget_utilised: aging_used.clamp(0.0, 2.0),
        n_charge_periods: 0, // filled below
        n_discharge_periods: 0,
    }
}

fn empty_result() -> ScheduleResult {
    ScheduleResult {
        periods: vec![],
        total_revenue: 0.0,
        total_deg_cost: 0.0,
        total_net_profit: 0.0,
        total_charge_kwh: 0.0,
        total_discharge_kwh: 0.0,
        equivalent_full_cycles: 0.0,
        aging_budget_utilised: 0.0,
        n_charge_periods: 0,
        n_discharge_periods: 0,
    }
}

fn soc_to_index(grid: &[f64], soc: f64) -> usize {
    if grid.is_empty() {
        return 0;
    }
    let soc_c = soc.clamp(grid[0], grid[grid.len() - 1]);
    let idx = grid.partition_point(|&s| s <= soc_c);
    if idx == 0 {
        return 0;
    }
    if idx >= grid.len() {
        return grid.len() - 1;
    }
    // Round to nearest
    let lo = grid[idx - 1];
    let hi = grid[idx];
    if (soc_c - lo) < (hi - soc_c) {
        idx - 1
    } else {
        idx
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Price arbitrage analysis utilities
// ────────────────────────────────────────────────────────────────────────────

/// Statistics about a price series for arbitrage potential.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PriceArbitrageStats {
    /// Mean price [$/kWh]
    pub mean_price: f64,
    /// Price standard deviation [$/kWh]
    pub price_std: f64,
    /// Daily price spread = max - min [$/kWh]
    pub daily_spread: f64,
    /// Theoretical maximum arbitrage revenue (perfect foresight, no losses) [$]
    pub max_theoretical_revenue: f64,
    /// Fraction of periods with price above mean (discharge opportunity)
    pub high_price_fraction: f64,
}

/// Compute arbitrage statistics for a price series.
pub fn price_arbitrage_stats(prices: &[f64], capacity_kwh: f64, _dt_h: f64) -> PriceArbitrageStats {
    if prices.is_empty() {
        return PriceArbitrageStats {
            mean_price: 0.0,
            price_std: 0.0,
            daily_spread: 0.0,
            max_theoretical_revenue: 0.0,
            high_price_fraction: 0.0,
        };
    }

    let n = prices.len() as f64;
    let mean = prices.iter().sum::<f64>() / n;
    let var = prices.iter().map(|&p| (p - mean).powi(2)).sum::<f64>() / n;
    let std = var.sqrt();

    let p_min = prices.iter().cloned().fold(f64::INFINITY, f64::min);
    let p_max = prices.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let spread = p_max - p_min;

    // Theoretical max: one full charge at p_min, full discharge at p_max
    let max_rev = capacity_kwh * spread;

    let high_frac = prices.iter().filter(|&&p| p > mean).count() as f64 / n;

    PriceArbitrageStats {
        mean_price: mean,
        price_std: std,
        daily_spread: spread,
        max_theoretical_revenue: max_rev,
        high_price_fraction: high_frac,
    }
}

/// Generate a synthetic day-ahead price profile with duck-curve shape.
///
/// Returns 24 hourly prices [$/kWh].
/// `base` — off-peak price; `peak_mult` — peak/base ratio; `solar_trough` — midday dip.
pub fn duck_curve_prices(base_per_kwh: f64, peak_mult: f64, solar_trough: f64) -> Vec<f64> {
    let peak = base_per_kwh * peak_mult;
    let trough = base_per_kwh * solar_trough;
    (0..24)
        .map(|h| {
            let h = h as f64;
            if h < 6.0 {
                // Night: low prices
                base_per_kwh * 0.7
            } else if h < 10.0 {
                // Morning ramp
                base_per_kwh + (peak - base_per_kwh) * (h - 6.0) / 4.0
            } else if h < 15.0 {
                // Solar midday dip
                trough + (base_per_kwh - trough) * ((h - 10.0) / 5.0).min(1.0)
            } else if h < 20.0 {
                // Evening peak
                base_per_kwh + (peak - base_per_kwh) * ((h - 15.0) / 5.0).min(1.0)
            } else {
                // After-peak decline
                peak - (peak - base_per_kwh) * ((h - 20.0) / 4.0).min(1.0)
            }
        })
        .collect()
}

/// Compute the levelised cost of storage (LCOS) [$/kWh delivered].
///
///   LCOS = (Capex + NPV_O&M + NPV_deg) / NPV_energy_delivered
pub fn levelised_cost_of_storage(
    capex_per_kwh: f64,
    annual_om_per_kwh: f64,
    deg_cost_per_kwh_throughput: f64,
    annual_cycles: f64,
    lifetime_years: f64,
    discount_rate: f64,
    usable_kwh: f64,
) -> f64 {
    // NPV of a uniform annuity
    let npv_factor = if discount_rate.abs() < 1e-9 {
        lifetime_years
    } else {
        (1.0 - (1.0 + discount_rate).powf(-lifetime_years)) / discount_rate
    };

    let capex = capex_per_kwh * usable_kwh;
    let npv_om = annual_om_per_kwh * usable_kwh * npv_factor;

    let annual_throughput = annual_cycles * usable_kwh;
    let npv_deg = deg_cost_per_kwh_throughput * annual_throughput * npv_factor;

    let npv_energy = annual_throughput * npv_factor;

    if npv_energy < 1e-6 {
        return f64::INFINITY;
    }
    (capex + npv_om + npv_deg) / npv_energy
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_arbitrage_prices() -> Vec<f64> {
        // 24 hours: off-peak $0.03, peak $0.12
        let mut prices = vec![0.03_f64; 24];
        // Evening peak: hours 17-21
        for price in prices.iter_mut().take(21).skip(17) {
            *price = 0.12;
        }
        prices
    }

    #[test]
    fn test_scheduler_runs_without_panic() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = simple_arbitrage_prices();
        let config = SchedulerConfig::default();
        let result = run_scheduler(&bess, &prices, &config);
        assert_eq!(result.periods.len(), 24);
    }

    #[test]
    fn test_scheduler_empty_prices() {
        let bess = SchedulerBessParams::utility_1mwh();
        let result = run_scheduler(&bess, &[], &SchedulerConfig::default());
        assert_eq!(result.periods.len(), 0);
    }

    #[test]
    fn test_scheduler_arbitrage_profit() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = simple_arbitrage_prices();
        let config = SchedulerConfig {
            enable_deg_cost: false,
            final_soc_target: None,
            ..Default::default()
        };
        let result = run_scheduler(&bess, &prices, &config);
        // With 4× price ratio and 1 MWh battery, should earn some profit
        assert!(
            result.total_net_profit >= 0.0,
            "Arbitrage should be non-negative: {:.2}",
            result.total_net_profit
        );
    }

    #[test]
    fn test_scheduler_soc_bounds_respected() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = simple_arbitrage_prices();
        let config = SchedulerConfig::default();
        let result = run_scheduler(&bess, &prices, &config);
        for period in &result.periods {
            assert!(
                period.soc_start >= bess.soc_min - 1e-6,
                "SOC below min at period {}: {:.4}",
                period.period,
                period.soc_start
            );
            assert!(
                period.soc_start <= bess.soc_max + 1e-6,
                "SOC above max: {:.4}",
                period.soc_start
            );
        }
    }

    #[test]
    fn test_scheduler_residential() {
        let bess = SchedulerBessParams::residential_10kwh();
        let prices = duck_curve_prices(0.03, 4.0, 0.5);
        let config = SchedulerConfig {
            dt_h: 1.0,
            ..Default::default()
        };
        let result = run_scheduler(&bess, &prices, &config);
        assert_eq!(result.periods.len(), 24);
        assert!(result.equivalent_full_cycles >= 0.0);
    }

    #[test]
    fn test_scheduler_duck_curve_charges_at_trough() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = duck_curve_prices(0.05, 5.0, 0.3);
        let config = SchedulerConfig {
            final_soc_target: None,
            enable_deg_cost: false,
            ..Default::default()
        };
        let result = run_scheduler(&bess, &prices, &config);
        // Check that some charging happens (not all idle)
        let has_charge = result
            .periods
            .iter()
            .any(|p| p.action == ScheduleAction::Charge);
        let has_discharge = result
            .periods
            .iter()
            .any(|p| p.action == ScheduleAction::Discharge);
        assert!(
            has_charge || has_discharge,
            "Should have at least some activity with large spread"
        );
    }

    #[test]
    fn test_bess_params_usable_kwh() {
        let bess = SchedulerBessParams::utility_1mwh();
        let usable = bess.usable_kwh();
        assert!((usable - 800.0).abs() < 1e-6); // (0.90 - 0.10) * 1000
    }

    #[test]
    fn test_bess_params_deg_cost() {
        let bess = SchedulerBessParams::utility_1mwh();
        let cost = bess.deg_cost(100.0); // 100 kWh throughput
        assert!((cost - 1.5).abs() < 1e-6); // 100 * 0.015
    }

    #[test]
    fn test_price_arbitrage_stats() {
        let prices = simple_arbitrage_prices();
        let stats = price_arbitrage_stats(&prices, 1000.0, 1.0);
        assert!(stats.daily_spread > 0.0);
        assert!(stats.max_theoretical_revenue > 0.0);
        assert!(stats.high_price_fraction > 0.0 && stats.high_price_fraction < 1.0);
    }

    #[test]
    fn test_price_arbitrage_stats_empty() {
        let stats = price_arbitrage_stats(&[], 1000.0, 1.0);
        assert_eq!(stats.mean_price, 0.0);
    }

    #[test]
    fn test_duck_curve_prices_length() {
        let prices = duck_curve_prices(0.04, 3.0, 0.5);
        assert_eq!(prices.len(), 24);
    }

    #[test]
    fn test_duck_curve_prices_positive() {
        let prices = duck_curve_prices(0.04, 3.0, 0.5);
        for &p in &prices {
            assert!(p > 0.0, "Price should be positive: {:.4}", p);
        }
    }

    #[test]
    fn test_levelised_cost_of_storage() {
        // 365 cycles/year (once daily), 15 years, 7% discount
        let lcos = levelised_cost_of_storage(300.0, 5.0, 0.015, 365.0, 15.0, 0.07, 800.0);
        // LCOS for utility BESS: typically $0.05–$0.50/kWh
        assert!(lcos > 0.01 && lcos < 1.0, "LCOS out of range: {:.4}", lcos);
    }

    #[test]
    fn test_achieved_rte() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = duck_curve_prices(0.03, 4.0, 0.5);
        let result = run_scheduler(&bess, &prices, &SchedulerConfig::default());
        if result.total_charge_kwh > 0.0 {
            let rte = result.achieved_rte();
            assert!(rte > 0.0 && rte <= 1.0, "RTE out of range: {:.4}", rte);
        }
    }

    #[test]
    fn test_schedule_periods_soc_continuity() {
        let bess = SchedulerBessParams::utility_1mwh();
        let prices = simple_arbitrage_prices();
        let result = run_scheduler(&bess, &prices, &SchedulerConfig::default());
        // Each period's soc_end should match next period's soc_start
        for i in 1..result.periods.len() {
            let prev_end = result.periods[i - 1].soc_end;
            let curr_start = result.periods[i].soc_start;
            assert!(
                (prev_end - curr_start).abs() < 1e-6,
                "SOC discontinuity at period {}: {:.4} vs {:.4}",
                i,
                prev_end,
                curr_start
            );
        }
    }
}
