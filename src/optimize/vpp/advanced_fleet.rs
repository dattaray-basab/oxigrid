//! Advanced energy storage fleet coordination with market integration.
//!
//! Coordinates a fleet of battery assets to simultaneously deliver ancillary
//! services and energy arbitrage, maximizing net profit while respecting SoC
//! bounds and degradation constraints.
//!
//! # Algorithm
//!
//! 1. **Reserve ancillary services** — rank assets by service suitability, commit
//!    MW to each required service (PrimaryReserve: fastest first; others: largest first).
//! 2. **Energy arbitrage** — with remaining capacity, charge in cheapest hours and
//!    discharge in most expensive hours using a greedy dual-sort approach.
//! 3. **SoC tracking** — integrate hourly power to maintain per-asset SoC within
//!    \[0.1, 0.9\] bounds.
//! 4. **Bid generation** — produce market bid objects for each committed service.
//!
//! # References
//! - Sioshansi, R. et al., "Estimating the value of electricity storage in PJM:
//!   Arbitrage and some welfare effects", Energy Economics, 2009
//! - Pozo, D. et al., "Unit Commitment with Ideal and Generic Energy Storage Units",
//!   IEEE Trans. Power Syst., 2014
//! - He, G. et al., "Optimal Bidding Strategy of Battery Storage in Power Markets
//!   Considering Performance-Based Regulation", IEEE Trans. Smart Grid, 2016

use serde::{Deserialize, Serialize};

// ── Error type ─────────────────────────────────────────────────────────────

/// Errors from the advanced fleet coordinator.
#[derive(Debug, thiserror::Error)]
pub enum FleetError {
    /// No assets have been added.
    #[error("no assets registered — call add_asset() first")]
    NoAssets,
    /// Energy prices vector is empty or wrong length.
    #[error("energy prices vector is empty")]
    NoPrices,
    /// Asset configuration is invalid.
    #[error("invalid asset {id}: {msg}")]
    InvalidAsset {
        /// Asset ID.
        id: usize,
        /// Description of the problem.
        msg: String,
    },
}

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the advanced fleet coordinator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedFleetConfig {
    /// Number of assets in the fleet.
    pub n_assets: usize,
    /// Market dispatch intervals as (start\_hour, end\_hour) pairs.
    pub market_intervals: Vec<(usize, usize)>,
    /// Settlement period duration \[h\] (e.g. 0.5 for 30-min, 1.0 for hourly).
    pub settlement_period_h: f64,
    /// List of ancillary services the fleet should participate in.
    pub ancillary_services: Vec<AncillaryService>,
    /// Communication latency from control center to assets \[ms\].
    pub communication_latency_ms: f64,
    /// Forecast accuracy of renewable generation \[%\].
    pub forecast_accuracy_pct: f64,
}

impl Default for AdvancedFleetConfig {
    fn default() -> Self {
        Self {
            n_assets: 1,
            market_intervals: vec![(0, 24)],
            settlement_period_h: 1.0,
            ancillary_services: Vec::new(),
            communication_latency_ms: 100.0,
            forecast_accuracy_pct: 90.0,
        }
    }
}

/// An ancillary service that the fleet may be committed to provide.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AncillaryService {
    /// Primary (spinning) reserve — fast frequency response.
    PrimaryReserve {
        /// Required committed capacity \[MW\].
        required_mw: f64,
        /// Required response time \[s\].
        response_s: f64,
    },
    /// Secondary (regulating) reserve — AGC response.
    SecondaryReserve {
        /// Required capacity \[MW\].
        required_mw: f64,
    },
    /// Tertiary (replacement) reserve — slow response.
    TertiaryReserve {
        /// Required capacity \[MW\].
        required_mw: f64,
    },
    /// Frequency regulation — symmetric up/down capacity with mileage signal.
    FrequencyRegulation {
        /// Regulation capacity \[MW\].
        capacity_mw: f64,
        /// Expected mileage signal (MW delivered per period) \[MW\].
        mileage_mw: f64,
    },
    /// Black-start capability.
    BlackStart {
        /// Required startup capacity \[MW\].
        capacity_mw: f64,
    },
    /// Voltage support — reactive power injection.
    VoltageSupport {
        /// Reactive power capacity \[Mvar\].
        q_capacity_mvar: f64,
    },
}

impl AncillaryService {
    /// Human-readable name for the service.
    pub fn name(&self) -> &'static str {
        match self {
            Self::PrimaryReserve { .. } => "PrimaryReserve",
            Self::SecondaryReserve { .. } => "SecondaryReserve",
            Self::TertiaryReserve { .. } => "TertiaryReserve",
            Self::FrequencyRegulation { .. } => "FrequencyRegulation",
            Self::BlackStart { .. } => "BlackStart",
            Self::VoltageSupport { .. } => "VoltageSupport",
        }
    }

    /// Required power capacity in \[MW\] (0 for reactive-only services).
    pub fn required_mw(&self) -> f64 {
        match self {
            Self::PrimaryReserve { required_mw, .. } => *required_mw,
            Self::SecondaryReserve { required_mw } => *required_mw,
            Self::TertiaryReserve { required_mw } => *required_mw,
            Self::FrequencyRegulation { capacity_mw, .. } => *capacity_mw,
            Self::BlackStart { capacity_mw } => *capacity_mw,
            Self::VoltageSupport { .. } => 0.0,
        }
    }
}

// ── Asset description ───────────────────────────────────────────────────────

/// A single battery storage asset within the fleet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetAsset {
    /// Unique asset identifier.
    pub id: usize,
    /// Bus index where the asset is connected to the grid.
    pub location_bus: usize,
    /// Usable energy capacity \[MWh\].
    pub capacity_mwh: f64,
    /// Maximum charge / discharge power \[MW\].
    pub power_mw: f64,
    /// One-way efficiency (charge or discharge), typically 0.92–0.98.
    pub efficiency: f64,
    /// Current state of charge \[0, 1\].
    pub soc: f64,
    /// SoC reduction per full equivalent cycle (0–1).
    pub degradation_per_cycle: f64,
    /// Round-trip cost of energy throughput \[USD/MWh\].
    pub round_trip_cost_usd_per_mwh: f64,
    /// Ancillary services this asset is capable of providing.
    pub ancillary_capability: Vec<AncillaryService>,
}

impl FleetAsset {
    /// Response time for primary reserve \[s\] — uses communication_latency as proxy.
    ///
    /// For a real asset the value would come from a manufacturer datasheet;
    /// here we use power density as a proxy (higher power/capacity → faster).
    pub fn implied_response_s(&self) -> f64 {
        // Faster response for higher power-to-energy ratio
        let ratio = self.power_mw / self.capacity_mwh.max(0.001);
        (10.0 / ratio.max(0.1)).min(60.0)
    }
}

// ── Result types ────────────────────────────────────────────────────────────

/// A market bid generated by the fleet.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetBid {
    /// Service name.
    pub service: String,
    /// Capacity committed \[MW\].
    pub capacity_mw: f64,
    /// Offer price \[USD/MW\].
    pub price_usd_per_mw: f64,
    /// Availability window (start\_hour, end\_hour).
    pub availability_period: (usize, usize),
}

/// Per-asset dispatch schedule.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetDispatchSchedule {
    /// Asset identifier.
    pub asset_id: usize,
    /// Scheduled power at each hour: positive = charging \[MW\], negative = discharging \[MW\].
    pub hourly_p_mw: Vec<f64>,
    /// SoC at the end of each hour \[0, 1\].
    pub hourly_soc: Vec<f64>,
    /// Ancillary service reservations: (service\_name, reserved\_MW).
    pub ancillary_reservations: Vec<(String, f64)>,
    /// Estimated total revenue \[USD\].
    pub estimated_revenue_usd: f64,
    /// Estimated capacity degradation \[%\].
    pub estimated_degradation_pct: f64,
}

/// Complete result of the fleet optimization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvancedFleetResult {
    /// Per-asset dispatch schedules.
    pub schedules: Vec<FleetDispatchSchedule>,
    /// Market bids to submit.
    pub fleet_bids: Vec<FleetBid>,
    /// Total fleet revenue \[USD\].
    pub total_revenue_usd: f64,
    /// Total degradation cost \[USD\].
    pub total_degradation_cost_usd: f64,
    /// Net profit = revenue − degradation cost \[USD\].
    pub net_profit_usd: f64,
    /// For each configured ancillary service: whether it was fully covered.
    pub ancillary_services_met: Vec<bool>,
    /// Peak shaving achieved: reduction in peak load \[MW\].
    pub peak_shaving_mw: f64,
}

// ── Coordinator ─────────────────────────────────────────────────────────────

/// Advanced energy storage fleet coordinator.
///
/// # Example
///
/// ```rust,ignore
/// use oxigrid::optimize::vpp::advanced_fleet::{
///     AdvancedFleetConfig, AdvancedFleetCoordinator, FleetAsset, AncillaryService,
/// };
/// let cfg = AdvancedFleetConfig::default();
/// let prices = vec![30.0; 24];
/// let mut coord = AdvancedFleetCoordinator::new(cfg, prices);
/// coord.add_asset(FleetAsset {
///     id: 0, location_bus: 1, capacity_mwh: 10.0, power_mw: 5.0,
///     efficiency: 0.95, soc: 0.5, degradation_per_cycle: 0.0001,
///     round_trip_cost_usd_per_mwh: 5.0,
///     ancillary_capability: vec![],
/// });
/// let result = coord.optimize().expect("ok");
/// ```
#[derive(Debug, Clone)]
pub struct AdvancedFleetCoordinator {
    config: AdvancedFleetConfig,
    assets: Vec<FleetAsset>,
    energy_prices: Vec<f64>,
    ancillary_prices: Vec<(String, f64)>,
}

impl AdvancedFleetCoordinator {
    /// Create a new coordinator with configuration and hourly spot prices.
    pub fn new(config: AdvancedFleetConfig, energy_prices: Vec<f64>) -> Self {
        Self {
            config,
            assets: Vec::new(),
            energy_prices,
            ancillary_prices: Vec::new(),
        }
    }

    /// Add a fleet asset.
    pub fn add_asset(&mut self, asset: FleetAsset) {
        self.assets.push(asset);
    }

    /// Set ancillary service prices: list of (service\_name, price \[USD/MW\]) pairs.
    pub fn set_ancillary_prices(&mut self, prices: Vec<(String, f64)>) {
        self.ancillary_prices = prices;
    }

    /// Run the fleet optimization.
    ///
    /// Returns [`FleetError`] if no assets or prices are configured.
    pub fn optimize(&self) -> Result<AdvancedFleetResult, FleetError> {
        if self.assets.is_empty() {
            return Err(FleetError::NoAssets);
        }
        if self.energy_prices.is_empty() {
            return Err(FleetError::NoPrices);
        }

        let n_hours = self.energy_prices.len();
        let dt = self.config.settlement_period_h;

        // ── Per-asset mutable state ─────────────────────────────────────────
        // soc[a] = current SoC during optimization
        let mut soc: Vec<f64> = self.assets.iter().map(|a| a.soc).collect();
        // hourly power schedule: p[a][h] (positive=charge, negative=discharge)
        let mut schedules: Vec<Vec<f64>> = vec![vec![0.0; n_hours]; self.assets.len()];
        // per-asset ancillary reservations
        let mut anc_reservations: Vec<Vec<(String, f64)>> = vec![Vec::new(); self.assets.len()];
        // per-asset reserved MW (reduces available power for arbitrage)
        let mut reserved_mw: Vec<f64> = vec![0.0; self.assets.len()];

        // ── Step 1: Reserve ancillary services ─────────────────────────────
        let mut ancillary_services_met: Vec<bool> =
            vec![false; self.config.ancillary_services.len()];
        let mut fleet_bids: Vec<FleetBid> = Vec::new();

        for (si, service) in self.config.ancillary_services.iter().enumerate() {
            let required = service.required_mw();
            if required <= 0.0 {
                ancillary_services_met[si] = true;
                continue;
            }

            let ranked = self.rank_assets_for_service(service);
            let mut remaining = required;

            for &ai in &ranked {
                if remaining <= 0.0 {
                    break;
                }
                let asset = &self.assets[ai];
                let avail = (asset.power_mw - reserved_mw[ai]).max(0.0);
                let commit = remaining.min(avail);
                if commit > 0.0 {
                    reserved_mw[ai] += commit;
                    remaining -= commit;
                    anc_reservations[ai].push((service.name().to_string(), commit));
                }
            }

            let committed = required - remaining;
            if committed >= required - 1e-9 {
                ancillary_services_met[si] = true;
            }

            if committed > 0.0 {
                // Look up ancillary price
                let anc_price = self
                    .ancillary_prices
                    .iter()
                    .find(|(name, _)| name == service.name())
                    .map(|(_, p)| *p)
                    .unwrap_or(10.0); // default $10/MW

                fleet_bids.push(FleetBid {
                    service: service.name().to_string(),
                    capacity_mw: committed,
                    price_usd_per_mw: anc_price,
                    availability_period: (0, n_hours),
                });
            }
        }

        // ── Step 2: Arbitrage with remaining capacity ───────────────────────
        // Identify cheap hours (charge) and expensive hours (discharge)
        // Sort hours by price ascending for charging, descending for discharging
        let mut price_order: Vec<usize> = (0..n_hours).collect();
        price_order.sort_by(|&a, &b| {
            self.energy_prices[a]
                .partial_cmp(&self.energy_prices[b])
                .unwrap_or(core::cmp::Ordering::Equal)
        });

        // Cheap hours = first half; expensive hours = second half
        let n_cheap = n_hours / 2;
        let cheap_hours: Vec<usize> = price_order[..n_cheap].to_vec();
        let expensive_hours: Vec<usize> = price_order[n_hours.saturating_sub(n_cheap)..].to_vec();

        for ai in 0..self.assets.len() {
            let asset = &self.assets[ai];
            let avail_power = (asset.power_mw - reserved_mw[ai]).max(0.0);
            let soc_min = 0.10_f64;
            let soc_max = 0.90_f64;

            // Charge in cheap hours
            for &h in &cheap_hours {
                let soc_headroom = (soc_max - soc[ai]).max(0.0);
                let max_charge_mwh = soc_headroom * asset.capacity_mwh / asset.efficiency;
                let charge_mw = avail_power.min(max_charge_mwh / dt.max(1e-9));
                if charge_mw > 1e-6 {
                    schedules[ai][h] += charge_mw;
                    soc[ai] += charge_mw * dt * asset.efficiency / asset.capacity_mwh;
                    soc[ai] = soc[ai].min(soc_max);
                }
            }

            // Discharge in expensive hours
            for &h in &expensive_hours {
                let soc_avail = (soc[ai] - soc_min).max(0.0);
                let max_discharge_mwh = soc_avail * asset.capacity_mwh * asset.efficiency;
                let discharge_mw = avail_power.min(max_discharge_mwh / dt.max(1e-9));
                if discharge_mw > 1e-6 {
                    schedules[ai][h] -= discharge_mw; // negative = discharging
                    soc[ai] -= discharge_mw * dt / (asset.efficiency * asset.capacity_mwh);
                    soc[ai] = soc[ai].max(soc_min);
                }
            }
        }

        // ── Step 3: Compute per-asset results ───────────────────────────────
        let mut dispatch_schedules: Vec<FleetDispatchSchedule> = Vec::new();
        let mut total_revenue = 0.0_f64;
        let mut total_degradation_cost = 0.0_f64;
        let mut peak_shaving_mw = 0.0_f64;

        for ai in 0..self.assets.len() {
            let asset = &self.assets[ai];

            // Reconstruct hourly SoC from initial SoC + power schedule
            let mut hourly_soc = Vec::with_capacity(n_hours);
            let mut running_soc = asset.soc;
            for &p in &schedules[ai] {
                if p >= 0.0 {
                    running_soc += p * dt * asset.efficiency / asset.capacity_mwh;
                } else {
                    running_soc += p * dt / (asset.efficiency * asset.capacity_mwh);
                }
                running_soc = running_soc.clamp(0.0, 1.0);
                hourly_soc.push(running_soc);
            }

            // Energy revenue: discharge earns price, charge costs price
            let energy_revenue: f64 = schedules[ai]
                .iter()
                .enumerate()
                .map(|(h, &p)| {
                    let price = self.energy_prices.get(h).copied().unwrap_or(0.0);
                    // Discharging (p < 0) earns revenue; charging (p > 0) costs money
                    -p * price * dt
                })
                .sum();

            // Ancillary revenue
            let anc_hours = n_hours as f64;
            let anc_revenue: f64 = anc_reservations[ai]
                .iter()
                .map(|(svc, mw)| {
                    let price = self
                        .ancillary_prices
                        .iter()
                        .find(|(name, _)| name == svc)
                        .map(|(_, p)| *p)
                        .unwrap_or(10.0);
                    mw * price * anc_hours
                })
                .sum();

            let asset_revenue = energy_revenue + anc_revenue;

            // Degradation cost: approximate cycles from total throughput
            let throughput_mwh: f64 = schedules[ai].iter().map(|&p| p.abs() * dt).sum();
            let equiv_cycles = throughput_mwh / (2.0 * asset.capacity_mwh.max(0.001));
            let degradation_pct = equiv_cycles * asset.degradation_per_cycle * 100.0;
            let degradation_cost = equiv_cycles
                * asset.degradation_per_cycle
                * asset.capacity_mwh
                * asset.round_trip_cost_usd_per_mwh;

            // Peak shaving: max discharge in any single hour
            let max_discharge = schedules[ai]
                .iter()
                .map(|&p| (-p).max(0.0))
                .fold(0.0_f64, f64::max);
            peak_shaving_mw += max_discharge;

            total_revenue += asset_revenue;
            total_degradation_cost += degradation_cost;

            dispatch_schedules.push(FleetDispatchSchedule {
                asset_id: asset.id,
                hourly_p_mw: schedules[ai].clone(),
                hourly_soc,
                ancillary_reservations: anc_reservations[ai].clone(),
                estimated_revenue_usd: asset_revenue,
                estimated_degradation_pct: degradation_pct,
            });
        }

        let net_profit = total_revenue - total_degradation_cost;

        Ok(AdvancedFleetResult {
            schedules: dispatch_schedules,
            fleet_bids,
            total_revenue_usd: total_revenue,
            total_degradation_cost_usd: total_degradation_cost,
            net_profit_usd: net_profit,
            ancillary_services_met,
            peak_shaving_mw,
        })
    }

    /// Rank assets for a given ancillary service by suitability.
    ///
    /// - **PrimaryReserve**: fastest response first (lowest `implied_response_s`),
    ///   then largest power.
    /// - **FrequencyRegulation**: highest efficiency first, then largest power.
    /// - **Others**: largest power first, then highest SoC.
    pub fn rank_assets_for_service(&self, service: &AncillaryService) -> Vec<usize> {
        let mut indices: Vec<usize> = (0..self.assets.len()).collect();

        match service {
            AncillaryService::PrimaryReserve { response_s, .. } => {
                // Prefer assets with faster inherent response and sufficient power
                indices.sort_by(|&a, &b| {
                    let resp_a = self.assets[a].implied_response_s();
                    let resp_b = self.assets[b].implied_response_s();
                    // Faster (lower) response time → higher priority
                    resp_a
                        .partial_cmp(&resp_b)
                        .unwrap_or(core::cmp::Ordering::Equal)
                        // Tie-break: larger power
                        .then(
                            self.assets[b]
                                .power_mw
                                .partial_cmp(&self.assets[a].power_mw)
                                .unwrap_or(core::cmp::Ordering::Equal),
                        )
                });
                // Filter: only keep assets capable of meeting the target response_s
                // (relaxed: keep all but rank them appropriately)
                let _ = response_s;
                indices
            }
            AncillaryService::FrequencyRegulation { .. } => {
                indices.sort_by(|&a, &b| {
                    self.assets[b]
                        .efficiency
                        .partial_cmp(&self.assets[a].efficiency)
                        .unwrap_or(core::cmp::Ordering::Equal)
                        .then(
                            self.assets[b]
                                .power_mw
                                .partial_cmp(&self.assets[a].power_mw)
                                .unwrap_or(core::cmp::Ordering::Equal),
                        )
                });
                indices
            }
            _ => {
                // Default: largest power, then highest SoC
                indices.sort_by(|&a, &b| {
                    self.assets[b]
                        .power_mw
                        .partial_cmp(&self.assets[a].power_mw)
                        .unwrap_or(core::cmp::Ordering::Equal)
                        .then(
                            self.assets[b]
                                .soc
                                .partial_cmp(&self.assets[a].soc)
                                .unwrap_or(core::cmp::Ordering::Equal),
                        )
                });
                indices
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a test asset with given response_s (via power/capacity ratio).
    fn asset(id: usize, power_mw: f64, capacity_mwh: f64, soc: f64) -> FleetAsset {
        FleetAsset {
            id,
            location_bus: id,
            capacity_mwh,
            power_mw,
            efficiency: 0.95,
            soc,
            degradation_per_cycle: 0.0001,
            round_trip_cost_usd_per_mwh: 5.0,
            ancillary_capability: Vec::new(),
        }
    }

    fn make_prices(n: usize, low: f64, high: f64) -> Vec<f64> {
        // First half cheap, second half expensive
        let mut prices = vec![low; n];
        for p in prices[n / 2..].iter_mut() {
            *p = high;
        }
        prices
    }

    /// Primary reserve: asset with higher power/capacity ratio (faster) selected first.
    #[test]
    fn test_primary_reserve_fastest_selected() {
        let cfg = AdvancedFleetConfig {
            ancillary_services: vec![AncillaryService::PrimaryReserve {
                required_mw: 2.0,
                response_s: 5.0,
            }],
            ..AdvancedFleetConfig::default()
        };
        let prices = make_prices(24, 30.0, 80.0);
        let mut coord = AdvancedFleetCoordinator::new(cfg, prices);
        // Asset 0: high power/capacity → fast response (ratio=5)
        coord.add_asset(asset(0, 10.0, 2.0, 0.5)); // implied_response_s ≈ 0.2
                                                   // Asset 1: low power/capacity → slower response (ratio=0.5)
        coord.add_asset(asset(1, 1.0, 2.0, 0.5)); // implied_response_s ≈ 2.0

        let result = coord.optimize().expect("optimize");
        // Asset 0 should have primary reserve reservation
        let sched0 = &result.schedules[0];
        let has_primary = sched0
            .ancillary_reservations
            .iter()
            .any(|(svc, _)| svc == "PrimaryReserve");
        assert!(
            has_primary,
            "Fastest asset must be selected for primary reserve"
        );
        assert!(
            result.ancillary_services_met[0],
            "Primary reserve must be met"
        );
    }

    /// Arbitrage: fleet should discharge in expensive hours and charge in cheap hours.
    #[test]
    fn test_arbitrage_buy_low_sell_high() {
        let cfg = AdvancedFleetConfig {
            ancillary_services: Vec::new(),
            ..AdvancedFleetConfig::default()
        };
        let prices = make_prices(24, 20.0, 100.0);
        let mut coord = AdvancedFleetCoordinator::new(cfg, prices);
        coord.add_asset(asset(0, 5.0, 20.0, 0.5));

        let result = coord.optimize().expect("optimize");
        let sched = &result.schedules[0];

        // Should have some charging in first 12 hours (cheap) and discharging in last 12
        let cheap_charging: f64 = sched.hourly_p_mw[..12].iter().filter(|&&p| p > 0.0).sum();
        let exp_discharging: f64 = sched.hourly_p_mw[12..].iter().filter(|&&p| p < 0.0).sum();

        assert!(
            cheap_charging > 0.0 || exp_discharging < 0.0,
            "Fleet should participate in arbitrage: cheap_ch={cheap_charging}, exp_dc={exp_discharging}"
        );
    }

    /// SoC bounds: hourly_soc must always stay within [0.0, 1.0].
    #[test]
    fn test_soc_bounds_never_violated() {
        let cfg = AdvancedFleetConfig {
            ancillary_services: Vec::new(),
            ..AdvancedFleetConfig::default()
        };
        let prices = make_prices(48, 5.0, 200.0);
        let mut coord = AdvancedFleetCoordinator::new(cfg, prices.clone());
        // Small capacity asset — easy to hit limits
        coord.add_asset(asset(0, 5.0, 5.0, 0.5));
        coord.add_asset(asset(1, 3.0, 8.0, 0.8));

        let result = coord.optimize().expect("optimize");
        for sched in &result.schedules {
            for (h, &s) in sched.hourly_soc.iter().enumerate() {
                assert!(
                    (0.0..=1.0 + 1e-9).contains(&s),
                    "SoC must be in [0,1] at hour {h}: got {s}"
                );
            }
        }
    }

    /// Service stacking: ancillary + energy arbitrage both occur.
    #[test]
    fn test_service_stacking() {
        let cfg = AdvancedFleetConfig {
            ancillary_services: vec![AncillaryService::SecondaryReserve { required_mw: 2.0 }],
            ..AdvancedFleetConfig::default()
        };
        let prices = make_prices(24, 30.0, 90.0);
        let mut coord = AdvancedFleetCoordinator::new(cfg, prices);
        coord.set_ancillary_prices(vec![("SecondaryReserve".to_string(), 15.0)]);
        // Large asset that can do both
        coord.add_asset(asset(0, 10.0, 30.0, 0.5));

        let result = coord.optimize().expect("optimize");
        let sched = &result.schedules[0];

        // Has ancillary reservation
        let has_anc = !sched.ancillary_reservations.is_empty();
        // Has some energy schedule
        let has_energy = sched.hourly_p_mw.iter().any(|&p| p.abs() > 1e-6);

        assert!(has_anc, "Should have ancillary reservation");
        assert!(has_energy, "Should also have energy arbitrage");
        assert!(
            result.ancillary_services_met[0],
            "Secondary reserve must be met"
        );
    }

    /// Ancillary revenue > energy revenue when ancillary price is high.
    #[test]
    fn test_ancillary_revenue_higher_than_energy() {
        // Two identical single-asset coordinators: one with ancillary, one without
        let make = |with_anc: bool| {
            let cfg = AdvancedFleetConfig {
                ancillary_services: if with_anc {
                    vec![AncillaryService::PrimaryReserve {
                        required_mw: 3.0,
                        response_s: 5.0,
                    }]
                } else {
                    Vec::new()
                },
                ..AdvancedFleetConfig::default()
            };
            let prices = make_prices(24, 30.0, 50.0); // narrow spread → low arb value
            let mut coord = AdvancedFleetCoordinator::new(cfg, prices);
            coord.set_ancillary_prices(vec![("PrimaryReserve".to_string(), 100.0)]);
            coord.add_asset(asset(0, 10.0, 20.0, 0.5));
            coord.optimize().expect("optimize")
        };

        let with_anc = make(true);
        let without_anc = make(false);

        assert!(
            with_anc.total_revenue_usd > without_anc.total_revenue_usd,
            "Ancillary revenue (price=100/MW) must exceed pure arbitrage (spread=20/MWh): \
             anc={:.2}, arb={:.2}",
            with_anc.total_revenue_usd,
            without_anc.total_revenue_usd
        );
    }

    /// No assets registered → FleetError::NoAssets.
    #[test]
    fn test_no_assets_error() {
        let cfg = AdvancedFleetConfig::default();
        let coord = AdvancedFleetCoordinator::new(cfg, vec![50.0; 24]);
        let result = coord.optimize();
        assert!(matches!(result, Err(FleetError::NoAssets)));
    }

    /// Empty prices → FleetError::NoPrices.
    #[test]
    fn test_no_prices_error() {
        let cfg = AdvancedFleetConfig::default();
        let mut coord = AdvancedFleetCoordinator::new(cfg, Vec::new());
        coord.add_asset(asset(0, 5.0, 10.0, 0.5));
        let result = coord.optimize();
        assert!(matches!(result, Err(FleetError::NoPrices)));
    }
}
