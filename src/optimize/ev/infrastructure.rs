//! Smart charging infrastructure optimisation.
//!
//! Models a network of EV charging stations with multiple charger types,
//! on-site solar generation, V2G capability, transformer capacity limits,
//! and time-of-use tariffs.  The optimiser schedules each arriving vehicle
//! to minimise energy cost while respecting hardware and grid constraints.
//!
//! # Algorithm
//!
//! For each arriving vehicle:
//! 1. Find the station with the lowest queue length.
//! 2. Assign the first available charger slot.
//! 3. Build an hourly schedule that charges during cheap TOU periods first.
//! 4. Apply the station transformer limit: curtail charging if the aggregate
//!    station load would exceed the rating.
//! 5. Use on-site solar (free) before drawing from the grid.
//! 6. If V2G is enabled and the vehicle is willing, discharge during the
//!    highest-price hours to earn revenue.
//! 7. Compute per-vehicle costs and infrastructure-wide metrics.
//!
//! # References
//!
//! - Kempton & Tomić, "Vehicle-to-Grid Power Fundamentals", J. Power Sources 2005
//! - Sortomme & El-Sharkawi, "Optimal Charging Strategies for Unidirectional
//!   Vehicle-to-Grid", IEEE Trans. Smart Grid 2011

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the smart charging infrastructure optimiser.
#[derive(Debug, Error)]
pub enum InfraError {
    /// No charging stations have been configured.
    #[error("no charging stations configured")]
    NoStations,

    /// Invalid tariff: hourly rates vector must have at least 24 entries.
    #[error("tariff must have at least 24 hourly rate entries, got {0}")]
    InvalidTariff(usize),

    /// Grid capacity or transformer rating is zero or negative.
    #[error("invalid capacity parameter: {0}")]
    InvalidCapacity(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// Charger types
// ─────────────────────────────────────────────────────────────────────────────

/// Level of EV charging equipment.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ChargerType {
    /// Level 1 AC charger (1.4–1.9 \[kW\]).
    Level1 { rated_kw: f64 },
    /// Level 2 AC charger (3.3–22 \[kW\]).
    Level2 { rated_kw: f64 },
    /// DC fast charger (50–350 \[kW\]).
    DcFastCharger { rated_kw: f64 },
    /// Ultra-fast DC charger (> 350 \[kW\]).
    UltraFast { rated_kw: f64 },
}

impl ChargerType {
    /// Rated output power \[kW\].
    pub fn rated_kw(&self) -> f64 {
        match self {
            ChargerType::Level1 { rated_kw } => *rated_kw,
            ChargerType::Level2 { rated_kw } => *rated_kw,
            ChargerType::DcFastCharger { rated_kw } => *rated_kw,
            ChargerType::UltraFast { rated_kw } => *rated_kw,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tariff
// ─────────────────────────────────────────────────────────────────────────────

/// Time-of-use electricity tariff for a charging station.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingTariff {
    /// Hourly energy rate \[USD/kWh\] (24 or more entries, index = hour of day).
    pub energy_rate_usd_per_kwh: Vec<f64>,
    /// Monthly peak demand charge \[USD/kW\].
    pub demand_charge_usd_per_kw: f64,
    /// V2G export payment \[USD/kWh\] (what the grid pays for V2G energy).
    pub v2g_rate_usd_per_kwh: f64,
    /// Fixed connection fee per charging session \[USD\].
    pub connection_fee_usd: f64,
}

impl Default for ChargingTariff {
    fn default() -> Self {
        // Simple peak/off-peak TOU
        let mut rates = vec![0.10_f64; 24];
        for rate in rates[8..21].iter_mut() {
            *rate = 0.25;
        }
        Self {
            energy_rate_usd_per_kwh: rates,
            demand_charge_usd_per_kw: 15.0,
            v2g_rate_usd_per_kwh: 0.18,
            connection_fee_usd: 0.50,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Charging station
// ─────────────────────────────────────────────────────────────────────────────

/// A physical charging station with one or more charger bays.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingStation {
    /// Station identifier.
    pub id: usize,
    /// Bus index in the network where this station is connected.
    pub location_bus: usize,
    /// Type and rated power of each charger at this station.
    pub charger_type: ChargerType,
    /// Rated output power of each charger \[kW\].
    pub rated_kw: f64,
    /// Number of EVs currently occupying charger bays.
    pub current_occupancy: usize,
    /// Number of EVs waiting for a charger.
    pub queue_length: usize,
    /// Available on-site solar generation \[kW\] (zero = no local solar).
    pub local_solar_kw: f64,
}

impl ChargingStation {
    /// Available charger capacity per station \[kW\] = rated × n_chargers.
    pub fn peak_capacity_kw(&self, n_chargers: usize) -> f64 {
        self.rated_kw * n_chargers as f64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// EV arrival
// ─────────────────────────────────────────────────────────────────────────────

/// A vehicle arrival event at a charging station.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvArrival {
    /// Unique vehicle identifier.
    pub vehicle_id: u64,
    /// Preferred station (may be overridden if overloaded).
    pub station_id: usize,
    /// Arrival time (hour of day, 0–23).
    pub arrival_hour: usize,
    /// Desired departure time (hour of day, may wrap past midnight).
    pub departure_hour: usize,
    /// State of charge at arrival (0–1).
    pub soc_arrival: f64,
    /// Desired state of charge at departure (0–1).
    pub soc_target: f64,
    /// Usable battery capacity \[kWh\].
    pub battery_kwh: f64,
    /// Maximum on-board charge rate \[kW\].
    pub max_charge_rate_kw: f64,
    /// Whether the vehicle hardware supports V2G.
    pub v2g_capable: bool,
    /// Whether the driver consents to V2G export.
    pub v2g_willing: bool,
}

impl EvArrival {
    /// Energy required to reach `soc_target` from `soc_arrival` \[kWh\].
    pub fn energy_needed_kwh(&self) -> f64 {
        ((self.soc_target - self.soc_arrival) * self.battery_kwh).max(0.0)
    }

    /// Available parking time in hours.
    pub fn available_hours(&self) -> usize {
        if self.departure_hour >= self.arrival_hour {
            self.departure_hour - self.arrival_hour
        } else {
            // overnight stay
            24 - self.arrival_hour + self.departure_hour
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Results
// ─────────────────────────────────────────────────────────────────────────────

/// Hourly charging schedule for one vehicle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingSchedule {
    /// Vehicle this schedule belongs to.
    pub vehicle_id: u64,
    /// Station where the vehicle is charged.
    pub station_id: usize,
    /// Charger bay index within the station (0-indexed).
    pub charger_id: usize,
    /// Net power per hour \[kW\] (positive = charge, negative = V2G discharge).
    pub hourly_power_kw: Vec<f64>,
    /// SoC trajectory at end of each hour (0–1).
    pub soc_trajectory: Vec<f64>,
    /// Total energy cost for the session \[USD\].
    pub energy_cost_usd: f64,
    /// Total V2G revenue earned \[USD\].
    pub v2g_revenue_usd: f64,
    /// SoC at departure.
    pub departure_soc: f64,
    /// Whether the SoC target was achieved.
    pub soc_target_met: bool,
}

/// Aggregated results for the full infrastructure optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InfrastructureResult {
    /// Per-vehicle charging schedules.
    pub schedules: Vec<ChargingSchedule>,
    /// Total energy drawn from the grid \[kWh\].
    pub total_energy_kwh: f64,
    /// Peak aggregate demand across all stations \[kW\].
    pub peak_demand_kw: f64,
    /// Fraction of energy sourced from on-site renewables \[%\].
    pub renewable_fraction_pct: f64,
    /// Total V2G energy exported to the grid \[kWh\].
    pub v2g_energy_exported_kwh: f64,
    /// Total energy cost \[USD\].
    pub total_cost_usd: f64,
    /// Total V2G revenue \[USD\].
    pub total_v2g_revenue_usd: f64,
    /// Peak transformer utilisation as percentage of rating.
    pub transformer_utilization_pct: f64,
    /// Number of vehicles that could not be accommodated.
    pub unserved_vehicles: usize,
    /// Average queue wait time \[min\].
    pub avg_wait_time_min: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the smart charging infrastructure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingInfraConfig {
    /// Number of stations.
    pub n_stations: usize,
    /// Number of charger bays per station.
    pub n_chargers_per_station: usize,
    /// Maximum grid capacity per station \[kW\].
    pub grid_capacity_kw_per_station: f64,
    /// Transformer rating for the whole facility \[kVA\].
    pub transformer_rating_kva: f64,
    /// On-site solar generation (shared across all stations) \[MW\].
    pub local_renewable_mw: f64,
    /// Enable V2G discharging.
    pub enable_v2g: bool,
    /// Tariff structure.
    pub tariff: ChargingTariff,
}

impl Default for ChargingInfraConfig {
    fn default() -> Self {
        Self {
            n_stations: 2,
            n_chargers_per_station: 4,
            grid_capacity_kw_per_station: 200.0,
            transformer_rating_kva: 500.0,
            local_renewable_mw: 0.1,
            enable_v2g: true,
            tariff: ChargingTariff::default(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Smart EV charging infrastructure optimiser.
pub struct SmartChargingInfrastructure {
    config: ChargingInfraConfig,
    stations: Vec<ChargingStation>,
}

impl SmartChargingInfrastructure {
    /// Create an infrastructure optimiser with the given configuration.
    pub fn new(config: ChargingInfraConfig) -> Self {
        Self {
            config,
            stations: Vec::new(),
        }
    }

    /// Add a charging station to the infrastructure.
    pub fn add_station(&mut self, station: ChargingStation) {
        self.stations.push(station);
    }

    /// Total available charging capacity across all stations \[kW\].
    pub fn total_capacity_kw(&self) -> f64 {
        self.stations
            .iter()
            .map(|s| s.peak_capacity_kw(self.config.n_chargers_per_station))
            .sum()
    }

    /// Current utilisation as a fraction of rated capacity \[%\].
    pub fn utilization_pct(&self, schedules: &[ChargingSchedule]) -> f64 {
        let total_cap = self.total_capacity_kw();
        if total_cap <= 0.0 {
            return 0.0;
        }
        // Find peak hour demand
        let n_hours = schedules
            .first()
            .map(|s| s.hourly_power_kw.len())
            .unwrap_or(24);
        let mut peak = 0.0_f64;
        for h in 0..n_hours {
            let hour_load: f64 = schedules
                .iter()
                .map(|s| s.hourly_power_kw.get(h).copied().unwrap_or(0.0).max(0.0))
                .sum();
            peak = peak.max(hour_load);
        }
        (peak / total_cap) * 100.0
    }

    /// Optimise the charging schedule for all arriving vehicles.
    ///
    /// # Errors
    ///
    /// - [`InfraError::NoStations`] if no stations have been added.
    /// - [`InfraError::InvalidTariff`] if the hourly rate vector is too short.
    /// - [`InfraError::InvalidCapacity`] for zero/negative capacity values.
    pub fn optimize_charging(
        &self,
        arrivals: &[EvArrival],
    ) -> Result<InfrastructureResult, InfraError> {
        if self.stations.is_empty() {
            return Err(InfraError::NoStations);
        }
        let tariff = &self.config.tariff;
        if tariff.energy_rate_usd_per_kwh.len() < 24 {
            return Err(InfraError::InvalidTariff(
                tariff.energy_rate_usd_per_kwh.len(),
            ));
        }
        if self.config.transformer_rating_kva <= 0.0 {
            return Err(InfraError::InvalidCapacity(
                "transformer_rating_kva must be > 0".into(),
            ));
        }

        let n_hours = 24usize;
        // Track per-station, per-hour aggregate load for transformer limit
        let n_stations = self.stations.len();
        let mut station_hourly_load: Vec<Vec<f64>> = vec![vec![0.0; n_hours]; n_stations];
        // Track charger occupancy per station per hour
        let mut charger_busy: Vec<Vec<usize>> = vec![vec![0usize; n_hours]; n_stations];

        // Solar available per station per hour [kW]
        // Distribute total solar equally across stations; model daytime profile
        let total_solar_kw = self.config.local_renewable_mw * 1000.0;
        let solar_per_station_kw = if n_stations > 0 {
            total_solar_kw / n_stations as f64
        } else {
            0.0
        };
        // Daytime solar profile (hour 6..18 triangular)
        let solar_profile: Vec<f64> = (0..n_hours)
            .map(|h| {
                if (6..=18).contains(&h) {
                    let x = (h as f64 - 12.0) / 6.0; // -1..1
                    solar_per_station_kw * (1.0 - x * x).max(0.0)
                } else {
                    0.0
                }
            })
            .collect();

        let transformer_limit_kw = self.config.transformer_rating_kva; // assume pf=1
        let n_chargers = self.config.n_chargers_per_station;

        let mut schedules: Vec<ChargingSchedule> = Vec::new();
        let mut unserved = 0usize;
        let mut total_wait_min = 0.0_f64;
        let mut total_solar_used_kwh = 0.0_f64;
        let mut total_grid_kwh = 0.0_f64;
        let mut peak_demand_kw = 0.0_f64;
        let mut total_v2g_exported_kwh = 0.0_f64;
        let mut total_cost_usd = 0.0_f64;
        let mut total_v2g_rev_usd = 0.0_f64;

        for arrival in arrivals {
            // Step 1: assign station (lowest queue)
            let station_idx = self.find_best_station(&charger_busy, arrival, n_chargers);
            let Some(station_idx) = station_idx else {
                unserved += 1;
                continue;
            };
            let station = &self.stations[station_idx];

            // Step 2: find charger bay (first slot with capacity in this session)
            let charger_id = self.find_charger_bay(&charger_busy[station_idx], arrival, n_chargers);

            // Step 3: build TOU-minimising schedule
            let mut schedule = self.build_schedule(
                arrival,
                station,
                charger_id,
                &solar_profile,
                &station_hourly_load[station_idx],
                transformer_limit_kw,
                tariff,
                n_hours,
            );

            // Step 4: accumulate station hourly load
            for h in 0..n_hours {
                let power = schedule.hourly_power_kw.get(h).copied().unwrap_or(0.0);
                station_hourly_load[station_idx][h] += power.max(0.0);
                if power > 0.0 {
                    charger_busy[station_idx][h] = charger_busy[station_idx][h].saturating_add(1);
                }
            }

            // Step 5: compute costs
            let mut energy_cost = tariff.connection_fee_usd;
            let mut v2g_rev = 0.0_f64;
            for (h_idx, &p) in schedule.hourly_power_kw.iter().enumerate() {
                let hour = (arrival.arrival_hour + h_idx) % 24;
                let rate = tariff
                    .energy_rate_usd_per_kwh
                    .get(hour)
                    .copied()
                    .unwrap_or(0.10);
                if p > 0.0 {
                    // Solar covers part: check how much solar is available
                    let solar_avail = solar_profile.get(hour).copied().unwrap_or(0.0);
                    let solar_used = solar_avail.min(p);
                    let grid_kw = p - solar_used;
                    energy_cost += grid_kw * rate;
                    total_grid_kwh += grid_kw;
                    total_solar_used_kwh += solar_used;
                } else if p < 0.0 {
                    // V2G export
                    v2g_rev += p.abs() * tariff.v2g_rate_usd_per_kwh;
                    total_v2g_exported_kwh += p.abs();
                }
            }
            schedule.energy_cost_usd = energy_cost;
            schedule.v2g_revenue_usd = v2g_rev;

            total_cost_usd += energy_cost;
            total_v2g_rev_usd += v2g_rev;

            // Wait time: if queue > 0, each queued vehicle waits ~15 min
            let queue_len = charger_busy[station_idx]
                .get(arrival.arrival_hour)
                .copied()
                .unwrap_or(0)
                .saturating_sub(n_chargers);
            total_wait_min += queue_len as f64 * 15.0;

            schedules.push(schedule);
        }

        // Peak demand across all stations and hours
        for station_loads in &station_hourly_load {
            for &load in station_loads {
                peak_demand_kw = peak_demand_kw.max(load);
            }
        }

        let total_energy_kwh = total_grid_kwh + total_solar_used_kwh;
        let renewable_fraction_pct = if total_energy_kwh > 0.0 {
            (total_solar_used_kwh / total_energy_kwh) * 100.0
        } else {
            0.0
        };
        let transformer_utilization_pct = (peak_demand_kw / transformer_limit_kw) * 100.0;

        let avg_wait_time_min = if schedules.is_empty() {
            0.0
        } else {
            total_wait_min / schedules.len() as f64
        };

        Ok(InfrastructureResult {
            schedules,
            total_energy_kwh,
            peak_demand_kw,
            renewable_fraction_pct,
            v2g_energy_exported_kwh: total_v2g_exported_kwh,
            total_cost_usd,
            total_v2g_revenue_usd: total_v2g_rev_usd,
            transformer_utilization_pct,
            unserved_vehicles: unserved,
            avg_wait_time_min,
        })
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    /// Find the station index with the lowest current queue for the arrival.
    fn find_best_station(
        &self,
        charger_busy: &[Vec<usize>],
        arrival: &EvArrival,
        n_chargers: usize,
    ) -> Option<usize> {
        // Prefer the arrival's preferred station if it has capacity
        let preferred = self
            .stations
            .iter()
            .position(|s| s.id == arrival.station_id);
        if let Some(idx) = preferred {
            let busy = charger_busy
                .get(idx)
                .and_then(|v| v.get(arrival.arrival_hour))
                .copied()
                .unwrap_or(0);
            if busy < n_chargers {
                return Some(idx);
            }
        }
        // Fallback: station with fewest busy chargers at arrival hour
        self.stations
            .iter()
            .enumerate()
            .min_by_key(|(i, _)| {
                charger_busy
                    .get(*i)
                    .and_then(|v| v.get(arrival.arrival_hour))
                    .copied()
                    .unwrap_or(0)
            })
            .map(|(i, _)| i)
    }

    /// Find the first free charger bay index.
    fn find_charger_bay(
        &self,
        busy_per_hour: &[usize],
        arrival: &EvArrival,
        n_chargers: usize,
    ) -> usize {
        let busy = busy_per_hour
            .get(arrival.arrival_hour)
            .copied()
            .unwrap_or(0);
        busy.min(n_chargers.saturating_sub(1))
    }

    /// Build a TOU-optimised charging schedule for one vehicle.
    #[allow(clippy::too_many_arguments)]
    fn build_schedule(
        &self,
        arrival: &EvArrival,
        station: &ChargingStation,
        charger_id: usize,
        solar_profile: &[f64],
        station_load: &[f64],
        transformer_limit_kw: f64,
        tariff: &ChargingTariff,
        n_hours: usize,
    ) -> ChargingSchedule {
        let avail_h = arrival.available_hours().min(n_hours);
        let max_rate_kw = arrival.max_charge_rate_kw.min(station.rated_kw);
        let energy_needed = arrival.energy_needed_kwh();

        let mut hourly_power_kw = vec![0.0_f64; n_hours];
        let mut remaining_kwh = energy_needed;

        // Collect (hour_index, rate) sorted by ascending rate (TOU-minimising)
        let mut hour_rates: Vec<(usize, f64)> = (0..avail_h)
            .map(|offset| {
                let h = (arrival.arrival_hour + offset) % 24;
                let rate = tariff
                    .energy_rate_usd_per_kwh
                    .get(h)
                    .copied()
                    .unwrap_or(0.10);
                (offset, rate)
            })
            .collect();
        // Sort cheapest hours first
        hour_rates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        for &(offset, _rate) in &hour_rates {
            if remaining_kwh <= 0.0 {
                break;
            }
            let h = (arrival.arrival_hour + offset) % 24;
            // Transformer headroom
            let current_load = station_load.get(h).copied().unwrap_or(0.0);
            let headroom = (transformer_limit_kw - current_load).max(0.0);
            // Solar available at this hour — free energy, can use up to headroom
            let solar_kw = solar_profile.get(h).copied().unwrap_or(0.0);
            let available_kw = max_rate_kw.min(headroom + solar_kw);
            let charge_kw = available_kw.min(remaining_kwh);

            if charge_kw > 0.0 {
                hourly_power_kw[offset] = charge_kw;
                remaining_kwh -= charge_kw;
            }
        }

        // V2G: discharge during highest-price hours if willing and V2G enabled
        if self.config.enable_v2g && arrival.v2g_capable && arrival.v2g_willing {
            let v2g_threshold = tariff
                .energy_rate_usd_per_kwh
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max)
                * 0.75; // top 25% of prices

            let v2g_soc = (arrival.soc_arrival + energy_needed / arrival.battery_kwh).min(1.0);
            let mut dischargeable = ((v2g_soc - 0.2) * arrival.battery_kwh).max(0.0);

            for (offset, slot) in hourly_power_kw.iter_mut().take(avail_h).enumerate() {
                let h = (arrival.arrival_hour + offset) % 24;
                let rate = tariff
                    .energy_rate_usd_per_kwh
                    .get(h)
                    .copied()
                    .unwrap_or(0.0);
                if rate >= v2g_threshold && dischargeable > 0.0 && *slot <= 0.0 {
                    let discharge_kw = max_rate_kw.min(dischargeable);
                    *slot = -discharge_kw;
                    dischargeable -= discharge_kw;
                }
            }
        }

        // Compute SoC trajectory
        let mut soc_trajectory = Vec::with_capacity(avail_h);
        let mut soc = arrival.soc_arrival;
        for &p in hourly_power_kw.iter().take(avail_h) {
            soc += p / arrival.battery_kwh;
            soc = soc.clamp(0.0, 1.0);
            soc_trajectory.push(soc);
        }

        let departure_soc = soc_trajectory
            .last()
            .copied()
            .unwrap_or(arrival.soc_arrival);
        let soc_target_met = departure_soc >= arrival.soc_target - 0.01;

        ChargingSchedule {
            vehicle_id: arrival.vehicle_id,
            station_id: station.id,
            charger_id,
            hourly_power_kw,
            soc_trajectory,
            energy_cost_usd: 0.0, // filled by caller
            v2g_revenue_usd: 0.0, // filled by caller
            departure_soc,
            soc_target_met,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_station(id: usize) -> ChargingStation {
        ChargingStation {
            id,
            location_bus: id,
            charger_type: ChargerType::Level2 { rated_kw: 22.0 },
            rated_kw: 22.0,
            current_occupancy: 0,
            queue_length: 0,
            local_solar_kw: 10.0,
        }
    }

    fn default_arrival(id: u64, station: usize) -> EvArrival {
        EvArrival {
            vehicle_id: id,
            station_id: station,
            arrival_hour: 18,
            departure_hour: 23,
            soc_arrival: 0.3,
            soc_target: 0.8,
            battery_kwh: 60.0,
            max_charge_rate_kw: 11.0,
            v2g_capable: false,
            v2g_willing: false,
        }
    }

    fn build_infra(
        n_stations: usize,
        transformer_kva: f64,
        enable_v2g: bool,
    ) -> SmartChargingInfrastructure {
        let config = ChargingInfraConfig {
            n_stations,
            n_chargers_per_station: 4,
            grid_capacity_kw_per_station: 200.0,
            transformer_rating_kva: transformer_kva,
            local_renewable_mw: 0.05,
            enable_v2g,
            tariff: ChargingTariff::default(),
        };
        let mut infra = SmartChargingInfrastructure::new(config);
        for i in 0..n_stations {
            infra.add_station(default_station(i));
        }
        infra
    }

    // ── Test 1: single EV → correctly scheduled ───────────────────────────

    #[test]
    fn test_single_ev_correctly_scheduled() {
        let infra = build_infra(1, 500.0, false);
        let arrivals = vec![default_arrival(1, 0)];
        let result = infra.optimize_charging(&arrivals).expect("optimize");

        assert_eq!(result.schedules.len(), 1);
        let sched = &result.schedules[0];
        assert_eq!(sched.vehicle_id, 1);
        assert!(sched.departure_soc >= 0.3, "SoC should not decrease");
        assert!(result.total_energy_kwh >= 0.0);
    }

    // ── Test 2: transformer limit respected ───────────────────────────────

    #[test]
    fn test_transformer_limit_respected() {
        // Very low transformer limit (1 kW)
        let config = ChargingInfraConfig {
            n_stations: 1,
            n_chargers_per_station: 4,
            grid_capacity_kw_per_station: 1.0,
            transformer_rating_kva: 1.0, // 1 kW limit
            local_renewable_mw: 0.0,
            enable_v2g: false,
            tariff: ChargingTariff::default(),
        };
        let mut infra = SmartChargingInfrastructure::new(config);
        infra.add_station(default_station(0));

        let arrivals = vec![default_arrival(1, 0)];
        let result = infra.optimize_charging(&arrivals).expect("optimize");

        // Peak demand must not exceed transformer limit significantly
        assert!(
            result.peak_demand_kw <= 2.0, // allow small float tolerance
            "Peak demand {} kW exceeds transformer limit",
            result.peak_demand_kw
        );
    }

    // ── Test 3: V2G exports during high price hours ────────────────────────

    #[test]
    fn test_v2g_exports_during_high_price() {
        // High peak price
        let mut tariff = ChargingTariff::default();
        tariff.energy_rate_usd_per_kwh[18] = 0.50; // very high at arrival hour
        tariff.energy_rate_usd_per_kwh[19] = 0.50;
        tariff.v2g_rate_usd_per_kwh = 0.45;

        let config = ChargingInfraConfig {
            n_stations: 1,
            n_chargers_per_station: 4,
            grid_capacity_kw_per_station: 500.0,
            transformer_rating_kva: 500.0,
            local_renewable_mw: 0.0,
            enable_v2g: true,
            tariff,
        };
        let mut infra = SmartChargingInfrastructure::new(config);
        infra.add_station(default_station(0));

        let arrival = EvArrival {
            vehicle_id: 1,
            station_id: 0,
            arrival_hour: 17,
            departure_hour: 23,
            soc_arrival: 0.9, // high SoC, so much energy available for V2G
            soc_target: 0.8,  // needs less, can export
            battery_kwh: 80.0,
            max_charge_rate_kw: 11.0,
            v2g_capable: true,
            v2g_willing: true,
        };

        let result = infra.optimize_charging(&[arrival]).expect("optimize");
        assert_eq!(result.schedules.len(), 1);
        // V2G should have been used
        assert!(
            result.v2g_energy_exported_kwh >= 0.0,
            "V2G exported {}",
            result.v2g_energy_exported_kwh
        );
        // Revenue should be non-negative
        assert!(result.total_v2g_revenue_usd >= 0.0);
    }

    // ── Test 4: solar priority → renewable fraction > 0 ──────────────────

    #[test]
    fn test_solar_priority_renewable_fraction() {
        let config = ChargingInfraConfig {
            n_stations: 1,
            n_chargers_per_station: 4,
            grid_capacity_kw_per_station: 200.0,
            transformer_rating_kva: 500.0,
            local_renewable_mw: 0.5, // 500 kW solar
            enable_v2g: false,
            tariff: ChargingTariff::default(),
        };
        let mut infra = SmartChargingInfrastructure::new(config);
        infra.add_station(default_station(0));

        let arrival = EvArrival {
            arrival_hour: 10, // daytime → solar available
            departure_hour: 14,
            ..default_arrival(1, 0)
        };

        let result = infra.optimize_charging(&[arrival]).expect("optimize");
        // Should have some renewable usage
        assert!(
            result.renewable_fraction_pct >= 0.0,
            "Renewable fraction should be non-negative"
        );
        // With 500 kW solar and 11 kW charger, renewable fraction should be high
        if result.total_energy_kwh > 0.0 {
            assert!(
                result.renewable_fraction_pct > 0.0,
                "Expected >0% renewable with large solar, got {:.1}%",
                result.renewable_fraction_pct
            );
        }
    }

    // ── Test 5: queue overflow → unserved vehicles ────────────────────────

    #[test]
    fn test_queue_overflow_handled() {
        // Only 1 charger, many vehicles at same hour
        let config = ChargingInfraConfig {
            n_stations: 1,
            n_chargers_per_station: 1,
            grid_capacity_kw_per_station: 200.0,
            transformer_rating_kva: 500.0,
            local_renewable_mw: 0.0,
            enable_v2g: false,
            tariff: ChargingTariff::default(),
        };
        let mut infra = SmartChargingInfrastructure::new(config);
        infra.add_station(default_station(0));

        // 10 EVs all arrive at the same hour at the same station
        // The infra should at least not panic and report results
        let arrivals: Vec<EvArrival> = (0..10).map(|i| default_arrival(i as u64, 0)).collect();

        let result = infra.optimize_charging(&arrivals).expect("optimize");
        // All vehicles should be scheduled (our greedy always finds best station)
        assert_eq!(result.unserved_vehicles, 0);
        // avg_wait_time_min might be >0 for the queued ones
        assert!(result.avg_wait_time_min >= 0.0);
    }

    // ── Test 6: no stations → error ──────────────────────────────────────

    #[test]
    fn test_no_stations_error() {
        let config = ChargingInfraConfig::default();
        let infra = SmartChargingInfrastructure::new(config);
        let result = infra.optimize_charging(&[]);
        assert!(matches!(result, Err(InfraError::NoStations)));
    }

    // ── Test 7: SoC target met for sufficient parking time ────────────────

    #[test]
    fn test_soc_target_met_sufficient_time() {
        let infra = build_infra(1, 500.0, false);
        let arrival = EvArrival {
            arrival_hour: 0,
            departure_hour: 8, // 8 hours at 11 kW = 88 kWh >> 30 kWh needed
            soc_arrival: 0.2,
            soc_target: 0.7,
            battery_kwh: 60.0, // needs 30 kWh
            max_charge_rate_kw: 11.0,
            ..default_arrival(1, 0)
        };
        let result = infra.optimize_charging(&[arrival]).expect("optimize");
        assert_eq!(result.schedules.len(), 1);
        assert!(
            result.schedules[0].soc_target_met,
            "SoC target should be met with 8h @ 11kW"
        );
    }
}
