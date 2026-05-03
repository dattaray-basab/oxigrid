//! EV–Grid Integration Analysis.
//!
//! Provides comprehensive tools for analysing the impact of electric vehicles
//! on electrical distribution networks, coordinating smart charging, calculating
//! V2G revenue, managing demand-response programmes, optimising charging network
//! placement, and quantifying EV–renewable synergy.
//!
//! # Modules overview
//!
//! | Struct | Purpose |
//! |---|---|
//! | [`EvGridImpact`] | Feeder/transformer impact for a fleet |
//! | [`SmartChargingCoordinator`] | Multi-EV scheduling under four modes |
//! | [`V2gRevenueCalculator`] | Frequency regulation + arbitrage economics |
//! | [`EvDemandResponse`] | DR curtailment and event activation |
//! | [`ChargingNetworkOptimizer`] | Greedy charger site selection |
//! | [`GridEvSynergy`] | Renewable–EV synergy metrics |

// ─────────────────────────────────────────────────────────────────────────────
// 1. EV Fleet Profile
// ─────────────────────────────────────────────────────────────────────────────

/// Statistical profile of an EV fleet connected to a distribution feeder.
#[derive(Debug, Clone)]
pub struct EvFleetProfile {
    /// Total number of EVs in the fleet.
    pub total_evs: usize,
    /// Average usable battery capacity \[kWh\].
    pub avg_battery_kwh: f64,
    /// Average daily distance driven per vehicle \[km\].
    pub avg_daily_km: f64,
    /// Average energy consumption \[kWh/km\].
    pub energy_per_km_kwh: f64,
    /// Fraction charging at home (0–1).
    pub home_charging_fraction: f64,
    /// Fraction charging at workplace (0–1).
    pub workplace_charging_fraction: f64,
    /// Fraction charging at public stations (0–1).
    pub public_charging_fraction: f64,
    /// Market adoption rate \[%\] (0–100).
    pub adoption_rate_pct: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Grid Impact Assessment
// ─────────────────────────────────────────────────────────────────────────────

/// Grid impact of an EV fleet on a single distribution feeder.
#[derive(Debug, Clone)]
pub struct EvGridImpact {
    /// Feeder identifier.
    pub feeder_id: usize,
    /// Distribution transformer rating \[kVA\].
    pub transformer_kva: f64,
    /// Existing peak feeder load before EV adoption \[MW\].
    pub feeder_peak_mw: f64,
    /// Hourly base-load profile (24 h) \[MW\].
    pub base_load_profile_mw: Vec<f64>,
    /// EV fleet statistics.
    pub ev_fleet: EvFleetProfile,
}

impl EvGridImpact {
    /// Total fleet daily energy demand \[kWh\].
    ///
    /// Formula: `total_evs × avg_daily_km × energy_per_km_kwh × adoption_rate_pct / 100`
    pub fn daily_energy_demand_kwh(&self) -> f64 {
        let f = &self.ev_fleet;
        f.total_evs as f64 * f.avg_daily_km * f.energy_per_km_kwh * f.adoption_rate_pct / 100.0
    }

    /// Uncontrolled charging load profile \[MW\] for 24 h.
    ///
    /// Assumes all home-charging EVs plug in at 18:00 and charge uniformly
    /// over 4 hours (hours 18–21 inclusive).
    pub fn uncontrolled_charging_profile(&self) -> Vec<f64> {
        let mut profile = vec![0.0_f64; 24];
        let energy_home_kwh = self.daily_energy_demand_kwh() * self.ev_fleet.home_charging_fraction;
        // Spread over 4 h → power = energy / 4 h
        let power_per_hour_kw = energy_home_kwh / 4.0;
        let power_per_hour_mw = power_per_hour_kw / 1_000.0;
        for item in profile.iter_mut().take(22).skip(18) {
            *item = power_per_hour_mw;
        }
        profile
    }

    /// Peak demand increase caused by uncontrolled EV charging \[MW\].
    pub fn peak_demand_increase_mw(&self) -> f64 {
        self.uncontrolled_charging_profile()
            .into_iter()
            .fold(0.0_f64, f64::max)
    }

    /// Transformer loading for a given load \[%\].
    ///
    /// `transformer_loading_pct = load_mw × 1000 / transformer_kva × 100`
    pub fn transformer_loading_pct(&self, load_mw: f64) -> f64 {
        if self.transformer_kva <= 0.0 {
            return 0.0;
        }
        load_mw * 1_000.0 / self.transformer_kva * 100.0
    }

    /// Maximum number of additional EVs the feeder can host while keeping
    /// transformer loading below `max_loading_pct` \[%\].
    pub fn hosting_capacity_evs(&self, max_loading_pct: f64) -> usize {
        // Allowed additional load [MW]
        let max_load_mw = max_loading_pct / 100.0 * self.transformer_kva / 1_000.0;
        let headroom_mw = (max_load_mw - self.feeder_peak_mw).max(0.0);

        // Energy per EV per 4-h charging window → average power [MW]
        let energy_per_ev_kwh = self.ev_fleet.avg_daily_km
            * self.ev_fleet.energy_per_km_kwh
            * self.ev_fleet.adoption_rate_pct
            / 100.0;
        let avg_power_per_ev_mw = energy_per_ev_kwh / 4.0 / 1_000.0;

        if avg_power_per_ev_mw <= 0.0 {
            return 0;
        }
        (headroom_mw / avg_power_per_ev_mw).floor() as usize
    }

    /// Approximate voltage impact of EV peak charging \[%\].
    ///
    /// Uses the simplified formula ΔV ≈ P·R / V² where V = 1 pu.
    /// `impedance_pu` is the feeder impedance in per unit.
    pub fn voltage_impact_estimate_pct(&self, impedance_pu: f64) -> f64 {
        let p_pu = self.peak_demand_increase_mw() / self.transformer_kva * 1_000.0;
        p_pu * impedance_pu * 100.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Smart Charging Coordinator
// ─────────────────────────────────────────────────────────────────────────────

/// A single EV charging session within a 24-hour window.
#[derive(Debug, Clone)]
pub struct EvChargingSession {
    /// Vehicle identifier.
    pub ev_id: usize,
    /// Hour of arrival (0–23).
    pub arrival_hour: usize,
    /// Hour of departure (1–24; may be ≥ 24 for overnight).
    pub departure_hour: usize,
    /// Energy required to reach target SoC \[kWh\].
    pub energy_needed_kwh: f64,
    /// Charger rated power \[kW\].
    pub charger_kw: f64,
    /// State of charge on arrival (0–1).
    pub initial_soc: f64,
    /// Usable battery capacity \[kWh\].
    pub battery_kwh: f64,
    /// Hourly scheduled power (24 slots) \[kW\] — filled by coordinator.
    pub scheduled_power_kw: Vec<f64>,
}

impl EvChargingSession {
    /// Create a new session with zeroed schedule.
    pub fn new(
        ev_id: usize,
        arrival_hour: usize,
        departure_hour: usize,
        energy_needed_kwh: f64,
        charger_kw: f64,
        initial_soc: f64,
        battery_kwh: f64,
    ) -> Self {
        Self {
            ev_id,
            arrival_hour,
            departure_hour,
            energy_needed_kwh,
            charger_kw,
            initial_soc,
            battery_kwh,
            scheduled_power_kw: vec![0.0; 24],
        }
    }

    /// Total energy scheduled \[kWh\].
    pub fn total_scheduled_kwh(&self) -> f64 {
        self.scheduled_power_kw.iter().sum()
    }

    /// Whether energy target is satisfied within a tolerance.
    pub fn is_satisfied(&self) -> bool {
        self.total_scheduled_kwh() >= self.energy_needed_kwh - 1e-6
    }
}

/// Charging coordination strategy.
#[derive(Debug, Clone, PartialEq)]
pub enum CoordinationMode {
    /// Charge immediately at full rated power from arrival.
    Uncontrolled,
    /// Schedule to minimise electricity cost (TOU pricing).
    TouOptimal,
    /// Schedule to minimise carbon emissions.
    CarbonMinimal,
    /// Valley-filling: prioritise hours with lowest aggregated load.
    GridFriendly,
}

/// Result of a single coordination run.
#[derive(Debug, Clone)]
pub struct CoordinationResult {
    /// Mode used.
    pub mode: CoordinationMode,
    /// Total electricity cost \[$/day\].
    pub total_cost: f64,
    /// Total carbon emitted by EV charging \[kg CO₂\].
    pub total_carbon_kg: f64,
    /// Peak aggregated EV power \[MW\].
    pub peak_mw: f64,
    /// Fraction of sessions whose energy target was fully met \[%\].
    pub sessions_satisfied_pct: f64,
}

/// Coordinates multiple EV charging sessions under different optimisation modes.
pub struct SmartChargingCoordinator {
    /// Registered sessions (schedule filled after `coordinate()`).
    pub sessions: Vec<EvChargingSession>,
    /// Feeder capacity limit \[MW\].
    pub grid_capacity_mw: f64,
    /// Hourly TOU electricity prices (24 h) \[$/MWh\].
    pub tou_prices: Vec<f64>,
    /// Hourly carbon intensity (24 h) \[gCO₂/kWh\].
    pub carbon_intensity: Vec<f64>,
    /// Active coordination mode.
    pub mode: CoordinationMode,
}

impl SmartChargingCoordinator {
    /// Construct a new coordinator with the given grid capacity and price/carbon vectors.
    pub fn new(capacity_mw: f64, prices: Vec<f64>, carbon: Vec<f64>) -> Self {
        Self {
            sessions: Vec::new(),
            grid_capacity_mw: capacity_mw,
            tou_prices: prices,
            carbon_intensity: carbon,
            mode: CoordinationMode::GridFriendly,
        }
    }

    /// Register an EV session.
    pub fn add_session(&mut self, session: EvChargingSession) {
        self.sessions.push(session);
    }

    /// Run coordination for the current mode and return a [`CoordinationResult`].
    pub fn coordinate(&mut self) -> CoordinationResult {
        let hours = 24_usize;
        // Reset schedules
        for s in &mut self.sessions {
            s.scheduled_power_kw = vec![0.0; hours];
        }

        // Clone price/carbon/mode data to avoid borrow conflicts in closures
        let tou_prices = self.tou_prices.clone();
        let carbon_intensity = self.carbon_intensity.clone();
        let mode = self.mode.clone();

        match mode {
            CoordinationMode::Uncontrolled => self.schedule_uncontrolled(),
            CoordinationMode::TouOptimal => self.schedule_sorted(|a, b, _, _| {
                let pa = tou_prices.get(a).copied().unwrap_or(f64::MAX);
                let pb = tou_prices.get(b).copied().unwrap_or(f64::MAX);
                pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
            }),
            CoordinationMode::CarbonMinimal => self.schedule_sorted(|a, b, _, _| {
                let ca = carbon_intensity.get(a).copied().unwrap_or(f64::MAX);
                let cb = carbon_intensity.get(b).copied().unwrap_or(f64::MAX);
                ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
            }),
            CoordinationMode::GridFriendly => {
                // Build current total-load profile for hour-ordering
                let base: Vec<f64> = (0..hours)
                    .map(|h| {
                        self.sessions
                            .iter()
                            .map(|s| s.scheduled_power_kw.get(h).copied().unwrap_or(0.0))
                            .sum::<f64>()
                            / 1_000.0
                    })
                    .collect();
                self.schedule_sorted(|a, b, _, _| {
                    let la = base.get(a).copied().unwrap_or(0.0);
                    let lb = base.get(b).copied().unwrap_or(0.0);
                    la.partial_cmp(&lb).unwrap_or(std::cmp::Ordering::Equal)
                })
            }
        };

        self.build_result()
    }

    fn schedule_uncontrolled(&mut self) {
        let hours = 24_usize;
        for s in &mut self.sessions {
            let mut remaining_kwh = s.energy_needed_kwh;
            let dep = s.departure_hour.min(hours);
            for h in s.arrival_hour..dep {
                if remaining_kwh <= 0.0 {
                    break;
                }
                let power = s.charger_kw.min(remaining_kwh);
                s.scheduled_power_kw[h] = power;
                remaining_kwh -= power;
            }
        }
    }

    /// Generic sorted-priority scheduler.
    /// `cmp` receives `(hour_a, hour_b, session_idx, hour_count)` and returns ordering.
    fn schedule_sorted<F>(&mut self, cmp: F)
    where
        F: Fn(usize, usize, usize, usize) -> std::cmp::Ordering,
    {
        let hours = 24_usize;
        for (idx, s) in self.sessions.iter_mut().enumerate() {
            let dep = s.departure_hour.min(hours);
            let window: Vec<usize> = (s.arrival_hour..dep).collect();
            let mut sorted_window = window.clone();
            sorted_window.sort_by(|&a, &b| cmp(a, b, idx, hours));

            let mut remaining_kwh = s.energy_needed_kwh;
            for h in sorted_window {
                if remaining_kwh <= 0.0 {
                    break;
                }
                let power = s.charger_kw.min(remaining_kwh);
                s.scheduled_power_kw[h] = power;
                remaining_kwh -= power;
            }
        }
    }

    /// Aggregated hourly EV demand profile \[MW\].
    pub fn total_demand_profile(&self) -> Vec<f64> {
        let hours = 24_usize;
        let mut profile = vec![0.0_f64; hours];
        for s in &self.sessions {
            for (h, slot) in profile.iter_mut().enumerate().take(hours) {
                *slot += s.scheduled_power_kw.get(h).copied().unwrap_or(0.0) / 1_000.0;
            }
        }
        profile
    }

    /// Run all four modes on cloned sessions and return total cost for each.
    ///
    /// Returns `(uncontrolled_cost, tou_cost, carbon_cost, grid_cost)` \[$/day\].
    pub fn cost_comparison(&self) -> (f64, f64, f64, f64) {
        let modes = [
            CoordinationMode::Uncontrolled,
            CoordinationMode::TouOptimal,
            CoordinationMode::CarbonMinimal,
            CoordinationMode::GridFriendly,
        ];
        let mut costs = [0.0_f64; 4];
        for (i, mode) in modes.iter().enumerate() {
            let mut clone = SmartChargingCoordinator {
                sessions: self.sessions.clone(),
                grid_capacity_mw: self.grid_capacity_mw,
                tou_prices: self.tou_prices.clone(),
                carbon_intensity: self.carbon_intensity.clone(),
                mode: mode.clone(),
            };
            let result = clone.coordinate();
            costs[i] = result.total_cost;
        }
        (costs[0], costs[1], costs[2], costs[3])
    }

    fn build_result(&self) -> CoordinationResult {
        let hours = 24_usize;
        let profile = self.total_demand_profile();
        let peak_mw = profile.iter().cloned().fold(0.0_f64, f64::max);

        let mut total_cost = 0.0_f64;
        let mut total_carbon_kg = 0.0_f64;
        for s in &self.sessions {
            for h in 0..hours {
                let p_kw = s.scheduled_power_kw.get(h).copied().unwrap_or(0.0);
                let p_mwh = p_kw / 1_000.0; // MWh charged in 1 h
                let price = self.tou_prices.get(h).copied().unwrap_or(0.0);
                let carbon = self.carbon_intensity.get(h).copied().unwrap_or(0.0);
                total_cost += p_mwh * price;
                total_carbon_kg += p_kw * carbon / 1_000.0; // kWh * gCO2/kWh / 1000 → kg
            }
        }

        let satisfied = self.sessions.iter().filter(|s| s.is_satisfied()).count();
        let sessions_satisfied_pct = if self.sessions.is_empty() {
            100.0
        } else {
            satisfied as f64 / self.sessions.len() as f64 * 100.0
        };

        CoordinationResult {
            mode: self.mode.clone(),
            total_cost,
            total_carbon_kg,
            peak_mw,
            sessions_satisfied_pct,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. V2G Revenue Calculator
// ─────────────────────────────────────────────────────────────────────────────

/// Calculates V2G revenue streams for a fleet.
#[derive(Debug, Clone)]
pub struct V2gRevenueCalculator {
    /// Number of vehicles in the fleet.
    pub fleet_vehicles: usize,
    /// Battery capacity per vehicle \[kWh\].
    pub battery_kwh_per_vehicle: f64,
    /// Usable SoC window for V2G `(soc_min, soc_max)`.
    pub usable_soc_range: (f64, f64),
    /// Bidirectional charger rated power \[kW\].
    pub charger_kw: f64,
    /// Round-trip charging+discharging efficiency (0–1).
    pub round_trip_efficiency: f64,
    /// Average hours per day vehicles are grid-connected \[h/day\].
    pub availability_hours_per_day: f64,
    /// Battery degradation cost per \[kWh\] of energy throughput \[$/kWh\].
    pub degradation_cost_per_kwh: f64,
}

impl V2gRevenueCalculator {
    /// Maximum V2G discharge power for the fleet \[MW\].
    ///
    /// Formula: `fleet_vehicles × charger_kw / 1000`
    pub fn fleet_v2g_capacity_mw(&self) -> f64 {
        self.fleet_vehicles as f64 * self.charger_kw / 1_000.0
    }

    /// Usable fleet energy available for V2G \[kWh\].
    ///
    /// Accounts for discharge-half efficiency: η_discharge ≈ √η_rt.
    pub fn fleet_energy_available_kwh(&self) -> f64 {
        let (soc_min, soc_max) = self.usable_soc_range;
        let eta_discharge = self.round_trip_efficiency.max(0.0).sqrt();
        self.fleet_vehicles as f64
            * self.battery_kwh_per_vehicle
            * (soc_max - soc_min).max(0.0)
            * eta_discharge
    }

    /// Gross daily frequency-regulation revenue minus degradation cost \[$/day\].
    ///
    /// `revenue = capacity_mw × availability_h × fr_price_per_mw_h`
    /// `degr_cost = degradation_cost_per_kwh × energy_available_kwh × (availability_h / 8)`
    pub fn frequency_regulation_revenue_per_day(&self, fr_price_per_mw_h: f64) -> f64 {
        let revenue =
            self.fleet_v2g_capacity_mw() * self.availability_hours_per_day * fr_price_per_mw_h;
        let energy_cycled =
            self.fleet_energy_available_kwh() * self.availability_hours_per_day / 8.0;
        let degradation = self.degradation_cost_per_kwh * energy_cycled;
        revenue - degradation
    }

    /// Net daily arbitrage revenue \[$/day\].
    ///
    /// Charges at `buy_price_per_mwh`, discharges at `sell_price_per_mwh` for
    /// `cycles` full charge–discharge cycles per day.
    pub fn arbitrage_revenue_per_day(
        &self,
        buy_price_per_mwh: f64,
        sell_price_per_mwh: f64,
        cycles: f64,
    ) -> f64 {
        let energy_kwh = self.fleet_energy_available_kwh();
        let gross = cycles * energy_kwh * (sell_price_per_mwh - buy_price_per_mwh) / 1_000.0
            * self.round_trip_efficiency.max(0.0).sqrt();
        let degradation = cycles * energy_kwh * self.degradation_cost_per_kwh;
        gross - degradation
    }

    /// Net annual value combining frequency regulation and arbitrage \[$/year\].
    ///
    /// `fr_price` in \[$/MW·h\], `peak_arbitrage_spread` in \[$/MWh\],
    /// `utilization_days` active days per year.
    pub fn net_annual_value(
        &self,
        fr_price: f64,
        peak_arbitrage_spread: f64,
        utilization_days: f64,
    ) -> f64 {
        let daily_fr = self.frequency_regulation_revenue_per_day(fr_price);
        let daily_arb = self.arbitrage_revenue_per_day(0.0, peak_arbitrage_spread, 1.0);
        (daily_fr + daily_arb) * utilization_days
    }

    /// Number of full battery cycles at which V2G revenue covers `installation_cost` \[cycles\].
    pub fn break_even_battery_cycles(&self, installation_cost: f64) -> f64 {
        let energy = self.fleet_energy_available_kwh();
        if energy <= 0.0 || self.degradation_cost_per_kwh <= 0.0 {
            return f64::INFINITY;
        }
        installation_cost / (self.degradation_cost_per_kwh * energy)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. EV Demand Response
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a demand-response activation event.
#[derive(Debug, Clone)]
pub struct DrActivationResult {
    /// Power actually curtailed \[MW\].
    pub activated_mw: f64,
    /// Unmet curtailment request \[MW\].
    pub shortfall_mw: f64,
    /// Number of vehicles participating.
    pub participating_evs: usize,
    /// Estimated compensation paid to EV owners \[$/event\].
    pub estimated_cost: f64,
}

/// Models an EV demand-response programme.
#[derive(Debug, Clone)]
pub struct EvDemandResponse {
    /// Total enrolled vehicles.
    pub enrolled_evs: usize,
    /// Fraction willing to respond on a given event (0–1).
    pub response_capacity_fraction: f64,
    /// Minimum fleet average SoC required to participate (0–1).
    pub min_soc_for_dr: f64,
    /// Notification-to-response delay \[min\].
    pub notification_delay_min: f64,
    /// Maximum power reduction as fraction of current demand \[%\].
    pub max_curtailment_pct: f64,
    /// Compensation paid to EV owner per curtailed \[kWh\] \[$/kWh\].
    pub compensation_per_kwh: f64,
}

impl EvDemandResponse {
    /// Available curtailment power given current EV demand and average SoC \[MW\].
    ///
    /// Returns 0 if `avg_soc < min_soc_for_dr`.
    pub fn available_curtailment_mw(&self, current_demand_mw: f64, avg_soc: f64) -> f64 {
        if avg_soc < self.min_soc_for_dr {
            return 0.0;
        }
        current_demand_mw * self.response_capacity_fraction * self.max_curtailment_pct / 100.0
    }

    /// Activate a DR event.
    ///
    /// `requested_mw` — grid operator's curtailment request \[MW\].
    /// `current_soc` — fleet average SoC (0–1).
    /// `duration_h` — event duration \[h\].
    pub fn activate_event(
        &self,
        requested_mw: f64,
        current_soc: f64,
        duration_h: f64,
    ) -> DrActivationResult {
        if current_soc < self.min_soc_for_dr {
            return DrActivationResult {
                activated_mw: 0.0,
                shortfall_mw: requested_mw,
                participating_evs: 0,
                estimated_cost: 0.0,
            };
        }

        let max_curtail =
            requested_mw * self.response_capacity_fraction * self.max_curtailment_pct / 100.0;
        let activated_mw = max_curtail.min(requested_mw);
        let shortfall_mw = (requested_mw - activated_mw).max(0.0);
        let participating_evs =
            (self.enrolled_evs as f64 * self.response_capacity_fraction).round() as usize;
        let curtailed_kwh = activated_mw * 1_000.0 * duration_h;
        let estimated_cost = curtailed_kwh * self.compensation_per_kwh;

        DrActivationResult {
            activated_mw,
            shortfall_mw,
            participating_evs,
            estimated_cost,
        }
    }

    /// Annual demand-response programme value \[$/year\].
    pub fn annual_dr_value(
        &self,
        events_per_year: usize,
        avg_event_mw: f64,
        avg_duration_h: f64,
    ) -> f64 {
        let curtailed_kwh_per_event = avg_event_mw * 1_000.0 * avg_duration_h;
        let cost_per_event = curtailed_kwh_per_event * self.compensation_per_kwh;
        // Value to the grid operator (avoided cost) minus EV owner compensation
        // Simplified: net value = curtailment MW * 100 $/MWh avoided - compensation
        let avoided_cost_per_event = avg_event_mw * 100.0 * avg_duration_h;
        (avoided_cost_per_event - cost_per_event) * events_per_year as f64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Charging Network Optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// Charger hardware category for network planning.
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, PartialEq)]
pub enum NetworkChargerType {
    /// Level 1 AC ~1 \[kW\], residential.
    Level1_1kw,
    /// Level 2 AC ~7 \[kW\], commercial/residential.
    Level2_7kw,
    /// DC fast charge 50 \[kW\].
    Dcfc_50kw,
    /// DC fast charge 150 \[kW\].
    Dcfc_150kw,
    /// High-power charge 350 \[kW\].
    Hpc_350kw,
}

/// A candidate charging site.
#[derive(Debug, Clone)]
pub struct ChargingLocation {
    /// Location identifier.
    pub id: usize,
    /// Easting coordinate \[km\] (or arbitrary units).
    pub x: f64,
    /// Northing coordinate \[km\].
    pub y: f64,
    /// Charger hardware type.
    pub charger_type: NetworkChargerType,
    /// One-time installation cost \[$/site\].
    pub install_cost: f64,
    /// Annual operating cost \[$/year\].
    pub annual_opex: f64,
    /// Rated charger output power \[kW\].
    pub charger_kw: f64,
    /// Throughput capacity \[vehicles/day\].
    pub capacity: usize,
}

/// Greedy charging network site selection.
pub struct ChargingNetworkOptimizer {
    /// All candidate locations.
    pub candidate_locations: Vec<ChargingLocation>,
    /// Total capital budget \[$/\].
    pub budget: f64,
    /// EV demand nodes `(x, y, daily_demand_kwh)`.
    pub ev_demand_nodes: Vec<(f64, f64, f64)>,
}

impl ChargingNetworkOptimizer {
    /// Service radius of each charger type \[km\].
    pub fn coverage_radius_km(&self, charger_type: &NetworkChargerType) -> f64 {
        match charger_type {
            NetworkChargerType::Level1_1kw => 1.0,
            NetworkChargerType::Level2_7kw => 5.0,
            NetworkChargerType::Dcfc_50kw => 20.0,
            NetworkChargerType::Dcfc_150kw => 50.0,
            NetworkChargerType::Hpc_350kw => 100.0,
        }
    }

    /// Demand \[kWh/day\] served by a location (sum of nodes within coverage radius).
    fn demand_served(&self, loc: &ChargingLocation) -> f64 {
        let radius = self.coverage_radius_km(&loc.charger_type);
        self.ev_demand_nodes
            .iter()
            .filter(|(nx, ny, _)| {
                let dx = nx - loc.x;
                let dy = ny - loc.y;
                (dx * dx + dy * dy).sqrt() <= radius
            })
            .map(|(_, _, d)| *d)
            .sum()
    }

    /// Greedy site selection: rank by demand_served/install_cost, pick within budget.
    ///
    /// Returns a list of selected location IDs.
    pub fn greedy_placement(&self) -> Vec<usize> {
        // Score each candidate
        let mut scored: Vec<(usize, f64)> = self
            .candidate_locations
            .iter()
            .filter(|l| l.install_cost > 0.0)
            .map(|l| {
                let score = self.demand_served(l) / l.install_cost;
                (l.id, score)
            })
            .collect();
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut selected = Vec::new();
        let mut spent = 0.0_f64;
        for (id, _) in &scored {
            if let Some(loc) = self.candidate_locations.iter().find(|l| l.id == *id) {
                if spent + loc.install_cost <= self.budget {
                    selected.push(*id);
                    spent += loc.install_cost;
                }
            }
        }
        selected
    }

    /// Total installed charger capacity for selected sites \[kW\].
    pub fn total_capacity_selected(&self, selected: &[usize]) -> f64 {
        self.candidate_locations
            .iter()
            .filter(|l| selected.contains(&l.id))
            .map(|l| l.charger_kw)
            .sum()
    }

    /// Fraction of demand nodes covered by at least one selected location \[0–1\].
    pub fn population_coverage_pct(&self, selected: &[usize]) -> f64 {
        if self.ev_demand_nodes.is_empty() {
            return 0.0;
        }
        let selected_locs: Vec<&ChargingLocation> = self
            .candidate_locations
            .iter()
            .filter(|l| selected.contains(&l.id))
            .collect();

        let covered = self
            .ev_demand_nodes
            .iter()
            .filter(|(nx, ny, _)| {
                selected_locs.iter().any(|l| {
                    let dx = nx - l.x;
                    let dy = ny - l.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    dist <= self.coverage_radius_km(&l.charger_type)
                })
            })
            .count();
        covered as f64 / self.ev_demand_nodes.len() as f64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Grid–EV Synergy Metrics
// ─────────────────────────────────────────────────────────────────────────────

/// Metrics quantifying how well smart EV charging integrates with renewable generation.
#[derive(Debug, Clone)]
pub struct GridEvSynergy {
    /// Hourly renewable generation \[MWh\].
    pub renewable_gen_mwh: Vec<f64>,
    /// Hourly EV smart-charging load \[MWh\].
    pub ev_load_mwh: Vec<f64>,
    /// Hourly conventional (non-EV) load \[MWh\].
    pub conventional_load_mwh: Vec<f64>,
}

impl GridEvSynergy {
    fn len(&self) -> usize {
        self.renewable_gen_mwh
            .len()
            .min(self.ev_load_mwh.len())
            .min(self.conventional_load_mwh.len())
    }

    /// Fraction of renewable generation absorbed by EV charging (0–1).
    pub fn renewable_utilization_pct(&self) -> f64 {
        let n = self.len();
        if n == 0 {
            return 0.0;
        }
        let absorbed: f64 = (0..n)
            .map(|i| self.ev_load_mwh[i].min(self.renewable_gen_mwh[i]).max(0.0))
            .sum();
        let total_re: f64 = self.renewable_gen_mwh[..n].iter().sum();
        if total_re <= 0.0 {
            return 0.0;
        }
        (absorbed / total_re).clamp(0.0, 1.0)
    }

    /// Fraction of EV energy supplied by renewables (0–1).
    pub fn ev_renewable_match_pct(&self) -> f64 {
        let n = self.len();
        if n == 0 {
            return 0.0;
        }
        let matched: f64 = (0..n)
            .map(|i| self.ev_load_mwh[i].min(self.renewable_gen_mwh[i]).max(0.0))
            .sum();
        let total_ev: f64 = self.ev_load_mwh[..n].iter().sum();
        if total_ev <= 0.0 {
            return 0.0;
        }
        (matched / total_ev).clamp(0.0, 1.0)
    }

    /// Peak demand reduction attributable to smart charging relative to uncontrolled \[MW\].
    ///
    /// Approximated as the difference between peak total load and average total load,
    /// representing how much valley-filling reduces the peak.
    pub fn peak_shaving_mw(&self) -> f64 {
        let n = self.len();
        if n == 0 {
            return 0.0;
        }
        let total: Vec<f64> = (0..n)
            .map(|i| self.conventional_load_mwh[i] + self.ev_load_mwh[i])
            .collect();
        let peak = total.iter().cloned().fold(0.0_f64, f64::max);
        let mean = total.iter().sum::<f64>() / n as f64;
        (peak - mean).max(0.0)
    }

    /// Valley-filling quality score (0–1, higher is better).
    ///
    /// `score = 1 − std(total_load) / std(conventional_load)`
    /// Clamped to \[0, 1\].
    pub fn valley_filling_score(&self) -> f64 {
        let n = self.len();
        if n == 0 {
            return 0.0;
        }

        let std_dev = |v: &[f64]| -> f64 {
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            let var = v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / v.len() as f64;
            var.sqrt()
        };

        let total: Vec<f64> = (0..n)
            .map(|i| self.conventional_load_mwh[i] + self.ev_load_mwh[i])
            .collect();

        let std_conv = std_dev(&self.conventional_load_mwh[..n]);
        let std_total = std_dev(&total);

        if std_conv <= 0.0 {
            return 0.0;
        }
        (1.0 - std_total / std_conv).clamp(0.0, 1.0)
    }

    /// Fraction of EV load met by local renewable generation (0–1).
    ///
    /// Equivalent to `ev_renewable_match_pct`.
    pub fn self_sufficiency_pct(&self) -> f64 {
        self.ev_renewable_match_pct()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_fleet_profile() -> EvFleetProfile {
        EvFleetProfile {
            total_evs: 1000,
            avg_battery_kwh: 60.0,
            avg_daily_km: 40.0,
            energy_per_km_kwh: 0.2,
            home_charging_fraction: 0.7,
            workplace_charging_fraction: 0.2,
            public_charging_fraction: 0.1,
            adoption_rate_pct: 50.0,
        }
    }

    fn make_impact() -> EvGridImpact {
        EvGridImpact {
            feeder_id: 1,
            transformer_kva: 1000.0, // 1 MVA
            feeder_peak_mw: 0.5,
            base_load_profile_mw: vec![0.3; 24],
            ev_fleet: make_fleet_profile(),
        }
    }

    // ── EvGridImpact ──────────────────────────────────────────────────────────

    #[test]
    fn test_daily_energy_demand() {
        let impact = make_impact();
        // 1000 * 40 * 0.2 * 50/100 = 4000 kWh
        let expected = 1000.0 * 40.0 * 0.2 * 50.0 / 100.0;
        let got = impact.daily_energy_demand_kwh();
        assert!(
            (got - expected).abs() < 1e-6,
            "got={got}, expected={expected}"
        );
    }

    #[test]
    fn test_hosting_capacity_returns_usize() {
        let impact = make_impact();
        let cap: usize = impact.hosting_capacity_evs(90.0);
        // Should be a non-negative integer
        assert!(cap < usize::MAX, "Capacity should be finite");
    }

    #[test]
    fn test_transformer_loading_pct() {
        let impact = make_impact();
        // 0.5 MW * 1000 / 1000 kVA * 100 = 50%
        let loading = impact.transformer_loading_pct(0.5);
        assert!((loading - 50.0).abs() < 1e-9, "loading={loading}");
    }

    #[test]
    fn test_uncontrolled_has_charge_at_arrival() {
        let impact = make_impact();
        let profile = impact.uncontrolled_charging_profile();
        assert_eq!(profile.len(), 24);
        assert!(
            profile[18] > 0.0,
            "Hour 18 should have positive EV load, got {}",
            profile[18]
        );
    }

    #[test]
    fn test_hosting_capacity_positive_for_reasonable_input() {
        let impact = make_impact();
        // 90% loading → headroom = 0.9 MW - 0.5 MW = 0.4 MW available
        let cap = impact.hosting_capacity_evs(90.0);
        // Should be > 0
        assert!(cap > 0, "Expected positive hosting capacity, got {cap}");
    }

    // ── SmartChargingCoordinator ──────────────────────────────────────────────

    fn make_coordinator(mode: CoordinationMode) -> SmartChargingCoordinator {
        let prices: Vec<f64> = (0..24)
            .map(|h| if (9..21).contains(&h) { 0.25 } else { 0.05 })
            .collect();
        let carbon: Vec<f64> = (0..24)
            .map(|h| if (12..18).contains(&h) { 50.0 } else { 200.0 })
            .collect();
        let mut coord = SmartChargingCoordinator::new(10.0, prices, carbon);
        coord.mode = mode;
        coord
    }

    fn add_test_session(coord: &mut SmartChargingCoordinator) {
        // Arrives 18h, departs 23h, needs 10 kWh, 7 kW charger
        let session = EvChargingSession::new(0, 18, 23, 10.0, 7.0, 0.3, 60.0);
        coord.add_session(session);
    }

    #[test]
    fn test_uncontrolled_has_charge_at_arrival_hour() {
        let mut coord = make_coordinator(CoordinationMode::Uncontrolled);
        add_test_session(&mut coord);
        coord.coordinate();
        let power_at_18 = coord.sessions[0].scheduled_power_kw[18];
        assert!(
            power_at_18 > 0.0,
            "Uncontrolled should charge at arrival hour 18, got {power_at_18}"
        );
    }

    #[test]
    fn test_tou_charges_in_cheapest_hours() {
        // Cheap: h<9 and h>=21, expensive: 9≤h<21
        // Session arrives at 8, departs at 23 — window includes both cheap and expensive hours
        let prices: Vec<f64> = (0..24)
            .map(|h| if (9..21).contains(&h) { 0.25 } else { 0.05 })
            .collect();
        let carbon = vec![100.0; 24];
        let mut coord = SmartChargingCoordinator::new(10.0, prices, carbon);
        coord.mode = CoordinationMode::TouOptimal;
        // Arrive 8h, depart 23h, need 10 kWh, 7 kW charger
        let session = EvChargingSession::new(0, 8, 23, 10.0, 7.0, 0.3, 60.0);
        coord.add_session(session);
        coord.coordinate();
        let sched = &coord.sessions[0].scheduled_power_kw;
        // Power at h=8 (cheap) should be >= power at h=9 (expensive)
        let cheap_hour = sched[8];
        let expensive_hour = sched[9];
        assert!(
            cheap_hour >= expensive_hour,
            "TOU should prefer cheap hours: h8={cheap_hour}, h9={expensive_hour}"
        );
    }

    #[test]
    fn test_total_demand_profile_sums_sessions() {
        let mut coord = make_coordinator(CoordinationMode::Uncontrolled);
        add_test_session(&mut coord);
        // Add second session
        coord.add_session(EvChargingSession::new(1, 19, 23, 5.0, 7.0, 0.5, 40.0));
        coord.coordinate();

        let profile = coord.total_demand_profile();
        let profile_total_kwh: f64 = profile.iter().sum::<f64>() * 1_000.0; // MW*h→kWh
        let sessions_total_kwh: f64 = coord.sessions.iter().map(|s| s.total_scheduled_kwh()).sum();
        assert!(
            (profile_total_kwh - sessions_total_kwh).abs() < 1e-6,
            "profile sum={profile_total_kwh}, sessions sum={sessions_total_kwh}"
        );
    }

    // ── V2gRevenueCalculator ──────────────────────────────────────────────────

    fn make_v2g() -> V2gRevenueCalculator {
        V2gRevenueCalculator {
            fleet_vehicles: 100,
            battery_kwh_per_vehicle: 60.0,
            usable_soc_range: (0.2, 0.8),
            charger_kw: 11.0,
            round_trip_efficiency: 0.9,
            availability_hours_per_day: 8.0,
            degradation_cost_per_kwh: 0.05,
        }
    }

    #[test]
    fn test_v2g_fleet_capacity_formula() {
        let v2g = make_v2g();
        let expected = 100.0 * 11.0 / 1_000.0;
        let got = v2g.fleet_v2g_capacity_mw();
        assert!(
            (got - expected).abs() < 1e-9,
            "got={got}, expected={expected}"
        );
    }

    #[test]
    fn test_v2g_net_annual_value_positive() {
        let v2g = make_v2g();
        // High FR price, good spread, many days → positive
        let val = v2g.net_annual_value(100.0, 80.0, 250.0);
        assert!(val > 0.0, "Net annual value should be positive, got {val}");
    }

    // ── EvDemandResponse ──────────────────────────────────────────────────────

    fn make_dr() -> EvDemandResponse {
        EvDemandResponse {
            enrolled_evs: 500,
            response_capacity_fraction: 0.8,
            min_soc_for_dr: 0.3,
            notification_delay_min: 15.0,
            max_curtailment_pct: 50.0,
            compensation_per_kwh: 0.10,
        }
    }

    #[test]
    fn test_dr_curtailment_lte_demand() {
        let dr = make_dr();
        let current_demand_mw = 5.0;
        let curtail = dr.available_curtailment_mw(current_demand_mw, 0.6);
        assert!(
            curtail <= current_demand_mw,
            "Curtailment {curtail} exceeds demand {current_demand_mw}"
        );
    }

    #[test]
    fn test_dr_shortfall_when_low_soc() {
        let dr = make_dr();
        // SoC below minimum → no response
        let result = dr.activate_event(2.0, 0.1, 1.0);
        assert!(
            result.shortfall_mw > 0.0,
            "Expected shortfall when SoC too low, got shortfall={}",
            result.shortfall_mw
        );
        assert_eq!(result.activated_mw, 0.0);
    }

    // ── ChargingNetworkOptimizer ──────────────────────────────────────────────

    fn make_optimizer() -> ChargingNetworkOptimizer {
        let locations = vec![
            ChargingLocation {
                id: 0,
                x: 0.0,
                y: 0.0,
                charger_type: NetworkChargerType::Level2_7kw,
                install_cost: 5_000.0,
                annual_opex: 500.0,
                charger_kw: 7.0,
                capacity: 30,
            },
            ChargingLocation {
                id: 1,
                x: 3.0,
                y: 0.0,
                charger_type: NetworkChargerType::Dcfc_50kw,
                install_cost: 30_000.0,
                annual_opex: 3_000.0,
                charger_kw: 50.0,
                capacity: 100,
            },
            ChargingLocation {
                id: 2,
                x: 1.0,
                y: 1.0,
                charger_type: NetworkChargerType::Level1_1kw,
                install_cost: 1_000.0,
                annual_opex: 100.0,
                charger_kw: 1.0,
                capacity: 10,
            },
        ];
        let demand_nodes = vec![(0.5, 0.5, 500.0), (2.5, 0.0, 300.0), (0.0, 2.0, 200.0)];
        ChargingNetworkOptimizer {
            candidate_locations: locations,
            budget: 10_000.0,
            ev_demand_nodes: demand_nodes,
        }
    }

    #[test]
    fn test_greedy_placement_within_budget() {
        let opt = make_optimizer();
        let selected = opt.greedy_placement();
        let total_cost: f64 = opt
            .candidate_locations
            .iter()
            .filter(|l| selected.contains(&l.id))
            .map(|l| l.install_cost)
            .sum();
        assert!(
            total_cost <= opt.budget + 1e-6,
            "Selected cost {total_cost} exceeds budget {}",
            opt.budget
        );
    }

    #[test]
    fn test_coverage_radius_positive_for_all_types() {
        let opt = make_optimizer();
        for ct in [
            NetworkChargerType::Level1_1kw,
            NetworkChargerType::Level2_7kw,
            NetworkChargerType::Dcfc_50kw,
            NetworkChargerType::Dcfc_150kw,
            NetworkChargerType::Hpc_350kw,
        ] {
            let r = opt.coverage_radius_km(&ct);
            assert!(r > 0.0, "Coverage radius for {ct:?} should be > 0, got {r}");
        }
    }

    // ── GridEvSynergy ─────────────────────────────────────────────────────────

    fn make_synergy() -> GridEvSynergy {
        // 24-hour profiles: renewable peaks midday, EV charges evening
        let renewable: Vec<f64> = (0..24)
            .map(|h| if (10..16).contains(&h) { 5.0 } else { 0.5 })
            .collect();
        let ev_load: Vec<f64> = (0..24)
            .map(|h| if (18..22).contains(&h) { 2.0 } else { 0.1 })
            .collect();
        let conv_load: Vec<f64> = (0..24)
            .map(|h| if (8..20).contains(&h) { 8.0 } else { 4.0 })
            .collect();
        GridEvSynergy {
            renewable_gen_mwh: renewable,
            ev_load_mwh: ev_load,
            conventional_load_mwh: conv_load,
        }
    }

    #[test]
    fn test_renewable_utilization_between_0_and_1() {
        let synergy = make_synergy();
        let util = synergy.renewable_utilization_pct();
        assert!(
            (0.0..=1.0).contains(&util),
            "Renewable utilization {util} not in [0,1]"
        );
    }

    #[test]
    fn test_valley_filling_score_non_negative() {
        // For a flat EV load (valley-filling scenario), score should be ≥ 0
        let conv_load = vec![4.0, 4.0, 4.0, 8.0, 8.0, 8.0, 4.0, 4.0];
        // Flat EV load → total std ≈ conv std → score ≈ 0 (not negative)
        let synergy = GridEvSynergy {
            renewable_gen_mwh: vec![1.0; 8],
            ev_load_mwh: vec![2.0; 8], // flat EV load
            conventional_load_mwh: conv_load,
        };
        let score = synergy.valley_filling_score();
        assert!(
            score >= 0.0,
            "Valley filling score should be ≥ 0, got {score}"
        );
    }

    #[test]
    fn test_valley_filling_score_positive_for_good_charging() {
        // EV charges when conv load is low → reduces std of total → score > 0
        let conv_load: Vec<f64> = vec![2.0, 2.0, 8.0, 8.0, 8.0, 2.0, 2.0, 2.0];
        // EV charges in off-peak hours (indices 0,1,5,6,7)
        let ev_load: Vec<f64> = vec![3.0, 3.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0];
        let synergy = GridEvSynergy {
            renewable_gen_mwh: vec![1.0; 8],
            ev_load_mwh: ev_load,
            conventional_load_mwh: conv_load,
        };
        let score = synergy.valley_filling_score();
        assert!(
            score > 0.0,
            "Valley filling score should be > 0 for good charging, got {score}"
        );
    }

    #[test]
    fn test_ev_renewable_match_pct_in_range() {
        let synergy = make_synergy();
        let pct = synergy.ev_renewable_match_pct();
        assert!(
            (0.0..=1.0).contains(&pct),
            "EV renewable match {pct} not in [0,1]"
        );
    }

    // ── 8 new tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_peak_demand_increase_mw_positive() {
        let impact = make_impact();
        let peak = impact.peak_demand_increase_mw();
        assert!(
            peak > 0.0,
            "peak_demand_increase_mw should be positive, got {peak}"
        );
    }

    #[test]
    fn test_voltage_impact_estimate_pct_proportional_to_impedance() {
        let impact = make_impact();
        let v1 = impact.voltage_impact_estimate_pct(0.01);
        let v2 = impact.voltage_impact_estimate_pct(0.02);
        // doubling impedance should double the voltage impact
        assert!(
            (v2 - 2.0 * v1).abs() < 1e-9,
            "voltage impact should scale linearly with impedance: v1={v1}, v2={v2}"
        );
    }

    #[test]
    fn test_charging_session_is_satisfied_after_full_schedule() {
        let mut session = EvChargingSession::new(0, 18, 23, 10.0, 7.0, 0.2, 60.0);
        // schedule 10 kW across 2 hours → 10 kWh total
        session.scheduled_power_kw[18] = 5.0;
        session.scheduled_power_kw[19] = 5.0;
        assert!(
            (session.total_scheduled_kwh() - 10.0).abs() < 1e-9,
            "total should be 10.0, got {}",
            session.total_scheduled_kwh()
        );
        assert!(session.is_satisfied(), "session should be satisfied");
    }

    #[test]
    fn test_cost_comparison_tou_lte_uncontrolled_cost() {
        let coord = make_coordinator(CoordinationMode::Uncontrolled);
        let (uncontrolled, tou, _carbon, _grid) = coord.cost_comparison();
        assert!(
            tou <= uncontrolled + 1e-6,
            "TOU-optimal cost ({tou:.4}) should not exceed uncontrolled cost ({uncontrolled:.4})"
        );
    }

    #[test]
    fn test_fleet_energy_available_kwh_scales_with_fleet_size() {
        let make_calc = |n: usize| V2gRevenueCalculator {
            fleet_vehicles: n,
            battery_kwh_per_vehicle: 60.0,
            usable_soc_range: (0.2, 0.8),
            charger_kw: 11.0,
            round_trip_efficiency: 0.9,
            availability_hours_per_day: 8.0,
            degradation_cost_per_kwh: 0.05,
        };
        let e100 = make_calc(100).fleet_energy_available_kwh();
        let e200 = make_calc(200).fleet_energy_available_kwh();
        assert!(
            (e200 - 2.0 * e100).abs() < 1e-6,
            "doubling fleet should double energy: e100={e100}, e200={e200}"
        );
    }

    #[test]
    fn test_arbitrage_revenue_per_day_positive_spread() {
        let calc = V2gRevenueCalculator {
            fleet_vehicles: 100,
            battery_kwh_per_vehicle: 60.0,
            usable_soc_range: (0.2, 0.8),
            charger_kw: 11.0,
            round_trip_efficiency: 0.9,
            availability_hours_per_day: 8.0,
            degradation_cost_per_kwh: 0.01, // low degradation
        };
        let revenue = calc.arbitrage_revenue_per_day(50.0, 200.0, 1.0);
        assert!(
            revenue > 0.0,
            "arbitrage revenue should be positive with a large spread, got {revenue}"
        );
    }

    #[test]
    fn test_available_curtailment_mw_zero_below_min_soc() {
        let dr = EvDemandResponse {
            enrolled_evs: 500,
            response_capacity_fraction: 0.8,
            min_soc_for_dr: 0.3,
            notification_delay_min: 5.0,
            max_curtailment_pct: 50.0,
            compensation_per_kwh: 0.10,
        };
        // avg_soc below threshold → zero curtailment
        let curtail = dr.available_curtailment_mw(10.0, 0.1);
        assert!(
            curtail.abs() < 1e-9,
            "curtailment should be zero below min SoC, got {curtail}"
        );
        // avg_soc above threshold → positive curtailment
        let curtail_ok = dr.available_curtailment_mw(10.0, 0.5);
        assert!(
            curtail_ok > 0.0,
            "curtailment should be positive above min SoC, got {curtail_ok}"
        );
    }

    #[test]
    fn test_population_coverage_pct_full_when_in_radius() {
        let optimizer = ChargingNetworkOptimizer {
            candidate_locations: vec![ChargingLocation {
                id: 1,
                x: 0.0,
                y: 0.0,
                charger_type: NetworkChargerType::Dcfc_150kw,
                install_cost: 10_000.0,
                annual_opex: 1_000.0,
                charger_kw: 150.0,
                capacity: 20,
            }],
            budget: 50_000.0,
            // All demand nodes within 5 km (Dcfc_150kw radius = 50 km)
            ev_demand_nodes: vec![(1.0, 1.0, 100.0), (2.0, 2.0, 80.0), (3.0, 3.0, 60.0)],
        };
        let coverage = optimizer.population_coverage_pct(&[1]);
        assert!(
            (coverage - 1.0).abs() < 1e-9,
            "all demand nodes should be covered, got {coverage}"
        );
    }
}
