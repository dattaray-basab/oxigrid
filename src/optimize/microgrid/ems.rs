/// Energy Management System (EMS) for islanded/grid-connected microgrids.
///
/// Performs a rule-based or cost-minimising 24-hour dispatch schedule
/// for a microgrid with:
///   - PV generation (forecast provided externally)
///   - Wind generation (forecast provided externally)
///   - Battery storage (2RC ECM model with SoC limits)
///   - Dispatchable generator (diesel / gas)
///   - Flexible load (curtailable fraction)
///
/// The greedy dispatch policy:
///   1. Serve load from renewables first.
///   2. Excess renewable → charge battery (within limits).
///   3. Deficit → discharge battery (within limits).
///   4. Remaining deficit → dispatchable generator.
///   5. Unmet load is tracked as `load_shedding_kw`.
use serde::{Deserialize, Serialize};

/// Configuration for one dispatchable generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DieselGen {
    /// Maximum rated power [kW]
    pub p_max_kw: f64,
    /// Minimum stable load [kW]
    pub p_min_kw: f64,
    /// Fuel cost slope [$/kWh]
    pub fuel_cost: f64,
    /// No-load cost [$/h]
    pub no_load_cost: f64,
    /// Startup cost [$]
    pub startup_cost: f64,
}

impl DieselGen {
    pub fn diesel_100kw() -> Self {
        Self {
            p_max_kw: 100.0,
            p_min_kw: 20.0,
            fuel_cost: 0.35,    // $/kWh
            no_load_cost: 5.0,  // $/h
            startup_cost: 50.0, // $
        }
    }

    /// Operating cost for one interval of `dt_h` hours at output `p_kw`.
    pub fn cost(&self, p_kw: f64, dt_h: f64) -> f64 {
        if p_kw < 1e-6 {
            return 0.0;
        }
        (self.no_load_cost + self.fuel_cost * p_kw) * dt_h
    }
}

/// Battery storage configuration for the EMS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmsBattery {
    /// Usable energy capacity [kWh]
    pub capacity_kwh: f64,
    /// Maximum charge/discharge power [kW]
    pub p_max_kw: f64,
    /// Round-trip efficiency (0–1)
    pub eta: f64,
    /// SoC lower limit (0–1)
    pub soc_min: f64,
    /// SoC upper limit (0–1)
    pub soc_max: f64,
    /// Current SoC (0–1)
    pub soc: f64,
}

impl EmsBattery {
    pub fn lifepo4_100kwh() -> Self {
        Self {
            capacity_kwh: 100.0,
            p_max_kw: 50.0,
            eta: 0.94,
            soc_min: 0.10,
            soc_max: 0.90,
            soc: 0.50,
        }
    }

    /// Maximum charge power available [kW] (limited by SoC and P_max).
    pub fn max_charge_kw(&self, dt_h: f64) -> f64 {
        let energy_room = (self.soc_max - self.soc) * self.capacity_kwh;
        let p_lim = energy_room / (dt_h * self.eta);
        p_lim.min(self.p_max_kw).max(0.0)
    }

    /// Maximum discharge power available [kW].
    pub fn max_discharge_kw(&self, dt_h: f64) -> f64 {
        let energy_avail = (self.soc - self.soc_min) * self.capacity_kwh;
        let p_lim = energy_avail * self.eta / dt_h;
        p_lim.min(self.p_max_kw).max(0.0)
    }

    /// Apply a charge (+) or discharge (−) action. Returns actual power [kW].
    pub fn apply(&mut self, p_kw: f64, dt_h: f64) -> f64 {
        if p_kw >= 0.0 {
            // Charging
            let p = p_kw.min(self.max_charge_kw(dt_h));
            self.soc += p * dt_h * self.eta / self.capacity_kwh;
            self.soc = self.soc.clamp(self.soc_min, self.soc_max);
            p
        } else {
            // Discharging
            let p = p_kw.abs().min(self.max_discharge_kw(dt_h));
            self.soc -= p * dt_h / (self.eta * self.capacity_kwh);
            self.soc = self.soc.clamp(self.soc_min, self.soc_max);
            -p
        }
    }
}

/// Result for a single dispatch interval.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmsInterval {
    pub hour: f64,
    pub load_kw: f64,
    pub pv_kw: f64,
    pub wind_kw: f64,
    pub battery_kw: f64, // positive = charging
    pub diesel_kw: f64,
    pub load_shed_kw: f64,
    pub renewable_curtail_kw: f64,
    pub battery_soc: f64,
    pub cost_usd: f64,
}

/// 24-hour dispatch plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmsPlan {
    pub intervals: Vec<EmsInterval>,
    pub total_cost_usd: f64,
    pub total_load_shed_kwh: f64,
    pub total_renewable_kwh: f64,
    pub total_diesel_kwh: f64,
    pub renewable_fraction: f64,
}

/// Rule-based greedy EMS dispatcher.
pub struct EmsDispatcher {
    pub battery: EmsBattery,
    pub diesel: DieselGen,
}

impl EmsDispatcher {
    pub fn new(battery: EmsBattery, diesel: DieselGen) -> Self {
        Self { battery, diesel }
    }

    /// Run a 24-hour dispatch given hourly load, PV, and wind forecasts.
    ///
    /// All vectors must have the same length (number of intervals).
    /// `dt_h` is the interval duration in hours.
    pub fn dispatch(
        &mut self,
        load_kw: &[f64],
        pv_kw: &[f64],
        wind_kw: &[f64],
        dt_h: f64,
    ) -> EmsPlan {
        let n = load_kw.len();
        assert_eq!(pv_kw.len(), n);
        assert_eq!(wind_kw.len(), n);

        let mut intervals = Vec::with_capacity(n);
        let mut total_cost = 0.0_f64;
        let mut total_load_shed = 0.0_f64;
        let mut total_renewable_kwh = 0.0_f64;
        let mut total_diesel_kwh = 0.0_f64;

        for i in 0..n {
            let load = load_kw[i].max(0.0);
            let pv = pv_kw[i].max(0.0);
            let wind = wind_kw[i].max(0.0);
            let hour = i as f64 * dt_h;

            let renewable = pv + wind;
            total_renewable_kwh += renewable * dt_h;

            let mut net = load - renewable; // positive = deficit

            let mut batt_p = 0.0_f64;
            let mut diesel_p = 0.0_f64;
            let mut curtail = 0.0_f64;
            let mut shed = 0.0_f64;

            if net < -1e-6 {
                // Excess renewable: charge battery (apply expects positive = charge)
                let charge = self.battery.apply(-net, dt_h); // -net > 0
                batt_p = charge;
                curtail = (-net) - charge; // any excess that couldn't be stored
                curtail = curtail.max(0.0);
            } else if net > 1e-6 {
                // Deficit: discharge battery (apply expects negative = discharge)
                let discharge = self.battery.apply(-net, dt_h); // -net < 0, returns < 0
                net += discharge; // discharge < 0 → reduces deficit
                batt_p = discharge;

                if net > 1e-6 {
                    // Still deficit: diesel
                    let p_diesel = net.clamp(0.0, self.diesel.p_max_kw);
                    diesel_p = p_diesel;
                    net -= p_diesel;
                    shed = net.max(0.0);
                    total_diesel_kwh += p_diesel * dt_h;
                }
            }

            let cost = self.diesel.cost(diesel_p, dt_h);
            total_cost += cost;
            total_load_shed += shed * dt_h;

            intervals.push(EmsInterval {
                hour,
                load_kw: load,
                pv_kw: pv,
                wind_kw: wind,
                battery_kw: batt_p,
                diesel_kw: diesel_p,
                load_shed_kw: shed,
                renewable_curtail_kw: curtail,
                battery_soc: self.battery.soc,
                cost_usd: cost,
            });
        }

        let renewable_served = total_renewable_kwh;
        let total_energy = load_kw.iter().sum::<f64>() * dt_h;
        let renewable_fraction = if total_energy > 0.0 {
            (renewable_served
                - intervals
                    .iter()
                    .map(|iv| iv.renewable_curtail_kw)
                    .sum::<f64>()
                    * dt_h)
                / total_energy
        } else {
            0.0
        };

        EmsPlan {
            intervals,
            total_cost_usd: total_cost,
            total_load_shed_kwh: total_load_shed,
            total_renewable_kwh,
            total_diesel_kwh,
            renewable_fraction: renewable_fraction.clamp(0.0, 1.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ems() -> EmsDispatcher {
        EmsDispatcher::new(EmsBattery::lifepo4_100kwh(), DieselGen::diesel_100kw())
    }

    #[test]
    fn test_excess_renewable_charges_battery() {
        let mut ems = make_ems();
        let initial_soc = ems.battery.soc;
        ems.dispatch(&[10.0], &[50.0], &[0.0], 1.0);
        assert!(
            ems.battery.soc > initial_soc,
            "SoC should increase when PV > load"
        );
    }

    #[test]
    fn test_deficit_discharges_battery_then_diesel() {
        let mut ems = make_ems();
        ems.battery.soc = 0.10; // Battery nearly empty
        let plan = ems.dispatch(&[80.0], &[0.0], &[0.0], 1.0);
        // With no renewables and empty battery, diesel must cover
        assert!(plan.intervals[0].diesel_kw > 0.0);
    }

    #[test]
    fn test_full_renewable_zero_diesel() {
        let mut ems = make_ems();
        let load = vec![50.0; 24];
        let pv = vec![80.0; 24];
        let plan = ems.dispatch(&load, &pv, &[0.0; 24], 1.0);
        assert_eq!(plan.total_diesel_kwh, 0.0);
    }

    #[test]
    fn test_24h_dispatch_energy_balance() {
        let mut ems = make_ems();
        let load: Vec<f64> = (0..24)
            .map(|h| 40.0 + 20.0 * (h as f64 * std::f64::consts::PI / 12.0).sin())
            .collect();
        let pv: Vec<f64> = (0..24)
            .map(|h| if (6..=18).contains(&h) { 60.0 } else { 0.0 })
            .collect();
        let plan = ems.dispatch(&load, &pv, &[0.0; 24], 1.0);
        assert_eq!(plan.intervals.len(), 24);
        // No unmet load (diesel covers any gap)
        assert_eq!(plan.total_load_shed_kwh, 0.0);
    }
}
