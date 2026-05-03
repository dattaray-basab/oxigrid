/// DR + EMS Coupling: integrating Demand Response into the Energy Management System.
///
/// Extends the EMS with active demand-side management:
///   - Price-responsive load shifting and curtailment
///   - DR programme participation (commercial/industrial)
///   - EMS co-optimisation with DR resources
///   - DR performance measurement and verification (M&V)
///
/// # DR Stack in EMS
/// The EMS dispatch order with DR:
///   1. Renewables (solar, wind) — zero marginal cost
///   2. Price-responsive load reduction (virtual generation)
///   3. Battery discharge
///   4. Controllable loads shifted from off-peak
///   5. Dispatchable generator
///   6. Emergency curtailment
///   7. Load shedding
use serde::{Deserialize, Serialize};

// ─── DR resource ─────────────────────────────────────────────────────────────

/// Type of demand response resource.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrResourceType {
    /// Shed/curtail load voluntarily (e.g., HVAC setpoint raise)
    Curtailment,
    /// Shift load from peak to off-peak (washing machine, EV charging)
    TimeShift,
    /// Interruptible load with contractual curtailment obligation
    Interruptible,
    /// Direct load control by utility/aggregator
    DirectControl,
}

/// A demand response resource in the EMS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrResource {
    /// Resource identifier
    pub id: usize,
    pub name: String,
    pub resource_type: DrResourceType,
    /// Baseline load `kW`
    pub baseline_kw: f64,
    /// Maximum curtailable power `kW`
    pub max_curtail_kw: f64,
    /// Minimum curtail duration `h` per event
    pub min_event_h: f64,
    /// Maximum curtail duration `h` per event
    pub max_event_h: f64,
    /// Maximum events per day
    pub max_events_per_day: usize,
    /// Incentive payment received [$/kWh curtailed]
    pub incentive_per_kwh: f64,
    /// Discomfort cost (customer penalty) [$/kWh curtailed]
    pub discomfort_per_kwh: f64,
    /// Price trigger threshold [$/kWh]: curtail when price > threshold
    pub price_trigger: f64,
    /// Elasticity (% load change per % price change), negative value
    pub price_elasticity: f64,
}

impl DrResource {
    /// Commercial building HVAC (large curtailment, moderate discomfort).
    pub fn commercial_hvac(id: usize) -> Self {
        Self {
            id,
            name: format!("HVAC-{id}"),
            resource_type: DrResourceType::Curtailment,
            baseline_kw: 50.0,
            max_curtail_kw: 30.0,
            min_event_h: 0.5,
            max_event_h: 4.0,
            max_events_per_day: 3,
            incentive_per_kwh: 0.15,
            discomfort_per_kwh: 0.05,
            price_trigger: 0.20,
            price_elasticity: -0.3,
        }
    }

    /// Industrial interruptible load.
    pub fn industrial_interruptible(id: usize) -> Self {
        Self {
            id,
            name: format!("Industrial-{id}"),
            resource_type: DrResourceType::Interruptible,
            baseline_kw: 200.0,
            max_curtail_kw: 180.0,
            min_event_h: 1.0,
            max_event_h: 8.0,
            max_events_per_day: 2,
            incentive_per_kwh: 0.30,
            discomfort_per_kwh: 0.10,
            price_trigger: 0.30,
            price_elasticity: -0.5,
        }
    }

    /// EV charging (shiftable load).
    pub fn ev_charging(id: usize) -> Self {
        Self {
            id,
            name: format!("EV-{id}"),
            resource_type: DrResourceType::TimeShift,
            baseline_kw: 7.4,
            max_curtail_kw: 7.4,
            min_event_h: 0.25,
            max_event_h: 2.0,
            max_events_per_day: 4,
            incentive_per_kwh: 0.08,
            discomfort_per_kwh: 0.02,
            price_trigger: 0.15,
            price_elasticity: -0.8,
        }
    }

    /// Net benefit of curtailing this resource [$/kW·h].
    pub fn net_benefit_per_kwh(&self) -> f64 {
        self.incentive_per_kwh - self.discomfort_per_kwh
    }

    /// Price-responsive load reduction given current price `kW`.
    pub fn price_response_kw(&self, price_kwh: f64) -> f64 {
        if price_kwh <= self.price_trigger {
            return 0.0;
        }
        let price_ratio = (price_kwh - self.price_trigger) / self.price_trigger.max(1e-9);
        let load_reduction_frac = (-self.price_elasticity * price_ratio).min(1.0);
        (self.baseline_kw * load_reduction_frac).min(self.max_curtail_kw)
    }
}

// ─── EMS + DR dispatch ────────────────────────────────────────────────────────

/// DR + EMS combined dispatch result for one period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrEmsPeriod {
    /// Period index
    pub period: usize,
    /// Renewable generation `kW`
    pub renewable_kw: f64,
    /// DR curtailment per resource `kW`
    pub dr_curtail_kw: Vec<f64>,
    /// Total DR reduction `kW`
    pub total_dr_kw: f64,
    /// Battery discharge `kW` (net)
    pub battery_net_kw: f64,
    /// Generator output `kW`
    pub gen_kw: f64,
    /// Residual load `kW`
    pub residual_load_kw: f64,
    /// Load shedding `kW`
    pub load_shed_kw: f64,
    /// Electricity price [$/kWh]
    pub price_kwh: f64,
    /// DR incentive earned [$]
    pub dr_incentive: f64,
    /// Generator cost [$]
    pub gen_cost: f64,
    /// Battery SOC at end of period
    pub soc_end: f64,
}

/// Configuration for the DR-EMS optimiser.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrEmsConfig {
    /// Time step `h`
    pub dt_h: f64,
    /// Electricity prices [$/kWh] per period
    pub prices: Vec<f64>,
    /// Baseline load `kW` per period
    pub load_kw: Vec<f64>,
    /// Renewable forecast `kW` per period (PV + wind combined)
    pub renewable_kw: Vec<f64>,
    /// Battery capacity `kWh`
    pub battery_kwh: f64,
    /// Battery max power `kW`
    pub battery_p_max_kw: f64,
    /// Battery round-trip efficiency
    pub battery_eta: f64,
    /// Battery initial SOC
    pub soc_init: f64,
    /// Generator max power `kW`
    pub gen_p_max_kw: f64,
    /// Generator cost [$/kWh]
    pub gen_cost_kwh: f64,
    /// Generator min load `kW`
    pub gen_p_min_kw: f64,
}

/// Full DR + EMS dispatch result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrEmsResult {
    pub periods: Vec<DrEmsPeriod>,
    pub total_gen_cost: f64,
    pub total_dr_incentive: f64,
    pub total_load_shed_kwh: f64,
    pub total_dr_reduction_kwh: f64,
    pub renewable_utilisation_pct: f64,
    pub peak_load_kw: f64,
    pub peak_after_dr_kw: f64,
    pub peak_reduction_pct: f64,
}

/// Run the DR + EMS co-optimiser.
///
/// Dispatches DR resources before generator, reducing peak demand
/// and total operating cost.
pub fn run_dr_ems(dr_resources: &[DrResource], config: &DrEmsConfig) -> DrEmsResult {
    let n_t = config.prices.len();
    let n_dr = dr_resources.len();
    let dt = config.dt_h;

    let mut soc = config.soc_init;
    let mut periods = Vec::with_capacity(n_t);
    let mut total_gen_cost = 0.0;
    let mut total_dr_incentive = 0.0;
    let mut total_shed = 0.0;
    let mut total_dr_kwh = 0.0;
    let mut total_renewable = 0.0;
    let mut total_renewable_avail = 0.0;
    let peak_load_kw = config.load_kw.iter().cloned().fold(0.0f64, f64::max);

    // Track DR events per resource
    let mut dr_events = vec![0usize; n_dr];
    let mut dr_event_h = vec![0.0f64; n_dr];

    for t in 0..n_t {
        let price = config.prices[t];
        let load = config.load_kw[t];
        let renew = config.renewable_kw.get(t).copied().unwrap_or(0.0);
        total_renewable_avail += renew;

        // ── Step 1: DR curtailment (sorted by net benefit) ──
        let mut dr_order: Vec<usize> = (0..n_dr).collect();
        dr_order.sort_by(|&a, &b| {
            dr_resources[b]
                .net_benefit_per_kwh()
                .partial_cmp(&dr_resources[a].net_benefit_per_kwh())
                .unwrap()
        });

        let mut dr_curtail = vec![0.0f64; n_dr];
        let mut total_dr_kw = 0.0;
        let mut remaining_load = load;

        // Price-responsive DR
        for &gi in &dr_order {
            let res = &dr_resources[gi];
            if price > res.price_trigger
                && dr_events[gi] < res.max_events_per_day
                && dr_event_h[gi] < res.max_event_h
            {
                let curtail = res.price_response_kw(price).min(remaining_load);
                dr_curtail[gi] = curtail;
                total_dr_kw += curtail;
                remaining_load = (remaining_load - curtail).max(0.0);
                if curtail > 0.1 {
                    dr_events[gi] += 1;
                    dr_event_h[gi] += dt;
                }
            }
        }

        // ── Step 2: Serve residual load from renewables ──
        let after_renew = (remaining_load - renew).max(0.0);
        let renew_used = remaining_load.min(renew);
        total_renewable += renew_used;
        remaining_load = after_renew;

        // ── Step 3: Battery ──
        let (batt_net_kw, new_soc) = battery_dispatch(
            soc,
            remaining_load,
            renew - renew_used,
            config.battery_kwh,
            config.battery_p_max_kw,
            config.battery_eta,
            dt,
        );
        remaining_load = (remaining_load - batt_net_kw.max(0.0)).max(0.0);
        soc = new_soc.clamp(0.05, 0.95);

        // ── Step 4: Generator ──
        let gen_kw = if remaining_load > config.gen_p_min_kw {
            remaining_load.min(config.gen_p_max_kw)
        } else if remaining_load > 0.0 {
            config.gen_p_min_kw.min(config.gen_p_max_kw)
        } else {
            0.0
        };
        let gen_cost = gen_kw * config.gen_cost_kwh * dt;
        remaining_load = (remaining_load - gen_kw).max(0.0);

        // ── Step 5: Load shedding ──
        let shed = remaining_load;
        total_shed += shed * dt;

        // DR incentive
        let dr_incentive = dr_curtail
            .iter()
            .zip(dr_resources.iter())
            .map(|(&c, res)| c * res.incentive_per_kwh * dt)
            .sum::<f64>();

        total_gen_cost += gen_cost;
        total_dr_incentive += dr_incentive;
        total_dr_kwh += total_dr_kw * dt;

        periods.push(DrEmsPeriod {
            period: t,
            renewable_kw: renew_used,
            dr_curtail_kw: dr_curtail,
            total_dr_kw,
            battery_net_kw: batt_net_kw,
            gen_kw,
            residual_load_kw: load - total_dr_kw,
            load_shed_kw: shed,
            price_kwh: price,
            dr_incentive,
            gen_cost,
            soc_end: soc,
        });
    }

    let peak_after_dr = periods
        .iter()
        .map(|p| p.residual_load_kw)
        .fold(0.0f64, f64::max);
    let peak_reduction = if peak_load_kw > 0.0 {
        (peak_load_kw - peak_after_dr) / peak_load_kw * 100.0
    } else {
        0.0
    };

    let renew_util = if total_renewable_avail > 0.0 {
        total_renewable / total_renewable_avail * 100.0
    } else {
        100.0
    };

    DrEmsResult {
        periods,
        total_gen_cost,
        total_dr_incentive,
        total_load_shed_kwh: total_shed,
        total_dr_reduction_kwh: total_dr_kwh,
        renewable_utilisation_pct: renew_util,
        peak_load_kw,
        peak_after_dr_kw: peak_after_dr,
        peak_reduction_pct: peak_reduction,
    }
}

/// Simple battery dispatch: discharge for load, charge from surplus renewable.
fn battery_dispatch(
    soc: f64,
    load_kw: f64,
    surplus_kw: f64,
    cap_kwh: f64,
    p_max_kw: f64,
    eta: f64,
    dt_h: f64,
) -> (f64, f64) {
    if load_kw > 0.0 {
        // Discharge
        let energy_avail = (soc - 0.10) * cap_kwh * eta;
        let p_discharge = load_kw.min(p_max_kw).min(energy_avail / dt_h);
        let p_discharge = p_discharge.max(0.0);
        let new_soc = soc - p_discharge * dt_h / (cap_kwh * eta).max(1e-9);
        (p_discharge, new_soc)
    } else if surplus_kw > 0.0 {
        // Charge
        let energy_head = (0.95 - soc) * cap_kwh;
        let p_charge = surplus_kw.min(p_max_kw).min(energy_head / dt_h);
        let p_charge = p_charge.max(0.0);
        let new_soc = soc + p_charge * dt_h * eta / cap_kwh.max(1e-9);
        (-p_charge, new_soc)
    } else {
        (0.0, soc)
    }
}

// ─── DR M&V (Measurement & Verification) ────────────────────────────────────

/// DR event verification: compares actual consumption vs. adjusted baseline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrMvResult {
    /// Verified load reduction `kWh`
    pub verified_reduction_kwh: f64,
    /// Adjusted baseline `kWh`
    pub baseline_kwh: f64,
    /// Actual consumption during event `kWh`
    pub actual_kwh: f64,
    /// Performance factor (actual / committed)
    pub performance_factor: f64,
    /// Incentive earned [$]
    pub incentive_earned: f64,
    /// Pass/fail
    pub passes: bool,
}

/// Verify a DR event using the IPMVP Option A (metered baseline) approach.
///
/// # Arguments
/// - `baseline` — pre-event average load `kW` per period
/// - `actual`   — measured consumption during event `kW` per period
/// - `committed_reduction_kw` — contracted reduction `kW`
/// - `incentive_per_kwh` — payment rate [$/kWh]
/// - `dt_h` — period duration `h`
pub fn verify_dr_event(
    baseline: &[f64],
    actual: &[f64],
    committed_reduction_kw: f64,
    incentive_per_kwh: f64,
    dt_h: f64,
) -> DrMvResult {
    let baseline_kwh: f64 = baseline.iter().sum::<f64>() * dt_h;
    let actual_kwh: f64 = actual.iter().sum::<f64>() * dt_h;
    let verified_kwh = (baseline_kwh - actual_kwh).max(0.0);
    let committed_kwh = committed_reduction_kw * baseline.len() as f64 * dt_h;
    let pf = if committed_kwh > 0.0 {
        verified_kwh / committed_kwh
    } else {
        0.0
    };
    let incentive = verified_kwh * incentive_per_kwh;

    DrMvResult {
        verified_reduction_kwh: verified_kwh,
        baseline_kwh,
        actual_kwh,
        performance_factor: pf,
        incentive_earned: incentive,
        passes: pf >= 0.80,
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_config() -> DrEmsConfig {
        DrEmsConfig {
            dt_h: 1.0,
            prices: vec![0.10, 0.12, 0.35, 0.40, 0.15, 0.08],
            load_kw: vec![80.0, 90.0, 120.0, 130.0, 100.0, 70.0],
            renewable_kw: vec![20.0, 30.0, 50.0, 60.0, 40.0, 10.0],
            battery_kwh: 100.0,
            battery_p_max_kw: 30.0,
            battery_eta: 0.95,
            soc_init: 0.5,
            gen_p_max_kw: 150.0,
            gen_cost_kwh: 0.30,
            gen_p_min_kw: 20.0,
        }
    }

    fn sample_dr() -> Vec<DrResource> {
        vec![
            DrResource::commercial_hvac(0),
            DrResource::industrial_interruptible(1),
            DrResource::ev_charging(2),
        ]
    }

    #[test]
    fn test_dr_ems_runs() {
        let result = run_dr_ems(&sample_dr(), &sample_config());
        assert_eq!(result.periods.len(), 6);
    }

    #[test]
    fn test_gen_cost_positive() {
        let result = run_dr_ems(&sample_dr(), &sample_config());
        assert!(result.total_gen_cost >= 0.0);
    }

    #[test]
    fn test_no_negative_shedding() {
        let result = run_dr_ems(&sample_dr(), &sample_config());
        for p in &result.periods {
            assert!(
                p.load_shed_kw >= -1e-9,
                "Load shed negative: {}",
                p.load_shed_kw
            );
        }
    }

    #[test]
    fn test_dr_reduces_peak() {
        let result = run_dr_ems(&sample_dr(), &sample_config());
        // Peak after DR should be ≤ original peak
        assert!(result.peak_after_dr_kw <= result.peak_load_kw + 1e-9);
    }

    #[test]
    fn test_dr_incentive_non_negative() {
        let result = run_dr_ems(&sample_dr(), &sample_config());
        assert!(result.total_dr_incentive >= 0.0);
    }

    #[test]
    fn test_price_response_zero_below_trigger() {
        let res = DrResource::commercial_hvac(0);
        assert_eq!(res.price_response_kw(0.05), 0.0);
    }

    #[test]
    fn test_price_response_positive_above_trigger() {
        let res = DrResource::commercial_hvac(0);
        let response = res.price_response_kw(0.50);
        assert!(
            response > 0.0,
            "Expected DR response at high price: {}",
            response
        );
    }

    #[test]
    fn test_mv_perfect_compliance() {
        let baseline = vec![100.0, 100.0];
        let actual = vec![70.0, 70.0];
        let mv = verify_dr_event(&baseline, &actual, 30.0, 0.20, 1.0);
        assert!((mv.verified_reduction_kwh - 60.0).abs() < 1e-6);
        assert!(mv.passes, "Should pass: PF = {:.2}", mv.performance_factor);
    }

    #[test]
    fn test_mv_poor_compliance_fails() {
        let baseline = vec![100.0, 100.0];
        let actual = vec![95.0, 95.0]; // only 5% reduction vs 30% committed
        let mv = verify_dr_event(&baseline, &actual, 30.0, 0.20, 1.0);
        assert!(!mv.passes, "Should fail: PF = {:.2}", mv.performance_factor);
    }

    #[test]
    fn test_dr_curtail_vector_length() {
        let drs = sample_dr();
        let result = run_dr_ems(&drs, &sample_config());
        for p in &result.periods {
            assert_eq!(p.dr_curtail_kw.len(), drs.len());
        }
    }

    #[test]
    fn test_empty_dr_still_runs() {
        let result = run_dr_ems(&[], &sample_config());
        assert_eq!(result.periods.len(), 6);
        assert_eq!(result.total_dr_reduction_kwh, 0.0);
    }
}
