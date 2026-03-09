#![cfg(feature = "optimize")]

use oxigrid::optimize::microgrid::ems::{DieselGen, EmsBattery, EmsDispatcher};

fn make_ems() -> EmsDispatcher {
    EmsDispatcher::new(EmsBattery::lifepo4_100kwh(), DieselGen::diesel_100kw())
}

/// Build a realistic 24-hour load, PV, and wind profile (hourly).
fn build_24h_profiles() -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let load: Vec<f64> = (0..24)
        .map(|h| {
            // Residential-style load: low at night, peak in morning/evening
            let base = 40.0_f64;
            let morning_peak = if (7..=9).contains(&h) { 25.0 } else { 0.0 };
            let evening_peak = if (18..=21).contains(&h) { 30.0 } else { 0.0 };
            let night_low = if (0..6).contains(&h) { -10.0 } else { 0.0 };
            (base + morning_peak + evening_peak + night_low).max(10.0)
        })
        .collect();

    let pv: Vec<f64> = (0..24)
        .map(|h| {
            // Solar: bell curve between 6am and 8pm
            if !(6..=20).contains(&h) {
                0.0
            } else {
                let t = (h as f64 - 13.0) / 7.0; // normalised offset from solar noon
                80.0 * (-3.0 * t * t).exp()
            }
        })
        .collect();

    let wind: Vec<f64> = (0..24)
        .map(|h| {
            // Wind: higher at night, lower during day (anti-correlated with PV)
            let base = 20.0_f64;
            let night_bonus = if !(6..=20).contains(&h) { 15.0 } else { 0.0 };
            base + night_bonus
        })
        .collect();

    (load, pv, wind)
}

#[test]
fn test_ems_24h_plan_structure() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    assert_eq!(plan.intervals.len(), 24, "Should have 24 hourly intervals");

    for (h, iv) in plan.intervals.iter().enumerate() {
        assert!(iv.load_kw >= 0.0, "Hour {h}: load_kw must be non-negative");
        assert!(iv.pv_kw >= 0.0, "Hour {h}: pv_kw must be non-negative");
        assert!(iv.wind_kw >= 0.0, "Hour {h}: wind_kw must be non-negative");
        assert!(
            iv.diesel_kw >= 0.0,
            "Hour {h}: diesel_kw must be non-negative"
        );
        assert!(
            iv.load_shed_kw >= 0.0,
            "Hour {h}: load_shed_kw must be non-negative"
        );
        assert!(
            iv.renewable_curtail_kw >= 0.0,
            "Hour {h}: curtailment must be non-negative"
        );
        assert!(
            iv.battery_soc >= 0.0 && iv.battery_soc <= 1.0,
            "Hour {h}: SoC {:.3} out of [0,1]",
            iv.battery_soc
        );
    }
}

#[test]
fn test_ems_24h_no_load_shed_with_diesel() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    // With full diesel backup available, no load should be shed
    assert_eq!(
        plan.total_load_shed_kwh, 0.0,
        "No load shedding expected when diesel is available"
    );
}

#[test]
fn test_ems_24h_positive_cost() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    assert!(
        plan.total_cost_usd >= 0.0,
        "Total cost must be non-negative, got {}",
        plan.total_cost_usd
    );
}

#[test]
fn test_ems_24h_renewable_fraction_valid() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    assert!(
        plan.renewable_fraction >= 0.0 && plan.renewable_fraction <= 1.0,
        "Renewable fraction {} out of [0,1]",
        plan.renewable_fraction
    );
}

#[test]
fn test_ems_24h_high_renewable_reduces_diesel() {
    // High renewable: PV 150 kW all day (massive oversupply)
    let mut ems_high = make_ems();
    let load = vec![50.0; 24];
    let pv_high = vec![150.0; 24];
    let wind = vec![0.0; 24];
    let plan_high = ems_high.dispatch(&load, &pv_high, &wind, 1.0);

    // Zero renewable: loads must come from diesel
    let mut ems_zero = make_ems();
    let pv_zero = vec![0.0; 24];
    let plan_zero = ems_zero.dispatch(&load, &pv_zero, &wind, 1.0);

    assert!(
        plan_high.total_diesel_kwh < plan_zero.total_diesel_kwh,
        "High renewable ({:.0} kWh diesel) should use less diesel than no renewable ({:.0} kWh)",
        plan_high.total_diesel_kwh,
        plan_zero.total_diesel_kwh
    );
    assert!(
        plan_high.renewable_fraction > plan_zero.renewable_fraction,
        "High renewable fraction ({:.2}) should exceed zero-renewable fraction ({:.2})",
        plan_high.renewable_fraction,
        plan_zero.renewable_fraction
    );
}

#[test]
fn test_ems_24h_battery_soc_continuity() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    // SoC should transition smoothly (no instantaneous jumps > 50%)
    let socs: Vec<f64> = plan.intervals.iter().map(|iv| iv.battery_soc).collect();
    for w in socs.windows(2) {
        let delta = (w[1] - w[0]).abs();
        assert!(
            delta <= 0.5,
            "SoC jump {:.3} between consecutive hours exceeds 0.5",
            delta
        );
    }
}

#[test]
fn test_ems_24h_diesel_within_limits() {
    let mut ems = make_ems();
    let load = vec![80.0; 24]; // moderate load, needs some diesel
    let pv = vec![0.0; 24];
    let wind = vec![0.0; 24];
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    for (h, iv) in plan.intervals.iter().enumerate() {
        assert!(
            iv.diesel_kw <= 100.0 + 1e-9,
            "Hour {h}: diesel_kw {:.1} exceeds p_max 100 kW",
            iv.diesel_kw
        );
    }
}

#[test]
fn test_ems_24h_energy_balance_per_interval() {
    let mut ems = make_ems();
    let (load, pv, wind) = build_24h_profiles();
    let plan = ems.dispatch(&load, &pv, &wind, 1.0);

    for (h, iv) in plan.intervals.iter().enumerate() {
        // Nodal power balance (lossless grid):
        //   pv + wind - curtail - battery_kw + diesel = load - shed
        // battery_kw > 0 means charging (consuming power), < 0 means discharging (producing power)
        let lhs = iv.pv_kw + iv.wind_kw - iv.renewable_curtail_kw - iv.battery_kw + iv.diesel_kw;
        let rhs = iv.load_kw - iv.load_shed_kw;

        assert!(
            (lhs - rhs).abs() < 1.0,
            "Hour {h}: supply {lhs:.2} kW ≠ demand {rhs:.2} kW (diff {:.3})",
            (lhs - rhs).abs()
        );
    }
}
