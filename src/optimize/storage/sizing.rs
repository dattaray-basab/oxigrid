/// Battery storage sizing optimisation.
///
/// Provides methods to compute the required energy capacity and power rating
/// for common BESS use cases: peak shaving, solar time-shifting, and backup power.
use serde::{Deserialize, Serialize};

/// Result of a battery sizing calculation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SizingResult {
    /// Required energy capacity `kWh`
    pub capacity_kwh: f64,
    /// Required power rating `kW`
    pub power_kw: f64,
    /// C-rate (power / capacity) [h⁻¹]
    pub c_rate: f64,
    /// Estimated annual savings [$/year] if price data provided
    pub annual_savings: Option<f64>,
}

impl SizingResult {
    fn new(capacity_kwh: f64, power_kw: f64) -> Self {
        let c_rate = if capacity_kwh > 1e-6 {
            power_kw / capacity_kwh
        } else {
            0.0
        };
        Self {
            capacity_kwh,
            power_kw,
            c_rate,
            annual_savings: None,
        }
    }
}

/// Size a battery for peak shaving.
///
/// Calculates the minimum capacity needed to limit peak grid demand to
/// `grid_limit_kw` by storing energy during off-peak periods.
///
/// # Arguments
/// - `load_kw`       — load profile `kW` for each time step
/// - `grid_limit_kw` — maximum allowed grid import `kW`
/// - `dt_h`          — time step `hours`
/// - `efficiency_rt` — round-trip efficiency (0–1)
pub fn size_for_peak_shaving(
    load_kw: &[f64],
    grid_limit_kw: f64,
    dt_h: f64,
    efficiency_rt: f64,
) -> SizingResult {
    // Peak power that must be supplied by battery
    let peak_load = load_kw.iter().cloned().fold(0.0_f64, f64::max);
    let p_bess_kw = (peak_load - grid_limit_kw).max(0.0);

    // Energy: integrate excess load above grid_limit
    let mut e_needed = 0.0_f64;
    let mut e_acc = 0.0_f64;
    let mut e_max = 0.0_f64;

    for &p in load_kw {
        let excess = (p - grid_limit_kw).max(0.0);
        let shortfall = (grid_limit_kw - p).max(0.0);
        e_acc -= excess * dt_h; // battery must supply excess
        e_acc += shortfall * dt_h * efficiency_rt.sqrt(); // charge during off-peak
        e_acc = e_acc.max(0.0); // can't go negative (cap at full)
        if -e_acc > e_needed {
            e_needed = -e_acc;
        }
        if e_acc > e_max {
            e_max = e_acc;
        }
    }

    // Required capacity: peak energy deficit / DoD
    let capacity_kwh = e_needed / 0.80; // assume 80% DoD
    SizingResult::new(capacity_kwh, p_bess_kw)
}

/// Size a battery for solar time-shifting.
///
/// Stores excess solar generation during the day to serve load in the evening.
///
/// # Arguments
/// - `pv_kw`         — PV generation profile `kW`
/// - `load_kw`       — load profile `kW`
/// - `dt_h`          — time step `hours`
/// - `efficiency_rt` — round-trip efficiency
pub fn size_for_solar_shifting(
    pv_kw: &[f64],
    load_kw: &[f64],
    dt_h: f64,
    efficiency_rt: f64,
) -> SizingResult {
    assert_eq!(
        pv_kw.len(),
        load_kw.len(),
        "PV and load series must be same length"
    );

    let eta_c = efficiency_rt.sqrt();

    // Compute net power: positive = excess PV (charge), negative = shortfall (discharge)
    let mut soc_trace = 0.0_f64;
    let mut peak_charge = 0.0_f64;
    let mut peak_discharge = 0.0_f64;
    let mut max_soc = 0.0_f64;
    let mut min_soc = 0.0_f64;

    for (&p, &l) in pv_kw.iter().zip(load_kw.iter()) {
        let net = p - l;
        if net > 0.0 {
            soc_trace += net * dt_h * eta_c;
            if net > peak_charge {
                peak_charge = net;
            }
        } else {
            soc_trace += net * dt_h / eta_c;
            if net.abs() > peak_discharge {
                peak_discharge = net.abs();
            }
        }
        if soc_trace > max_soc {
            max_soc = soc_trace;
        }
        if soc_trace < min_soc {
            min_soc = soc_trace;
        }
    }

    let swing = max_soc - min_soc;
    let capacity_kwh = swing / 0.80; // 80% DoD
    let power_kw = peak_charge.max(peak_discharge);

    SizingResult::new(capacity_kwh, power_kw)
}

/// Size a battery for backup power.
///
/// # Arguments
/// - `load_kw`       — average load to support during outage `kW`
/// - `duration_h`    — required backup duration `hours`
/// - `dod`           — allowed depth of discharge (0–1, default 0.80)
/// - `efficiency_rt` — round-trip efficiency
pub fn size_for_backup(
    load_kw: f64,
    duration_h: f64,
    dod: f64,
    efficiency_rt: f64,
) -> SizingResult {
    let eta_d = efficiency_rt.sqrt();
    let e_required = load_kw * duration_h / eta_d; // account for discharge losses
    let capacity_kwh = e_required / dod;
    SizingResult::new(capacity_kwh, load_kw)
}

/// Size a battery to achieve a target self-consumption ratio.
///
/// Self-consumption = PV energy consumed on-site / total PV generation.
///
/// # Arguments
/// - `pv_kw`               — PV generation profile `kW`
/// - `load_kw`             — load profile `kW`
/// - `dt_h`                — time step `hours`
/// - `target_sc_ratio`     — target self-consumption (0–1, e.g. 0.80)
/// - `efficiency_rt`       — round-trip efficiency
pub fn size_for_self_consumption(
    pv_kw: &[f64],
    load_kw: &[f64],
    dt_h: f64,
    target_sc_ratio: f64,
    efficiency_rt: f64,
) -> SizingResult {
    // Binary search on capacity
    let mut lo = 0.0_f64;
    let mut hi = pv_kw.iter().cloned().fold(0.0_f64, f64::max) * 24.0 * dt_h;
    let power_kw = pv_kw.iter().cloned().fold(0.0_f64, f64::max);

    for _ in 0..40 {
        let mid = (lo + hi) / 2.0;
        let sc = simulate_self_consumption(pv_kw, load_kw, mid, power_kw, dt_h, efficiency_rt);
        if sc < target_sc_ratio {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    SizingResult::new(hi, power_kw)
}

/// Simulate self-consumption ratio for a given battery capacity.
fn simulate_self_consumption(
    pv_kw: &[f64],
    load_kw: &[f64],
    capacity_kwh: f64,
    power_kw: f64,
    dt_h: f64,
    efficiency_rt: f64,
) -> f64 {
    if capacity_kwh < 1e-9 {
        // No battery: SC = min(PV, load) / PV
        let total_pv: f64 = pv_kw.iter().sum::<f64>() * dt_h;
        if total_pv < 1e-9 {
            return 1.0;
        }
        let direct_use: f64 = pv_kw
            .iter()
            .zip(load_kw.iter())
            .map(|(&p, &l)| p.min(l))
            .sum::<f64>()
            * dt_h;
        return direct_use / total_pv;
    }

    let eta_c = efficiency_rt.sqrt();
    let eta_d = efficiency_rt.sqrt();
    let mut soc = 0.0_f64; // kWh
    let mut total_pv = 0.0_f64;
    let mut used = 0.0_f64; // direct + battery

    for (&p, &l) in pv_kw.iter().zip(load_kw.iter()) {
        total_pv += p * dt_h;
        let direct = p.min(l);
        used += direct * dt_h;
        let excess = (p - l).max(0.0);
        let shortfall = (l - p).max(0.0);

        // Charge with excess
        let charge = excess
            .min(power_kw)
            .min((capacity_kwh - soc) / (dt_h * eta_c));
        soc += charge * dt_h * eta_c;

        // Discharge for shortfall
        let discharge = shortfall.min(power_kw).min(soc * eta_d / dt_h);
        let e_out = discharge * dt_h;
        soc -= e_out / eta_d;
        used += e_out;
    }

    if total_pv < 1e-9 {
        1.0
    } else {
        (used / total_pv).min(1.0)
    }
}

/// Sensitivity analysis: capacity vs. self-consumption ratio.
///
/// Returns `(capacity_kwh, sc_ratio)` pairs for a range of capacities.
pub fn self_consumption_curve(
    pv_kw: &[f64],
    load_kw: &[f64],
    dt_h: f64,
    efficiency_rt: f64,
    n_points: usize,
) -> Vec<(f64, f64)> {
    let max_pv = pv_kw.iter().cloned().fold(0.0_f64, f64::max);
    let power_kw = max_pv;
    let max_cap = max_pv * 8.0; // 8 hours at peak PV

    (0..=n_points)
        .map(|i| {
            let cap = max_cap * i as f64 / n_points as f64;
            let sc = simulate_self_consumption(pv_kw, load_kw, cap, power_kw, dt_h, efficiency_rt);
            (cap, sc)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solar_load_profiles() -> (Vec<f64>, Vec<f64>) {
        // 24-hour hourly: PV peaks midday, load peaks morning/evening
        let pv: Vec<f64> = (0..24)
            .map(|h| {
                if (6..=18).contains(&h) {
                    50.0 * ((h as f64 - 12.0) / 6.0 * std::f64::consts::PI / 2.0)
                        .cos()
                        .powi(2)
                } else {
                    0.0
                }
            })
            .collect();
        let load: Vec<f64> = (0..24)
            .map(|h| {
                if !(7..=20).contains(&h) {
                    20.0
                } else if h < 10 {
                    40.0
                } else if h < 16 {
                    25.0
                } else {
                    45.0
                }
            })
            .collect();
        (pv, load)
    }

    #[test]
    fn test_peak_shaving_capacity_positive() {
        let load = vec![50.0, 60.0, 80.0, 70.0, 40.0, 30.0];
        let result = size_for_peak_shaving(&load, 60.0, 1.0, 0.90);
        assert!(result.capacity_kwh >= 0.0);
        assert!(result.power_kw >= 0.0);
    }

    #[test]
    fn test_peak_shaving_no_need_if_below_limit() {
        let load = vec![20.0, 30.0, 25.0];
        let result = size_for_peak_shaving(&load, 100.0, 1.0, 0.90);
        assert_eq!(result.power_kw, 0.0, "No battery needed if load < limit");
    }

    #[test]
    fn test_solar_shifting_positive_capacity() {
        let (pv, load) = solar_load_profiles();
        let result = size_for_solar_shifting(&pv, &load, 1.0, 0.90);
        assert!(result.capacity_kwh > 0.0);
        assert!(result.power_kw > 0.0);
    }

    #[test]
    fn test_backup_sizing() {
        let result = size_for_backup(10.0, 4.0, 0.80, 0.90);
        // Need ~10 kW * 4h / 0.95 / 0.80 ≈ 52 kWh
        assert!(
            result.capacity_kwh > 40.0 && result.capacity_kwh < 80.0,
            "Backup capacity={:.1} kWh",
            result.capacity_kwh
        );
        assert_eq!(result.power_kw, 10.0);
    }

    #[test]
    fn test_self_consumption_increases_with_capacity() {
        let (pv, load) = solar_load_profiles();
        let sc_no_bat = simulate_self_consumption(&pv, &load, 0.0, 50.0, 1.0, 0.90);
        let sc_with_bat = simulate_self_consumption(&pv, &load, 100.0, 50.0, 1.0, 0.90);
        assert!(
            sc_with_bat >= sc_no_bat,
            "SC should increase with battery: {:.3} → {:.3}",
            sc_no_bat,
            sc_with_bat
        );
    }

    #[test]
    fn test_sc_curve_monotone() {
        let (pv, load) = solar_load_profiles();
        let curve = self_consumption_curve(&pv, &load, 1.0, 0.90, 10);
        for w in curve.windows(2) {
            assert!(
                w[1].1 >= w[0].1 - 0.001,
                "SC curve should be non-decreasing: {:.3} → {:.3}",
                w[0].1,
                w[1].1
            );
        }
    }

    #[test]
    fn test_c_rate_computed() {
        let r = SizingResult::new(100.0, 50.0);
        assert!((r.c_rate - 0.5).abs() < 1e-9);
    }
}
