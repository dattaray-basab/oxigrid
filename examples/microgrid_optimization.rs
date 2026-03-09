use oxigrid::optimize::microgrid::ems::{DieselGen, EmsBattery, EmsDispatcher};
use oxigrid::renewable::solar::irradiance::{poa_isotropic, SolarPosition};

fn main() {
    println!("Microgrid 24-Hour Energy Management System");
    println!("===========================================");
    println!("Site: latitude 35°N, south-facing PV array (tilt 35°), 500 kWp installed");
    println!("Storage: 100 kWh LFP battery (50 kW max power)");
    println!("Backup: 100 kW diesel generator");

    // ── Synthetic 24-hour PV generation profile ──────────────────────────────
    let lat = 35.0;
    let day = 172_u32; // ~summer solstice
    let pv_installed_kw = 500.0;
    let pv_efficiency = 0.18; // module efficiency
    let pv_area_m2 = pv_installed_kw * 1000.0 / (1000.0 * pv_efficiency); // m²

    let pv_kw: Vec<f64> = (0..24)
        .map(|h| {
            let pos = SolarPosition::compute(lat, day, h as f64 + 0.5);
            if pos.is_daytime() {
                let ghi_peak = 900.0; // W/m² clear-sky GHI
                let ghi = ghi_peak * (pos.elevation.sin()).max(0.0);
                let poa = poa_isotropic(ghi, &pos, 35.0, 0.0, 0.2, day);
                poa.total * pv_area_m2 * pv_efficiency / 1000.0
            } else {
                0.0
            }
        })
        .collect();

    // ── Synthetic load profile (commercial building) ─────────────────────────
    let load_kw: Vec<f64> = (0..24)
        .map(|h| {
            // Base load 40 kW, morning/evening peaks
            let base = 40.0_f64;
            let morning = 30.0 * (-(((h as f64) - 9.0) / 2.0).powi(2)).exp();
            let evening = 50.0 * (-(((h as f64) - 18.5) / 2.5).powi(2)).exp();
            base + morning + evening
        })
        .collect();

    // ── Wind (not available at this site: zero) ───────────────────────────────
    let wind_kw = vec![0.0_f64; 24];

    // ── Run EMS dispatch ─────────────────────────────────────────────────────
    let battery = EmsBattery::lifepo4_100kwh();
    let diesel = DieselGen::diesel_100kw();
    let mut ems = EmsDispatcher::new(battery, diesel);
    let plan = ems.dispatch(&load_kw, &pv_kw, &wind_kw, 1.0);

    // ── Print results ─────────────────────────────────────────────────────────
    println!(
        "\n{:>4}  {:>8}  {:>8}  {:>8}  {:>8}  {:>8}  {:>7}",
        "Hour", "Load(kW)", "PV(kW)", "Batt(kW)", "Diesel(kW)", "SoC(%)", "Cost($)"
    );
    println!("{}", "-".repeat(60));

    for iv in &plan.intervals {
        let batt_str = if iv.battery_kw.abs() < 0.1 {
            format!("{:>8.1}", 0.0)
        } else {
            format!("{:>8.1}", iv.battery_kw)
        };

        println!(
            "{:>4.0}  {:>8.1}  {:>8.1}  {}  {:>10.1}  {:>7.1}  {:>7.2}",
            iv.hour,
            iv.load_kw,
            iv.pv_kw,
            batt_str,
            iv.diesel_kw,
            iv.battery_soc * 100.0,
            iv.cost_usd,
        );
    }

    println!("\n--- 24-Hour Summary ---");
    println!("  Total cost:         ${:.2}", plan.total_cost_usd);
    println!("  Total diesel:       {:.1} kWh", plan.total_diesel_kwh);
    println!(
        "  Renewable fraction: {:.1}%",
        plan.renewable_fraction * 100.0
    );
    println!("  Load shed:          {:.1} kWh", plan.total_load_shed_kwh);
    println!("  PV generated:       {:.1} kWh", pv_kw.iter().sum::<f64>());
    println!(
        "  Load served:        {:.1} kWh",
        load_kw.iter().sum::<f64>()
    );

    // ── Weekly energy cost comparison ─────────────────────────────────────────
    println!("\n--- Annualised Economics ---");
    let annual_cost = plan.total_cost_usd * 365.0;
    let diesel_price_per_litre = 1.20;
    let litres_per_kwh = 0.28;
    let diesel_kwh_annual = plan.total_diesel_kwh * 365.0;
    let fuel_cost = diesel_kwh_annual * litres_per_kwh * diesel_price_per_litre;
    println!("  Estimated annual operating cost: ${:.0}", annual_cost);
    println!(
        "  Annual diesel fuel (est.):       {:.0} L  (${:.0})",
        diesel_kwh_annual * litres_per_kwh,
        fuel_cost
    );

    // ── Weekend PV production estimate ────────────────────────────────────────
    let peak_pv: f64 = pv_kw.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let pv_hour_count = pv_kw.iter().filter(|&&p| p > 1.0).count();
    println!("  Peak PV power:                   {:.1} kW", peak_pv);
    println!("  Daily PV generation hours:        {} h", pv_hour_count);

    // Polar plot ticks to illustrate load shape
    let max_load = load_kw.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!("\nLoad profile (normalised):");
    for (h, &l) in load_kw.iter().enumerate() {
        let bars = (l / max_load * 20.0).round() as usize;
        println!("  {:02}h  {:<20}  {:.1} kW", h, "#".repeat(bars), l);
    }

    // Demonstrate cost savings with different battery sizes
    println!("\n--- Battery Sizing Sensitivity ---");
    println!(
        "{:>12}  {:>14}  {:>20}",
        "Battery(kWh)", "DieselCost($/day)", "RenewFraction(%)"
    );
    for &cap in &[0.0_f64, 50.0, 100.0, 200.0] {
        let mut b = EmsBattery::lifepo4_100kwh();
        b.capacity_kwh = cap.max(1.0);
        b.p_max_kw = (cap * 0.5).max(1.0);
        let d = DieselGen::diesel_100kw();
        let mut e = EmsDispatcher::new(b, d);
        let p = e.dispatch(&load_kw, &pv_kw, &wind_kw, 1.0);
        println!(
            "{:>12.0}  {:>14.2}  {:>20.1}",
            cap,
            p.total_cost_usd,
            p.renewable_fraction * 100.0
        );
    }
}
