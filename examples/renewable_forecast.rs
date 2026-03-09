use oxigrid::renewable::forecast::persistence::{
    skill_score, DiurnalPersistence, PersistenceForecast,
};
use oxigrid::renewable::solar::irradiance::SolarPosition;

fn main() {
    println!("Renewable Energy Forecasting — Persistence & Diurnal Models");
    println!("=============================================================");

    // ── Generate a synthetic 5-day hourly GHI dataset ─────────────────────────
    let lat = 40.0;
    let n_days = 5_usize;
    let n_hours = 24 * n_days;

    // Simulate clear-sky GHI with random cloud perturbations
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let pseudo_rand = |seed: usize| -> f64 {
        let mut h = DefaultHasher::new();
        seed.hash(&mut h);
        let v = h.finish();
        (v % 1000) as f64 / 1000.0
    };

    let mut observations: Vec<f64> = Vec::with_capacity(n_hours);
    for i in 0..n_hours {
        let day = (i / 24 + 1) as u32 + 90; // April days
        let h = (i % 24) as f64 + 0.5;
        let pos = SolarPosition::compute(lat, day, h);
        let ghi_clearsky = if pos.is_daytime() {
            850.0 * pos.elevation.sin().max(0.0)
        } else {
            0.0
        };
        // Random cloud factor 0.3–1.0
        let cloud = 0.3 + 0.7 * pseudo_rand(i * 31 + day as usize);
        observations.push(ghi_clearsky * cloud);
    }

    // ── Naive persistence forecast ─────────────────────────────────────────────
    let mut naive = PersistenceForecast::new(observations[0]);
    let mut naive_errors: Vec<f64> = Vec::new();

    // ── Diurnal persistence (same-hour yesterday) ─────────────────────────────
    let mut diurnal = DiurnalPersistence::new(24);
    // Warm up with first day
    for obs in &observations[..24] {
        diurnal.update(*obs);
    }
    let mut diurnal_errors: Vec<f64> = Vec::new();

    // Evaluate on days 2–5
    println!(
        "\n{:>5}  {:>10}  {:>10}  {:>10}  {:>10}",
        "Hour", "Obs(W/m²)", "Naive", "Diurnal", "Clear-sky"
    );
    println!("{}", "-".repeat(52));

    for (i, &obs) in observations.iter().enumerate().skip(24) {
        let day = (i / 24 + 1) as u32 + 90;
        let h = (i % 24) as f64 + 0.5;
        let pos = SolarPosition::compute(lat, day, h);
        let clearsky = if pos.is_daytime() {
            850.0 * pos.elevation.sin().max(0.0)
        } else {
            0.0
        };

        let naive_fc = naive.last_value;
        let diurnal_fc = diurnal.forecast_next_period()[0];

        naive_errors.push((obs - naive_fc).powi(2));
        diurnal_errors.push((obs - diurnal_fc).powi(2));

        naive.update(obs);
        diurnal.update(obs);

        if i % 8 == 0 {
            println!(
                "{:>5}  {:>10.1}  {:>10.1}  {:>10.1}  {:>10.1}",
                i, obs, naive_fc, diurnal_fc, clearsky
            );
        }
    }

    // ── Compute RMSE ─────────────────────────────────────────────────────────
    let rmse = |errs: &[f64]| -> f64 { (errs.iter().sum::<f64>() / errs.len() as f64).sqrt() };
    let naive_rmse = rmse(&naive_errors);
    let diurnal_rmse = rmse(&diurnal_errors);
    let skill = skill_score(diurnal_rmse, naive_rmse);

    println!("\n--- Forecast Verification (days 2–5) ---");
    println!("  Naive persistence RMSE:   {:.1} W/m²", naive_rmse);
    println!("  Diurnal persistence RMSE: {:.1} W/m²", diurnal_rmse);
    println!(
        "  Diurnal skill score:      {:.3}  (vs naive; 1.0 = perfect)",
        skill
    );

    // ── Multi-step ahead forecast example ─────────────────────────────────────
    println!("\n--- 12-hour ahead naive persistence forecast ---");
    let fc12 = naive.forecast_k_steps(12);
    for (k, &v) in fc12.iter().enumerate() {
        println!("  t+{:02}: {:.1} W/m²", k + 1, v);
    }

    // ── Tomorrow's diurnal forecast profile ──────────────────────────────────
    println!("\n--- Tomorrow's diurnal profile forecast ---");
    let tomorrow = diurnal.forecast_next_period();
    for (h, &v) in tomorrow.iter().enumerate() {
        let bars = if v > 0.0 {
            (v / 900.0 * 30.0).round() as usize
        } else {
            0
        };
        println!("  {:02}h  {:<30}  {:.1} W/m²", h, "#".repeat(bars), v);
    }
}
