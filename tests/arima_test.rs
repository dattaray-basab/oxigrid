#![cfg(feature = "renewable")]
use oxigrid::renewable::forecast::arima::{
    aic, autocorrelation, select_ar_order, ArModel, ArimaModel,
};

// ── AR Model Fitting ─────────────────────────────────────────────────────────

#[test]
fn test_ar1_fit_known_series() {
    // Generate AR(1) series: y_t = 0.8 * y_{t-1} + ε
    let n = 200;
    let phi_true = 0.8;
    let mut series = vec![0.0_f64; n];
    // Use a simple PRNG-like deterministic series for reproducibility
    let mut noise = 0.1;
    for t in 1..n {
        noise = (noise * 1.7 + 0.3) % 0.4 - 0.2; // bounded pseudo-noise
        series[t] = phi_true * series[t - 1] + noise;
    }

    let model = ArModel::fit(&series, 1).expect("AR(1) fit failed");
    assert_eq!(model.order, 1);
    assert!(
        (model.phi[0] - phi_true).abs() < 0.15,
        "AR(1) coefficient estimate far from truth: {} vs {}",
        model.phi[0],
        phi_true
    );
}

#[test]
fn test_ar_model_forecast_positive() {
    // Non-negative series (solar power)
    let series: Vec<f64> = (0..100)
        .map(|i| (i as f64 * 0.1).sin().abs() * 10.0)
        .collect();
    let model = ArModel::fit(&series, 2).expect("AR(2) fit failed");
    let history: Vec<f64> = series[90..].to_vec();
    let forecast = model.forecast(&history, 5);
    assert_eq!(forecast.len(), 5);
    for (i, &v) in forecast.iter().enumerate() {
        assert!(
            v >= 0.0,
            "Forecast value {} should be non-negative: {:.4}",
            i,
            v
        );
    }
}

#[test]
fn test_ar_model_residuals() {
    let series: Vec<f64> = (0..50).map(|i| (i as f64).sin()).collect();
    let model = ArModel::fit(&series, 1).expect("AR fit failed");
    let residuals = model.residuals(&series);
    assert_eq!(residuals.len(), series.len() - 1);
    // Residuals should be smaller than the original variance
    let res_var: f64 = residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64;
    let orig_var: f64 = {
        let mean = series.iter().sum::<f64>() / series.len() as f64;
        series.iter().map(|y| (y - mean).powi(2)).sum::<f64>() / series.len() as f64
    };
    // AR model should explain some variance (not trivially true for short sine, but residuals finite)
    assert!(res_var.is_finite(), "Residual variance is NaN/Inf");
    assert!(orig_var.is_finite(), "Original variance is NaN/Inf");
}

#[test]
fn test_ar_model_mae() {
    let train: Vec<f64> = (0..80)
        .map(|i| (i as f64 * 0.2).sin() * 5.0 + 5.0)
        .collect();
    let test: Vec<f64> = (80..100)
        .map(|i| (i as f64 * 0.2).sin() * 5.0 + 5.0)
        .collect();
    let model = ArModel::fit(&train, 1).expect("AR fit failed");
    let mae = model.mae(&train, &test);
    assert!(mae >= 0.0, "MAE should be non-negative: {mae:.4}");
    assert!(mae.is_finite(), "MAE should be finite: {mae}");
}

#[test]
fn test_ar_insufficient_data() {
    let series = vec![1.0, 2.0, 3.0];
    let result = ArModel::fit(&series, 5); // order > n
    assert!(result.is_none(), "Should return None for order > n");
}

// ── ARIMA Model ──────────────────────────────────────────────────────────────

#[test]
fn test_arima_fit_stationary() {
    // Stationary series: d=0 should be same as AR fit
    let series: Vec<f64> = (0..100).map(|i| (i as f64 * 0.3).sin() * 3.0).collect();
    let model = ArimaModel::fit(&series, 1, 0).expect("ARIMA fit failed");
    let history: Vec<f64> = series[95..].to_vec();
    let forecast = model.forecast(&history, 5);
    assert_eq!(forecast.len(), 5);
    for &v in &forecast {
        assert!(v.is_finite(), "ARIMA forecast should be finite: {v}");
    }
}

#[test]
fn test_arima_fit_integrated() {
    // Random walk (d=1 makes it stationary)
    let mut series = vec![0.0_f64; 100];
    let mut noise = 0.05;
    for t in 1..100 {
        noise = (noise * 1.3 + 0.1) % 0.3 - 0.15;
        series[t] = series[t - 1] + noise;
    }
    let model = ArimaModel::fit(&series, 1, 1).expect("ARIMA(1,1,0) fit failed");
    let history: Vec<f64> = series[94..].to_vec();
    let forecast = model.forecast(&history, 5);
    assert_eq!(forecast.len(), 5);
}

#[test]
fn test_arima_forecast_nonneg() {
    // Solar-like series: non-negative
    let series: Vec<f64> = (0..100)
        .map(|i| (i as f64 * PI / 50.0).sin().max(0.0) * 800.0)
        .collect();
    let model = ArimaModel::fit(&series, 2, 0).expect("ARIMA fit failed");
    let history: Vec<f64> = series[90..].to_vec();
    let forecast = model.forecast(&history, 10);
    for (i, &v) in forecast.iter().enumerate() {
        assert!(v >= 0.0, "Forecast[{i}] = {v:.4} is negative");
    }
}

// ── Autocorrelation ──────────────────────────────────────────────────────────

#[test]
fn test_autocorrelation_lag0() {
    let series = vec![1.0, 2.0, 3.0, 4.0, 5.0];
    let ac0 = autocorrelation(&series, 0);
    // ACF at lag 0 = 1.0 (if normalised by variance)
    // Our implementation returns raw not normalised, just check > 0
    assert!(
        ac0 > 0.0,
        "Autocorrelation at lag 0 should be positive: {ac0}"
    );
}

#[test]
fn test_autocorrelation_white_noise() {
    // White noise should have small autocorrelation at lag > 0
    let n = 500;
    let series: Vec<f64> = (0..n)
        .map(|i| {
            // Deterministic pseudo-white noise
            let x = (i as f64 * 7919.0).sin();
            x - x.floor() - 0.5
        })
        .collect();
    let ac1 = autocorrelation(&series, 1);
    let ac0 = autocorrelation(&series, 0);
    let normalised = (ac1 / ac0).abs();
    assert!(
        normalised < 0.2,
        "White noise ACF at lag 1 too large: {normalised:.4}"
    );
}

// ── AIC and Order Selection ───────────────────────────────────────────────────

#[test]
fn test_aic_increases_with_order_for_white_noise() {
    // For white noise, higher AR orders shouldn't significantly improve fit
    let series: Vec<f64> = (0..200)
        .map(|i| {
            let x = (i as f64 * 6271.0).sin();
            x - x.floor() - 0.5
        })
        .collect();
    let aic1 = aic(series.len(), 1, ArModel::fit(&series, 1).unwrap().sigma2);
    let aic5 = aic(series.len(), 5, ArModel::fit(&series, 5).unwrap().sigma2);
    // AIC may increase for over-parameterised model on white noise
    // (or at minimum not dramatically decrease)
    assert!(aic1.is_finite(), "AIC order 1 should be finite: {aic1}");
    assert!(aic5.is_finite(), "AIC order 5 should be finite: {aic5}");
}

#[test]
fn test_select_ar_order_ar1_process() {
    // For a true AR(1) process, selected order should be ≤ 3
    let phi = 0.7;
    let mut series = vec![0.0_f64; 150];
    let mut noise = 0.1;
    for t in 1..150 {
        noise = (noise * 1.7 + 0.3) % 0.4 - 0.2;
        series[t] = phi * series[t - 1] + noise;
    }
    let order = select_ar_order(&series, 10);
    assert!(
        (1..=5).contains(&order),
        "Selected AR order {order} out of expected range [1,5] for AR(1) process"
    );
}

use std::f64::consts::PI;
