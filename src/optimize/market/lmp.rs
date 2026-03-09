//! Locational Marginal Price (LMP) decomposition and computation.
//!
//! LMP_i = λ_energy + congestion_i + loss_i
//!
//! # References
//! - Schweppe et al., "Spot Pricing of Electricity", Springer, 1988
//! - PJM "LMP Calculation Guide", rev. 2023
use serde::{Deserialize, Serialize};

/// LMP decomposition at a bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LmpComponents {
    pub bus_id: usize,
    /// Energy component (system marginal price) \[$/MWh\]
    pub energy: f64,
    /// Congestion component (shift factor × shadow price) \[$/MWh\]
    pub congestion: f64,
    /// Loss component (marginal loss factor × energy price) \[$/MWh\]
    pub loss: f64,
    /// Total LMP \[$/MWh\]
    pub lmp: f64,
}

impl LmpComponents {
    /// Compute LMP decomposition given GSF and shadow prices.
    ///
    /// LMP_i = λ + Σ_l (GSF_{l,i} × μ_l) + (∂loss/∂P_i) × λ
    ///
    /// # Arguments
    /// - `bus_id`        — bus index
    /// - `lambda`        — system energy price \[$/MWh\]
    /// - `gsf`           — generation shift factors for this bus (one per branch)
    /// - `shadow_prices` — branch shadow prices \[$/MWh\] (one per branch)
    /// - `mlf`           — marginal loss factor for this bus \[pu\]
    pub fn compute(
        bus_id: usize,
        lambda: f64,
        gsf: &[f64],
        shadow_prices: &[f64],
        mlf: f64,
    ) -> Self {
        let congestion: f64 = gsf
            .iter()
            .zip(shadow_prices.iter())
            .map(|(&g, &mu)| g * mu)
            .sum();
        let loss = mlf * lambda;
        let lmp = lambda + congestion + loss;
        Self {
            bus_id,
            energy: lambda,
            congestion,
            loss,
            lmp,
        }
    }
}

/// Compute LMP decomposition for all buses.
///
/// # Arguments
/// - `lambda`        — system marginal price \[$/MWh\]
/// - `gsf`           — generation shift factor matrix `[branch][bus]`
/// - `shadow_prices` — branch shadow prices \[$/MWh\]
/// - `mlfs`          — marginal loss factors per bus \[pu\]
pub fn compute_lmps(
    lambda: f64,
    gsf: &[Vec<f64>],
    shadow_prices: &[f64],
    mlfs: &[f64],
) -> Vec<LmpComponents> {
    let n_buses = gsf.first().map(|r| r.len()).unwrap_or(0);
    (0..n_buses)
        .map(|b| {
            let gsf_row: Vec<f64> = gsf
                .iter()
                .map(|row| if b < row.len() { row[b] } else { 0.0 })
                .collect();
            let mlf = if b < mlfs.len() { mlfs[b] } else { 0.0 };
            LmpComponents::compute(b, lambda, &gsf_row, shadow_prices, mlf)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lmp_uncongested_equals_lambda() {
        let lambda = 40.0;
        let gsf = vec![vec![0.5, -0.3, 0.1], vec![-0.2, 0.4, 0.1]];
        let shadow_prices = vec![0.0, 0.0];
        let mlfs = vec![0.0, 0.0, 0.0];
        let lmps = compute_lmps(lambda, &gsf, &shadow_prices, &mlfs);
        for lmp in &lmps {
            assert!(
                (lmp.lmp - lambda).abs() < 1e-10,
                "Uncongested LMP should equal lambda: {:.4}",
                lmp.lmp
            );
        }
    }

    #[test]
    fn test_lmp_congestion_component() {
        let lambda = 40.0;
        let gsf = vec![vec![0.5, 0.3]];
        let shadow_prices = vec![10.0];
        let mlfs = vec![0.0, 0.0];
        let lmps = compute_lmps(lambda, &gsf, &shadow_prices, &mlfs);
        assert!((lmps[0].congestion - 5.0).abs() < 1e-10);
        assert!((lmps[1].congestion - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_lmp_components_sum() {
        let lmp_comp = LmpComponents::compute(0, 35.0, &[0.3, -0.1], &[5.0, 2.0], 0.02);
        let expected = 35.0 + 0.3 * 5.0 + (-0.1) * 2.0 + 0.02 * 35.0;
        assert!((lmp_comp.lmp - expected).abs() < 1e-8);
    }
}
