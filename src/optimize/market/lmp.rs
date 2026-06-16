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

    #[test]
    fn test_lmp_energy_component_stored() {
        let lmp_comp = LmpComponents::compute(2, 50.0, &[0.1, -0.2], &[5.0, 3.0], 0.0);
        assert!(
            (lmp_comp.energy - 50.0).abs() < 1e-10,
            "energy component should equal lambda: {:.4}",
            lmp_comp.energy
        );
        assert_eq!(lmp_comp.bus_id, 2);
    }

    #[test]
    fn test_lmp_congestion_positive_on_congested_line() {
        let lambda = 30.0;
        let gsf = vec![vec![0.8_f64, 0.2_f64]];
        let shadow_prices = vec![20.0_f64];
        let mlfs = vec![0.0_f64, 0.0_f64];
        let lmps = compute_lmps(lambda, &gsf, &shadow_prices, &mlfs);
        assert!(
            (lmps[0].congestion - 16.0).abs() < 1e-10,
            "bus 0 congestion should be 16.0: {:.4}",
            lmps[0].congestion
        );
        assert!(
            (lmps[1].congestion - 4.0).abs() < 1e-10,
            "bus 1 congestion should be 4.0: {:.4}",
            lmps[1].congestion
        );
    }

    #[test]
    fn test_lmp_reference_bus_zero_gsf() {
        let lmp_comp = LmpComponents::compute(0, 45.0, &[0.0, 0.0, 0.0], &[10.0, 5.0, 8.0], 0.0);
        assert!(
            lmp_comp.congestion.abs() < 1e-10,
            "reference bus congestion should be zero: {:.4}",
            lmp_comp.congestion
        );
        assert!(
            (lmp_comp.lmp - 45.0).abs() < 1e-10,
            "reference bus LMP should equal energy price: {:.4}",
            lmp_comp.lmp
        );
    }

    #[test]
    fn test_lmp_loss_component_proportional_to_lambda() {
        let lmp_comp = LmpComponents::compute(0, 60.0, &[], &[], 0.05);
        assert!(
            (lmp_comp.loss - 0.05 * 60.0).abs() < 1e-10,
            "loss component should be 0.05 * 60.0 = 3.0: {:.4}",
            lmp_comp.loss
        );
        assert!(
            (lmp_comp.lmp - 63.0).abs() < 1e-10,
            "lmp should be 63.0: {:.4}",
            lmp_comp.lmp
        );
    }

    #[test]
    fn test_lmp_negative_gsf_reduces_lmp() {
        let lmp_comp = LmpComponents::compute(1, 40.0, &[-0.6], &[15.0], 0.0);
        assert!(
            (lmp_comp.congestion - (-9.0)).abs() < 1e-10,
            "congestion should be -9.0: {:.4}",
            lmp_comp.congestion
        );
        assert!(
            (lmp_comp.lmp - 31.0).abs() < 1e-10,
            "lmp should be 31.0: {:.4}",
            lmp_comp.lmp
        );
    }

    #[test]
    fn test_compute_lmps_bus_count() {
        let lmps = compute_lmps(
            35.0,
            &[vec![0.3_f64, 0.5_f64, -0.2_f64, 0.1_f64]],
            &[8.0_f64],
            &[0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64],
        );
        assert_eq!(lmps.len(), 4, "should have one LmpComponents per bus");
        for (i, lmp) in lmps.iter().enumerate() {
            assert_eq!(
                lmp.bus_id, i,
                "bus_id at index {i} should equal {i}, got {}",
                lmp.bus_id
            );
        }
    }
}
