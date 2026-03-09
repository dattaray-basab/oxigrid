/// Economic load dispatch.
///
/// Classic equal-incremental-cost problem: find the generator outputs P_i
/// that minimise total cost while meeting the load balance constraint.
///
/// This module provides utilities for multi-period dispatch, unit
/// commitment helpers, and reporting.
use crate::optimize::opf::dc_opf::{economic_dispatch_pub, GenCost};
use serde::{Deserialize, Serialize};

/// A single dispatch interval result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispatchInterval {
    pub interval_idx: usize,
    pub load_mw: f64,
    pub p_gen_mw: Vec<f64>,
    pub total_cost: f64,
    pub lambda: f64,
}

/// Run economic dispatch for a sequence of load values.
///
/// Returns one `DispatchInterval` per load value.
pub fn multi_period_dispatch(
    costs: &[GenCost],
    loads_mw: &[f64],
) -> crate::error::Result<Vec<DispatchInterval>> {
    loads_mw
        .iter()
        .enumerate()
        .map(|(idx, &load)| {
            let p = economic_dispatch_pub(costs, load)?;
            let total_cost: f64 = costs
                .iter()
                .zip(p.iter())
                .map(|(c, &pi)| c.total_cost(pi))
                .sum();
            let lambda = costs
                .iter()
                .zip(p.iter())
                .filter(|(c, &pi)| pi > c.p_min + 1e-3 && pi < c.p_max - 1e-3)
                .map(|(c, &pi)| c.marginal_cost(pi))
                .next()
                .unwrap_or(0.0);
            Ok(DispatchInterval {
                interval_idx: idx,
                load_mw: load,
                p_gen_mw: p,
                total_cost,
                lambda,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multi_period_basic() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 10.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 10.0, 150.0),
        ];
        let loads = vec![50.0, 80.0, 120.0];
        let intervals = multi_period_dispatch(&costs, &loads).unwrap();
        assert_eq!(intervals.len(), 3);
        for (iv, &load) in intervals.iter().zip(loads.iter()) {
            let total: f64 = iv.p_gen_mw.iter().sum();
            assert!(
                (total - load).abs() < 1e-2,
                "iv={} total={:.4}",
                iv.interval_idx,
                total
            );
        }
    }
}
