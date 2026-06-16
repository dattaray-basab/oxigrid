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

    #[test]
    fn test_empty_load_series() {
        let costs = vec![GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 100.0)];
        let intervals = multi_period_dispatch(&costs, &[]).expect("empty loads should succeed");
        assert_eq!(intervals.len(), 0, "empty loads should yield empty result");
    }

    #[test]
    fn test_single_generator_takes_all_load() {
        let costs = vec![GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 200.0)];
        let loads = vec![100.0];
        let intervals = multi_period_dispatch(&costs, &loads)
            .expect("single-generator dispatch should succeed");
        assert_eq!(intervals.len(), 1);
        let iv = &intervals[0];
        assert!(
            (iv.p_gen_mw[0] - 100.0).abs() < 1.0,
            "single generator should take all load; got {}",
            iv.p_gen_mw[0]
        );
    }

    #[test]
    fn test_interval_idx_sequential() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 0.0, 150.0),
        ];
        let loads = vec![10.0, 20.0, 30.0];
        let intervals =
            multi_period_dispatch(&costs, &loads).expect("interval_idx dispatch should succeed");
        assert_eq!(intervals.len(), 3);
        for (expected_idx, iv) in intervals.iter().enumerate() {
            assert_eq!(
                iv.interval_idx, expected_idx,
                "interval_idx should be sequential; expected {} got {}",
                expected_idx, iv.interval_idx
            );
        }
    }

    #[test]
    fn test_total_cost_positive() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 0.0, 150.0),
        ];
        let loads = vec![80.0];
        let intervals =
            multi_period_dispatch(&costs, &loads).expect("total_cost dispatch should succeed");
        assert_eq!(intervals.len(), 1);
        let iv = &intervals[0];
        assert!(
            iv.total_cost > 0.0,
            "total_cost should be positive; got {}",
            iv.total_cost
        );
    }

    #[test]
    fn test_lambda_nonnegative() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 0.0, 150.0),
        ];
        let loads = vec![50.0, 80.0, 120.0];
        let intervals =
            multi_period_dispatch(&costs, &loads).expect("lambda dispatch should succeed");
        for iv in &intervals {
            assert!(
                iv.lambda >= 0.0,
                "lambda should be non-negative; got {} at interval {}",
                iv.lambda,
                iv.interval_idx
            );
        }
    }

    #[test]
    fn test_higher_load_higher_cost() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 200.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 0.0, 200.0),
        ];
        let intervals_low =
            multi_period_dispatch(&costs, &[100.0]).expect("low-load dispatch should succeed");
        let intervals_high =
            multi_period_dispatch(&costs, &[200.0]).expect("high-load dispatch should succeed");
        let cost_low = intervals_low[0].total_cost;
        let cost_high = intervals_high[0].total_cost;
        assert!(
            cost_high > cost_low,
            "higher load should yield higher total cost; low={} high={}",
            cost_low,
            cost_high
        );
    }

    #[test]
    fn test_dispatch_interval_fields() {
        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 100.0),
            GenCost::quadratic(0.0, 30.0, 0.03, 0.0, 150.0),
        ];
        let loads = vec![60.0, 90.0, 110.0];
        let intervals = multi_period_dispatch(&costs, &loads)
            .expect("dispatch_interval_fields dispatch should succeed");
        assert_eq!(intervals.len(), loads.len());
        for (iv, &expected_load) in intervals.iter().zip(loads.iter()) {
            assert!(
                (iv.load_mw - expected_load).abs() < 1e-9,
                "load_mw should match input load; expected {} got {}",
                expected_load,
                iv.load_mw
            );
        }
    }
}
