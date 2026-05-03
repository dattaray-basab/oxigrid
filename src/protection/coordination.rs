/// Protection coordination for overcurrent relays.
///
/// Checks that primary and backup relays have sufficient coordination margins
/// (coordination time interval, CTI) and generates a coordination chart.
///
/// # Method
/// For each pair (primary relay, backup relay), the time–current characteristic
/// (IEC 60255 standard inverse) is computed at the maximum fault current seen
/// by the backup.  The coordination margin must be ≥ CTI (typically 0.2–0.3 s).
///
/// # Reference
/// IEEE Std C37.112-1996 "IEEE Standard Inverse-Time Characteristic Equations".
use serde::{Deserialize, Serialize};

use crate::protection::relay::{OcRelay, RelayCharacteristic};

/// A coordination pair: primary relay + backup relay with results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationPair {
    /// Index of primary (fastest) relay
    pub primary_idx: usize,
    /// Index of backup relay
    pub backup_idx: usize,
    /// Maximum fault current seen by the backup relay `A`
    pub i_fault_a: f64,
    /// Operating time of primary relay at i_fault_a `s`
    pub t_primary_s: f64,
    /// Operating time of backup relay at i_fault_a `s`
    pub t_backup_s: f64,
    /// Coordination margin = t_backup − t_primary `s`
    pub margin_s: f64,
    /// True if margin ≥ CTI
    pub coordinated: bool,
}

/// Coordination violation for a relay pair.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationViolation {
    pub primary_idx: usize,
    pub backup_idx: usize,
    pub margin_s: f64,
    pub required_cti_s: f64,
}

/// Full coordination study result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationStudy {
    pub pairs: Vec<CoordinationPair>,
    pub violations: Vec<CoordinationViolation>,
    /// Required coordination time interval `s`
    pub cti_s: f64,
}

impl CoordinationStudy {
    /// True if all relay pairs are coordinated.
    pub fn is_fully_coordinated(&self) -> bool {
        self.violations.is_empty()
    }

    /// Number of violating pairs.
    pub fn n_violations(&self) -> usize {
        self.violations.len()
    }
}

/// Check coordination between a set of relay pairs.
///
/// # Arguments
/// * `pairs` — list of `(primary_relay_idx, backup_relay_idx, i_fault_a)` tuples
/// * `relays` — slice of all relays (indexed by the pair tuples)
/// * `cti_s`  — required coordination time interval `s` (typically 0.2–0.3 s)
pub fn check_coordination(
    relays: &[OcRelay],
    pairs: &[(usize, usize, f64)],
    cti_s: f64,
) -> CoordinationStudy {
    let mut coord_pairs = Vec::new();
    let mut violations = Vec::new();

    for &(pi, bi, i_fault) in pairs {
        let t_p = relays[pi].trip_time(i_fault).unwrap_or(f64::INFINITY);
        let t_b = relays[bi].trip_time(i_fault).unwrap_or(f64::INFINITY);
        let margin = t_b - t_p;
        let coordinated = margin >= cti_s;

        if !coordinated {
            violations.push(CoordinationViolation {
                primary_idx: pi,
                backup_idx: bi,
                margin_s: margin,
                required_cti_s: cti_s,
            });
        }

        coord_pairs.push(CoordinationPair {
            primary_idx: pi,
            backup_idx: bi,
            i_fault_a: i_fault,
            t_primary_s: t_p,
            t_backup_s: t_b,
            margin_s: margin,
            coordinated,
        });
    }

    CoordinationStudy {
        pairs: coord_pairs,
        violations,
        cti_s,
    }
}

/// Generate a time–current characteristic curve for a relay.
///
/// Returns `(current `A`, time `s`)` pairs over the range `[i_min, i_max]`
/// with `n_points` logarithmically spaced values.
pub fn tcc_curve(relay: &OcRelay, i_min: f64, i_max: f64, n_points: usize) -> Vec<(f64, f64)> {
    let log_min = i_min.ln();
    let log_max = i_max.ln();
    (0..n_points)
        .filter_map(|i| {
            let frac = i as f64 / (n_points - 1).max(1) as f64;
            let current = (log_min + frac * (log_max - log_min)).exp();
            relay
                .trip_time(current)
                .filter(|&t| t.is_finite() && t > 0.0)
                .map(|time| (current, time))
        })
        .collect()
}

/// Recommended TMS for a backup relay to achieve coordination with a primary.
///
/// Given a target backup trip time `t_target_s`, compute the TMS that achieves
/// it at the given fault current.
///
/// For IEC standard inverse: t = TMS · K / (M^α − 1)
/// → TMS = t · (M^α − 1) / K
pub fn recommend_tms(relay: &OcRelay, t_target_s: f64, i_fault_a: f64) -> f64 {
    if relay.i_pickup < 1e-10 || i_fault_a <= relay.i_pickup {
        return relay.tms;
    }
    let m = i_fault_a / relay.i_pickup;
    let (k, alpha): (f64, f64) = match relay.curve {
        RelayCharacteristic::StandardInverse => (0.14, 0.02),
        RelayCharacteristic::VeryInverse => (13.5, 1.0),
        RelayCharacteristic::ExtremelyInverse => (80.0, 2.0),
        RelayCharacteristic::LongTimeInverse => (120.0, 1.0),
        _ => (0.14, 0.02), // default to SI for IEEE curves
    };
    let denom = m.powf(alpha) - 1.0;
    if denom < 1e-10 {
        return relay.tms;
    }
    t_target_s * denom / k
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protection::relay::{OcRelay, RelayCharacteristic};

    fn make_relay(pickup: f64, tms: f64) -> OcRelay {
        OcRelay::new(pickup, tms, RelayCharacteristic::StandardInverse)
    }

    #[test]
    fn test_coordinated_pair() {
        // Backup relay has higher TMS → longer operating time
        let relays = vec![
            make_relay(100.0, 0.10), // R0: primary
            make_relay(100.0, 0.30), // R1: backup
        ];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        assert!(
            study.is_fully_coordinated(),
            "Backup TMS=0.3 should coordinate with primary TMS=0.1"
        );
    }

    #[test]
    fn test_uncoordinated_pair() {
        let relays = vec![
            make_relay(100.0, 0.30), // R0: primary — slower than backup
            make_relay(100.0, 0.31), // R1: backup — barely faster
        ];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        assert!(
            !study.is_fully_coordinated(),
            "Margin too small → violation expected"
        );
        assert_eq!(study.n_violations(), 1);
    }

    #[test]
    fn test_margin_calculation() {
        let relays = vec![make_relay(100.0, 0.10), make_relay(100.0, 0.40)];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        let pair = &study.pairs[0];
        assert!((pair.margin_s - (pair.t_backup_s - pair.t_primary_s)).abs() < 1e-10);
        assert!(pair.margin_s > 0.0);
    }

    #[test]
    fn test_tcc_curve_length() {
        let relay = make_relay(100.0, 0.20);
        let curve = tcc_curve(&relay, 150.0, 5000.0, 20);
        assert!(!curve.is_empty());
        assert!(curve.len() <= 20);
    }

    #[test]
    fn test_tcc_curve_monotone_decreasing() {
        // Higher current → faster trip
        let relay = make_relay(100.0, 0.20);
        let curve = tcc_curve(&relay, 200.0, 2000.0, 10);
        for i in 1..curve.len() {
            assert!(curve[i].0 > curve[i - 1].0, "Currents should be increasing");
            assert!(
                curve[i].1 < curve[i - 1].1 + 1e-6,
                "Times should be decreasing"
            );
        }
    }

    #[test]
    fn test_recommend_tms_positive() {
        let relay = make_relay(100.0, 0.20);
        let tms_new = recommend_tms(&relay, 0.60, 500.0);
        assert!(
            tms_new > 0.0,
            "Recommended TMS should be positive: {:.4}",
            tms_new
        );
    }

    #[test]
    fn test_multiple_pairs() {
        let relays = vec![
            make_relay(100.0, 0.10), // R0
            make_relay(100.0, 0.35), // R1
            make_relay(80.0, 0.15),  // R2
            make_relay(80.0, 0.40),  // R3
        ];
        let study = check_coordination(&relays, &[(0, 1, 600.0), (2, 3, 400.0)], 0.20);
        assert_eq!(study.pairs.len(), 2);
    }
}
