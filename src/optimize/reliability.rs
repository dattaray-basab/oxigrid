/// Generation adequacy and reliability assessment.
///
/// Computes probabilistic reliability indices used in power system planning:
///
/// - **LOLP** (Loss of Load Probability): probability that load exceeds available generation
/// - **LOLE** (Loss of Load Expectation): expected hours/days per year with insufficient capacity
/// - **EENS** (Expected Energy Not Served): expected unserved energy [MWh/year]
/// - **EUE** (Expected Unserved Energy): same as EENS, per IEEE 762
/// - **Capacity Credit**: effective load-carrying capability (ELCC) of a resource
/// - **Reserve margin adequacy**: probability that reserve margin is adequate
///
/// # Method
/// Uses the **Capacity Outage Probability Table (COPT)** method:
/// 1. Build COPT from individual unit availabilities (FOR/availability)
/// 2. Convolve with load duration curve
/// 3. Compute expectation over outage states
///
/// # References
/// - Billinton & Allan, "Reliability Evaluation of Power Systems", 2nd Ed., Plenum 1996
/// - IEEE Std 762-2006 — Definitions for Use in Reporting Electric Generating Unit Reliability
/// - NERC, "Probabilistic Assessment", December 2019
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Generator unit model
// ─────────────────────────────────────────────────────────────────────────────

/// Generating unit for reliability assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityUnit {
    /// Unit name / identifier
    pub name: String,
    /// Installed capacity [MW]
    pub capacity_mw: f64,
    /// Forced Outage Rate (FOR) — probability of being unavailable [0, 1]
    pub forced_outage_rate: f64,
    /// Planned outage hours per year
    pub planned_outage_hours: f64,
}

impl ReliabilityUnit {
    /// Create a unit with a given capacity and FOR.
    pub fn new(name: impl Into<String>, capacity_mw: f64, for_rate: f64) -> Self {
        Self {
            name: name.into(),
            capacity_mw,
            forced_outage_rate: for_rate.clamp(0.0, 1.0),
            planned_outage_hours: 0.0,
        }
    }

    /// Unit availability (complement of total outage rate).
    pub fn availability(&self) -> f64 {
        let planned_rate = self.planned_outage_hours / 8760.0;
        (1.0 - self.forced_outage_rate) * (1.0 - planned_rate)
    }

    /// Expected available capacity [MW].
    pub fn expected_capacity_mw(&self) -> f64 {
        self.capacity_mw * self.availability()
    }

    /// Typical baseload unit (coal/nuclear): large capacity, low FOR.
    pub fn baseload(name: impl Into<String>, capacity_mw: f64) -> Self {
        Self::new(name, capacity_mw, 0.05)
    }

    /// Typical peaking unit (gas turbine): smaller capacity, higher FOR.
    pub fn peaking(name: impl Into<String>, capacity_mw: f64) -> Self {
        Self::new(name, capacity_mw, 0.10)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Capacity Outage Probability Table (COPT)
// ─────────────────────────────────────────────────────────────────────────────

/// A single entry in the Capacity Outage Probability Table.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoptEntry {
    /// Capacity on outage [MW]
    pub outage_mw: f64,
    /// Exact probability of this outage state
    pub probability: f64,
    /// Cumulative probability (P ≥ this outage level)
    pub cumulative_prob: f64,
}

/// Capacity Outage Probability Table.
///
/// Entries are sorted by ascending outage level.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Copt {
    pub entries: Vec<CoptEntry>,
    /// Total installed capacity [MW]
    pub total_capacity_mw: f64,
}

impl Copt {
    /// Build COPT by successively convolving unit outage distributions.
    ///
    /// For each unit with capacity C_i and FOR p_i:
    ///   P(X + C_i outage) = p_i * P(X) + (1-p_i) * P(X + C_i outage before)
    ///
    /// This is the recursive convolution method (Billinton & Allan, Ch. 2).
    pub fn from_units(units: &[ReliabilityUnit]) -> Self {
        // Use integer bucketing of 1 MW resolution
        let total_cap: f64 = units.iter().map(|u| u.capacity_mw).sum();
        let n_states = (total_cap.ceil() as usize) + 1;

        // prob_table[k] = P(exactly k MW on outage)
        let mut prob_table = vec![0.0_f64; n_states];
        prob_table[0] = 1.0; // initially no outage

        for unit in units {
            let cap_i = unit.capacity_mw.round() as usize;
            let for_i = unit.forced_outage_rate;
            let avail_i = 1.0 - for_i;

            // Convolve from high to low to avoid using updated values
            for k in (0..n_states).rev() {
                let p_stay = prob_table[k] * avail_i;
                let p_out = prob_table[k] * for_i;
                prob_table[k] = p_stay;
                let new_k = k + cap_i;
                if new_k < n_states {
                    prob_table[new_k] += p_out;
                }
            }
        }

        // Build cumulative probabilities
        let mut cumulative = vec![0.0_f64; n_states];
        cumulative[n_states - 1] = prob_table[n_states - 1];
        for k in (0..n_states - 1).rev() {
            cumulative[k] = cumulative[k + 1] + prob_table[k];
        }

        // Build entries (only keep nonzero states)
        let entries: Vec<CoptEntry> = (0..n_states)
            .filter(|&k| prob_table[k] > 1e-15)
            .map(|k| CoptEntry {
                outage_mw: k as f64,
                probability: prob_table[k],
                cumulative_prob: cumulative[k],
            })
            .collect();

        Self {
            entries,
            total_capacity_mw: total_cap,
        }
    }

    /// Probability that available capacity ≤ threshold [MW].
    /// Equivalently, P(outage ≥ total_capacity - threshold).
    pub fn prob_capacity_below(&self, threshold_mw: f64) -> f64 {
        let required_outage = self.total_capacity_mw - threshold_mw;
        if required_outage <= 0.0 {
            return 1.0;
        }
        self.entries
            .iter()
            .filter(|e| e.outage_mw >= required_outage)
            .map(|e| e.probability)
            .sum()
    }

    /// Expected available capacity [MW].
    pub fn expected_available_capacity(&self) -> f64 {
        self.entries
            .iter()
            .map(|e| (self.total_capacity_mw - e.outage_mw) * e.probability)
            .sum()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Load Duration Curve
// ─────────────────────────────────────────────────────────────────────────────

/// Discretised load duration curve.
///
/// `loads[i]` is the load [MW] at hour `i` of the year (8760 hours total).
/// The LDC is derived by sorting in descending order.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadDurationCurve {
    /// Sorted load levels (descending), length = 8760
    pub ldc: Vec<f64>,
    /// Peak load [MW]
    pub peak_load_mw: f64,
    /// Average load [MW]
    pub average_load_mw: f64,
    /// Total energy [MWh/year]
    pub total_energy_mwh: f64,
}

impl LoadDurationCurve {
    /// Build LDC from hourly load profile (8760 values).
    pub fn from_hourly(hourly_loads: &[f64]) -> Self {
        let n = hourly_loads.len();
        let total_energy_mwh: f64 = hourly_loads.iter().sum();
        let average_load_mw = total_energy_mwh / n as f64;
        let peak_load_mw = hourly_loads.iter().cloned().fold(0.0_f64, f64::max);

        let mut ldc = hourly_loads.to_vec();
        ldc.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

        Self {
            ldc,
            peak_load_mw,
            average_load_mw,
            total_energy_mwh,
        }
    }

    /// Number of hours per year that load exceeds `threshold_mw`.
    pub fn hours_exceeding(&self, threshold_mw: f64) -> usize {
        self.ldc.iter().filter(|&&l| l > threshold_mw).count()
    }

    /// Fraction of year that load exceeds `threshold_mw`.
    pub fn fraction_exceeding(&self, threshold_mw: f64) -> f64 {
        self.hours_exceeding(threshold_mw) as f64 / self.ldc.len() as f64
    }

    /// Load at given percentile (0 = peak, 100 = minimum).
    pub fn load_at_percentile(&self, pct: f64) -> f64 {
        let idx = ((pct / 100.0) * (self.ldc.len() - 1) as f64).round() as usize;
        self.ldc[idx.min(self.ldc.len() - 1)]
    }

    /// Synthesise a simple LDC from peak, average, and n_hours.
    /// Models load as a half-sinusoid between min and peak.
    pub fn synthesise(peak_mw: f64, load_factor: f64, n_hours: usize) -> Self {
        let min_mw = (2.0 * load_factor - 1.0) * peak_mw;
        let min_mw = min_mw.max(0.1 * peak_mw);
        let loads: Vec<f64> = (0..n_hours)
            .map(|i| {
                let t = i as f64 / n_hours as f64;
                min_mw + (peak_mw - min_mw) * (std::f64::consts::PI * t).sin()
            })
            .collect();
        Self::from_hourly(&loads)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reliability indices
// ─────────────────────────────────────────────────────────────────────────────

/// Generation adequacy reliability indices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityIndices {
    /// Loss of Load Probability [0, 1]
    pub lolp: f64,
    /// Loss of Load Expectation [hours/year]
    pub lole_hours: f64,
    /// Expected Unserved Energy [MWh/year]
    pub eens_mwh: f64,
    /// Installed reserve margin [%] = (capacity - peak) / peak * 100
    pub installed_reserve_margin_pct: f64,
    /// Effective reserve margin accounting for FOR [%]
    pub effective_reserve_margin_pct: f64,
    /// Total installed capacity [MW]
    pub total_capacity_mw: f64,
    /// Peak load [MW]
    pub peak_load_mw: f64,
}

impl ReliabilityIndices {
    /// Assess reliability using COPT and LDC.
    ///
    /// # Method
    /// For each COPT state (available capacity = total_cap - outage):
    ///   If available < load → contribute to LOLP weighted by probability and time
    pub fn compute(copt: &Copt, ldc: &LoadDurationCurve) -> Self {
        let n_hours = ldc.ldc.len() as f64;
        let peak = ldc.peak_load_mw;
        let total_cap = copt.total_capacity_mw;

        let mut lole_hours = 0.0_f64;
        let mut eens_mwh = 0.0_f64;
        let mut lolp = 0.0_f64;

        for entry in &copt.entries {
            let available = total_cap - entry.outage_mw;
            // Hours where load exceeds available capacity
            let h_loss = ldc.hours_exceeding(available) as f64;
            if h_loss > 0.0 {
                lole_hours += entry.probability * h_loss;
                lolp += entry.probability * (h_loss / n_hours);
                // Expected unserved energy: average of excess load × hours
                let eue: f64 = ldc
                    .ldc
                    .iter()
                    .filter(|&&l| l > available)
                    .map(|&l| l - available)
                    .sum::<f64>();
                eens_mwh += entry.probability * eue;
            }
        }

        // Expected available from COPT
        let expected_avail = copt.expected_available_capacity();
        let effective_reserve_pct = if peak > 0.0 {
            (expected_avail - peak) / peak * 100.0
        } else {
            0.0
        };
        let installed_reserve_pct = if peak > 0.0 {
            (total_cap - peak) / peak * 100.0
        } else {
            0.0
        };

        Self {
            lolp,
            lole_hours,
            eens_mwh,
            installed_reserve_margin_pct: installed_reserve_pct,
            effective_reserve_margin_pct: effective_reserve_pct,
            total_capacity_mw: total_cap,
            peak_load_mw: peak,
        }
    }

    /// Assess reliability from unit data and hourly loads.
    pub fn from_units_and_loads(units: &[ReliabilityUnit], hourly_loads: &[f64]) -> Self {
        let copt = Copt::from_units(units);
        let ldc = LoadDurationCurve::from_hourly(hourly_loads);
        Self::compute(&copt, &ldc)
    }

    /// Check NERC planning reserve margin criterion (typically 15–20%).
    pub fn meets_reserve_criterion(&self, target_reserve_pct: f64) -> bool {
        self.installed_reserve_margin_pct >= target_reserve_pct
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Capacity Credit / ELCC
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate the capacity credit (ELCC) of a new resource using LOLE matching.
///
/// The ELCC is the amount of perfectly reliable capacity that would produce
/// the same LOLE improvement as the new resource.
///
/// # Arguments
/// - `base_units`   — existing generation fleet
/// - `new_unit`     — candidate new resource (e.g., wind farm)
/// - `hourly_loads` — hourly load profile [MW]
///
/// Returns the ELCC in MW (fraction of the new unit's capacity).
pub fn capacity_credit_elcc(
    base_units: &[ReliabilityUnit],
    new_unit: &ReliabilityUnit,
    hourly_loads: &[f64],
) -> f64 {
    let ldc = LoadDurationCurve::from_hourly(hourly_loads);

    // LOLE without new unit
    let copt_base = Copt::from_units(base_units);
    let idx_base = ReliabilityIndices::compute(&copt_base, &ldc);
    let lole_base = idx_base.lole_hours;

    // LOLE with new unit
    let mut units_with_new = base_units.to_vec();
    units_with_new.push(new_unit.clone());
    let copt_with = Copt::from_units(&units_with_new);
    let idx_with = ReliabilityIndices::compute(&copt_with, &ldc);
    let lole_with = idx_with.lole_hours;

    if lole_base <= lole_with {
        return 0.0; // No improvement
    }

    // Binary search for the equivalent perfectly-reliable capacity increment
    // that produces the same LOLE reduction
    let target_lole = lole_with;
    let mut lo = 0.0_f64;
    let mut hi = new_unit.capacity_mw;

    for _ in 0..30 {
        let mid = (lo + hi) / 2.0;
        // Add a perfectly reliable unit of `mid` MW
        let reliable_unit = ReliabilityUnit::new("elcc_ref", mid, 0.0);
        let mut units_ref = base_units.to_vec();
        units_ref.push(reliable_unit);
        let copt_ref = Copt::from_units(&units_ref);
        let idx_ref = ReliabilityIndices::compute(&copt_ref, &ldc);
        if idx_ref.lole_hours > target_lole {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    (lo + hi) / 2.0
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_fleet() -> Vec<ReliabilityUnit> {
        vec![
            ReliabilityUnit::baseload("G1", 400.0),
            ReliabilityUnit::baseload("G2", 400.0),
            ReliabilityUnit::peaking("G3", 200.0),
        ]
    }

    fn simple_loads(peak: f64, n: usize) -> Vec<f64> {
        // Sinusoidal profile from 0.6*peak to peak
        (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let min = 0.6 * peak;
                min + (peak - min) * (std::f64::consts::PI * t).sin()
            })
            .collect()
    }

    #[test]
    fn test_copt_total_probability_sums_to_one() {
        let fleet = simple_fleet();
        let copt = Copt::from_units(&fleet);
        let total: f64 = copt.entries.iter().map(|e| e.probability).sum();
        assert!(
            (total - 1.0).abs() < 1e-6,
            "COPT probabilities should sum to 1: {total:.6}"
        );
    }

    #[test]
    fn test_copt_no_outage_state_dominates() {
        let fleet = simple_fleet();
        let copt = Copt::from_units(&fleet);
        // State with 0 outage (all units available) should have highest probability
        let zero_outage = copt.entries.iter().find(|e| e.outage_mw < 0.5).unwrap();
        let max_prob = copt
            .entries
            .iter()
            .map(|e| e.probability)
            .fold(0.0_f64, f64::max);
        assert!((zero_outage.probability - max_prob).abs() < 1e-10);
    }

    #[test]
    fn test_copt_expected_capacity_near_installed() {
        // All units with FOR=0 → expected capacity = installed
        let units = vec![
            ReliabilityUnit::new("G1", 100.0, 0.0),
            ReliabilityUnit::new("G2", 200.0, 0.0),
        ];
        let copt = Copt::from_units(&units);
        assert!((copt.expected_available_capacity() - 300.0).abs() < 1e-4);
    }

    #[test]
    fn test_ldc_from_hourly_sorted() {
        let loads: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let ldc = LoadDurationCurve::from_hourly(&loads);
        // LDC should be sorted descending
        for i in 1..ldc.ldc.len() {
            assert!(ldc.ldc[i] <= ldc.ldc[i - 1]);
        }
    }

    #[test]
    fn test_ldc_peak_load() {
        let loads = simple_loads(800.0, 8760);
        let ldc = LoadDurationCurve::from_hourly(&loads);
        assert!((ldc.peak_load_mw - 800.0).abs() < 1.0);
    }

    #[test]
    fn test_ldc_hours_exceeding() {
        let loads: Vec<f64> = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let ldc = LoadDurationCurve::from_hourly(&loads);
        // After sort descending: [500, 400, 300, 200, 100]
        // > 250: 500, 400, 300 → 3
        let h = ldc.hours_exceeding(250.0);
        assert_eq!(h, 3);
    }

    #[test]
    fn test_ldc_fraction_exceeding() {
        let loads: Vec<f64> = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let ldc = LoadDurationCurve::from_hourly(&loads);
        let frac = ldc.fraction_exceeding(250.0);
        assert!((frac - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_ldc_synthesise_peak_match() {
        let ldc = LoadDurationCurve::synthesise(1000.0, 0.7, 8760);
        assert!(
            (ldc.peak_load_mw - 1000.0).abs() < 50.0,
            "peak={:.1}",
            ldc.peak_load_mw
        );
    }

    #[test]
    fn test_reliability_indices_high_reserve() {
        // System with huge reserve: LOLP should be very low
        let units: Vec<ReliabilityUnit> = (0..10)
            .map(|i| ReliabilityUnit::new(format!("G{i}"), 200.0, 0.05))
            .collect();
        let loads = simple_loads(500.0, 8760); // peak 500 MW, total cap 2000 MW
        let idx = ReliabilityIndices::from_units_and_loads(&units, &loads);
        assert!(
            idx.lolp < 0.001,
            "LOLP should be tiny with large reserve: {:.6}",
            idx.lolp
        );
        assert!(idx.lole_hours < 1.0);
    }

    #[test]
    fn test_reliability_indices_tight_system() {
        // Barely enough capacity: higher LOLP
        let units = vec![ReliabilityUnit::new("G1", 800.0, 0.10)];
        let loads = simple_loads(780.0, 8760);
        let idx = ReliabilityIndices::from_units_and_loads(&units, &loads);
        // With 10% FOR on single unit, significant probability of outage at peak
        assert!(idx.lolp > 0.0, "LOLP should be positive in tight system");
        assert!(idx.eens_mwh >= 0.0);
    }

    #[test]
    fn test_reserve_margin_installed() {
        let units = simple_fleet(); // 400+400+200 = 1000 MW
        let loads = simple_loads(800.0, 8760);
        let copt = Copt::from_units(&units);
        let ldc = LoadDurationCurve::from_hourly(&loads);
        let idx = ReliabilityIndices::compute(&copt, &ldc);
        let expected_rm = (1000.0 - 800.0) / 800.0 * 100.0; // 25%
        assert!((idx.installed_reserve_margin_pct - expected_rm).abs() < 0.1);
    }

    #[test]
    fn test_reserve_criterion_met() {
        let units = simple_fleet();
        let loads = simple_loads(700.0, 8760);
        let copt = Copt::from_units(&units);
        let ldc = LoadDurationCurve::from_hourly(&loads);
        let idx = ReliabilityIndices::compute(&copt, &ldc);
        assert!(idx.meets_reserve_criterion(15.0));
    }

    #[test]
    fn test_unit_availability() {
        let unit = ReliabilityUnit {
            name: "test".into(),
            capacity_mw: 100.0,
            forced_outage_rate: 0.05,
            planned_outage_hours: 438.0, // 5% of 8760
        };
        // Availability ≈ (1-0.05) * (1-0.05) ≈ 0.9025
        assert!((unit.availability() - 0.9025).abs() < 1e-6);
    }

    #[test]
    fn test_capacity_credit_perfectly_reliable() {
        // A perfectly reliable unit (FOR=0) should have ELCC ≈ its full capacity
        let base = vec![ReliabilityUnit::new("G1", 600.0, 0.08)];
        let new_unit = ReliabilityUnit::new("New_reliable", 100.0, 0.0);
        let loads = simple_loads(650.0, 1000); // tight system
        let elcc = capacity_credit_elcc(&base, &new_unit, &loads);
        assert!(elcc > 0.0 && elcc <= 100.0, "ELCC={elcc:.2}");
    }

    #[test]
    fn test_eens_nonnegative() {
        let fleet = simple_fleet();
        let loads = simple_loads(900.0, 8760);
        let copt = Copt::from_units(&fleet);
        let ldc = LoadDurationCurve::from_hourly(&loads);
        let idx = ReliabilityIndices::compute(&copt, &ldc);
        assert!(idx.eens_mwh >= 0.0);
        assert!(idx.lole_hours >= 0.0);
    }

    #[test]
    fn test_copt_prob_capacity_below() {
        let units = vec![ReliabilityUnit::new("G1", 100.0, 0.0)]; // FOR=0 always available
        let copt = Copt::from_units(&units);
        // 100% available: prob(capacity >= 100) = 1, prob(capacity < 100) = 0
        let p = copt.prob_capacity_below(50.0); // prob available cap <= 50 → outage >= 50
        assert!(
            p < 1e-10,
            "FOR=0 unit should never be below capacity: {p:.2e}"
        );
    }

    #[test]
    fn test_single_unit_for_zero_lolp_zero() {
        // Single unit, FOR=0, always available at 200 MW; peak load 100 MW
        let units = vec![ReliabilityUnit::new("G1", 200.0, 0.0)];
        let loads = vec![100.0_f64; 8760];
        let idx = ReliabilityIndices::from_units_and_loads(&units, &loads);
        assert!(idx.lolp < 1e-12, "LOLP must be 0 when FOR=0 and cap > load");
        assert!(idx.lole_hours < 1e-12);
        assert!(idx.eens_mwh < 1e-12);
    }
}
