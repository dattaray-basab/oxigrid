/// Battery cycle aging — Rainflow counting and Wöhler-curve degradation map.
///
/// # Overview
///
/// Battery cycle aging is driven by the cumulative damage of charge/discharge
/// cycles.  Two complementary methods are implemented:
///
/// 1. **Rainflow counting** (ASTM E1049): extracts half-cycles and full cycles
///    from an arbitrary SoC (or depth-of-discharge, DoD) profile.  Used widely
///    in fatigue analysis; adapted here for battery DoD trajectories.
///
/// 2. **Wöhler curve** (S-N curve): maps each DoD level to the number of
///    cycles to failure (N_f).  Total aging consumption:
///    D = Σ n_i / N_f(DoD_i)   (Miner's rule)
///    where n_i = counted cycles at DoD = DoD_i.
///
/// 3. **Degradation map**: 2D lookup table (DoD × SoC_avg) for more accurate
///    cycle life estimation incorporating the average SoC during cycling.
///
/// # Wöhler Curve Fits
///
/// Common empirical model:  N_f(DoD) = K · DoD^{-β}
/// with parameters fit from manufacturer cycle life data.
///
/// # References
/// - ASTM E1049-85 (2011), "Practices for Cycle Counting in Fatigue Analysis".
/// - Schmalstieg et al., "A holistic aging model for Li(NiMnCo)O2 based 18650
///   lithium-ion batteries", J. Power Sources 2014.
use serde::{Deserialize, Serialize};

/// A counted cycle from rainflow analysis.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RainflowCycle {
    /// Depth of discharge (DoD) for this cycle [0, 1]
    pub dod: f64,
    /// Mean SoC during this cycle [0, 1]
    pub mean_soc: f64,
    /// Cycle count (0.5 for half-cycle, 1.0 for full cycle)
    pub count: f64,
}

impl RainflowCycle {
    /// Effective damage contribution under Miner's rule [fraction of life].
    pub fn damage(&self, woehler: &WoehlerCurve) -> f64 {
        let n_f = woehler.cycles_to_failure(self.dod);
        if n_f <= 0.0 {
            return 0.0;
        }
        self.count / n_f
    }
}

/// Rainflow cycle counting on a SoC time series.
///
/// Extracts cycles using the 3-point (4-point) rainflow algorithm
/// (ASTM E1049 reduced-range method).
///
/// `soc_series` — time series of SoC values ∈ [0, 1].
/// Returns a list of counted cycles with DoD, mean SoC, and count.
pub fn rainflow_count(soc_series: &[f64]) -> Vec<RainflowCycle> {
    if soc_series.len() < 2 {
        return vec![];
    }

    // Extract turning points (local extrema)
    let peaks = extract_turning_points(soc_series);
    if peaks.len() < 2 {
        return vec![];
    }

    rainflow_from_peaks(&peaks)
}

/// Extract local minima and maxima (turning points) from a series.
pub fn extract_turning_points(series: &[f64]) -> Vec<f64> {
    if series.len() < 2 {
        return series.to_vec();
    }

    let mut peaks = vec![series[0]];

    for i in 1..series.len() - 1 {
        let prev = series[i - 1];
        let curr = series[i];
        let next = series[i + 1];
        if (curr >= prev && curr >= next) || (curr <= prev && curr <= next) {
            peaks.push(curr);
        }
    }

    peaks.push(
        *series
            .last()
            .expect("invariant: series non-empty checked by caller"),
    );

    // Deduplicate adjacent identical values
    peaks.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    peaks
}

/// Apply the 4-point rainflow algorithm to a sequence of turning points.
///
/// Returns closed cycles extracted from the sequence.
fn rainflow_from_peaks(peaks: &[f64]) -> Vec<RainflowCycle> {
    let mut stack: Vec<f64> = Vec::new();
    let mut cycles: Vec<RainflowCycle> = Vec::new();

    for &p in peaks {
        stack.push(p);
        // Attempt to extract cycles from top of stack
        loop {
            let n = stack.len();
            if n < 3 {
                break;
            }

            // 3-point check: ranges of last 3 points
            let x = (stack[n - 3] - stack[n - 2]).abs();
            let y = (stack[n - 2] - stack[n - 1]).abs();

            if x <= y {
                // Extract cycle of range x
                let s_lo = stack[n - 3].min(stack[n - 2]);
                let s_hi = stack[n - 3].max(stack[n - 2]);
                let dod = s_hi - s_lo;
                let mean_soc = (s_hi + s_lo) / 2.0;

                if dod > 1e-6 {
                    cycles.push(RainflowCycle {
                        dod,
                        mean_soc,
                        count: 1.0,
                    });
                }

                // Remove the two points that formed the cycle
                let p_last = stack.remove(n - 1);
                stack.remove(n - 3);
                stack.remove(stack.len() - 1);
                stack.push(p_last);
            } else {
                break;
            }
        }
    }

    // Drain remaining stack as half-cycles
    let n = stack.len();
    for i in 0..n.saturating_sub(1) {
        let s_lo = stack[i].min(stack[i + 1]);
        let s_hi = stack[i].max(stack[i + 1]);
        let dod = s_hi - s_lo;
        let mean_soc = (s_hi + s_lo) / 2.0;
        if dod > 1e-6 {
            cycles.push(RainflowCycle {
                dod,
                mean_soc,
                count: 0.5,
            });
        }
    }

    cycles
}

/// Wöhler (S-N) curve: cycles-to-failure vs. depth of discharge.
///
/// Model: N_f(DoD) = K · DoD^{-β}
/// Parameters K and β are fit from manufacturer cycle life data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WoehlerCurve {
    /// Scale constant K `cycles`
    pub k: f64,
    /// Shape exponent β (typical 1.5–3.0)
    pub beta: f64,
    /// Minimum DoD below which cycles are not damaging
    pub dod_min: f64,
}

impl WoehlerCurve {
    /// LFP cell typical parameters (shallow cycling tolerant).
    pub fn lfp_typical() -> Self {
        Self {
            k: 5_000.0,
            beta: 1.8,
            dod_min: 0.01,
        }
    }

    /// NMC/NCM cell typical parameters.
    pub fn nmc_typical() -> Self {
        Self {
            k: 2_000.0,
            beta: 2.0,
            dod_min: 0.01,
        }
    }

    /// LTO (lithium titanate) — very high cycle life.
    pub fn lto_typical() -> Self {
        Self {
            k: 20_000.0,
            beta: 1.5,
            dod_min: 0.01,
        }
    }

    /// Custom curve.
    pub fn custom(k: f64, beta: f64) -> Self {
        Self {
            k,
            beta,
            dod_min: 0.005,
        }
    }

    /// Cycles to failure at given DoD.
    pub fn cycles_to_failure(&self, dod: f64) -> f64 {
        let dod = dod.max(self.dod_min);
        self.k * dod.powf(-self.beta)
    }

    /// DoD at which the cell reaches N cycles of life.
    pub fn max_dod_for_cycles(&self, n_cycles: f64) -> f64 {
        if n_cycles <= 0.0 {
            return 0.0;
        }
        (self.k / n_cycles).powf(1.0 / self.beta)
    }

    /// Compute the Miner's rule damage from a list of rainflow cycles [0, 1].
    pub fn miners_damage(&self, cycles: &[RainflowCycle]) -> f64 {
        cycles.iter().map(|c| c.damage(self)).sum()
    }

    /// Remaining useful life fraction [0, 1] given accumulated damage.
    pub fn remaining_life_fraction(&self, total_damage: f64) -> f64 {
        (1.0 - total_damage).max(0.0)
    }
}

/// Degradation map: 2D cycle life as a function of DoD and mean SoC.
///
/// Accounts for the SoC-dependent stress factor: high SoC → shorter life.
/// Interpolates between tabulated (DoD, SoC_avg) → N_f data points.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationMap {
    /// DoD breakpoints [0, 1]
    pub dod_pts: Vec<f64>,
    /// Average SoC breakpoints [0, 1]
    pub soc_pts: Vec<f64>,
    /// N_f values: `cycles_to_failure[dod_idx][soc_idx]`
    pub n_f: Vec<Vec<f64>>,
}

impl DegradationMap {
    /// Build from a Wöhler curve with SoC correction factor.
    ///
    /// SoC stress factor: f(SoC_avg) = exp(κ · (SoC_avg − 0.5))
    /// High SoC → more stress → lower N_f.
    pub fn from_woehler_with_soc(
        woehler: &WoehlerCurve,
        dod_pts: Vec<f64>,
        soc_pts: Vec<f64>,
        kappa: f64,
    ) -> Self {
        let n_dod = dod_pts.len();
        let n_soc = soc_pts.len();
        let mut n_f = vec![vec![0.0f64; n_soc]; n_dod];

        for (i, &dod) in dod_pts.iter().enumerate() {
            let base = woehler.cycles_to_failure(dod);
            for (j, &soc) in soc_pts.iter().enumerate() {
                let stress = (kappa * (soc - 0.5)).exp();
                n_f[i][j] = base / stress;
            }
        }

        Self {
            dod_pts,
            soc_pts,
            n_f,
        }
    }

    /// Interpolate N_f at arbitrary (dod, soc_avg).
    pub fn cycles_to_failure(&self, dod: f64, soc_avg: f64) -> f64 {
        let dod = dod.clamp(0.0, 1.0);
        let soc = soc_avg.clamp(0.0, 1.0);

        // Find bracket indices
        let (di, df) = interp_idx(&self.dod_pts, dod);
        let (si, sf) = interp_idx(&self.soc_pts, soc);

        // Bilinear interpolation
        let n00 = self.n_f[di][si];
        let n10 = self.n_f[(di + 1).min(self.n_f.len() - 1)][si];
        let n01 = self.n_f[di][(si + 1).min(self.n_f[di].len() - 1)];
        let n11 = self.n_f[(di + 1).min(self.n_f.len() - 1)][(si + 1).min(self.n_f[di].len() - 1)];

        let n0 = n00 + df * (n10 - n00);
        let n1 = n01 + df * (n11 - n01);

        n0 + sf * (n1 - n0)
    }

    /// Miner's rule damage using the 2D map.
    pub fn miners_damage_2d(&self, cycles: &[RainflowCycle]) -> f64 {
        cycles
            .iter()
            .map(|c| {
                let n_f = self.cycles_to_failure(c.dod, c.mean_soc);
                if n_f <= 0.0 {
                    0.0
                } else {
                    c.count / n_f
                }
            })
            .sum()
    }
}

/// Linear interpolation index: returns (lower_idx, fractional_position).
fn interp_idx(pts: &[f64], x: f64) -> (usize, f64) {
    if pts.len() <= 1 {
        return (0, 0.0);
    }
    if x <= pts[0] {
        return (0, 0.0);
    }
    if x >= pts[pts.len() - 1] {
        return (pts.len() - 2, 1.0);
    }

    let pos = pts.partition_point(|&p| p <= x);
    let lo = pos.saturating_sub(1);
    let hi = pos.min(pts.len() - 1);
    let frac = if (pts[hi] - pts[lo]).abs() > 1e-12 {
        (x - pts[lo]) / (pts[hi] - pts[lo])
    } else {
        0.0
    };

    (lo, frac)
}

/// Battery cycle life estimator that accumulates damage over a multi-day profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleLifeEstimator {
    pub woehler: WoehlerCurve,
    pub total_damage: f64,
    pub total_cycles: f64,
    /// SoC history buffer (cleared after each call to `process_profile`)
    soc_buffer: Vec<f64>,
}

impl CycleLifeEstimator {
    pub fn new(woehler: WoehlerCurve) -> Self {
        Self {
            woehler,
            total_damage: 0.0,
            total_cycles: 0.0,
            soc_buffer: Vec::new(),
        }
    }

    /// Append SoC samples to the internal buffer.
    pub fn append_soc(&mut self, soc_samples: &[f64]) {
        self.soc_buffer.extend_from_slice(soc_samples);
    }

    /// Process the current SoC buffer: run rainflow counting and accumulate damage.
    ///
    /// Returns the damage accumulated this call.
    pub fn process_buffer(&mut self) -> f64 {
        if self.soc_buffer.len() < 2 {
            return 0.0;
        }

        let cycles = rainflow_count(&self.soc_buffer);
        let damage: f64 = cycles.iter().map(|c| c.damage(&self.woehler)).sum();
        let cycle_count: f64 = cycles.iter().map(|c| c.count).sum();

        self.total_damage += damage;
        self.total_cycles += cycle_count;
        self.soc_buffer.clear();

        damage
    }

    /// Equivalent full cycles (EFC) = total DoD·count / 1.0 (normalised to 100% DoD).
    pub fn equivalent_full_cycles(&self) -> f64 {
        self.total_cycles // approximate
    }

    /// Remaining life fraction [0, 1].
    pub fn remaining_life(&self) -> f64 {
        (1.0 - self.total_damage).max(0.0)
    }

    /// State of health (SoH) estimate based on cycle damage [0, 1].
    pub fn state_of_health(&self) -> f64 {
        self.remaining_life()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn simple_soc() -> Vec<f64> {
        // One full cycle: charge from 0.2 to 0.9 then discharge to 0.2
        let mut v = Vec::new();
        for i in 0..=70 {
            v.push(0.2 + i as f64 / 100.0);
        }
        for i in (0..=70).rev() {
            v.push(0.2 + i as f64 / 100.0);
        }
        v
    }

    #[test]
    fn test_turning_points_simple() {
        let s = vec![0.5, 0.9, 0.2, 0.8, 0.3];
        let peaks = extract_turning_points(&s);
        // Should have: 0.5 (start), 0.9 (max), 0.2 (min), 0.8 (max), 0.3 (end)
        assert!(
            peaks.len() >= 3,
            "Expected at least 3 turning points: {:?}",
            peaks
        );
    }

    #[test]
    fn test_rainflow_simple_cycle() {
        let soc = simple_soc();
        let cycles = rainflow_count(&soc);
        // Should detect at least one cycle of DoD ≈ 0.7
        assert!(!cycles.is_empty(), "Should count at least one cycle");
        let max_dod = cycles.iter().map(|c| c.dod).fold(0.0f64, f64::max);
        assert!(max_dod > 0.5, "Max DoD should be ~0.7: {:.3}", max_dod);
    }

    #[test]
    fn test_rainflow_constant_soc_no_cycles() {
        let soc = vec![0.5; 100];
        let cycles = rainflow_count(&soc);
        // No variation → no cycles
        assert!(cycles.is_empty(), "Constant SoC should yield no cycles");
    }

    #[test]
    fn test_rainflow_half_cycle() {
        // Only charge, no discharge → half cycle
        let soc: Vec<f64> = (0..=100).map(|i| i as f64 / 100.0).collect();
        let cycles = rainflow_count(&soc);
        let total_count: f64 = cycles.iter().map(|c| c.count).sum();
        assert!(
            total_count > 0.0,
            "Should count half cycles: {}",
            total_count
        );
    }

    #[test]
    fn test_woehler_lfp_reasonable_life() {
        let curve = WoehlerCurve::lfp_typical();
        let n_100 = curve.cycles_to_failure(1.0);
        let n_50 = curve.cycles_to_failure(0.5);
        let n_20 = curve.cycles_to_failure(0.2);
        // Shallow cycling → more cycles
        assert!(
            n_20 > n_50,
            "N_f(20%) > N_f(50%): {:.0} > {:.0}",
            n_20,
            n_50
        );
        assert!(
            n_50 > n_100,
            "N_f(50%) > N_f(100%): {:.0} > {:.0}",
            n_50,
            n_100
        );
        // LFP at 100% DoD should be ~5000 (from K=5000, beta=1.8)
        assert!(
            n_100 > 1000.0 && n_100 < 100_000.0,
            "N_f at 100%: {:.0}",
            n_100
        );
    }

    #[test]
    fn test_woehler_nmc_less_than_lfp() {
        let lfp = WoehlerCurve::lfp_typical();
        let nmc = WoehlerCurve::nmc_typical();
        // LFP is more cycle-tolerant
        let nf_lfp = lfp.cycles_to_failure(0.8);
        let nf_nmc = nmc.cycles_to_failure(0.8);
        assert!(
            nf_lfp > nf_nmc,
            "LFP should outlast NMC: {:.0} > {:.0}",
            nf_lfp,
            nf_nmc
        );
    }

    #[test]
    fn test_miners_rule_damage_one_cycle() {
        let curve = WoehlerCurve::lfp_typical();
        let cycles = vec![RainflowCycle {
            dod: 1.0,
            mean_soc: 0.5,
            count: 1.0,
        }];
        let damage = curve.miners_damage(&cycles);
        let expected = 1.0 / curve.cycles_to_failure(1.0);
        assert_abs_diff_eq!(damage, expected, epsilon = 1e-12);
    }

    #[test]
    fn test_miners_rule_damage_zero_dod() {
        let curve = WoehlerCurve::lfp_typical();
        // DoD below threshold → very small damage
        let cycles = vec![RainflowCycle {
            dod: 0.0,
            mean_soc: 0.5,
            count: 100.0,
        }];
        let damage = curve.miners_damage(&cycles);
        assert!(
            damage < 1.0,
            "Zero DoD should not exhaust life: {:.6}",
            damage
        );
    }

    #[test]
    fn test_degradation_map_bilinear() {
        let curve = WoehlerCurve::lfp_typical();
        let dod_pts = vec![0.2, 0.5, 0.8, 1.0];
        let soc_pts = vec![0.2, 0.5, 0.8];
        let map = DegradationMap::from_woehler_with_soc(&curve, dod_pts, soc_pts, 1.0);

        // High SoC → lower N_f
        let nf_low_soc = map.cycles_to_failure(0.5, 0.2);
        let nf_high_soc = map.cycles_to_failure(0.5, 0.8);
        assert!(
            nf_low_soc > nf_high_soc,
            "Low SoC should have better life: {:.0} > {:.0}",
            nf_low_soc,
            nf_high_soc
        );
    }

    #[test]
    fn test_cycle_life_estimator_accumulates() {
        let estimator_init = CycleLifeEstimator::new(WoehlerCurve::lfp_typical());
        let mut est = estimator_init;
        est.append_soc(&simple_soc());
        let damage = est.process_buffer();
        assert!(damage > 0.0, "Should accumulate damage: {:.6e}", damage);
        assert_abs_diff_eq!(est.total_damage, damage, epsilon = 1e-12);
    }

    #[test]
    fn test_cycle_life_estimator_remaining_life() {
        let mut est = CycleLifeEstimator::new(WoehlerCurve::lfp_typical());
        // Apply many cycles
        for _ in 0..10 {
            est.append_soc(&simple_soc());
            est.process_buffer();
        }
        let rl = est.remaining_life();
        assert!(
            (0.0..=1.0).contains(&rl),
            "Remaining life out of range: {:.4}",
            rl
        );
        assert!(rl < 1.0, "After cycling, remaining life should decrease");
    }

    #[test]
    fn test_cycle_life_estimator_soh() {
        let mut est = CycleLifeEstimator::new(WoehlerCurve::nmc_typical());
        est.append_soc(&simple_soc());
        est.process_buffer();
        let soh = est.state_of_health();
        assert!(soh > 0.9 && soh <= 1.0, "SoH after one cycle: {:.6}", soh);
    }

    #[test]
    fn test_max_dod_for_cycles() {
        let curve = WoehlerCurve::lfp_typical();
        let target_cycles = 1000.0;
        let max_dod = curve.max_dod_for_cycles(target_cycles);
        // N_f at this DoD should equal target_cycles
        let n_f = curve.cycles_to_failure(max_dod);
        assert!(
            (n_f - target_cycles).abs() / target_cycles < 0.01,
            "max_dod={:.4}, N_f={:.1} (target={})",
            max_dod,
            n_f,
            target_cycles
        );
    }
}
