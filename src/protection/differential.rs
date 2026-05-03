/// Transformer differential protection (87T).
///
/// Implements:
/// - Percentage-bias differential characteristic (operate/restrain zones)
/// - 2nd and 5th harmonic inrush restraint (IEC 60255-12)
/// - CT saturation check and cross-blocking
/// - Trip time override for severe internal faults
///
/// # Operating principle
///
/// The differential current I_diff = |I_1 + I_2| and restraint current
/// I_rst = (|I_1| + |I_2|) / 2.  An operate condition exists when:
///
///   I_diff > I_min_pu  AND  I_diff > k_slope · I_rst
///
/// The inrush blocking region checks:
///
///   I_2nd / I_diff > h2_threshold  → block (magnetising inrush)
///   I_5th / I_diff > h5_threshold  → block (overexcitation)
///
/// # References
/// - IEC 60255-12:2021 — Transformer differential protection
/// - IEEE Std C37.91-2008 — Transformer protection guide
/// - Blackburn & Domin, "Protective Relaying", 3rd ed., Ch. 10
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Parameters
// ────────────────────────────────────────────────────────────────────────────

/// Relay characteristic parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiffRelayParams {
    /// Minimum pickup threshold I_min [p.u. of transformer rating]
    pub i_min_pu: f64,
    /// Slope 1 (low-current region): k1 `fraction`
    pub slope1: f64,
    /// Slope 2 (high-current region): k2 `fraction`, applied when I_rst > i_break_pu
    pub slope2: f64,
    /// Break-point current for dual-slope [p.u.]
    pub i_break_pu: f64,
    /// 2nd harmonic ratio threshold for inrush blocking `fraction`
    pub h2_threshold: f64,
    /// 5th harmonic ratio threshold for over-excitation blocking `fraction`
    pub h5_threshold: f64,
    /// Definite trip (instantaneous) threshold for severe faults [p.u.]
    pub i_instantaneous_pu: f64,
    /// Trip time delay `s` (for operate region, excluding instantaneous)
    pub trip_delay_s: f64,
}

impl DiffRelayParams {
    /// Typical 87T relay for a power transformer (IEC 60255-12 defaults).
    pub fn standard_87t() -> Self {
        Self {
            i_min_pu: 0.20, // 20% of rated current
            slope1: 0.30,   // 30% bias in low-current region
            slope2: 0.60,   // 60% bias in high-current region
            i_break_pu: 2.0,
            h2_threshold: 0.15,      // 15% 2nd harmonic → block
            h5_threshold: 0.35,      // 35% 5th harmonic → block (overexcitation)
            i_instantaneous_pu: 8.0, // Instantaneous trip above 8× rated
            trip_delay_s: 0.02,      // 20 ms operate time
        }
    }

    /// Sensitive differential relay for generator protection.
    pub fn generator_87g() -> Self {
        Self {
            i_min_pu: 0.05,
            slope1: 0.15,
            slope2: 0.30,
            i_break_pu: 1.0,
            h2_threshold: 0.10,
            h5_threshold: 0.25,
            i_instantaneous_pu: 5.0,
            trip_delay_s: 0.015,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// CT current sample
// ────────────────────────────────────────────────────────────────────────────

/// Current sample from one side of the transformer (after CT ratio correction).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CtSample {
    /// Fundamental component [p.u. of transformer rated current]
    pub i_fundamental: f64,
    /// 2nd harmonic content [p.u.]
    pub i_2nd_harmonic: f64,
    /// 5th harmonic content [p.u.]
    pub i_5th_harmonic: f64,
    /// Raw instantaneous current [p.u.]
    pub i_instantaneous: f64,
    /// CT saturation detected?
    pub ct_saturated: bool,
}

impl CtSample {
    pub fn ideal(i_pu: f64) -> Self {
        Self {
            i_fundamental: i_pu,
            i_2nd_harmonic: 0.0,
            i_5th_harmonic: 0.0,
            i_instantaneous: i_pu,
            ct_saturated: false,
        }
    }

    pub fn with_harmonics(i_pu: f64, h2: f64, h5: f64) -> Self {
        Self {
            i_fundamental: i_pu,
            i_2nd_harmonic: h2,
            i_5th_harmonic: h5,
            i_instantaneous: i_pu,
            ct_saturated: false,
        }
    }

    pub fn with_saturation(mut self) -> Self {
        self.ct_saturated = true;
        self
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Relay decision
// ────────────────────────────────────────────────────────────────────────────

/// Outcome of one relay evaluation cycle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RelayDecision {
    /// Normal operation — no operation
    Restrain,
    /// Operate region, but blocked by harmonic restraint
    BlockedByInrush,
    /// Operate region, but blocked by over-excitation (5th harmonic)
    BlockedByOverexcitation,
    /// Operate region, but blocked due to CT saturation on external fault
    BlockedBySaturation,
    /// Trip signal issued (operate + delay)
    Trip {
        trip_time_s: f64,
        i_diff: f64,
        i_rst: f64,
    },
    /// Instantaneous (high-set) trip — severe internal fault
    InstantaneousTrip { i_diff: f64 },
}

impl RelayDecision {
    /// Returns true if a trip (any kind) was issued.
    pub fn is_trip(&self) -> bool {
        matches!(
            self,
            RelayDecision::Trip { .. } | RelayDecision::InstantaneousTrip { .. }
        )
    }

    /// Returns true if in the operate zone (regardless of blocking).
    pub fn is_operate_zone(&self) -> bool {
        !matches!(self, RelayDecision::Restrain)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Differential relay logic
// ────────────────────────────────────────────────────────────────────────────

/// Evaluate one cycle of the transformer differential relay.
///
/// - `i1` — CT sample from the primary side (CT ratio-corrected, delta-compensated)
/// - `i2` — CT sample from the secondary side
/// - Returns the relay decision.
pub fn evaluate_87t(params: &DiffRelayParams, i1: &CtSample, i2: &CtSample) -> RelayDecision {
    // Differential and restraint current (fundamental)
    let i_diff = (i1.i_fundamental + i2.i_fundamental).abs();
    let i_rst = (i1.i_fundamental.abs() + i2.i_fundamental.abs()) / 2.0;

    // Instantaneous (high-set) element — unblocked
    let i_inst = (i1.i_instantaneous + i2.i_instantaneous).abs();
    if i_inst > params.i_instantaneous_pu {
        return RelayDecision::InstantaneousTrip { i_diff: i_inst };
    }

    // Percentage-bias characteristic
    let slope = if i_rst > params.i_break_pu {
        params.slope2
    } else {
        params.slope1
    };
    let operate = i_diff > params.i_min_pu && i_diff > slope * i_rst;

    if !operate {
        return RelayDecision::Restrain;
    }

    // CT saturation blocking (external fault with CT saturation)
    if i1.ct_saturated || i2.ct_saturated {
        return RelayDecision::BlockedBySaturation;
    }

    // 2nd harmonic inrush blocking (cross-restraint: either phase blocks all)
    let h2_ratio_1 = if i1.i_fundamental > 1e-6 {
        i1.i_2nd_harmonic / i1.i_fundamental
    } else {
        0.0
    };
    let h2_ratio_2 = if i2.i_fundamental > 1e-6 {
        i2.i_2nd_harmonic / i2.i_fundamental
    } else {
        0.0
    };
    if h2_ratio_1 > params.h2_threshold || h2_ratio_2 > params.h2_threshold {
        return RelayDecision::BlockedByInrush;
    }

    // 5th harmonic over-excitation blocking
    let h5_ratio_1 = if i1.i_fundamental > 1e-6 {
        i1.i_5th_harmonic / i1.i_fundamental
    } else {
        0.0
    };
    let h5_ratio_2 = if i2.i_fundamental > 1e-6 {
        i2.i_5th_harmonic / i2.i_fundamental
    } else {
        0.0
    };
    if h5_ratio_1 > params.h5_threshold || h5_ratio_2 > params.h5_threshold {
        return RelayDecision::BlockedByOverexcitation;
    }

    // Trip (time-delayed)
    RelayDecision::Trip {
        trip_time_s: params.trip_delay_s,
        i_diff,
        i_rst,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Bias characteristic utilities
// ────────────────────────────────────────────────────────────────────────────

/// Compute the minimum differential current required to operate at a given restraint level.
///
/// Returns I_diff_min for the bias characteristic.
pub fn bias_characteristic(params: &DiffRelayParams, i_rst: f64) -> f64 {
    let slope = if i_rst > params.i_break_pu {
        params.slope2
    } else {
        params.slope1
    };
    (params.i_min_pu).max(slope * i_rst)
}

/// Generate the operate/restrain boundary curve for plotting.
///
/// Returns (I_rst, I_diff_pickup) pairs from 0 to `i_rst_max`.
pub fn bias_curve(params: &DiffRelayParams, i_rst_max: f64, n_points: usize) -> Vec<(f64, f64)> {
    (0..n_points)
        .map(|i| {
            let i_rst = i_rst_max * i as f64 / (n_points - 1).max(1) as f64;
            (i_rst, bias_characteristic(params, i_rst))
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Extended differential state machine
// ────────────────────────────────────────────────────────────────────────────

/// State of the differential relay across multiple evaluation cycles.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiffRelayState {
    /// Continuous operate zone timer `s`
    pub operate_timer_s: f64,
    /// Has tripped on this fault?
    pub has_tripped: bool,
    /// Number of evaluations in operate zone
    pub operate_count: usize,
    /// Last decision
    pub last_decision: Option<RelayDecision>,
}

impl DiffRelayState {
    pub fn new() -> Self {
        Self {
            operate_timer_s: 0.0,
            has_tripped: false,
            operate_count: 0,
            last_decision: None,
        }
    }

    /// Step the relay state machine.
    ///
    /// `dt` — sample interval `s`. Returns Some(trip) if relay just tripped.
    pub fn step(
        &mut self,
        params: &DiffRelayParams,
        i1: &CtSample,
        i2: &CtSample,
        dt: f64,
    ) -> Option<RelayDecision> {
        if self.has_tripped {
            return None; // Already tripped, waiting for reset
        }

        let decision = evaluate_87t(params, i1, i2);

        match &decision {
            RelayDecision::InstantaneousTrip { .. } => {
                self.has_tripped = true;
                self.last_decision = Some(decision.clone());
                Some(decision)
            }
            RelayDecision::Trip { trip_time_s, .. } => {
                self.operate_timer_s += dt;
                self.operate_count += 1;
                if self.operate_timer_s >= *trip_time_s {
                    self.has_tripped = true;
                    self.last_decision = Some(decision.clone());
                    Some(decision)
                } else {
                    self.last_decision = Some(decision);
                    None
                }
            }
            RelayDecision::Restrain => {
                // Reset timer on restrain
                self.operate_timer_s = 0.0;
                self.operate_count = 0;
                self.last_decision = Some(decision);
                None
            }
            _ => {
                // Blocked — partial reset
                self.operate_timer_s = (self.operate_timer_s - dt).max(0.0);
                self.last_decision = Some(decision);
                None
            }
        }
    }

    /// Reset after fault clearance.
    pub fn reset(&mut self) {
        self.operate_timer_s = 0.0;
        self.has_tripped = false;
        self.operate_count = 0;
        self.last_decision = None;
    }
}

impl Default for DiffRelayState {
    fn default() -> Self {
        Self::new()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Sensitivity analysis
// ────────────────────────────────────────────────────────────────────────────

/// Minimum internal fault current (p.u.) that the relay will detect.
///
/// For a balanced internal fault with load current I_load,
/// the differential current = I_fault, restraint ≈ I_load + I_fault/2.
/// Find the minimum I_fault such that the operate condition is met.
pub fn minimum_detectable_fault(params: &DiffRelayParams, i_load_pu: f64) -> f64 {
    // i_diff = i_fault, i_rst = i_load + i_fault/2
    // Operate if: i_fault > i_min AND i_fault > slope * (i_load + i_fault/2)
    // i_fault > slope * i_load + slope * i_fault/2
    // i_fault * (1 - slope/2) > slope * i_load
    // i_fault > slope * i_load / (1 - slope/2)
    let slope = if i_load_pu > params.i_break_pu {
        params.slope2
    } else {
        params.slope1
    };
    let denom = (1.0 - slope / 2.0).max(0.01);
    let from_bias = slope * i_load_pu / denom;
    params.i_min_pu.max(from_bias)
}

/// Coverage of the winding for a turn-to-turn fault.
///
/// For a winding with N total turns, a turn-to-turn fault involving n turns
/// produces differential current proportional to n/N.
/// Returns the minimum turn fraction detectable.
pub fn minimum_turn_fault_coverage(params: &DiffRelayParams, i_load_pu: f64) -> f64 {
    let i_min_fault = minimum_detectable_fault(params, i_load_pu);
    // Turn-to-turn fault current ≈ Vrated / (n/N * Zshort)
    // Simplified: coverage = i_min_fault / (rated_current_pu=1)
    i_min_fault.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_internal_fault_trips() {
        let params = DiffRelayParams::standard_87t();
        // Pure internal fault: I1 = 0.5, I2 = -0.5 → I_diff = 1.0 (high)
        // Model as: both CTs see current, but they don't cancel
        let i1 = CtSample::ideal(1.0);
        let i2 = CtSample::ideal(0.0); // only primary side sees fault current
        let decision = evaluate_87t(&params, &i1, &i2);
        assert!(
            decision.is_trip(),
            "Internal fault should trip: {:?}",
            decision
        );
    }

    #[test]
    fn test_external_fault_restrains() {
        let params = DiffRelayParams::standard_87t();
        // External fault: both CTs carry same current → I_diff ≈ 0
        let i1 = CtSample::ideal(3.0);
        let i2 = CtSample::ideal(-3.0); // opposite polarity (through fault)
        let decision = evaluate_87t(&params, &i1, &i2);
        // i_diff = |3.0 - 3.0| = 0 → restrain
        assert_eq!(decision, RelayDecision::Restrain);
    }

    #[test]
    fn test_inrush_blocking() {
        let params = DiffRelayParams::standard_87t();
        // Inrush: high 2nd harmonic on primary
        let i1 = CtSample::with_harmonics(1.5, 0.4, 0.0); // 2nd harmonic = 26.7%
        let i2 = CtSample::ideal(0.0);
        let decision = evaluate_87t(&params, &i1, &i2);
        assert_eq!(
            decision,
            RelayDecision::BlockedByInrush,
            "High 2nd harmonic should block: {:?}",
            decision
        );
    }

    #[test]
    fn test_overexcitation_blocking() {
        let params = DiffRelayParams::standard_87t();
        // Overexcitation: high 5th harmonic
        let i1 = CtSample::with_harmonics(1.2, 0.0, 0.5); // 5th harmonic = 41.7%
        let i2 = CtSample::ideal(0.0);
        let decision = evaluate_87t(&params, &i1, &i2);
        assert_eq!(decision, RelayDecision::BlockedByOverexcitation);
    }

    #[test]
    fn test_ct_saturation_blocking() {
        let params = DiffRelayParams::standard_87t();
        let i1 = CtSample::ideal(2.0).with_saturation();
        let i2 = CtSample::ideal(0.0);
        let decision = evaluate_87t(&params, &i1, &i2);
        assert_eq!(decision, RelayDecision::BlockedBySaturation);
    }

    #[test]
    fn test_instantaneous_trip() {
        let params = DiffRelayParams::standard_87t();
        // Severe fault: above instantaneous threshold
        let i1 = CtSample {
            i_fundamental: 5.0,
            i_2nd_harmonic: 0.0,
            i_5th_harmonic: 0.0,
            i_instantaneous: 9.0,
            ct_saturated: false,
        };
        let i2 = CtSample::ideal(0.0);
        let decision = evaluate_87t(&params, &i1, &i2);
        assert!(matches!(decision, RelayDecision::InstantaneousTrip { .. }));
    }

    #[test]
    fn test_bias_characteristic_slope1() {
        let params = DiffRelayParams::standard_87t();
        // Below break-point: slope1 = 0.30
        let pickup = bias_characteristic(&params, 1.0);
        assert!((pickup - 0.30_f64.max(params.i_min_pu)).abs() < 1e-6);
    }

    #[test]
    fn test_bias_characteristic_slope2() {
        let params = DiffRelayParams::standard_87t();
        // Above break-point: slope2 = 0.60
        let pickup = bias_characteristic(&params, 3.0);
        assert!((pickup - 1.80_f64.max(params.i_min_pu)).abs() < 1e-6);
    }

    #[test]
    fn test_bias_curve_length() {
        let params = DiffRelayParams::standard_87t();
        let curve = bias_curve(&params, 5.0, 50);
        assert_eq!(curve.len(), 50);
        // Verify monotone increasing
        for w in curve.windows(2) {
            assert!(w[1].1 >= w[0].1, "Bias curve should be non-decreasing");
        }
    }

    #[test]
    fn test_relay_state_machine_trips_after_delay() {
        let params = DiffRelayParams::standard_87t();
        let mut state = DiffRelayState::new();
        let i1 = CtSample::ideal(1.0);
        let i2 = CtSample::ideal(0.0);
        let dt = 0.005;

        let mut tripped = false;
        for _ in 0..20 {
            if let Some(dec) = state.step(&params, &i1, &i2, dt) {
                assert!(dec.is_trip());
                tripped = true;
                break;
            }
        }
        assert!(tripped, "Should have tripped after 20 ms delay");
    }

    #[test]
    fn test_relay_state_machine_resets() {
        let _params = DiffRelayParams::standard_87t();
        let mut state = DiffRelayState::new();
        state.operate_timer_s = 0.01;
        state.reset();
        assert_eq!(state.operate_timer_s, 0.0);
        assert!(!state.has_tripped);
    }

    #[test]
    fn test_minimum_detectable_fault() {
        let params = DiffRelayParams::standard_87t();
        let i_fault_min = minimum_detectable_fault(&params, 0.5);
        assert!(
            i_fault_min >= params.i_min_pu,
            "Minimum fault must exceed pickup: {:.4}",
            i_fault_min
        );
    }

    #[test]
    fn test_minimum_turn_fault_coverage() {
        let params = DiffRelayParams::standard_87t();
        let coverage = minimum_turn_fault_coverage(&params, 0.5);
        assert!(coverage > 0.0 && coverage <= 1.0);
    }

    #[test]
    fn test_below_minimum_pickup_restrains() {
        let params = DiffRelayParams::standard_87t();
        // Tiny differential below pickup
        let i1 = CtSample::ideal(0.05);
        let i2 = CtSample::ideal(0.0);
        let decision = evaluate_87t(&params, &i1, &i2);
        assert_eq!(decision, RelayDecision::Restrain);
    }

    #[test]
    fn test_generator_relay_more_sensitive() {
        let params_gen = DiffRelayParams::generator_87g();
        let params_xfmr = DiffRelayParams::standard_87t();
        assert!(params_gen.i_min_pu < params_xfmr.i_min_pu);
        assert!(params_gen.slope1 < params_xfmr.slope1);
    }
}
