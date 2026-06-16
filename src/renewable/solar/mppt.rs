/// Maximum Power Point Tracking (MPPT) algorithms.
///
/// Two algorithms are provided:
///
/// 1. **Perturb & Observe (P&O)** — simple, widely deployed.
///    Perturbs the operating voltage by ±ΔV and observes the resulting
///    power change; steps toward higher power each cycle.
///
/// 2. **Incremental Conductance (InC)** — better under rapidly varying
///    irradiance. Tracks the condition dP/dV = 0 by comparing the
///    incremental conductance (dI/dV) to the instantaneous conductance
///    (−I/V).
use serde::{Deserialize, Serialize};

/// State for the Perturb & Observe MPPT controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerturbObserve {
    /// Current operating voltage `V`
    pub v_op: f64,
    /// Previous power measurement `W`
    prev_power: f64,
    /// Previous voltage `V`
    prev_voltage: f64,
    /// Perturbation step size `V`
    pub delta_v: f64,
    /// Voltage limits `V`
    pub v_min: f64,
    pub v_max: f64,
}

impl PerturbObserve {
    /// Create a new P&O controller.
    ///
    /// - `v_init`  — initial operating voltage `V`
    /// - `delta_v` — perturbation step `V` (typical: 0.5–2% of V_oc)
    /// - `v_min` / `v_max` — operating voltage window `V`
    pub fn new(v_init: f64, delta_v: f64, v_min: f64, v_max: f64) -> Self {
        Self {
            v_op: v_init,
            prev_power: -f64::INFINITY,
            prev_voltage: v_init,
            delta_v,
            v_min,
            v_max,
        }
    }

    /// Update the MPPT controller given measured (`v`, `i`).
    ///
    /// Returns the new reference voltage.
    pub fn update(&mut self, v: f64, i: f64) -> f64 {
        let p = v * i;
        let dp = p - self.prev_power;
        let dv = v - self.prev_voltage;

        let direction = if dp.abs() < 1e-9 {
            0.0 // At MPP (or stagnant)
        } else if dp > 0.0 {
            if dv >= 0.0 {
                1.0
            } else {
                -1.0
            } // Moving toward MPP
        } else if dv >= 0.0 {
            -1.0 // Moving away from MPP
        } else {
            1.0
        };

        self.prev_power = p;
        self.prev_voltage = v;
        self.v_op = (v + direction * self.delta_v).clamp(self.v_min, self.v_max);
        self.v_op
    }
}

/// State for the Incremental Conductance MPPT controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncrementalConductance {
    /// Current operating voltage reference `V`
    pub v_ref: f64,
    /// Previous voltage `V`
    prev_v: f64,
    /// Previous current `A`
    prev_i: f64,
    /// Step size `V`
    pub delta_v: f64,
    /// Voltage limits `V`
    pub v_min: f64,
    pub v_max: f64,
}

impl IncrementalConductance {
    pub fn new(v_init: f64, delta_v: f64, v_min: f64, v_max: f64) -> Self {
        Self {
            v_ref: v_init,
            prev_v: v_init,
            prev_i: 0.0,
            delta_v,
            v_min,
            v_max,
        }
    }

    /// Update given measured (`v`, `i`).  Returns new reference voltage.
    ///
    /// Uses the condition: at MPP, dI/dV = −I/V.
    pub fn update(&mut self, v: f64, i: f64) -> f64 {
        let dv = v - self.prev_v;
        let di = i - self.prev_i;

        let direction = if dv.abs() < 1e-9 {
            // dV ≈ 0: use dI/dt
            if di.abs() < 1e-9 {
                0.0 // At MPP
            } else if di > 0.0 {
                1.0
            } else {
                -1.0
            }
        } else {
            let inc_cond = di / dv; // dI/dV
            let inst_cond = -i / v.max(1e-9); // −I/V  (value at MPP)
            let err = inc_cond - inst_cond;
            if err.abs() < 1e-6 {
                0.0 // At MPP
            } else if err > 0.0 {
                1.0 // Left of MPP → increase V
            } else {
                -1.0 // Right of MPP → decrease V
            }
        };

        self.prev_v = v;
        self.prev_i = i;
        self.v_ref = (v + direction * self.delta_v).clamp(self.v_min, self.v_max);
        self.v_ref
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::renewable::solar::pv_cell::{diode_current, find_mpp, SingleDiodeParams};

    fn params() -> SingleDiodeParams {
        SingleDiodeParams::crystalline_si_250w()
    }

    #[test]
    fn test_po_converges_to_mpp() {
        let p = params();
        let mpp = find_mpp(&p, 1000.0, 298.15);
        let mut ctrl = PerturbObserve::new(mpp.voltage * 0.5, 0.5, 1.0, 50.0);

        // Run many iterations
        for _ in 0..200 {
            let v = ctrl.v_op;
            let i = diode_current(&p, v, 1000.0, 298.15);
            ctrl.update(v, i);
        }

        let v_final = ctrl.v_op;
        let i_final = diode_current(&p, v_final, 1000.0, 298.15);
        let p_final = v_final * i_final;

        assert!(
            (p_final - mpp.power).abs() / mpp.power < 0.02,
            "P&O: p_final={:.2} mpp={:.2}",
            p_final,
            mpp.power
        );
    }

    #[test]
    fn test_inc_converges_to_mpp() {
        let p = params();
        let mpp = find_mpp(&p, 1000.0, 298.15);
        let mut ctrl = IncrementalConductance::new(mpp.voltage * 0.5, 0.3, 1.0, 50.0);

        for _ in 0..300 {
            let v = ctrl.v_ref;
            let i = diode_current(&p, v, 1000.0, 298.15);
            ctrl.update(v, i);
        }

        let v_final = ctrl.v_ref;
        let i_final = diode_current(&p, v_final, 1000.0, 298.15);
        let p_final = v_final * i_final;

        assert!(
            (p_final - mpp.power).abs() / mpp.power < 0.02,
            "InC: p_final={:.2} mpp={:.2}",
            p_final,
            mpp.power
        );
    }

    #[test]
    fn test_po_duty_cycle_clamped_at_max() {
        let p = params();
        let mut ctrl = PerturbObserve::new(28.0, 1.0, 1.0, 30.0);

        for _ in 0..100 {
            let v = ctrl.v_op;
            let i = diode_current(&p, v, 1000.0, 298.15);
            ctrl.update(v, i);
            assert!(
                ctrl.v_op <= 30.0,
                "v_op={:.4} exceeded v_max=30.0",
                ctrl.v_op
            );
        }
    }

    #[test]
    fn test_po_duty_cycle_clamped_at_min() {
        let mut ctrl = PerturbObserve::new(7.0, 1.0, 5.0, 50.0);

        for _ in 0..50 {
            let v = ctrl.v_op;
            ctrl.update(v, 0.0);
            assert!(
                ctrl.v_op >= 5.0,
                "v_op={:.4} went below v_min=5.0",
                ctrl.v_op
            );
        }
    }

    #[test]
    fn test_po_power_increases_after_correct_perturbation() {
        let p = params();
        let mpp = find_mpp(&p, 1000.0, 298.15);
        let start_v = mpp.voltage * 0.7;
        let mut ctrl = PerturbObserve::new(start_v, 0.5, 1.0, 50.0);
        let v_init = ctrl.v_op;

        let v = ctrl.v_op;
        let i = diode_current(&p, v, 1000.0, 298.15);
        ctrl.update(v, i);

        assert!(
            ctrl.v_op > v_init,
            "Expected upward step from below MPP: v_op={:.4} v_init={:.4}",
            ctrl.v_op,
            v_init
        );
    }

    #[test]
    fn test_po_starts_above_mpp_converges_near_mpp() {
        let p = params();
        let mpp = find_mpp(&p, 1000.0, 298.15);
        // Start 15% above MPP (not too close to Voc where I≈0 creates P&O lock-up).
        // After many iterations the delivered power must be within 5% of the true MPP.
        let start_v = mpp.voltage * 1.15;
        let mut ctrl = PerturbObserve::new(start_v, 0.5, 1.0, 50.0);

        for _ in 0..300 {
            let v = ctrl.v_op;
            let i = diode_current(&p, v, 1000.0, 298.15);
            ctrl.update(v, i);
        }

        let v_final = ctrl.v_op;
        let i_final = diode_current(&p, v_final, 1000.0, 298.15);
        let p_final = v_final * i_final;

        assert!(
            (p_final - mpp.power).abs() / mpp.power < 0.05,
            "P&O starting above MPP: p_final={:.2} mpp={:.2} (>5% error)",
            p_final,
            mpp.power
        );
    }

    #[test]
    fn test_inc_dv_zero_branch_di_positive() {
        let mut ctrl = IncrementalConductance::new(20.0, 0.5, 1.0, 50.0);
        ctrl.update(20.0, 5.0);
        let v_ref_after = ctrl.update(20.0, 6.0);
        assert!(
            v_ref_after > 20.0,
            "Expected upward step when dI>0 and dV≈0: v_ref={:.4}",
            v_ref_after
        );
    }

    #[test]
    fn test_inc_dv_zero_branch_di_negative() {
        let mut ctrl = IncrementalConductance::new(20.0, 0.5, 1.0, 50.0);
        ctrl.update(20.0, 5.0);
        let v_ref_after = ctrl.update(20.0, 4.0);
        assert!(
            v_ref_after < 20.0,
            "Expected downward step when dI<0 and dV≈0: v_ref={:.4}",
            v_ref_after
        );
    }

    #[test]
    fn test_po_oscillation_within_delta_v() {
        let p = params();
        let mpp = find_mpp(&p, 1000.0, 298.15);
        let mut ctrl = PerturbObserve::new(mpp.voltage * 0.5, 0.5, 1.0, 50.0);

        let mut last_20 = Vec::with_capacity(20);

        for iter in 0..500 {
            let v = ctrl.v_op;
            let i = diode_current(&p, v, 1000.0, 298.15);
            ctrl.update(v, i);
            if iter >= 480 {
                last_20.push(ctrl.v_op);
            }
        }

        let v_max = last_20.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let v_min = last_20.iter().cloned().fold(f64::INFINITY, f64::min);
        let peak_to_peak = v_max - v_min;

        assert!(
            peak_to_peak <= 2.0 * ctrl.delta_v + 1e-9,
            "P&O steady-state oscillation too large: p2p={:.4} limit={:.4}",
            peak_to_peak,
            2.0 * ctrl.delta_v
        );
    }
}
