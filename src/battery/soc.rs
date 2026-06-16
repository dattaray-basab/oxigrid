/// State-of-Charge estimation algorithms.
///
/// # Coulomb Counting
/// Integrates current to track SoC:
///   SoC(k+1) = SoC(k) − I·Δt / (3600·Q_n·η)
///
/// # Extended Kalman Filter (EKF)
/// State: x = `SoC`
/// Process model (Rint):
///   SoC(k+1) = SoC(k) − I·Δt / (3600·Q_n)
/// Measurement model:
///   V_meas = OCV(SoC) − I·R0
///   H = dV/dSoC = dOCV/dSoC
use crate::battery::OcvSocCurve;
use crate::units::{Current, StateOfCharge, Temperature, Voltage};
use serde::{Deserialize, Serialize};

// ── Coulomb Counting ─────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoulombCounter {
    pub soc: f64,
    pub capacity_ah: f64,
    pub coulombic_efficiency: f64,
}

impl CoulombCounter {
    pub fn new(initial_soc: f64, capacity_ah: f64) -> Self {
        Self {
            soc: initial_soc.clamp(0.0, 1.0),
            capacity_ah,
            coulombic_efficiency: 0.98,
        }
    }

    /// Integrate one time step. Returns updated SoC.
    pub fn step(&mut self, current: Current, dt: f64) -> StateOfCharge {
        let eta = if current.0 >= 0.0 {
            1.0
        } else {
            self.coulombic_efficiency
        };
        let dsoc = -current.0 * dt / (3600.0 * self.capacity_ah) * eta;
        self.soc = (self.soc + dsoc).clamp(0.0, 1.0);
        StateOfCharge::new(self.soc)
    }

    pub fn current_soc(&self) -> StateOfCharge {
        StateOfCharge::new(self.soc)
    }
}

// ── Extended Kalman Filter ───────────────────────────────────────────────────

/// Single-state EKF for SoC estimation using a Rint battery model.
///
/// State vector x = `SoC`  (scalar)
/// Input u = current `A`
/// Measurement z = terminal voltage `V`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EkfSocEstimator {
    pub ocv_curve: OcvSocCurve,
    pub r0: f64,
    pub capacity_ah: f64,

    // EKF state
    pub x: f64, // SoC estimate
    pub p: f64, // State covariance (scalar)
    pub q: f64, // Process noise variance
    pub r: f64, // Measurement noise variance
}

impl EkfSocEstimator {
    pub fn new(ocv_curve: OcvSocCurve, r0: f64, capacity_ah: f64, initial_soc: f64) -> Self {
        Self {
            ocv_curve,
            r0,
            capacity_ah,
            x: initial_soc.clamp(0.0, 1.0),
            p: 0.01, // initial uncertainty
            q: 1e-6, // process noise
            r: 1e-4, // measurement noise (V²)
        }
    }

    /// Run one EKF predict-update cycle.
    ///
    /// Returns the updated SoC estimate.
    pub fn update(
        &mut self,
        current: Current,
        v_meas: Voltage,
        dt: f64,
        _temp: Temperature,
    ) -> StateOfCharge {
        // ── Predict ──
        let dsoc = -current.0 * dt / (3600.0 * self.capacity_ah);
        let x_pred = (self.x + dsoc).clamp(0.0, 1.0);
        // Jacobian of process wrt state: F = 1 (identity for SoC)
        let p_pred = self.p + self.q;

        // ── Update ──
        // Measurement prediction
        let v_pred = self.ocv_curve.ocv(x_pred) - current.0 * self.r0;
        // Innovation
        let y = v_meas.0 - v_pred;
        // Jacobian of measurement wrt state: H = dOCV/dSoC
        let h = self.ocv_curve.docv_dsoc(x_pred);
        // Innovation covariance: S = H*P*H' + R
        let s = h * p_pred * h + self.r;
        // Kalman gain: K = P*H'/S
        let k = p_pred * h / s;

        // Updated state
        self.x = (x_pred + k * y).clamp(0.0, 1.0);
        // Updated covariance: P = (1 - K*H)*P_pred
        self.p = (1.0 - k * h) * p_pred;

        StateOfCharge::new(self.x)
    }

    pub fn soc(&self) -> StateOfCharge {
        StateOfCharge::new(self.x)
    }
}

// ── Unscented Kalman Filter ──────────────────────────────────────────────────

/// Unscented Kalman Filter for SoC — 1-state implementation.
///
/// Uses sigma points to handle nonlinearity in the OCV curve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UkfSocEstimator {
    pub ocv_curve: OcvSocCurve,
    pub r0: f64,
    pub capacity_ah: f64,
    pub x: f64,
    pub p: f64,
    pub q: f64,
    pub r: f64,
    /// UKF tuning: alpha (spread), beta (distribution), kappa (secondary)
    pub alpha: f64,
    pub beta: f64,
    pub kappa: f64,
}

impl UkfSocEstimator {
    pub fn new(ocv_curve: OcvSocCurve, r0: f64, capacity_ah: f64, initial_soc: f64) -> Self {
        Self {
            ocv_curve,
            r0,
            capacity_ah,
            x: initial_soc.clamp(0.0, 1.0),
            p: 0.01,
            q: 1e-6,
            r: 1e-4,
            alpha: 1e-3,
            beta: 2.0,
            kappa: 0.0,
        }
    }

    pub fn update(
        &mut self,
        current: Current,
        v_meas: Voltage,
        dt: f64,
        _temp: Temperature,
    ) -> StateOfCharge {
        let n = 1_f64; // state dimension
        let lambda = self.alpha * self.alpha * (n + self.kappa) - n;

        // Sigma points (scalar case: 3 points)
        let spread = ((n + lambda) * self.p).sqrt();
        let sigma = [self.x, self.x + spread, self.x - spread];

        // Weights
        let wm0 = lambda / (n + lambda);
        let wc0 = wm0 + 1.0 - self.alpha * self.alpha + self.beta;
        let w1 = 0.5 / (n + lambda);

        // Propagate sigma points through process model
        let dsoc = -current.0 * dt / (3600.0 * self.capacity_ah);
        let sigma_pred: Vec<f64> = sigma.iter().map(|&s| (s + dsoc).clamp(0.0, 1.0)).collect();

        // Predicted mean and covariance
        let x_pred = wm0 * sigma_pred[0] + w1 * sigma_pred[1] + w1 * sigma_pred[2];
        let p_pred = wc0 * (sigma_pred[0] - x_pred).powi(2)
            + w1 * (sigma_pred[1] - x_pred).powi(2)
            + w1 * (sigma_pred[2] - x_pred).powi(2)
            + self.q;

        // Propagate through measurement model
        let z_sigma: Vec<f64> = sigma_pred
            .iter()
            .map(|&s| self.ocv_curve.ocv(s) - current.0 * self.r0)
            .collect();

        let z_pred = wm0 * z_sigma[0] + w1 * z_sigma[1] + w1 * z_sigma[2];
        let s = wc0 * (z_sigma[0] - z_pred).powi(2)
            + w1 * (z_sigma[1] - z_pred).powi(2)
            + w1 * (z_sigma[2] - z_pred).powi(2)
            + self.r;

        // Cross-covariance
        let p_xz = wc0 * (sigma_pred[0] - x_pred) * (z_sigma[0] - z_pred)
            + w1 * (sigma_pred[1] - x_pred) * (z_sigma[1] - z_pred)
            + w1 * (sigma_pred[2] - x_pred) * (z_sigma[2] - z_pred);

        // Kalman gain
        let k = p_xz / s;
        let y = v_meas.0 - z_pred;

        self.x = (x_pred + k * y).clamp(0.0, 1.0);
        self.p = p_pred - k * s * k;

        StateOfCharge::new(self.x)
    }

    pub fn soc(&self) -> StateOfCharge {
        StateOfCharge::new(self.x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coulomb_counter_discharge() {
        let mut cc = CoulombCounter::new(1.0, 10.0);
        // 10A for 3600s (1C for 1h)
        let soc = cc.step(Current(10.0), 3600.0);
        assert!((soc.0 - 0.0).abs() < 0.02);
    }

    #[test]
    fn test_coulomb_counter_charge() {
        let mut cc = CoulombCounter::new(0.0, 10.0);
        let soc = cc.step(Current(-10.0), 3600.0);
        // After 1C charge for 1h with η=0.98
        assert!(soc.0 > 0.95);
    }

    #[test]
    fn test_ekf_tracks_soc() {
        let curve = OcvSocCurve::nmc_default();
        let mut ekf = EkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.8);
        let current = Current(3.0); // 1C discharge
        let dt = 1.0;

        for _ in 0..100 {
            // True terminal voltage at true SoC ≈ 0.8 (simplified)
            let v_true = Voltage(curve.ocv(ekf.x) - current.0 * 0.05);
            let soc = ekf.update(current, v_true, dt, Temperature(298.15));
            // SoC should be decreasing
            let _ = soc;
        }
        // After ~33s of 1C discharge, SoC should be slightly less than 0.8
        assert!(ekf.x < 0.8);
        assert!(ekf.x > 0.7);
    }

    #[test]
    fn test_ukf_tracks_soc() {
        let curve = OcvSocCurve::nmc_default();
        let mut ukf = UkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.8);
        let current = Current(3.0);
        let dt = 1.0;

        for _ in 0..100 {
            let v_true = Voltage(curve.ocv(ukf.x) - current.0 * 0.05);
            let soc = ukf.update(current, v_true, dt, Temperature(298.15));
            let _ = soc;
        }
        assert!(ukf.x < 0.8);
        assert!(ukf.x > 0.7);
    }

    #[test]
    fn test_coulomb_counter_clamps_to_zero() {
        // Starting at SoC=0.1, discharge at 10A for far longer than needed
        // SoC must never go below 0.0
        let mut cc = CoulombCounter::new(0.1, 10.0);
        let mut min_soc = 1.0_f64;
        for _ in 0..500 {
            let soc = cc.step(Current(10.0), 10.0);
            if soc.0 < min_soc {
                min_soc = soc.0;
            }
        }
        assert!(min_soc >= 0.0, "SoC went below 0.0: {min_soc}");
        assert!(
            cc.soc <= 0.0 + 1e-9,
            "Final SoC should be at floor: {}",
            cc.soc
        );
    }

    #[test]
    fn test_coulomb_counter_clamps_to_one() {
        // Starting at SoC=0.9, charge at -10A for far longer than needed
        // SoC must never exceed 1.0
        let mut cc = CoulombCounter::new(0.9, 10.0);
        let mut max_soc = 0.0_f64;
        for _ in 0..500 {
            let soc = cc.step(Current(-10.0), 10.0);
            if soc.0 > max_soc {
                max_soc = soc.0;
            }
        }
        assert!(max_soc <= 1.0, "SoC exceeded 1.0: {max_soc}");
        assert!(
            cc.soc >= 1.0 - 1e-9,
            "Final SoC should be at ceiling: {}",
            cc.soc
        );
    }

    #[test]
    fn test_coulomb_counter_current_soc() {
        // After a step, current_soc() must equal the returned StateOfCharge
        let mut cc = CoulombCounter::new(0.5, 20.0);
        let returned = cc.step(Current(5.0), 60.0);
        let queried = cc.current_soc();
        assert!(
            (returned.0 - queried.0).abs() < 1e-12,
            "current_soc() {:.10} != step() return {:.10}",
            queried.0,
            returned.0
        );
    }

    #[test]
    fn test_coulomb_counter_zero_current() {
        // Zero current must leave SoC unchanged
        let initial = 0.65;
        let mut cc = CoulombCounter::new(initial, 50.0);
        for _ in 0..1000 {
            let soc = cc.step(Current(0.0), 1.0);
            assert!(
                (soc.0 - initial).abs() < 1e-12,
                "SoC changed under zero current: {:.10}",
                soc.0
            );
        }
        assert!(
            (cc.soc - initial).abs() < 1e-12,
            "Internal soc field changed: {:.10}",
            cc.soc
        );
    }

    #[test]
    fn test_ekf_soc_method() {
        // .soc() must return exactly the same value as the .x field wrapped in StateOfCharge
        let curve = OcvSocCurve::nmc_default();
        let mut ekf = EkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.5);
        let current = Current(1.5);
        let dt = 1.0;
        for _ in 0..50 {
            let v_sim = Voltage(curve.ocv(ekf.x) - current.0 * 0.05);
            ekf.update(current, v_sim, dt, Temperature(298.15));
        }
        let via_method = ekf.soc();
        let via_field = StateOfCharge::new(ekf.x);
        assert!(
            (via_method.0 - via_field.0).abs() < 1e-12,
            "soc() {:.10} != StateOfCharge::new(x) {:.10}",
            via_method.0,
            via_field.0
        );
    }

    #[test]
    fn test_ukf_soc_method() {
        // .soc() must return exactly the same value as the .x field wrapped in StateOfCharge
        let curve = OcvSocCurve::nmc_default();
        let mut ukf = UkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.5);
        let current = Current(1.5);
        let dt = 1.0;
        for _ in 0..50 {
            let v_sim = Voltage(curve.ocv(ukf.x) - current.0 * 0.05);
            ukf.update(current, v_sim, dt, Temperature(298.15));
        }
        let via_method = ukf.soc();
        let via_field = StateOfCharge::new(ukf.x);
        assert!(
            (via_method.0 - via_field.0).abs() < 1e-12,
            "soc() {:.10} != StateOfCharge::new(x) {:.10}",
            via_method.0,
            via_field.0
        );
    }

    #[test]
    fn test_ekf_with_lfp_curve() {
        // EKF must work with an LFP OCV curve and keep SoC in [0, 1]
        let curve = OcvSocCurve::lfp_default();
        let mut ekf = EkfSocEstimator::new(curve.clone(), 0.03, 5.0, 0.75);
        let current = Current(5.0); // 1C discharge
        let dt = 1.0;

        for _ in 0..200 {
            let v_sim = Voltage(curve.ocv(ekf.x) - current.0 * 0.03);
            let soc = ekf.update(current, v_sim, dt, Temperature(298.15));
            assert!(
                soc.0 >= 0.0 && soc.0 <= 1.0,
                "SoC out of range [0,1]: {:.6}",
                soc.0
            );
        }
        // After ~200s of 1C discharge from 0.75, SoC should have decreased
        assert!(
            ekf.x < 0.75,
            "SoC did not decrease under discharge: {:.6}",
            ekf.x
        );
        assert!(ekf.x >= 0.0, "SoC went negative: {:.6}", ekf.x);
    }

    #[test]
    fn coulomb_counter_clamps_soc_at_zero() {
        // Start near zero, discharge hard — SoC must not go below 0.0
        let mut cc = CoulombCounter::new(0.05, 10.0);
        let soc = cc.step(Current(10.0), 3600.0);
        assert!(soc.0 >= 0.0, "SoC clamped below zero: {}", soc.0);
        assert_eq!(soc.0, 0.0, "SoC should be exactly 0.0 after over-discharge");
    }

    #[test]
    fn coulomb_counter_clamps_soc_at_one() {
        // Start near full, charge hard — SoC must not exceed 1.0
        let mut cc = CoulombCounter::new(0.95, 10.0);
        let soc = cc.step(Current(-10.0), 3600.0);
        assert!(soc.0 <= 1.0, "SoC exceeded 1.0: {}", soc.0);
        assert_eq!(soc.0, 1.0, "SoC should be exactly 1.0 after over-charge");
    }

    #[test]
    fn coulomb_counter_current_soc_matches_step() {
        let mut cc = CoulombCounter::new(0.5, 10.0);
        let soc_from_step = cc.step(Current(1.0), 360.0);
        let soc_from_getter = cc.current_soc();
        assert!(
            (soc_from_step.0 - soc_from_getter.0).abs() < 1e-10,
            "current_soc() = {} does not match step() = {}",
            soc_from_getter.0,
            soc_from_step.0
        );
    }

    #[test]
    fn coulomb_counter_coulombic_efficiency_applied_on_charge() {
        // 10A charge for 3600s on 10Ah from 0.0 → should reach ~0.98 (not 1.0)
        let mut cc = CoulombCounter::new(0.0, 10.0);
        let soc = cc.step(Current(-10.0), 3600.0);
        // With η = 0.98, charged charge = 10Ah * 0.98 = 9.8Ah → SoC = 0.98
        assert!(
            (soc.0 - 0.98).abs() < 0.005,
            "Expected SoC ≈ 0.98 with coulombic efficiency, got {}",
            soc.0
        );
    }

    #[test]
    fn ekf_covariance_decreases() {
        let curve = OcvSocCurve::nmc_default();
        let mut ekf = EkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.8);
        let initial_p = ekf.p;
        let current = Current(1.0);
        let dt = 1.0;
        for _ in 0..50 {
            let v_meas = Voltage(curve.ocv(ekf.x) - current.0 * 0.05);
            ekf.update(current, v_meas, dt, Temperature(298.15));
        }
        assert!(
            ekf.p < initial_p,
            "Covariance did not decrease: initial={}, final={}",
            initial_p,
            ekf.p
        );
    }

    #[test]
    fn ekf_soc_getter_matches_state() {
        let curve = OcvSocCurve::nmc_default();
        let mut ekf = EkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.7);
        let current = Current(1.0);
        let dt = 1.0;
        for _ in 0..10 {
            let v_meas = Voltage(curve.ocv(ekf.x) - current.0 * 0.05);
            ekf.update(current, v_meas, dt, Temperature(298.15));
        }
        let soc_via_getter = ekf.soc();
        assert!(
            (soc_via_getter.0 - ekf.x).abs() < 1e-10,
            "soc() = {} does not match x field = {}",
            soc_via_getter.0,
            ekf.x
        );
    }

    #[test]
    fn ukf_soc_clamps_at_bounds() {
        // Start near 0.0 with large discharge — SoC must not go below 0.0
        let curve = OcvSocCurve::nmc_default();
        let mut ukf = UkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.02);
        let current = Current(30.0); // large discharge
        let dt = 360.0; // 6 minutes → removes 30*360/3600=3Ah >> 0.02*3Ah left
        let v_meas = Voltage(curve.ocv(0.0_f64.max(ukf.x)));
        let soc = ukf.update(current, v_meas, dt, Temperature(298.15));
        assert!(soc.0 >= 0.0, "UKF SoC went below 0.0: {}", soc.0);
    }

    #[test]
    fn ukf_soc_getter_matches_state() {
        let curve = OcvSocCurve::nmc_default();
        let mut ukf = UkfSocEstimator::new(curve.clone(), 0.05, 3.0, 0.6);
        let current = Current(1.0);
        let dt = 1.0;
        for _ in 0..10 {
            let v_meas = Voltage(curve.ocv(ukf.x) - current.0 * 0.05);
            ukf.update(current, v_meas, dt, Temperature(298.15));
        }
        let soc_via_getter = ukf.soc();
        assert!(
            (soc_via_getter.0 - ukf.x).abs() < 1e-10,
            "soc() = {} does not match x field = {}",
            soc_via_getter.0,
            ukf.x
        );
    }
}
