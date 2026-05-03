use serde::{Deserialize, Serialize};

/// Transmission line or transformer modelled as a π-circuit.
///
/// A `tap == 0.0` indicates a plain transmission line (tap defaults to 1.0).
/// Non-zero `tap` or non-zero `shift` indicates a transformer.
///
/// The π-model admittance equations (see [`crate::network::admittance`]):
/// ```text
///   Y_ii = ys / |tap|² + jb/2
///   Y_jj = ys + jb/2
///   Y_ij = -ys / tap*
///   Y_ji = -ys / tap
/// ```
/// where `ys = 1 / (r + jx)` is the series admittance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Branch {
    /// From-bus external ID.
    pub from_bus: usize,
    /// To-bus external ID.
    pub to_bus: usize,
    /// Series resistance [p.u.].
    pub r: f64,
    /// Series reactance [p.u.].
    pub x: f64,
    /// Total line-charging susceptance [p.u.] (split equally at each end).
    pub b: f64,
    /// Long-term MVA rating `MVA` (`0` = unlimited).
    pub rate_a: f64,
    /// Short-term MVA rating `MVA` (`0` = unlimited).
    pub rate_b: f64,
    /// Emergency MVA rating `MVA` (`0` = unlimited).
    pub rate_c: f64,
    /// Off-nominal turns ratio (transformer tap magnitude).  `0.0` = treated as `1.0`.
    pub tap: f64,
    /// Phase shift angle `degrees`.  `0.0` = no phase shift.
    pub shift: f64,
    /// In-service flag.  `false` = branch is open (removed from Y-bus).
    pub status: bool,
}

impl Branch {
    /// Effective tap ratio: returns `1.0` for lines (`tap == 0.0`), else `self.tap`.
    pub fn effective_tap(&self) -> f64 {
        if self.tap == 0.0 {
            1.0
        } else {
            self.tap
        }
    }

    /// Complex tap `a = t·e^{jφ}` where `t = effective_tap()`, `φ = shift` `rad`.
    pub fn tap_complex(&self) -> num_complex::Complex64 {
        let t = self.effective_tap();
        let shift_rad = self.shift.to_radians();
        num_complex::Complex64::new(t * shift_rad.cos(), t * shift_rad.sin())
    }
}
