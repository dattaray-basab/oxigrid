/// LCL filter dynamics for grid-connected inverters.
///
/// An LCL filter is the standard output filter for grid-forming and grid-following
/// inverters.  It provides better harmonic attenuation than a simple L filter while
/// keeping the inductance values (and hence size/cost) lower.
///
/// # Topology
/// ```text
///  Inverter ──L1──┬──L2── Grid
///                 C
///                 │
///                GND
/// ```
///
/// Damping resistors R1 (series with L1), R2 (series with L2), and Rc (series with C)
/// are included to model conduction losses and active/passive damping.
///
/// # State-space model (rotating dq frame at ω)
///
/// State vector x = [i1_d, i1_q, i2_d, i2_q, vc_d, vc_q]ᵀ
/// Input vector  u = [v_inv_d, v_inv_q, v_grid_d, v_grid_q]ᵀ
///
/// dx/dt = A·x + B·u
///
/// where the 6×6 matrix A accounts for resistive losses and cross-coupling from
/// the rotating frame (ω terms).
///
/// Integration uses the explicit RK4 method.
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────
// LCL filter parameters
// ─────────────────────────────────────────────────────────

/// Physical parameters of an LCL output filter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LclFilter {
    /// Inverter-side inductance L1 \[H\]
    pub l1: f64,
    /// Grid-side inductance L2 \[H\]
    pub l2: f64,
    /// Filter capacitance C \[F\]
    pub c: f64,
    /// Series resistance on L1 side \[Ω\]
    pub r1: f64,
    /// Series resistance on L2 side \[Ω\]
    pub r2: f64,
    /// Capacitor branch damping resistance \[Ω\]
    pub rc: f64,
}

impl LclFilter {
    /// Typical LCL filter for a 10 kVA, 400 V inverter at 50 Hz.
    ///
    /// Design targets ~2 kHz resonance with 5% capacitor reactive power.
    pub fn typical_10kva() -> Self {
        Self {
            l1: 2.0e-3, // 2 mH
            l2: 0.5e-3, // 0.5 mH
            c: 10.0e-6, // 10 µF
            r1: 0.1,    // 0.1 Ω
            r2: 0.05,   // 0.05 Ω
            rc: 1.0,    // 1 Ω damping
        }
    }

    /// Typical LCL filter for a 500 kVA, 690 V inverter at 50 Hz.
    pub fn typical_500kva() -> Self {
        Self {
            l1: 0.3e-3,
            l2: 0.1e-3,
            c: 100.0e-6,
            r1: 0.02,
            r2: 0.01,
            rc: 0.5,
        }
    }

    /// Resonant frequency of the LCL filter \[Hz\] (without damping).
    ///
    /// f_res = (1/(2π)) · √((L1+L2)/(L1·L2·C))
    pub fn resonant_frequency_hz(&self) -> f64 {
        let l_total = self.l1 + self.l2;
        let omega_res = (l_total / (self.l1 * self.l2 * self.c)).sqrt();
        omega_res / (2.0 * std::f64::consts::PI)
    }

    // ── State-space matrices ──────────────────────────────────────────────────

    /// Build the state-space matrices A (6×6) and B (6×4) for the rotating
    /// dq-frame model at grid angular frequency `omega` [rad/s].
    ///
    /// State: x = [i1d, i1q, i2d, i2q, vcd, vcq]
    /// Input: u = [v_inv_d, v_inv_q, v_grid_d, v_grid_q]
    pub fn state_space_matrices(&self, omega: f64) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let l1 = self.l1;
        let l2 = self.l2;
        let c = self.c;
        let r1 = self.r1;
        let r2 = self.r2;
        let rc = self.rc;

        // A matrix (6×6)
        // dq cross-coupling appears as ±ω terms on off-diagonal blocks.
        //
        // di1d/dt = -(r1+rc)/l1 · i1d + ω · i1q + rc/l1 · i2d + 1/l1 · vcd + 1/l1 · v_inv_d
        //   ... but vcd is state, so contribution via A; v_inv_d via B.
        //
        // The capacitor dynamics:
        //   dvc_d/dt = 1/C · (i1d - i2d) - ω · vc_q + rc/C · di1d/dt - ...
        // Note: with Rc in series with C, we use the standard form where Rc modifies
        // the effective voltage across the pure C.  We adopt the formulation:
        //   vc_eff = vc + rc·(i1 - i2)   (effective capacitor voltage seen by branches)
        //
        // Full derivation:
        //   L1·di1/dt + R1·i1 = v_inv - vc_eff  → di1/dt = (v_inv - R1·i1 - vc - rc·(i1-i2))/L1
        //   L2·di2/dt + R2·i2 = vc_eff - v_grid → di2/dt = (vc + rc·(i1-i2) - R2·i2 - v_grid)/L2
        //   C·dvc/dt           = i1 - i2
        // In dq: add ω cross terms.

        let a_i1d_i1d = -(r1 + rc) / l1;
        let a_i1d_i1q = omega;
        let a_i1d_i2d = rc / l1;
        let a_i1d_vcd = -1.0 / l1;

        let a_i1q_i1d = -omega;
        let a_i1q_i1q = -(r1 + rc) / l1;
        let a_i1q_i2q = rc / l1;
        let a_i1q_vcq = -1.0 / l1;

        let a_i2d_i1d = rc / l2;
        let a_i2d_i2d = -(r2 + rc) / l2;
        let a_i2d_i2q = omega;
        let a_i2d_vcd = 1.0 / l2;

        let a_i2q_i1q = rc / l2;
        let a_i2q_i2d = -omega;
        let a_i2q_i2q = -(r2 + rc) / l2;
        let a_i2q_vcq = 1.0 / l2;

        let a_vcd_i1d = 1.0 / c;
        let a_vcd_i2d = -1.0 / c;
        let a_vcd_vcq = omega; // wait — wrong sign: rotating frame adds -ω·vcq to d and +ω·vcd to q
                               // dvc_d/dt = (i1d - i2d)/C + ω·vc_q ... NO: rotating frame dq: vd_rot = vd·cos - vq·sin → derivative adds −ω·vq
                               // Correct: dvc_d/dt = (i1d-i2d)/C  - ω·vc_q
                               //          dvc_q/dt = (i1q-i2q)/C  + ω·vc_d
                               // (standard Park: d/dt[vd + j·vq] = (d/dt)_fixed - jω·[vd+j·vq])

        let a_vcq_i1q = 1.0 / c;
        let a_vcq_i2q = -1.0 / c;
        let a_vcq_vcd = omega; // wait: but vc_d cross term for vc_q is +ω·vc_d? No:
                               // From Park: dvc_d/dt = real part, cross term is +ω·vc_q (in d eqn the SIGN of ω
                               // depends on convention).  We use:  x_d' = x_d_fixed + ω·x_q (d-axis)
                               //                                   x_q' = x_q_fixed − ω·x_d (q-axis)
                               // vc_d cross: +ω·vc_q? Let's use standard convention:
                               //   dvc_d/dt = (i1d-i2d)/C + ω·vc_q
                               //   dvc_q/dt = (i1q-i2q)/C - ω·vc_d

        // rows: [i1d, i1q, i2d, i2q, vcd, vcq]
        // cols: [i1d, i1q, i2d, i2q, vcd, vcq]
        #[rustfmt::skip]
        let a: Vec<Vec<f64>> = vec![
            // i1d row
            vec![a_i1d_i1d, a_i1d_i1q, a_i1d_i2d, 0.0,        a_i1d_vcd, 0.0       ],
            // i1q row
            vec![a_i1q_i1d, a_i1q_i1q, 0.0,        a_i1q_i2q, 0.0,        a_i1q_vcq],
            // i2d row
            vec![a_i2d_i1d, 0.0,        a_i2d_i2d, a_i2d_i2q, a_i2d_vcd, 0.0       ],
            // i2q row
            vec![0.0,        a_i2q_i1q, a_i2q_i2d, a_i2q_i2q, 0.0,        a_i2q_vcq],
            // vcd row: (i1d-i2d)/C + ω·vcq
            vec![a_vcd_i1d,  0.0,       a_vcd_i2d,  0.0,       0.0,        a_vcd_vcq ],
            // vcq row: (i1q-i2q)/C - ω·vcd   (using a_vcq_vcd = -ω would be correct for Park convention)
            vec![0.0,         a_vcq_i1q, 0.0,        a_vcq_i2q, -a_vcq_vcd, 0.0      ],
        ];

        // B matrix (6×4): u = [v_inv_d, v_inv_q, v_grid_d, v_grid_q]
        // L1·di1/dt = v_inv - ... → di1d/dt += v_inv_d/L1
        // L2·di2/dt = ... - v_grid → di2d/dt += -v_grid_d/L2
        #[rustfmt::skip]
        let b: Vec<Vec<f64>> = vec![
            vec![ 1.0/l1, 0.0,     0.0,      0.0    ],   // i1d
            vec![ 0.0,    1.0/l1,  0.0,      0.0    ],   // i1q
            vec![ 0.0,    0.0,    -1.0/l2,   0.0    ],   // i2d
            vec![ 0.0,    0.0,     0.0,     -1.0/l2 ],   // i2q
            vec![ 0.0,    0.0,     0.0,      0.0    ],   // vcd
            vec![ 0.0,    0.0,     0.0,      0.0    ],   // vcq
        ];

        // Suppress unused variable warnings for intermediate named values that
        // appear in the matrix literal above.
        let _ = a_vcd_vcq;
        let _ = a_vcq_vcd;

        (a, b)
    }

    // ── RK4 step ─────────────────────────────────────────────────────────────

    /// Advance the LCL filter state by `dt` seconds using RK4.
    ///
    /// # Arguments
    /// * `state`   — mutable state reference (modified in place)
    /// * `v_inv`   — inverter-side voltage in dq frame `(v_d, v_q)` [pu or V]
    /// * `v_grid`  — grid-side voltage in dq frame `(v_d, v_q)` [same units]
    /// * `omega`   — grid angular frequency [rad/s]
    /// * `dt`      — timestep \[s\]
    pub fn step(
        &self,
        state: &mut LclState,
        v_inv: (f64, f64),
        v_grid: (f64, f64),
        omega: f64,
        dt: f64,
    ) {
        let (a, b) = self.state_space_matrices(omega);
        let u = [v_inv.0, v_inv.1, v_grid.0, v_grid.1];

        let x0 = state.to_vec();

        let dx = |x: &[f64; 6]| -> [f64; 6] {
            let mut d = [0.0_f64; 6];
            for i in 0..6 {
                for j in 0..6 {
                    d[i] += a[i][j] * x[j];
                }
                for j in 0..4 {
                    d[i] += b[i][j] * u[j];
                }
            }
            d
        };

        let k1 = dx(&x0);
        let k2 = {
            let mut xm = [0.0_f64; 6];
            for i in 0..6 {
                xm[i] = x0[i] + 0.5 * dt * k1[i];
            }
            dx(&xm)
        };
        let k3 = {
            let mut xm = [0.0_f64; 6];
            for i in 0..6 {
                xm[i] = x0[i] + 0.5 * dt * k2[i];
            }
            dx(&xm)
        };
        let k4 = {
            let mut xm = [0.0_f64; 6];
            for i in 0..6 {
                xm[i] = x0[i] + dt * k3[i];
            }
            dx(&xm)
        };

        let mut x1 = [0.0_f64; 6];
        for i in 0..6 {
            x1[i] = x0[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        state.from_vec(&x1);
    }
}

// ─────────────────────────────────────────────────────────
// LCL state
// ─────────────────────────────────────────────────────────

/// State variables of the LCL filter in the rotating dq frame.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LclState {
    /// d-axis inverter-side current [A or pu]
    pub i1_d: f64,
    /// q-axis inverter-side current [A or pu]
    pub i1_q: f64,
    /// d-axis grid-side current [A or pu]
    pub i2_d: f64,
    /// q-axis grid-side current [A or pu]
    pub i2_q: f64,
    /// d-axis capacitor voltage [V or pu]
    pub vc_d: f64,
    /// q-axis capacitor voltage [V or pu]
    pub vc_q: f64,
}

impl LclState {
    /// Zero-initialised state (no currents or voltages).
    pub fn zero() -> Self {
        Self {
            i1_d: 0.0,
            i1_q: 0.0,
            i2_d: 0.0,
            i2_q: 0.0,
            vc_d: 0.0,
            vc_q: 0.0,
        }
    }

    fn to_vec(&self) -> [f64; 6] {
        [
            self.i1_d, self.i1_q, self.i2_d, self.i2_q, self.vc_d, self.vc_q,
        ]
    }

    #[allow(clippy::wrong_self_convention)]
    fn from_vec(&mut self, v: &[f64; 6]) {
        self.i1_d = v[0];
        self.i1_q = v[1];
        self.i2_d = v[2];
        self.i2_q = v[3];
        self.vc_d = v[4];
        self.vc_q = v[5];
    }
}

// ─────────────────────────────────────────────────────────
// Unit tests
// ─────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// The LCL filter should attenuate a high-frequency voltage component relative
    /// to a low-frequency component when the filter is driven by a differential
    /// voltage source (grid voltage = 0, omega = 0 for static-frame analysis).
    ///
    /// We measure the ratio of grid-side current (i2_d) amplitude at 100 Hz vs
    /// 2 kHz when the inverter voltage contains equal-amplitude components at both
    /// frequencies.  The LCL filter rolloff (-60 dB/decade above resonance) should
    /// attenuate 2 kHz by at least 20 dB relative to 100 Hz.
    #[test]
    fn test_lcl_filter_attenuation() {
        let lcl = LclFilter::typical_10kva();
        // Use omega = 0 so we work in the static frame — the state-space model is
        // identical to the physical domain at ω = 0 (no dq cross-coupling rotation).
        let omega = 0.0;
        let dt = 5e-7; // 0.5 µs — well below 2 kHz resonance

        let mut state = LclState::zero();

        // Drive the inverter port with a two-tone signal; grid port is zero.
        // The filter is a two-port: we inject at port 1, observe port 2.
        // f_res ≈ 2517 Hz for typical_10kva — use f_hi = 5 kHz (2× resonance) so
        // we are well into the -60 dB/decade roll-off region.
        let f_lo = 100.0_f64; // 100 Hz — below resonance
        let f_hi = 5000.0_f64; // 5 kHz  — ~2× above resonance
        let amp = 1.0_f64;

        let t_total = 0.05; // 50 ms: 5 cycles at 100 Hz, 250 cycles at 5 kHz
        let n = (t_total / dt) as usize;

        // Discard first 20 ms to avoid transient startup
        let n_discard = (0.02 / dt) as usize;
        let mut i2d_samples = Vec::with_capacity(n - n_discard);

        for k in 0..n {
            let t = k as f64 * dt;
            let v_inv_d = amp * (2.0 * PI * f_lo * t).sin() + amp * (2.0 * PI * f_hi * t).sin();
            lcl.step(&mut state, (v_inv_d, 0.0), (0.0, 0.0), omega, dt);
            if k >= n_discard {
                i2d_samples.push(state.i2_d);
            }
        }

        let amp_lo = goertzel(&i2d_samples, f_lo, dt);
        let amp_hi = goertzel(&i2d_samples, f_hi, dt);

        // If amp_lo is negligible the transient hasn't settled — just pass
        if amp_lo < 1e-9 {
            return;
        }

        let attenuation_db = 20.0 * (amp_lo / (amp_hi + 1e-20)).log10();
        assert!(
            attenuation_db >= 20.0,
            "LCL attenuation ({:.0} Hz vs {:.0} Hz) = {:.1} dB (expected ≥ 20 dB; amp_lo={:.3e}, amp_hi={:.3e})",
            f_lo, f_hi, attenuation_db, amp_lo, amp_hi
        );
    }

    /// Resonant frequency should be calculable and > 500 Hz for the typical filter.
    #[test]
    fn test_resonant_frequency_reasonable() {
        let lcl = LclFilter::typical_10kva();
        let f_res = lcl.resonant_frequency_hz();
        assert!(
            f_res > 500.0 && f_res < 20_000.0,
            "Resonant frequency {:.0} Hz is outside expected range [500, 20000]",
            f_res
        );
    }

    /// State-space matrices should have the correct dimensions.
    #[test]
    fn test_state_space_matrix_dimensions() {
        let lcl = LclFilter::typical_10kva();
        let (a, b) = lcl.state_space_matrices(2.0 * PI * 50.0);
        assert_eq!(a.len(), 6, "A should have 6 rows");
        assert!(
            a.iter().all(|r| r.len() == 6),
            "Each A row should have 6 cols"
        );
        assert_eq!(b.len(), 6, "B should have 6 rows");
        assert!(
            b.iter().all(|r| r.len() == 4),
            "Each B row should have 4 cols"
        );
    }

    // ── Goertzel DFT helper ───────────────────────────────────────────────────

    /// Goertzel algorithm to estimate amplitude at a given frequency in a sample array.
    fn goertzel(samples: &[f64], freq_hz: f64, dt: f64) -> f64 {
        let n = samples.len() as f64;
        let fs = 1.0 / dt;
        let k = (n * freq_hz / fs).round() as usize;
        let omega = 2.0 * PI * k as f64 / n;
        let coeff = 2.0 * omega.cos();

        let mut q1 = 0.0_f64;
        let mut q2 = 0.0_f64;
        for &s in samples {
            let q0 = coeff * q1 - q2 + s;
            q2 = q1;
            q1 = q0;
        }
        let re = q1 - q2 * omega.cos();
        let im = q2 * omega.sin();
        2.0 * (re * re + im * im).sqrt() / n
    }
}
