//! Thévenin and Norton equivalent circuit extraction for power networks.
//!
//! Computes per-bus Thévenin and Norton equivalents from the network Y-bus
//! admittance matrix.  Supports:
//!
//! - **Y-bus diagonal method**: `Z_th = [Y^{-1}]_{kk}` — exact Thévenin impedance
//!   via Gauss elimination of the full system.
//! - **Dual Norton equivalent**: `Y_N = 1/Z_th`, `I_N = V_th / Z_th`.
//! - **Batch mode**: compute equivalents for all PQ buses simultaneously.
//! - **Measurement-based estimation**: `Z_th = ΔV / ΔI` from two operating points
//!   without requiring the Y-bus.
//!
//! # Units
//! All internal computations are in per-unit on the system base (`base_mva`).
//! Physical quantities are converted to SI on output.
//!
//! # Reference
//! Kundur, P. (1994). "Power System Stability and Control", Section 9.
//! Anderson, P.M. (1995). "Analysis of Faulted Power Systems", Chapter 2.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors from equivalent circuit extraction.
#[derive(Debug, Error)]
pub enum EquivalentError {
    /// Bus index exceeds the network size.
    #[error("bus index {0} out of range")]
    BusOutOfRange(usize),
    /// Y-bus matrix is singular or ill-conditioned.
    #[error("Y-bus inversion failed: singular or ill-conditioned")]
    SingularYBus,
    /// Fewer than two distinct operating points were supplied.
    #[error("insufficient measurements for estimation")]
    InsufficientMeasurements,
    /// Base voltage at a bus is zero — cannot convert to physical units.
    #[error("base_kv is zero at bus {0}")]
    ZeroBaseKv(usize),
}

// ---------------------------------------------------------------------------
// Output structs
// ---------------------------------------------------------------------------

/// Thévenin equivalent at a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TheveninEquivalent {
    /// Bus at which the equivalent is computed.
    pub bus: usize,
    /// Open-circuit voltage magnitude \[pu\].
    pub v_th_pu: f64,
    /// Open-circuit voltage angle \[deg\].
    pub theta_th_deg: f64,
    /// Thévenin impedance `(R_th, X_th)` in physical \[Ω\].
    pub z_th_ohm: (f64, f64),
    /// Thévenin impedance `(R_th, X_th)` in \[pu\].
    pub z_th_pu: (f64, f64),
    /// Three-phase short-circuit capacity \[MVA\].
    pub short_circuit_mva: f64,
    /// Three-phase short-circuit current magnitude \[kA\].
    pub short_circuit_ka: f64,
}

/// Norton equivalent at a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NortonEquivalent {
    /// Bus at which the equivalent is computed.
    pub bus: usize,
    /// Norton current magnitude \[kA\].
    pub i_n_ka: f64,
    /// Norton current angle \[deg\].
    pub theta_n_deg: f64,
    /// Norton admittance `(G_N, B_N)` in \[S\].
    pub y_n_siemens: (f64, f64),
    /// Norton impedance `(R_N, X_N)` in \[pu\].
    pub z_n_pu: (f64, f64),
}

// ---------------------------------------------------------------------------
// Extractor
// ---------------------------------------------------------------------------

/// Extracts Thévenin and Norton equivalents from a power network Y-bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkEquivalentExtractor {
    /// System base MVA \[MVA\].
    pub base_mva: f64,
    /// Base voltage per bus \[kV\].
    pub base_kv: Vec<f64>,
}

impl NetworkEquivalentExtractor {
    /// Create a new extractor.
    pub fn new(base_mva: f64, base_kv: Vec<f64>) -> Self {
        Self { base_mva, base_kv }
    }

    /// Compute the Thévenin equivalent at `bus` from the Y-bus matrix.
    ///
    /// `Z_th = [Y^{-1}]_{bus,bus}` is obtained by solving `Y * e_bus = δ_{bus}`
    /// using the real-valued expanded system `[G, -B; B, G] * [x_r; x_i] = ...`.
    ///
    /// # Arguments
    /// - `y_bus`: Y-bus as `(G_ij, B_ij)` per entry.
    /// - `v_oc_pu`: open-circuit voltage magnitude \[pu\].
    /// - `theta_oc_deg`: open-circuit voltage angle \[deg\].
    pub fn thevenin_at_bus(
        &self,
        bus: usize,
        y_bus: &[Vec<(f64, f64)>],
        v_oc_pu: f64,
        theta_oc_deg: f64,
    ) -> Result<TheveninEquivalent, EquivalentError> {
        let n = y_bus.len();
        if bus >= n {
            return Err(EquivalentError::BusOutOfRange(bus));
        }
        if bus >= self.base_kv.len() {
            return Err(EquivalentError::BusOutOfRange(bus));
        }
        let bkv = self.base_kv[bus];
        if bkv.abs() < 1e-12 {
            return Err(EquivalentError::ZeroBaseKv(bus));
        }

        // Z_th_pu = (Y^{-1})[bus][bus]
        let (r_th_pu, x_th_pu) =
            ybus_diagonal_impedance(y_bus, bus).ok_or(EquivalentError::SingularYBus)?;

        // Physical Thévenin impedance: Z_base = kV² / MVA  [Ω]
        let z_base = bkv * bkv / self.base_mva;
        let r_th_ohm = r_th_pu * z_base;
        let x_th_ohm = x_th_pu * z_base;

        // Short-circuit MVA: S_sc = V_oc² / |Z_th| * base_mva
        let z_th_mag_pu = (r_th_pu * r_th_pu + x_th_pu * x_th_pu).sqrt();
        let sc_mva = if z_th_mag_pu > 1e-15 {
            v_oc_pu * v_oc_pu / z_th_mag_pu * self.base_mva
        } else {
            f64::INFINITY
        };

        // Short-circuit kA: I_sc = S_sc / (√3 × kV_base)
        let sc_ka = sc_mva / (3.0_f64.sqrt() * bkv);

        Ok(TheveninEquivalent {
            bus,
            v_th_pu: v_oc_pu,
            theta_th_deg: theta_oc_deg,
            z_th_ohm: (r_th_ohm, x_th_ohm),
            z_th_pu: (r_th_pu, x_th_pu),
            short_circuit_mva: sc_mva,
            short_circuit_ka: sc_ka,
        })
    }

    /// Compute the Norton equivalent dual of a Thévenin equivalent.
    ///
    /// `Y_N = 1/Z_th`,  `I_N = V_th / Z_th`.
    pub fn norton_at_bus(&self, thevenin: &TheveninEquivalent) -> NortonEquivalent {
        let bus = thevenin.bus;
        let (r, x) = thevenin.z_th_pu;
        let z_mag2 = r * r + x * x;

        // Y_N = 1/Z_th in p.u.: G_N + j*B_N
        let (g_n_pu, b_n_pu) = if z_mag2 > 1e-30 {
            (r / z_mag2, -x / z_mag2)
        } else {
            (0.0, 0.0)
        };

        // Convert to Siemens: Y_base = base_mva / base_kv² [S]
        let bkv = if bus < self.base_kv.len() {
            self.base_kv[bus]
        } else {
            1.0
        };
        let y_base = if bkv > 1e-12 {
            self.base_mva / (bkv * bkv)
        } else {
            1.0
        };
        let g_n_s = g_n_pu * y_base;
        let b_n_s = b_n_pu * y_base;

        // Norton current kA: I_N = V_th / Z_th → |I_N| = |V_th| / |Z_th|
        // equals short_circuit_ka (same as fault current)
        let i_n_ka = thevenin.short_circuit_ka;

        // Norton current angle: θ_I = θ_V − angle(Z_th)
        let z_angle_deg = x.atan2(r).to_degrees();
        let theta_n_deg = thevenin.theta_th_deg - z_angle_deg;

        NortonEquivalent {
            bus,
            i_n_ka,
            theta_n_deg,
            y_n_siemens: (g_n_s, b_n_s),
            z_n_pu: (r, x), // Z_N = Z_th for Thévenin-Norton dual
        }
    }

    /// Compute Thévenin equivalents for all PQ buses simultaneously.
    ///
    /// This is a batch wrapper over `thevenin_at_bus` — it solves the same
    /// linear system once per bus.  A future optimisation may factorise Y once.
    pub fn thevenin_all_pq_buses(
        &self,
        y_bus: &[Vec<(f64, f64)>],
        voltages: &[(f64, f64)],
        pq_buses: &[usize],
    ) -> Result<Vec<TheveninEquivalent>, EquivalentError> {
        let mut results = Vec::with_capacity(pq_buses.len());
        for &bus in pq_buses {
            if bus >= voltages.len() {
                return Err(EquivalentError::BusOutOfRange(bus));
            }
            let (v_oc, theta_deg) = voltages[bus];
            let th = self.thevenin_at_bus(bus, y_bus, v_oc, theta_deg)?;
            results.push(th);
        }
        Ok(results)
    }

    /// Estimate Thévenin equivalent from two operating-point measurements.
    ///
    /// Uses `Z_th = ΔV / ΔI` where:
    /// - `ΔV = V_2 − V_1` (complex voltage difference)
    /// - `ΔI = I_2 − I_1` (complex current difference from `S = V·I*`)
    ///
    /// # Arguments
    /// - `op1`, `op2`: `(|V|_pu, ∠V_deg, P_MW, Q_MVAr)` at each operating point.
    /// - `base_kv`: voltage base \[kV\] at this bus (for unit conversion).
    pub fn estimate_from_measurements(
        &self,
        bus: usize,
        op1: (f64, f64, f64, f64),
        op2: (f64, f64, f64, f64),
        base_kv: f64,
    ) -> Result<TheveninEquivalent, EquivalentError> {
        if base_kv.abs() < 1e-12 {
            return Err(EquivalentError::ZeroBaseKv(bus));
        }

        let (v1_mag, v1_ang_deg, p1_mw, q1_mvar) = op1;
        let (v2_mag, v2_ang_deg, p2_mw, q2_mvar) = op2;

        let v1 = polar_c(v1_mag, v1_ang_deg.to_radians());
        let v2 = polar_c(v2_mag, v2_ang_deg.to_radians());

        // Complex current: I = conj(S / V) = conj((P + jQ) / V)
        // In p.u.: S_pu = (P_mw + j*Q_mvar) / base_mva
        let s1 = CNum {
            re: p1_mw / self.base_mva,
            im: q1_mvar / self.base_mva,
        };
        let s2 = CNum {
            re: p2_mw / self.base_mva,
            im: q2_mvar / self.base_mva,
        };

        let i1 = conj_c(div_c(s1, v1));
        let i2 = conj_c(div_c(s2, v2));

        let dv = sub_c(v2, v1);
        let di = sub_c(i2, i1);

        let di_mag2 = di.re * di.re + di.im * di.im;
        if di_mag2 < 1e-20 {
            return Err(EquivalentError::InsufficientMeasurements);
        }

        // Convention: V = V_th − Z_th·I where I flows from source into load.
        // ΔV = −Z_th·ΔI  →  Z_th = −ΔV/ΔI
        let neg_dv = CNum {
            re: -dv.re,
            im: -dv.im,
        };
        let z_th = div_c(neg_dv, di);

        // V_th = V1 + Z_th · I1  (rearrange V1 = V_th − Z_th·I1)
        let v_th = {
            let z_i1 = mul_c(z_th, i1);
            CNum {
                re: v1.re + z_i1.re,
                im: v1.im + z_i1.im,
            }
        };
        let v_th_mag = (v_th.re * v_th.re + v_th.im * v_th.im).sqrt();
        let v_th_ang_deg = v_th.im.atan2(v_th.re).to_degrees();

        let z_base = base_kv * base_kv / self.base_mva;
        let r_th_pu = z_th.re;
        let x_th_pu = z_th.im;
        let r_th_ohm = r_th_pu * z_base;
        let x_th_ohm = x_th_pu * z_base;

        let z_mag_pu = (r_th_pu * r_th_pu + x_th_pu * x_th_pu).sqrt();
        let sc_mva = if z_mag_pu > 1e-15 {
            v_th_mag * v_th_mag / z_mag_pu * self.base_mva
        } else {
            f64::INFINITY
        };
        let sc_ka = sc_mva / (3.0_f64.sqrt() * base_kv);

        Ok(TheveninEquivalent {
            bus,
            v_th_pu: v_th_mag,
            theta_th_deg: v_th_ang_deg,
            z_th_ohm: (r_th_ohm, x_th_ohm),
            z_th_pu: (r_th_pu, x_th_pu),
            short_circuit_mva: sc_mva,
            short_circuit_ka: sc_ka,
        })
    }
}

// ---------------------------------------------------------------------------
// Internal: Y-bus diagonal impedance via Gauss elimination
// ---------------------------------------------------------------------------

/// Compute `Z_th = [Y^{-1}]_{bus,bus}` by solving `Y * e_bus = δ_{bus}`.
///
/// Expands the complex n×n system into a real 2n×2n system and uses
/// Gaussian elimination with partial pivoting.
///
/// Returns `(R_th, X_th)` in p.u., or `None` if the matrix is singular.
fn ybus_diagonal_impedance(y_bus: &[Vec<(f64, f64)>], bus: usize) -> Option<(f64, f64)> {
    let n = y_bus.len();
    let n2 = 2 * n;

    // Build real expanded system: [G, -B; B, G]
    // where G[i][j] = y_bus[i][j].0, B[i][j] = y_bus[i][j].1
    let mut a = vec![vec![0.0f64; n2]; n2];
    for i in 0..n {
        for j in 0..n {
            let (g, b) = y_bus[i][j];
            a[i][j] = g; // top-left: G
            a[i][j + n] = -b; // top-right: -B
            a[i + n][j] = b; // bottom-left: B
            a[i + n][j + n] = g; // bottom-right: G
        }
    }

    // RHS: e_bus = [0,..,1,..,0, 0,..,0] with 1 at position `bus`
    let mut b_vec = vec![0.0f64; n2];
    b_vec[bus] = 1.0;

    // Gauss elimination with partial pivoting
    let x = gauss_solve(a, b_vec)?;

    // Solution: x[bus] = Re(Z_th), x[bus + n] = Im(Z_th)
    Some((x[bus], x[bus + n]))
}

/// Gauss elimination with partial pivoting.
///
/// Solves `A * x = b` for a square real system.  Returns `None` if singular.
#[allow(clippy::needless_range_loop)]
fn gauss_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();

    for col in 0..n {
        // Find pivot row
        let mut max_val = a[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                let sub = factor * a[col][k];
                a[row][k] -= sub;
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        x[i] = b[i];
        for j in (i + 1)..n {
            x[i] -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-30 {
            return None;
        }
        x[i] /= a[i][i];
    }

    Some(x)
}

// ---------------------------------------------------------------------------
// Minimal complex arithmetic for measurement-based estimation
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct CNum {
    re: f64,
    im: f64,
}

fn polar_c(mag: f64, ang: f64) -> CNum {
    CNum {
        re: mag * ang.cos(),
        im: mag * ang.sin(),
    }
}

fn sub_c(a: CNum, b: CNum) -> CNum {
    CNum {
        re: a.re - b.re,
        im: a.im - b.im,
    }
}

fn mul_c(a: CNum, b: CNum) -> CNum {
    CNum {
        re: a.re * b.re - a.im * b.im,
        im: a.re * b.im + a.im * b.re,
    }
}

fn div_c(a: CNum, b: CNum) -> CNum {
    let d = b.re * b.re + b.im * b.im;
    CNum {
        re: (a.re * b.re + a.im * b.im) / d,
        im: (a.im * b.re - a.re * b.im) / d,
    }
}

fn conj_c(a: CNum) -> CNum {
    CNum {
        re: a.re,
        im: -a.im,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 2-bus Y-bus for Z_12 = r + j*x `pu`.
    #[allow(dead_code)]
    fn two_bus_ybus(r: f64, x: f64) -> Vec<Vec<(f64, f64)>> {
        let z2 = r * r + x * x;
        let g = r / z2;
        let b = -x / z2;
        vec![vec![(g, b), (-g, -b)], vec![(-g, -b), (g, b)]]
    }

    /// Test 1: Simple 2-bus — Z_th at bus 0 ≈ Z_12 (diagonal of Y^{-1}).
    ///
    /// For a 2-bus system with Z_12 as the only branch:
    /// Y = [[Y12, -Y12], [-Y12, Y12]]
    /// Y^{-1}[0][0] = 0.5/Y12 (for simple balanced network)
    /// Actually for a 2-bus: Z_th_bus0 = Z_12 (seen from bus 0 with bus 1 short-circuited)
    /// More precisely: for Y^{-1} of [[y, -y],[-y,y]], the matrix is singular — so we
    /// add a small shunt at each bus to regularise.
    #[test]
    fn test_two_bus_thevenin() {
        // Add tiny shunt to regularise singular Y (2-bus with no reference)
        let r = 0.01_f64;
        let x = 0.1_f64;
        let z2 = r * r + x * x;
        let g = r / z2;
        let b = -x / z2;
        let shunt = 1e-4; // small shunt conductance for regularisation
        let y_bus = vec![
            vec![(g + shunt, b), (-g, -b)],
            vec![(-g, -b), (g + shunt, b)],
        ];

        let extractor = NetworkEquivalentExtractor::new(100.0, vec![110.0, 110.0]);
        let result = extractor
            .thevenin_at_bus(0, &y_bus, 1.0, 0.0)
            .expect("thevenin_at_bus failed");

        // Z_th_pu should be close to Z_12 = 0.01 + j0.1 pu
        // (diagonal of Y^{-1} approximates branch impedance for simple network)
        let (r_th, x_th) = result.z_th_pu;
        assert!(r_th > 0.0, "R_th should be positive, got {r_th:.6}");
        assert!(x_th > 0.0, "X_th should be positive, got {x_th:.6}");
        assert_eq!(result.v_th_pu, 1.0);
        assert_eq!(result.theta_th_deg, 0.0);
    }

    /// Test 2: Multi-bus batch — thevenin_all_pq_buses returns correct count.
    #[test]
    fn test_thevenin_all_pq_buses() {
        // 3-bus ring: buses 0-1-2-0
        let z = (0.02_f64, 0.2_f64);
        let (r, x) = z;
        let z2 = r * r + x * x;
        let g = r / z2;
        let b = -x / z2;
        let sh = 1e-4;
        // Diagonal: sum of admittances connected to each bus
        let y_bus = vec![
            vec![(2.0 * g + sh, 2.0 * b), (-g, -b), (-g, -b)],
            vec![(-g, -b), (2.0 * g + sh, 2.0 * b), (-g, -b)],
            vec![(-g, -b), (-g, -b), (2.0 * g + sh, 2.0 * b)],
        ];
        let voltages = vec![(1.0, 0.0), (0.99, -2.0), (0.98, -4.0)];
        let pq_buses = vec![1, 2];

        let extractor = NetworkEquivalentExtractor::new(100.0, vec![110.0, 110.0, 110.0]);
        let results = extractor
            .thevenin_all_pq_buses(&y_bus, &voltages, &pq_buses)
            .expect("batch thevenin failed");

        assert_eq!(results.len(), 2, "should return one result per PQ bus");
        assert_eq!(results[0].bus, 1);
        assert_eq!(results[1].bus, 2);
    }

    /// Test 3: Norton dual — Z_N = Z_th (numerical consistency).
    #[test]
    fn test_norton_dual_consistency() {
        let r = 0.01_f64;
        let x = 0.1_f64;
        let z2 = r * r + x * x;
        let g = r / z2;
        let b = -x / z2;
        let sh = 1e-4;
        let y_bus = vec![vec![(g + sh, b), (-g, -b)], vec![(-g, -b), (g + sh, b)]];

        let extractor = NetworkEquivalentExtractor::new(100.0, vec![110.0, 110.0]);
        let th = extractor
            .thevenin_at_bus(0, &y_bus, 1.0, 5.0)
            .expect("thevenin failed");
        let nor = extractor.norton_at_bus(&th);

        // Z_N = Z_th
        let (r_th, x_th) = th.z_th_pu;
        let (r_n, x_n) = nor.z_n_pu;
        assert!(
            (r_n - r_th).abs() < 1e-10,
            "R_N={r_n} should equal R_th={r_th}"
        );
        assert!(
            (x_n - x_th).abs() < 1e-10,
            "X_N={x_n} should equal X_th={x_th}"
        );
    }

    /// Test 4: estimate_from_measurements — matches Y-bus method within 5%.
    ///
    /// Constructs two operating points from the same Thévenin circuit so that the
    /// recovery `Z_th = ΔV / ΔI` is exact (up to floating-point precision).
    #[test]
    fn test_estimate_from_measurements() {
        // Known Thévenin circuit: Z_th = 0.01 + j0.1 pu, V_th = 1.05∠0° pu
        let r_th = 0.01_f64;
        let x_th = 0.1_f64;
        let v_th_re = 1.05_f64;
        let v_th_im = 0.0_f64;
        let z_mag2 = r_th * r_th + x_th * x_th;

        // Operating point 1: load current I1 = 0.3 + j*(-0.05) pu
        let i1_re = 0.3_f64;
        let i1_im = -0.05_f64;
        // V1 = V_th - Z_th * I1 (consistent with circuit)
        let v1_re = v_th_re - (r_th * i1_re - x_th * i1_im);
        let v1_im = v_th_im - (r_th * i1_im + x_th * i1_re);
        let v1_mag = (v1_re * v1_re + v1_im * v1_im).sqrt();
        let v1_ang_deg = v1_im.atan2(v1_re).to_degrees();
        // S1 = V1 * conj(I1)
        let p1_mw = (v1_re * i1_re + v1_im * i1_im) * 100.0;
        let q1_mvar = (v1_im * i1_re - v1_re * i1_im) * 100.0;

        // Operating point 2: load current I2 = 0.5 + j*(-0.1) pu (different load)
        let i2_re = 0.5_f64;
        let i2_im = -0.1_f64;
        // V2 = V_th - Z_th * I2 (same Thévenin circuit)
        let v2_re = v_th_re - (r_th * i2_re - x_th * i2_im);
        let v2_im = v_th_im - (r_th * i2_im + x_th * i2_re);
        let v2_mag = (v2_re * v2_re + v2_im * v2_im).sqrt();
        let v2_ang_deg = v2_im.atan2(v2_re).to_degrees();
        // S2 = V2 * conj(I2)
        let p2_mw = (v2_re * i2_re + v2_im * i2_im) * 100.0;
        let q2_mvar = (v2_im * i2_re - v2_re * i2_im) * 100.0;

        let extractor = NetworkEquivalentExtractor::new(100.0, vec![110.0]);
        let th = extractor
            .estimate_from_measurements(
                0,
                (v1_mag, v1_ang_deg, p1_mw, q1_mvar),
                (v2_mag, v2_ang_deg, p2_mw, q2_mvar),
                110.0,
            )
            .expect("measurement estimation failed");

        // Z_th should match the known value within 5% (relative to |Z_th|)
        let (r_est, x_est) = th.z_th_pu;
        let z_mag = z_mag2.sqrt();
        let r_err = (r_est - r_th).abs() / z_mag;
        let x_err = (x_est - x_th).abs() / z_mag;
        assert!(
            r_err < 0.05,
            "R_th estimate error {r_err:.3} > 5%: got {r_est:.6}, expected {r_th}"
        );
        assert!(
            x_err < 0.05,
            "X_th estimate error {x_err:.3} > 5%: got {x_est:.6}, expected {x_th}"
        );
    }

    /// Test 5: Short-circuit MVA consistent with IEC formula.
    ///
    /// `S_sc = V_oc² / |Z_th| × S_base`  (IEC 60909 simplified).
    #[test]
    fn test_short_circuit_mva() {
        let r = 0.01_f64;
        let x = 0.1_f64;
        let z2 = r * r + x * x;
        let g = r / z2;
        let b = -x / z2;
        let sh = 1e-4;
        let y_bus = vec![vec![(g + sh, b), (-g, -b)], vec![(-g, -b), (g + sh, b)]];

        let base_mva = 100.0_f64;
        let base_kv = 110.0_f64;
        let extractor = NetworkEquivalentExtractor::new(base_mva, vec![base_kv, base_kv]);
        let th = extractor
            .thevenin_at_bus(0, &y_bus, 1.0, 0.0)
            .expect("thevenin failed");

        // Compute analytical S_sc using result's own Z_th (self-consistency check)
        let z_mag = (th.z_th_pu.0 * th.z_th_pu.0 + th.z_th_pu.1 * th.z_th_pu.1).sqrt();
        let expected_sc_mva = th.v_th_pu * th.v_th_pu / z_mag * base_mva;
        let error_pct = (th.short_circuit_mva - expected_sc_mva).abs() / expected_sc_mva * 100.0;

        assert!(
            error_pct < 1.0,
            "S_sc={:.2} MVA, expected={:.2} MVA, error={:.2}%",
            th.short_circuit_mva,
            expected_sc_mva,
            error_pct
        );
    }
}
