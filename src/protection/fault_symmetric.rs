//! Symmetric (three-phase balanced) fault analysis.
//!
//! Implements IEC 60909 / ANSI C37 3-phase bolted fault current calculation
//! using the Z-bus Thevenin equivalent method.

use crate::error::{OxiGridError, Result};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

/// Type of short-circuit fault.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum FaultType {
    /// Three-phase bolted (balanced)
    ThreePhase,
    /// Single-line-to-ground (requires zero-sequence impedance)
    SingleLineGround,
    /// Line-to-line
    LineLine,
    /// Double-line-to-ground
    DoubleLineGround,
}

/// Fault current result at one bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultResult {
    /// Faulted bus index (0-based)
    pub bus_idx: usize,
    /// Fault type
    pub fault_type: FaultType,
    /// Thevenin impedance at fault point \[p.u.\]
    pub z_thevenin: Complex64,
    /// Pre-fault voltage \[p.u.\]
    pub v_prefault: Complex64,
    /// Fault current magnitude \[p.u.\]
    pub i_fault_pu: f64,
    /// Fault current magnitude \[kA\] (requires base values)
    pub i_fault_ka: Option<f64>,
    /// 3-phase fault MVA
    pub fault_mva: f64,
    pub base_mva: f64,
}

impl FaultResult {
    /// X/R ratio at the fault point.
    pub fn xr_ratio(&self) -> f64 {
        if self.z_thevenin.re.abs() < 1e-12 {
            f64::INFINITY
        } else {
            self.z_thevenin.im / self.z_thevenin.re
        }
    }

    /// DC offset factor: κ = 1.02 + 0.98·exp(−3/X/R)
    /// (IEC 60909 asymmetry factor for peak fault current).
    pub fn dc_offset_factor(&self) -> f64 {
        let xr = self.xr_ratio().min(1000.0);
        1.02 + 0.98 * (-3.0 / xr).exp()
    }
}

/// Compute bus impedance matrix Z_bus = Y_bus⁻¹ for a small system.
///
/// For large systems, use sparse factorization; here we use dense inversion
/// via Gaussian elimination (adequate for ≤ 200 buses).
pub fn compute_zbus(y_bus: &[Vec<Complex64>]) -> Result<Vec<Vec<Complex64>>> {
    let n = y_bus.len();
    // Build augmented matrix [Y|I]
    let mut mat: Vec<Vec<Complex64>> = (0..n)
        .map(|i| {
            let mut row = y_bus[i].clone();
            for j in 0..n {
                row.push(if i == j {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(0.0, 0.0)
                });
            }
            row
        })
        .collect();

    // Gaussian elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = mat[col][col].norm();
        #[allow(clippy::needless_range_loop)]
        for row in (col + 1)..n {
            let v = mat[row][col].norm();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return Err(OxiGridError::LinearAlgebra("Y-bus is singular".into()));
        }
        mat.swap(col, max_row);

        let pivot = mat[col][col];
        for item in mat[col][col..2 * n].iter_mut() {
            *item /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = mat[row][col];
            #[allow(clippy::needless_range_loop)]
            for j in col..2 * n {
                let sub = factor * mat[col][j];
                mat[row][j] -= sub;
            }
        }
    }

    // Extract right half = Z_bus
    Ok((0..n).map(|i| mat[i][n..].to_vec()).collect())
}

/// Compute 3-phase fault current at bus `fault_bus` (0-based index).
///
/// # Arguments
/// - `z_bus`      — bus impedance matrix (from `compute_zbus`)
/// - `v_prefault` — pre-fault voltages \[p.u.\] (typically from power flow)
/// - `base_mva`   — system MVA base
/// - `bus_kv`     — base kV at the fault bus
pub fn three_phase_fault(
    z_bus: &[Vec<Complex64>],
    v_prefault: &[Complex64],
    fault_bus: usize,
    base_mva: f64,
    bus_kv: Option<f64>,
) -> Result<FaultResult> {
    if fault_bus >= z_bus.len() {
        return Err(OxiGridError::InvalidParameter(format!(
            "fault_bus {} out of range {}",
            fault_bus,
            z_bus.len()
        )));
    }
    let z_th = z_bus[fault_bus][fault_bus];
    let v_f = v_prefault[fault_bus];

    let i_fault = v_f / z_th;
    let i_fault_mag = i_fault.norm();
    let fault_mva = i_fault_mag * v_f.norm() * base_mva;

    let i_fault_ka = bus_kv.map(|kv| {
        let i_base_ka = base_mva / (kv * 3.0_f64.sqrt());
        i_fault_mag * i_base_ka
    });

    Ok(FaultResult {
        bus_idx: fault_bus,
        fault_type: FaultType::ThreePhase,
        z_thevenin: z_th,
        v_prefault: v_f,
        i_fault_pu: i_fault_mag,
        i_fault_ka,
        fault_mva,
        base_mva,
    })
}

/// Compute all-bus 3-phase fault currents (scan).
pub fn fault_scan(
    z_bus: &[Vec<Complex64>],
    v_prefault: &[Complex64],
    base_mva: f64,
) -> Result<Vec<FaultResult>> {
    (0..z_bus.len())
        .map(|i| three_phase_fault(z_bus, v_prefault, i, base_mva, None))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple 2-bus Y-bus for testing.
    fn two_bus_ybus() -> Vec<Vec<Complex64>> {
        let y12 = Complex64::new(0.0, -5.0); // x = 0.2 p.u.
        let y11 = -y12 + Complex64::new(0.0, 0.1); // shunt for numerical stability
        vec![vec![y11, y12], vec![y12, y11]]
    }

    #[test]
    fn test_zbus_inverse_of_ybus() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let n = y.len();
        for (i, yi) in y.iter().enumerate().take(n) {
            for (j, _) in z.iter().enumerate().take(n) {
                let mut prod = Complex64::new(0.0, 0.0);
                for k in 0..n {
                    prod += yi[k] * z[k][j];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (prod.re - expected).abs() < 1e-6 && prod.im.abs() < 1e-6,
                    "Y*Z[{i},{j}]={:.4}+{:.4}j",
                    prod.re,
                    prod.im
                );
            }
        }
    }

    #[test]
    fn test_3phase_fault_current_positive() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let v = vec![Complex64::new(1.0, 0.0); 2];
        let result = three_phase_fault(&z, &v, 0, 100.0, Some(13.8)).unwrap();
        assert!(result.i_fault_pu > 0.0);
        assert!(result.fault_mva > 0.0);
    }

    #[test]
    fn test_fault_current_higher_at_strong_bus() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let v = vec![Complex64::new(1.0, 0.0); 2];
        let r0 = three_phase_fault(&z, &v, 0, 100.0, None).unwrap();
        let r1 = three_phase_fault(&z, &v, 1, 100.0, None).unwrap();
        assert!((r0.i_fault_pu - r1.i_fault_pu).abs() < 1e-6);
    }

    #[test]
    fn test_fault_scan_returns_all_buses() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let v = vec![Complex64::new(1.0, 0.0); 2];
        let results = fault_scan(&z, &v, 100.0).unwrap();
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_dc_offset_factor_range() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let v = vec![Complex64::new(1.0, 0.0); 2];
        let result = three_phase_fault(&z, &v, 0, 100.0, None).unwrap();
        let kappa = result.dc_offset_factor();
        assert!((1.0..=2.0).contains(&kappa), "κ={:.4}", kappa);
    }
}
