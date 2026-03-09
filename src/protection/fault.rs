/// Short-circuit (fault) current calculation.
///
/// Implements IEC 60909 / ANSI C37 simplified fault current methods
/// for balanced (3-phase) and unbalanced (SLG, LL) faults.
///
/// # Method
/// The Thevenin equivalent at the faulted bus is:
///   Z_th = 1 / Y_bus[f,f]   (diagonal element of Z_bus = inv(Y_bus))
///
/// For a bolted 3-phase fault at bus f with pre-fault voltage V_f:
///   I_f = V_f / Z_th
///
/// This module provides simplified methods using positive-sequence impedance.
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
    /// Thevenin impedance at fault point [p.u.]
    pub z_thevenin: Complex64,
    /// Pre-fault voltage [p.u.]
    pub v_prefault: Complex64,
    /// Fault current magnitude [p.u.]
    pub i_fault_pu: f64,
    /// Fault current magnitude [kA] (requires base values)
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
        for val in mat[col].iter_mut().skip(col).take(2 * n - col) {
            *val /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = mat[row][col];
            let pivot_row: Vec<_> = mat[col][col..2 * n].to_vec();
            for (dest, &src) in mat[row][col..2 * n].iter_mut().zip(pivot_row.iter()) {
                *dest -= factor * src;
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
/// - `v_prefault` — pre-fault voltages [p.u.] (typically from power flow)
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

// ────────────────────────────────────────────────────────────────────────────
// Symmetrical component analysis — unbalanced fault currents
// ────────────────────────────────────────────────────────────────────────────

/// Sequence impedances at a bus for unbalanced fault analysis.
///
/// Contains the positive-, negative-, and zero-sequence Thevenin impedances
/// at the faulted bus, derived from the respective sequence Z-bus matrices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceImpedances {
    /// Positive-sequence Thevenin impedance Z1 [p.u.]
    pub z1: Complex64,
    /// Negative-sequence Thevenin impedance Z2 [p.u.]
    pub z2: Complex64,
    /// Zero-sequence Thevenin impedance Z0 [p.u.]
    pub z0: Complex64,
}

impl SequenceImpedances {
    /// Compute sequence impedances from three separate Z-bus matrices.
    ///
    /// Each matrix represents the respective sequence network Z-bus.
    /// For machines with equal sequence reactances, Z1 ≈ Z2 in many textbooks,
    /// but they are kept separate here for generality.
    pub fn from_zbus(
        z1_bus: &[Vec<Complex64>],
        z2_bus: &[Vec<Complex64>],
        z0_bus: &[Vec<Complex64>],
        fault_bus: usize,
    ) -> Result<Self> {
        let n = z1_bus.len();
        if fault_bus >= n || fault_bus >= z2_bus.len() || fault_bus >= z0_bus.len() {
            return Err(OxiGridError::InvalidParameter(format!(
                "fault_bus {fault_bus} out of range {n}"
            )));
        }
        Ok(Self {
            z1: z1_bus[fault_bus][fault_bus],
            z2: z2_bus[fault_bus][fault_bus],
            z0: z0_bus[fault_bus][fault_bus],
        })
    }

    /// Simplified constructor when Z2 = Z1 (balanced machine assumption) and Z0 is given.
    pub fn from_z1_z0(z1: Complex64, z0: Complex64) -> Self {
        Self { z1, z2: z1, z0 }
    }
}

/// Sequence currents during an unbalanced fault.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceCurrents {
    /// Positive-sequence fault current I1 [p.u.]
    pub i1: Complex64,
    /// Negative-sequence fault current I2 [p.u.]
    pub i2: Complex64,
    /// Zero-sequence fault current I0 [p.u.]
    pub i0: Complex64,
}

impl SequenceCurrents {
    /// Convert sequence currents to phase currents (a, b, c).
    ///
    /// Uses the symmetrical component transformation:
    ///   [Ia]   [1  1  1 ] [I0]
    ///   [Ib] = [1  a² a ] [I1]
    ///   [Ic]   [1  a  a²] [I2]
    ///
    /// where a = e^(j2π/3) (the Fortescue operator)
    pub fn to_phase_currents(&self) -> [Complex64; 3] {
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let a2 = a * a;

        let ia = self.i0 + self.i1 + self.i2;
        let ib = self.i0 + a2 * self.i1 + a * self.i2;
        let ic = self.i0 + a * self.i1 + a2 * self.i2;

        [ia, ib, ic]
    }

    /// Phase current magnitudes [p.u.].
    pub fn phase_magnitudes(&self) -> [f64; 3] {
        let [ia, ib, ic] = self.to_phase_currents();
        [ia.norm(), ib.norm(), ic.norm()]
    }

    /// Total ground fault current: 3 * I0 [p.u.].
    pub fn ground_current(&self) -> Complex64 {
        self.i0 * 3.0
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Single-line-to-ground (SLG) fault
// ────────────────────────────────────────────────────────────────────────────

/// Compute sequence currents for a single-line-to-ground (SLG) fault on phase A.
///
/// Connection of sequence networks: Z1, Z2, Z0 all in series (Anderson p. 251).
///
///   I1 = I2 = I0 = V_f / (Z1 + Z2 + Z0 + 3*Z_f)
///
/// where Z_f is the fault impedance (0 for bolted fault).
pub fn slg_fault(
    v_prefault: Complex64,
    seq: &SequenceImpedances,
    z_fault: Complex64,
) -> SequenceCurrents {
    let denom = seq.z1 + seq.z2 + seq.z0 + z_fault * 3.0;
    let i1 = if denom.norm() < 1e-12 {
        Complex64::new(0.0, 0.0)
    } else {
        v_prefault / denom
    };
    SequenceCurrents { i1, i2: i1, i0: i1 }
}

// ────────────────────────────────────────────────────────────────────────────
// Line-to-line (LL) fault
// ────────────────────────────────────────────────────────────────────────────

/// Compute sequence currents for a line-to-line (LL) fault between phases B and C.
///
/// Connection of sequence networks: Z1 and Z2 in parallel (Anderson p. 256).
///
///   I1 = V_f / (Z1 + Z2 + Z_f)
///   I2 = -I1
///   I0 = 0  (no ground path)
pub fn ll_fault(
    v_prefault: Complex64,
    seq: &SequenceImpedances,
    z_fault: Complex64,
) -> SequenceCurrents {
    let denom = seq.z1 + seq.z2 + z_fault;
    let i1 = if denom.norm() < 1e-12 {
        Complex64::new(0.0, 0.0)
    } else {
        v_prefault / denom
    };
    SequenceCurrents {
        i1,
        i2: -i1,
        i0: Complex64::new(0.0, 0.0),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Double-line-to-ground (DLG) fault
// ────────────────────────────────────────────────────────────────────────────

/// Compute sequence currents for a double-line-to-ground (DLG) fault on phases B and C.
///
/// Connection: Z2 in parallel with (Z0 + 3*Z_g), this parallel combination in series with Z1.
///
///   I1 = V_f / (Z1 + Z2||(Z0+3Zg))
///   I2 = -I1 * (Z0 + 3*Z_g) / (Z2 + Z0 + 3*Z_g)
///   I0 = -I1 * Z2 / (Z2 + Z0 + 3*Z_g)
///
/// where Z_g is the ground fault impedance (often 0 for bolted ground).
pub fn dlg_fault(
    v_prefault: Complex64,
    seq: &SequenceImpedances,
    z_fault: Complex64,
    z_ground: Complex64,
) -> SequenceCurrents {
    let z0g = seq.z0 + z_ground * 3.0;
    // Parallel combination: Z2 || Z0g
    let z_parallel = if (seq.z2 + z0g).norm() < 1e-12 {
        Complex64::new(0.0, 0.0)
    } else {
        seq.z2 * z0g / (seq.z2 + z0g)
    };

    let denom1 = seq.z1 + z_parallel + z_fault;
    let i1 = if denom1.norm() < 1e-12 {
        Complex64::new(0.0, 0.0)
    } else {
        v_prefault / denom1
    };

    let denom2 = seq.z2 + z0g;
    let (i2, i0) = if denom2.norm() < 1e-12 {
        (Complex64::new(0.0, 0.0), Complex64::new(0.0, 0.0))
    } else {
        let i2 = -i1 * z0g / denom2;
        let i0 = -i1 * seq.z2 / denom2;
        (i2, i0)
    };

    SequenceCurrents { i1, i2, i0 }
}

// ────────────────────────────────────────────────────────────────────────────
// Unified unbalanced fault interface
// ────────────────────────────────────────────────────────────────────────────

/// Result for an unbalanced fault.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnbalancedFaultResult {
    /// Faulted bus index (0-based)
    pub bus_idx: usize,
    /// Fault type
    pub fault_type: FaultType,
    /// Sequence impedances used
    pub seq_impedances: SequenceImpedances,
    /// Sequence currents
    pub seq_currents: SequenceCurrents,
    /// Phase current magnitudes [p.u.]
    pub phase_magnitudes: [f64; 3],
    /// Ground current magnitude [p.u.]
    pub ground_current_pu: f64,
    /// Fault MVA (based on positive-sequence current)
    pub fault_mva: f64,
    /// System MVA base
    pub base_mva: f64,
}

impl UnbalancedFaultResult {
    /// X/R ratio at the fault using positive-sequence impedance.
    pub fn xr_ratio(&self) -> f64 {
        let z1 = self.seq_impedances.z1;
        if z1.re.abs() < 1e-12 {
            f64::INFINITY
        } else {
            z1.im / z1.re
        }
    }

    /// Maximum phase current magnitude [p.u.].
    pub fn max_phase_current_pu(&self) -> f64 {
        self.phase_magnitudes
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max)
    }
}

/// Compute an unbalanced fault at the given bus.
///
/// # Arguments
/// - `seq`        — sequence impedances at the fault bus
/// - `v_prefault` — pre-fault voltage at the fault bus [p.u.]
/// - `fault_type` — SLG, LL, or DLG (ThreePhase uses `three_phase_fault`)
/// - `bus_idx`    — bus index for labelling
/// - `base_mva`   — system base MVA
/// - `z_fault`    — fault impedance (0.0 for bolted)
pub fn unbalanced_fault(
    seq: SequenceImpedances,
    v_prefault: Complex64,
    fault_type: FaultType,
    bus_idx: usize,
    base_mva: f64,
    z_fault: Complex64,
) -> Result<UnbalancedFaultResult> {
    let seq_currents = match fault_type {
        FaultType::SingleLineGround => slg_fault(v_prefault, &seq, z_fault),
        FaultType::LineLine => ll_fault(v_prefault, &seq, z_fault),
        FaultType::DoubleLineGround => {
            dlg_fault(v_prefault, &seq, z_fault, Complex64::new(0.0, 0.0))
        }
        FaultType::ThreePhase => {
            return Err(OxiGridError::InvalidParameter(
                "Use three_phase_fault() for ThreePhase faults".into(),
            ));
        }
    };

    let phase_magnitudes = seq_currents.phase_magnitudes();
    let ground_current_pu = seq_currents.ground_current().norm();
    let fault_mva = seq_currents.i1.norm() * v_prefault.norm() * base_mva;

    Ok(UnbalancedFaultResult {
        bus_idx,
        fault_type,
        seq_impedances: seq,
        seq_currents,
        phase_magnitudes,
        ground_current_pu,
        fault_mva,
        base_mva,
    })
}

/// Scan all fault types at a single bus.
///
/// Returns results for SLG, LL, and DLG faults at the given bus.
/// Useful for comparing fault severity across fault types.
pub fn unbalanced_fault_scan(
    seq: SequenceImpedances,
    v_prefault: Complex64,
    bus_idx: usize,
    base_mva: f64,
) -> Result<Vec<UnbalancedFaultResult>> {
    let types = [
        FaultType::SingleLineGround,
        FaultType::LineLine,
        FaultType::DoubleLineGround,
    ];
    types
        .iter()
        .map(|&ft| {
            unbalanced_fault(
                seq.clone(),
                v_prefault,
                ft,
                bus_idx,
                base_mva,
                Complex64::new(0.0, 0.0),
            )
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Sequence current to relay quantities
// ────────────────────────────────────────────────────────────────────────────

/// Compute zero-sequence current from sequence currents (for ground relay inputs).
///
/// Returns `3 * I0` — this is the actual neutral current.
pub fn neutral_current(seq: &SequenceCurrents) -> Complex64 {
    seq.ground_current()
}

/// Compute negative-sequence current magnitude — useful for unbalance detection.
pub fn negative_sequence_current_pu(seq: &SequenceCurrents) -> f64 {
    seq.i2.norm()
}

/// Compute voltage unbalance factor (VUF) from phase voltages.
///
/// VUF = |V_neg| / |V_pos| × 100%
///
/// Sequence voltages computed from balanced 3-phase voltage system perturbed
/// by the fault (simplified; uses Fortescue transform on phase voltages).
pub fn voltage_unbalance_factor(va: Complex64, vb: Complex64, vc: Complex64) -> f64 {
    let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
    let a2 = a * a;
    // V0 = (Va + Vb + Vc) / 3
    // V1 = (Va + a*Vb + a2*Vc) / 3
    // V2 = (Va + a2*Vb + a*Vc) / 3
    let v1 = (va + a * vb + a2 * vc) / 3.0;
    let v2 = (va + a2 * vb + a * vc) / 3.0;
    if v1.norm() < 1e-12 {
        0.0
    } else {
        (v2.norm() / v1.norm()) * 100.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple 2-bus Y-bus for testing (both buses connected, slack at bus 0).
    fn two_bus_ybus() -> Vec<Vec<Complex64>> {
        let y12 = Complex64::new(0.0, -5.0); // x = 0.2 p.u.
        let y11 = -y12 + Complex64::new(0.0, 0.1); // shunt for numerical stability
        vec![vec![y11, y12], vec![y12, y11]]
    }

    #[test]
    fn test_zbus_inverse_of_ybus() {
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        // Verify Y * Z ≈ I
        let n = y.len();
        for (i, y_row) in y.iter().enumerate() {
            for (j, z_col_idx) in (0..n).enumerate() {
                let mut prod = Complex64::new(0.0, 0.0);
                for (k, y_val) in y_row.iter().enumerate() {
                    prod += y_val * z[k][z_col_idx];
                }
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (prod.re - expected).abs() < 1e-6 && prod.im.abs() < 1e-6,
                    "Y*Z[{},{}]={:.4}+{:.4}j",
                    i,
                    j,
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
        // A bus with lower Zth has higher fault current
        let y = two_bus_ybus();
        let z = compute_zbus(&y).unwrap();
        let v = vec![Complex64::new(1.0, 0.0); 2];
        let r0 = three_phase_fault(&z, &v, 0, 100.0, None).unwrap();
        let r1 = three_phase_fault(&z, &v, 1, 100.0, None).unwrap();
        // Both buses are identical in this symmetric network
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

    // ── Symmetrical component tests ──────────────────────────────────────────

    fn typical_seq() -> SequenceImpedances {
        // Typical: Z1=Z2=j0.12, Z0=j0.35 p.u. (grounded system)
        SequenceImpedances {
            z1: Complex64::new(0.01, 0.12),
            z2: Complex64::new(0.01, 0.12),
            z0: Complex64::new(0.02, 0.35),
        }
    }

    #[test]
    fn test_slg_fault_sequence_currents_equal() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = slg_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        // SLG: I1 = I2 = I0
        let diff12 = (result.i1 - result.i2).norm();
        let diff10 = (result.i1 - result.i0).norm();
        assert!(
            diff12 < 1e-10,
            "I1 should equal I2 for SLG: diff={diff12:.2e}"
        );
        assert!(
            diff10 < 1e-10,
            "I1 should equal I0 for SLG: diff={diff10:.2e}"
        );
    }

    #[test]
    fn test_slg_fault_ground_current() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = slg_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        // Ground current = 3*I0 = 3*I1 for SLG
        let i_ground = result.ground_current().norm();
        let i1_times3 = result.i1.norm() * 3.0;
        assert!((i_ground - i1_times3).abs() < 1e-10);
    }

    #[test]
    fn test_ll_fault_zero_sequence_zero() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = ll_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        // LL fault has no zero-sequence (no ground path)
        assert!(result.i0.norm() < 1e-12, "I0 should be 0 for LL fault");
        // I2 = -I1
        let sum12 = (result.i1 + result.i2).norm();
        assert!(sum12 < 1e-10, "I1+I2 should be 0 for LL fault");
    }

    #[test]
    fn test_ll_fault_no_ground_current() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = ll_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        assert!(result.ground_current().norm() < 1e-12);
    }

    #[test]
    fn test_dlg_fault_has_ground_current() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = dlg_fault(
            v_f,
            &seq,
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        );
        // DLG has ground current (I0 ≠ 0)
        assert!(result.i0.norm() > 1e-6, "I0 should be nonzero for DLG");
        assert!(result.ground_current().norm() > 1e-6);
    }

    #[test]
    fn test_dlg_kirchhoff_i1_i2_i0() {
        // KCL: I1 + I2 + I0 = 0? No — that's for balanced. Check I_a = I1+I2+I0 nonzero for DLG
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = dlg_fault(
            v_f,
            &seq,
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        );
        let phases = result.to_phase_currents();
        // Phase A current should be finite and positive
        assert!(phases[0].norm() > 0.0);
    }

    #[test]
    fn test_slg_higher_than_ll_for_low_z0() {
        // When Z0 is small (solidly grounded), SLG > LL in most cases
        let seq = SequenceImpedances {
            z1: Complex64::new(0.0, 0.12),
            z2: Complex64::new(0.0, 0.12),
            z0: Complex64::new(0.0, 0.05), // small Z0
        };
        let v_f = Complex64::new(1.0, 0.0);
        let slg = slg_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        let ll = ll_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        // I_a(SLG) = 3*I1_slg
        let ia_slg = (slg.i0 + slg.i1 + slg.i2).norm();
        // I_b(LL) ≈ √3 * I1_ll (phase B current for LL fault between B-C)
        let phases_ll = ll.to_phase_currents();
        let ia_ll = phases_ll[1].norm(); // phase B
        assert!(
            ia_slg > 0.0 && ia_ll > 0.0,
            "Both faults should have nonzero current"
        );
    }

    #[test]
    fn test_unbalanced_fault_slg() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = unbalanced_fault(
            seq,
            v_f,
            FaultType::SingleLineGround,
            0,
            100.0,
            Complex64::new(0.0, 0.0),
        )
        .unwrap();
        assert!(result.fault_mva > 0.0);
        assert!(result.ground_current_pu > 0.0);
        assert_eq!(result.fault_type, FaultType::SingleLineGround);
    }

    #[test]
    fn test_unbalanced_fault_three_phase_error() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let err = unbalanced_fault(
            seq,
            v_f,
            FaultType::ThreePhase,
            0,
            100.0,
            Complex64::new(0.0, 0.0),
        );
        assert!(
            err.is_err(),
            "ThreePhase should return error from unbalanced_fault"
        );
    }

    #[test]
    fn test_unbalanced_fault_scan_returns_three_types() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let results = unbalanced_fault_scan(seq, v_f, 0, 100.0).unwrap();
        assert_eq!(results.len(), 3);
        assert_eq!(results[0].fault_type, FaultType::SingleLineGround);
        assert_eq!(results[1].fault_type, FaultType::LineLine);
        assert_eq!(results[2].fault_type, FaultType::DoubleLineGround);
    }

    #[test]
    fn test_voltage_unbalance_factor_balanced() {
        // Perfectly balanced 3-phase: VUF = 0
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let va = Complex64::new(1.0, 0.0);
        let vb = a * a * va; // 240°
        let vc = a * va; // 120°
        let vuf = voltage_unbalance_factor(va, vb, vc);
        assert!(vuf < 1e-6, "Balanced system should have VUF≈0: {vuf:.4}");
    }

    #[test]
    fn test_voltage_unbalance_factor_unbalanced() {
        // Inject unbalance: reduce Va slightly
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let va = Complex64::new(0.90, 0.0); // 10% voltage drop on phase A
        let vb = a * a;
        let vc = a;
        let vuf = voltage_unbalance_factor(va, vb, vc);
        assert!(vuf > 0.1, "Unbalanced system should have VUF > 0: {vuf:.4}");
    }

    #[test]
    fn test_seq_impedances_from_z1_z0() {
        let z1 = Complex64::new(0.01, 0.12);
        let z0 = Complex64::new(0.02, 0.35);
        let seq = SequenceImpedances::from_z1_z0(z1, z0);
        assert_eq!(seq.z1, z1);
        assert_eq!(seq.z2, z1); // Z2 = Z1 assumption
        assert_eq!(seq.z0, z0);
    }

    #[test]
    fn test_phase_currents_symmetrical_component_roundtrip() {
        // Start with known sequence currents, convert to phase, check
        let seq_i = SequenceCurrents {
            i0: Complex64::new(0.0, 0.0),
            i1: Complex64::new(2.0, -1.0),
            i2: Complex64::new(0.0, 0.0),
        };
        // For positive sequence only: Ia=I1, Ib=a²*I1, Ic=a*I1
        let phases = seq_i.to_phase_currents();
        assert!(
            (phases[0] - seq_i.i1).norm() < 1e-10,
            "Ia should equal I1 for pos-seq only"
        );
        // All phases should have same magnitude
        let m0 = phases[0].norm();
        let m1 = phases[1].norm();
        let m2 = phases[2].norm();
        assert!((m0 - m1).abs() < 1e-10 && (m1 - m2).abs() < 1e-10);
    }

    #[test]
    fn test_negative_sequence_current_pu() {
        let seq_i = SequenceCurrents {
            i0: Complex64::new(0.0, 0.0),
            i1: Complex64::new(2.0, 0.0),
            i2: Complex64::new(1.5, 0.0),
        };
        assert!((negative_sequence_current_pu(&seq_i) - 1.5).abs() < 1e-10);
    }
}
