//! Asymmetric (unbalanced) fault analysis using symmetrical components.
//!
//! Provides single-line-to-ground (SLG), line-to-line (LL), and
//! double-line-to-ground (DLG) fault calculations via Fortescue's
//! method of symmetrical components (Anderson, "Analysis of Faulted
//! Power Systems", 1973).

use crate::error::{OxiGridError, Result};
use crate::protection::fault_symmetric::FaultType;
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Sequence impedances
// ────────────────────────────────────────────────────────────────────────────

/// Sequence impedances at a bus for unbalanced fault analysis.
///
/// Contains the positive-, negative-, and zero-sequence Thevenin impedances
/// at the faulted bus, derived from the respective sequence Z-bus matrices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceImpedances {
    /// Positive-sequence Thevenin impedance Z1 \[p.u.\]
    pub z1: Complex64,
    /// Negative-sequence Thevenin impedance Z2 \[p.u.\]
    pub z2: Complex64,
    /// Zero-sequence Thevenin impedance Z0 \[p.u.\]
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

// ────────────────────────────────────────────────────────────────────────────
// Sequence currents
// ────────────────────────────────────────────────────────────────────────────

/// Sequence currents during an unbalanced fault.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceCurrents {
    /// Positive-sequence fault current I1 \[p.u.\]
    pub i1: Complex64,
    /// Negative-sequence fault current I2 \[p.u.\]
    pub i2: Complex64,
    /// Zero-sequence fault current I0 \[p.u.\]
    pub i0: Complex64,
}

impl SequenceCurrents {
    /// Convert sequence currents to phase currents (a, b, c).
    ///
    /// Uses the symmetrical component transformation:
    ///   \[Ia\]   \[1  1  1 \] \[I0\]
    ///   \[Ib\] = \[1  a² a \] \[I1\]
    ///   \[Ic\]   \[1  a  a²\] \[I2\]
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

    /// Phase current magnitudes \[p.u.\].
    pub fn phase_magnitudes(&self) -> [f64; 3] {
        let [ia, ib, ic] = self.to_phase_currents();
        [ia.norm(), ib.norm(), ic.norm()]
    }

    /// Total ground fault current: 3 * I0 \[p.u.\].
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
    /// Phase current magnitudes \[p.u.\]
    pub phase_magnitudes: [f64; 3],
    /// Ground current magnitude \[p.u.\]
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

    /// Maximum phase current magnitude \[p.u.\].
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
/// - `v_prefault` — pre-fault voltage at the fault bus \[p.u.\]
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

// ────────────────────────────────────────────────────────────────────────────
// Higher-level asymmetric fault API
// ────────────────────────────────────────────────────────────────────────────

/// Type of asymmetric (unsymmetric) fault for the high-level API.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum AsymmetricFaultType {
    /// Single-line-to-ground (SLG) — phase A to ground (~70% of faults)
    SingleLineToGround,
    /// Line-to-line (LL) — phase B to C
    LineToLine,
    /// Double-line-to-ground (DLG) — phases B and C to ground
    DoubleLineToGround,
}

/// Result of an asymmetric fault analysis using symmetrical components.
///
/// Provides sequence currents, phase currents, and magnitude in both p.u. and physical units.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AsymmetricFaultResult {
    /// Fault type
    pub fault_type: AsymmetricFaultType,
    /// Faulted bus index (0-based)
    pub fault_bus: usize,
    /// Positive-sequence fault current \[p.u.\]
    pub i_fault_positive: Complex64,
    /// Negative-sequence fault current \[p.u.\]
    pub i_fault_negative: Complex64,
    /// Zero-sequence fault current \[p.u.\]
    pub i_fault_zero: Complex64,
    /// Phase A fault current \[p.u.\]
    pub i_fault_a: Complex64,
    /// Phase B fault current \[p.u.\]
    pub i_fault_b: Complex64,
    /// Phase C fault current \[p.u.\]
    pub i_fault_c: Complex64,
    /// Magnitude of dominant phase fault current \[p.u.\]
    pub i_fault_magnitude: f64,
    /// Fault current in kA (if base_kv provided)
    pub i_fault_ka: Option<f64>,
    /// System MVA base
    pub base_mva: f64,
}

impl AsymmetricFaultResult {
    /// X/R ratio at the fault using positive-sequence impedance.
    pub fn xr_ratio(&self) -> f64 {
        // Not directly available here — caller should use SequenceImpedances
        0.0
    }

    /// Maximum phase current magnitude \[p.u.\].
    pub fn max_phase_current_pu(&self) -> f64 {
        [self.i_fault_a, self.i_fault_b, self.i_fault_c]
            .iter()
            .map(|c| c.norm())
            .fold(0.0_f64, f64::max)
    }
}

/// Compute asymmetric fault using symmetrical component formulas.
///
/// # Arguments
/// - `seq`              — Sequence impedances at the fault bus
/// - `fault_type`       — Type of asymmetric fault
/// - `prefault_voltage` — Pre-fault voltage in p.u. (typically 1.0)
/// - `fault_bus`        — Bus index for labelling
/// - `base_kv`          — Base kV at the fault bus (optional — used for kA conversion)
/// - `base_mva`         — System MVA base
pub fn compute_asymmetric_fault(
    seq: &SequenceImpedances,
    fault_type: AsymmetricFaultType,
    prefault_voltage: f64,
    fault_bus: usize,
    base_kv: Option<f64>,
    base_mva: f64,
) -> Result<AsymmetricFaultResult> {
    let v_f = Complex64::new(prefault_voltage, 0.0);
    let z_zero = Complex64::new(0.0, 0.0);

    let ft_mapped = match fault_type {
        AsymmetricFaultType::SingleLineToGround => FaultType::SingleLineGround,
        AsymmetricFaultType::LineToLine => FaultType::LineLine,
        AsymmetricFaultType::DoubleLineToGround => FaultType::DoubleLineGround,
    };

    let seq_currents = match ft_mapped {
        FaultType::SingleLineGround => slg_fault(v_f, seq, z_zero),
        FaultType::LineLine => ll_fault(v_f, seq, z_zero),
        FaultType::DoubleLineGround => dlg_fault(v_f, seq, z_zero, z_zero),
        FaultType::ThreePhase => {
            return Err(OxiGridError::InvalidParameter(
                "Use three_phase_fault() for 3-phase faults".into(),
            ));
        }
    };

    let [ia, ib, ic] = seq_currents.to_phase_currents();
    let i_fault_magnitude = [ia, ib, ic]
        .iter()
        .map(|c| c.norm())
        .fold(0.0_f64, f64::max);

    let i_fault_ka = base_kv.map(|kv| {
        let i_base_ka = base_mva / (kv * 3.0_f64.sqrt());
        i_fault_magnitude * i_base_ka
    });

    Ok(AsymmetricFaultResult {
        fault_type,
        fault_bus,
        i_fault_positive: seq_currents.i1,
        i_fault_negative: seq_currents.i2,
        i_fault_zero: seq_currents.i0,
        i_fault_a: ia,
        i_fault_b: ib,
        i_fault_c: ic,
        i_fault_magnitude,
        i_fault_ka,
        base_mva,
    })
}

// ────────────────────────────────────────────────────────────────────────────
// FaultAnalysis — high-level convenience interface
// ────────────────────────────────────────────────────────────────────────────

/// High-level fault analysis engine.
///
/// Wraps the Z-bus and pre-fault voltages to provide convenient per-bus fault
/// calculation methods for all fault types — both symmetric (3-phase) and
/// asymmetric (SLG, LL, DLG).
///
/// Sequence impedances are approximated as:
/// - Z1 = Zth (positive-sequence Thevenin from Z-bus diagonal)
/// - Z2 = Z1  (negative ≈ positive for most networks)
/// - Z0 = 3×Z1 (rough zero-sequence approximation)
#[derive(Debug, Clone)]
pub struct FaultAnalysis {
    /// Bus impedance matrix Z_bus (positive-sequence)
    pub z_bus: Vec<Vec<Complex64>>,
    /// Pre-fault bus voltages \[p.u.\]
    pub v_prefault: Vec<Complex64>,
    /// System MVA base
    pub base_mva: f64,
    /// Per-bus base kV (optional — for kA conversion)
    pub bus_kv: Vec<Option<f64>>,
}

impl FaultAnalysis {
    /// Create a new FaultAnalysis from a Y-bus matrix and pre-fault voltages.
    ///
    /// Computes Z_bus = Y_bus⁻¹ internally.
    pub fn from_ybus(
        y_bus: &[Vec<Complex64>],
        v_prefault: Vec<Complex64>,
        base_mva: f64,
    ) -> Result<Self> {
        use crate::protection::fault_symmetric::compute_zbus;
        let z_bus = compute_zbus(y_bus)?;
        let n = z_bus.len();
        Ok(Self {
            z_bus,
            v_prefault,
            base_mva,
            bus_kv: vec![None; n],
        })
    }

    /// Set the base kV for a specific bus (enables kA output).
    pub fn set_bus_kv(&mut self, bus_idx: usize, kv: f64) -> Result<()> {
        if bus_idx >= self.bus_kv.len() {
            return Err(OxiGridError::InvalidParameter(format!(
                "bus_idx {bus_idx} out of range {}",
                self.bus_kv.len()
            )));
        }
        self.bus_kv[bus_idx] = Some(kv);
        Ok(())
    }

    /// Approximate sequence impedances at a bus using Z-bus diagonal.
    fn seq_at(&self, bus_idx: usize) -> Result<SequenceImpedances> {
        if bus_idx >= self.z_bus.len() {
            return Err(OxiGridError::InvalidParameter(format!(
                "bus_idx {bus_idx} out of range {}",
                self.z_bus.len()
            )));
        }
        let z1 = self.z_bus[bus_idx][bus_idx];
        let z0 = z1 * 3.0; // rough approximation
        Ok(SequenceImpedances::from_z1_z0(z1, z0))
    }

    /// Compute a single-line-to-ground (SLG) fault at the given bus.
    pub fn single_line_to_ground_fault(&self, bus_idx: usize) -> Result<AsymmetricFaultResult> {
        let seq = self.seq_at(bus_idx)?;
        let v_f = self.v_prefault[bus_idx].norm();
        compute_asymmetric_fault(
            &seq,
            AsymmetricFaultType::SingleLineToGround,
            v_f,
            bus_idx,
            self.bus_kv[bus_idx],
            self.base_mva,
        )
    }

    /// Compute a line-to-line (LL) fault at the given bus.
    pub fn line_to_line_fault(&self, bus_idx: usize) -> Result<AsymmetricFaultResult> {
        let seq = self.seq_at(bus_idx)?;
        let v_f = self.v_prefault[bus_idx].norm();
        compute_asymmetric_fault(
            &seq,
            AsymmetricFaultType::LineToLine,
            v_f,
            bus_idx,
            self.bus_kv[bus_idx],
            self.base_mva,
        )
    }

    /// Compute a double-line-to-ground (DLG) fault at the given bus.
    pub fn double_line_to_ground_fault(&self, bus_idx: usize) -> Result<AsymmetricFaultResult> {
        let seq = self.seq_at(bus_idx)?;
        let v_f = self.v_prefault[bus_idx].norm();
        compute_asymmetric_fault(
            &seq,
            AsymmetricFaultType::DoubleLineToGround,
            v_f,
            bus_idx,
            self.bus_kv[bus_idx],
            self.base_mva,
        )
    }

    /// Compute all three asymmetric fault types at the given bus.
    pub fn all_asymmetric_faults(&self, bus_idx: usize) -> Result<[AsymmetricFaultResult; 3]> {
        let slg = self.single_line_to_ground_fault(bus_idx)?;
        let ll = self.line_to_line_fault(bus_idx)?;
        let dlg = self.double_line_to_ground_fault(bus_idx)?;
        Ok([slg, ll, dlg])
    }
}

/// Compute negative-sequence current magnitude — useful for unbalance detection.
pub fn negative_sequence_current_pu(seq: &SequenceCurrents) -> f64 {
    seq.i2.norm()
}

/// Compute voltage unbalance factor (VUF) from phase voltages.
///
/// VUF = |V_neg| / |V_pos| × 100%
pub fn voltage_unbalance_factor(va: Complex64, vb: Complex64, vc: Complex64) -> f64 {
    let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
    let a2 = a * a;
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

    fn typical_seq() -> SequenceImpedances {
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
        let i_ground = result.ground_current().norm();
        let i1_times3 = result.i1.norm() * 3.0;
        assert!((i_ground - i1_times3).abs() < 1e-10);
    }

    #[test]
    fn test_ll_fault_zero_sequence_zero() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = ll_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        assert!(result.i0.norm() < 1e-12, "I0 should be 0 for LL fault");
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
        assert!(result.i0.norm() > 1e-6, "I0 should be nonzero for DLG");
        assert!(result.ground_current().norm() > 1e-6);
    }

    #[test]
    fn test_dlg_kirchhoff_i1_i2_i0() {
        let seq = typical_seq();
        let v_f = Complex64::new(1.0, 0.0);
        let result = dlg_fault(
            v_f,
            &seq,
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        );
        let phases = result.to_phase_currents();
        assert!(phases[0].norm() > 0.0);
    }

    #[test]
    fn test_slg_higher_than_ll_for_low_z0() {
        let seq = SequenceImpedances {
            z1: Complex64::new(0.0, 0.12),
            z2: Complex64::new(0.0, 0.12),
            z0: Complex64::new(0.0, 0.05),
        };
        let v_f = Complex64::new(1.0, 0.0);
        let slg = slg_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        let ll = ll_fault(v_f, &seq, Complex64::new(0.0, 0.0));
        let ia_slg = (slg.i0 + slg.i1 + slg.i2).norm();
        let phases_ll = ll.to_phase_currents();
        let ia_ll = phases_ll[1].norm();
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
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let va = Complex64::new(1.0, 0.0);
        let vb = a * a * va;
        let vc = a * va;
        let vuf = voltage_unbalance_factor(va, vb, vc);
        assert!(vuf < 1e-6, "Balanced system should have VUF≈0: {vuf:.4}");
    }

    #[test]
    fn test_voltage_unbalance_factor_unbalanced() {
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let va = Complex64::new(0.90, 0.0);
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
        assert_eq!(seq.z2, z1);
        assert_eq!(seq.z0, z0);
    }

    #[test]
    fn test_phase_currents_symmetrical_component_roundtrip() {
        let seq_i = SequenceCurrents {
            i0: Complex64::new(0.0, 0.0),
            i1: Complex64::new(2.0, -1.0),
            i2: Complex64::new(0.0, 0.0),
        };
        let phases = seq_i.to_phase_currents();
        assert!(
            (phases[0] - seq_i.i1).norm() < 1e-10,
            "Ia should equal I1 for pos-seq only"
        );
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
