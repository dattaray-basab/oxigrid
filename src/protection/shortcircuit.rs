//! Comprehensive short-circuit analysis per IEC 60909 and ANSI/IEEE C37 standards.
//!
//! This module provides:
//!
//! - [`Iec60909Calculator`] — IEC 60909 voltage-source equivalent method:
//!   3-phase, 2-phase, 1-phase, and 2-phase-to-earth fault currents,
//!   peak factor κ, thermal equivalent currents, breaking currents.
//! - [`InductionMotorGroup`] and [`SynchronousMotorGroup`] — motor short-circuit
//!   contributions with subtransient / transient decay models.
//! - [`short_circuit_survey`] — network-wide SC level survey at all buses.
//! - [`sc_level_extremes`] — minimum / maximum SC levels across the network.
//! - [`verify_relay_settings`] — relay coordination verification against SC levels.
//! - [`AnsiCalculator`] — ANSI/IEEE C37.010 first-cycle (momentary) and
//!   interrupting duty calculations.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::protection::fault_symmetric::FaultType;
use crate::protection::relay::OcRelay;
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// IEC 60909 constants and helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Voltage factor c (IEC 60909-0 Table 1).
///
/// - `c_max = 1.10` for LV networks (up to 1 kV) — maximum SC current
/// - `c_max = 1.05` for MV/HV networks — maximum SC current
/// - `c_min = 0.95` for all networks — minimum SC current
pub const C_MAX_LV: f64 = 1.10;
pub const C_MAX_HV: f64 = 1.05;
pub const C_MIN: f64 = 0.95;

// ─────────────────────────────────────────────────────────────────────────────
// IEC 60909 method selector
// ─────────────────────────────────────────────────────────────────────────────

/// Calculation method for IEC 60909.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum Iec60909Method {
    /// Equivalent voltage source at the fault point (IEC 60909 Clause 4.3).
    /// Uses c*V_n/√3 as the driving voltage; no pre-fault power flow required.
    EquivalentVoltage,
    /// Rigorous superposition method using pre-fault operating point.
    /// Requires network-wide power flow solution.
    Superposition,
}

// ─────────────────────────────────────────────────────────────────────────────
// Grounding configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Zero-sequence path configuration at a transformer neutral or generator grounding point.
///
/// Determines whether zero-sequence current can flow through a component,
/// which is essential for single-line-to-ground and double-line-to-ground
/// fault calculations (IEC 60909-0 Clause 3.9).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum GroundingConfig {
    /// Neutral directly connected to earth — zero-sequence path available.
    /// Typical for star-grounded transformers and generators.
    SolidlyGrounded,
    /// Delta winding — no zero-sequence path available.
    /// Zero-sequence current cannot flow through delta windings.
    UngroundedDelta,
    /// Neutral grounded through impedance Zn = r_ohm + j*x_ohm \[Ω\].
    /// Used when limiting fault current via neutral grounding resistor or reactor.
    Impedance { r_ohm: f64, x_ohm: f64 },
    /// Effectively grounded condition per ANSI/IEEE (X0/X1 < 3, R0/X1 < 1).
    EffectivelyGrounded,
}

impl GroundingConfig {
    /// Returns `true` if zero-sequence current can flow (any grounded configuration).
    pub fn allows_zero_sequence(&self) -> bool {
        !matches!(self, Self::UngroundedDelta)
    }

    /// Grounding impedance in per-unit (at system base).
    ///
    /// Solidly grounded → 0 Ω, delta → infinite (represented as large value),
    /// impedance → exact value.
    pub fn impedance_pu(&self, base_kv: f64, base_mva: f64) -> Complex64 {
        match self {
            Self::SolidlyGrounded | Self::EffectivelyGrounded => Complex64::new(0.0, 0.0),
            Self::UngroundedDelta => {
                // Effectively infinite impedance — no zero-sequence path
                Complex64::new(1e9, 0.0)
            }
            Self::Impedance { r_ohm, x_ohm } => {
                // Convert Ω to pu: Z_pu = Z_ohm / Z_base = Z_ohm * S_base / V_base^2
                let z_base = (base_kv * base_kv) / base_mva;
                if z_base < 1e-12 {
                    Complex64::new(0.0, 0.0)
                } else {
                    Complex64::new(r_ohm / z_base, x_ohm / z_base)
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Short-circuit input / output
// ─────────────────────────────────────────────────────────────────────────────

/// Input parameters for a short-circuit study at a specific bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShortCircuitInput {
    /// Faulted bus index (0-based internal index).
    pub fault_bus: usize,
    /// Type of short-circuit fault.
    pub fault_type: FaultType,
    /// Fault impedance Zf \[p.u.\] — `Complex64::new(0,0)` for bolted (zero-impedance) fault.
    pub fault_impedance: Complex64,
    /// Pre-fault terminal voltage at the fault bus \[p.u.\] — typically 1.0 in IEC 60909.
    pub pre_fault_voltage_pu: f64,
}

impl ShortCircuitInput {
    /// Construct a bolted 3-phase fault input (most common, gives maximum SC current).
    pub fn bolted_three_phase(fault_bus: usize) -> Self {
        Self {
            fault_bus,
            fault_type: FaultType::ThreePhase,
            fault_impedance: Complex64::new(0.0, 0.0),
            pre_fault_voltage_pu: 1.0,
        }
    }

    /// Construct a bolted single-line-to-ground fault input.
    pub fn bolted_slg(fault_bus: usize) -> Self {
        Self {
            fault_bus,
            fault_type: FaultType::SingleLineGround,
            fault_impedance: Complex64::new(0.0, 0.0),
            pre_fault_voltage_pu: 1.0,
        }
    }
}

/// Full IEC 60909 short-circuit analysis result at one bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShortCircuitResult {
    /// Faulted bus index (0-based).
    pub fault_bus: usize,
    /// Fault type computed.
    pub fault_type: FaultType,
    // ── IEC 60909 symmetrical currents ──────────────────────────────────────
    /// Initial symmetrical 3-phase SC current I"k3 \[kA RMS\].
    pub i_k3_ka: f64,
    /// Initial 2-phase SC current I"k2 \[kA RMS\].
    pub i_k2_ka: f64,
    /// Initial 1-phase (SLG) SC current I"k1 \[kA RMS\].
    pub i_k1_ka: f64,
    /// 2-phase-to-earth SC current I"kE2E1 \[kA RMS\] (max phase current for DLG).
    pub i_k_e2e1_ka: f64,
    /// Short-circuit power S"k \[MVA\] (3-phase basis).
    pub s_k_mva: f64,
    /// Driving-point (Thevenin) impedance at the fault bus \[Ω\].
    pub z_k_ohm: f64,
    /// R/X ratio at the fault point (determines DC offset).
    pub r_x_ratio: f64,
    // ── Peak current ────────────────────────────────────────────────────────
    /// Peak SC current ip \[kA\] — asymmetrical first-cycle peak.
    pub i_p_ka: f64,
    /// IEC 60909 peak factor κ (1.02 … 2.0).
    pub kappa: f64,
    // ── Thermal equivalent ──────────────────────────────────────────────────
    /// Thermal equivalent current Ith \[kA RMS\] over clearing time tc.
    pub i_th_ka: f64,
    /// I²t value \[kA²·s\] — Joule integral for cable/equipment thermal rating.
    pub thermal_energy_ka2s: f64,
    // ── Breaking current ────────────────────────────────────────────────────
    /// Symmetrical breaking current Ib \[kA\] at contact parting time (optional).
    pub i_b_ka: Option<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Dense Z-bus helper (reused from fault_symmetric logic)
// ─────────────────────────────────────────────────────────────────────────────

/// Dense Gaussian-elimination inversion of an n×n complex matrix.
///
/// Returns the inverse matrix or `OxiGridError::LinearAlgebra` if singular.
#[allow(clippy::needless_range_loop)]
fn dense_invert(mat_in: &[Vec<Complex64>]) -> Result<Vec<Vec<Complex64>>> {
    let n = mat_in.len();
    // Build augmented [A | I]
    let mut aug: Vec<Vec<Complex64>> = mat_in
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            for j in 0..n {
                r.push(if i == j {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(0.0, 0.0)
                });
            }
            r
        })
        .collect();

    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].norm();
        for row in (col + 1)..n {
            let v = aug[row][col].norm();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return Err(OxiGridError::LinearAlgebra(
                "Impedance matrix is singular — network may be disconnected".into(),
            ));
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for j in col..2 * n {
            aug[col][j] /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in col..2 * n {
                let sub = factor * aug[col][j];
                aug[row][j] -= sub;
            }
        }
    }

    Ok((0..n).map(|i| aug[i][n..].to_vec()).collect())
}

/// Extract the dense Y-bus as a Vec<Vec<Complex64>> from a PowerNetwork.
fn dense_ybus(network: &PowerNetwork) -> Result<Vec<Vec<Complex64>>> {
    let n = network.bus_count();
    let mut y: Vec<Vec<Complex64>> = vec![vec![Complex64::new(0.0, 0.0); n]; n];

    for branch in &network.branches {
        if !branch.status {
            continue;
        }
        let i = network.bus_index(branch.from_bus)?;
        let j = network.bus_index(branch.to_bus)?;

        let ys = Complex64::new(branch.r, branch.x).inv();
        let bc = Complex64::new(0.0, branch.b / 2.0);
        let tap = branch.tap_complex();
        let tap_conj = tap.conj();
        let tap_mag_sq = tap.norm_sqr();

        y[i][i] += ys / tap_mag_sq + bc;
        y[j][j] += ys + bc;
        y[i][j] += -ys / tap_conj;
        y[j][i] += -ys / tap;
    }

    // Shunt elements
    for (idx, bus) in network.buses.iter().enumerate() {
        if bus.gs != 0.0 || bus.bs != 0.0 {
            let y_shunt = Complex64::new(bus.gs, bus.bs) / network.base_mva;
            y[idx][idx] += y_shunt;
        }
    }

    Ok(y)
}

// ─────────────────────────────────────────────────────────────────────────────
// IEC 60909 Calculator
// ─────────────────────────────────────────────────────────────────────────────

/// IEC 60909-0:2016 short-circuit calculator.
///
/// Implements the equivalent voltage source method (Clause 4.3) which requires
/// no pre-fault power flow — the driving voltage is:
///
/// ```text
///   U_eq = c · V_n / √3   (phase-to-neutral)
/// ```
///
/// where `c` is the IEC 60909 voltage factor (typically 1.0–1.10 for maximum
/// SC current, 0.95 for minimum SC current verification).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Iec60909Calculator {
    /// Voltage factor c (see IEC 60909-0 Table 1).
    pub c_factor: f64,
    /// Calculation method (equivalent voltage source or superposition).
    pub method: Iec60909Method,
    /// Fault clearing (short-circuit) duration tc \[s\] for thermal rating.
    pub clearing_time_s: f64,
}

impl Iec60909Calculator {
    /// Create a new calculator for maximum SC current (c = 1.0, LV = 1.1).
    ///
    /// Defaults to the equivalent voltage source method with 0.1 s clearing time.
    pub fn new(c_factor: f64) -> Self {
        Self {
            c_factor,
            method: Iec60909Method::EquivalentVoltage,
            clearing_time_s: 0.1,
        }
    }

    /// Set calculation method.
    pub fn with_method(mut self, method: Iec60909Method) -> Self {
        self.method = method;
        self
    }

    /// Set fault clearing time \[s\] used for thermal equivalent current.
    pub fn with_clearing_time(mut self, tc_s: f64) -> Self {
        self.clearing_time_s = tc_s;
        self
    }

    // ── Z-matrix builders ─────────────────────────────────────────────────

    /// Build the positive-sequence bus impedance matrix Z1 = Y1⁻¹.
    ///
    /// For balanced networks the positive-sequence network is identical to
    /// the normal-frequency network (IEC 60909-0 Clause 3.7).
    pub fn build_z1_matrix(&self, network: &PowerNetwork) -> Result<Vec<Vec<Complex64>>> {
        let y1 = dense_ybus(network)?;
        dense_invert(&y1)
    }

    /// Build the negative-sequence bus impedance matrix Z2.
    ///
    /// For balanced, transmission-class networks Z2 ≈ Z1.  This returns
    /// the same matrix as Z1; machine negative-sequence impedance differences
    /// are handled via generator subtransient models (out of scope here).
    pub fn build_z2_matrix(&self, network: &PowerNetwork) -> Result<Vec<Vec<Complex64>>> {
        // For balanced power networks: Z2 = Z1
        self.build_z1_matrix(network)
    }

    /// Build the zero-sequence bus impedance matrix Z0.
    ///
    /// The zero-sequence network topology differs from the positive/negative-
    /// sequence network because:
    /// - Delta windings block zero-sequence current.
    /// - Solidly grounded neutrals short the zero-sequence to ground.
    /// - Impedance-grounded neutrals add 3·Zn in the zero-sequence path.
    ///
    /// The `grounding` slice maps bus index → grounding configuration.
    /// If `grounding` is empty or shorter than the bus count, missing buses
    /// default to `SolidlyGrounded`.
    pub fn build_z0_matrix(
        &self,
        network: &PowerNetwork,
        grounding: &[GroundingConfig],
    ) -> Result<Vec<Vec<Complex64>>> {
        // Start from Y0 = Y1 (same topology)
        let mut y0 = dense_ybus(network)?;

        // Apply grounding modifications to the diagonal:
        // A solidly-grounded bus has no additional diagonal term.
        // An ungrounded-delta bus adds a large impedance to block zero-seq.
        // An impedance-grounded bus adds (1/(3·Zn)) to the zero-sequence Y.
        for (idx, bus) in network.buses.iter().enumerate() {
            let gcfg = grounding
                .get(idx)
                .copied()
                .unwrap_or(GroundingConfig::SolidlyGrounded);
            // Base kV of this bus (use a nominal 1.0 pu if not set)
            let base_kv = if bus.base_kv.0 > 1e-6 {
                bus.base_kv.0
            } else {
                1.0
            };
            let zn = gcfg.impedance_pu(base_kv, network.base_mva);

            match gcfg {
                GroundingConfig::UngroundedDelta => {
                    // Block zero-sequence: add very large shunt admittance
                    // (equivalent to opening the zero-sequence path)
                    // We model this by subtracting all off-diagonal entries
                    // from this row/col — simplest: set diagonal to large value.
                    // For a proper Z0 build we zero out the row/col contributions
                    // from branches connected to this bus.  Approximation here:
                    // very low shunt = open circuit in zero-seq.
                    y0[idx][idx] += Complex64::new(0.0, -1e9);
                }
                GroundingConfig::Impedance { .. } => {
                    // Add 1/(3*Zn) to zero-sequence admittance
                    let three_zn = zn * 3.0;
                    if three_zn.norm() > 1e-14 {
                        y0[idx][idx] += Complex64::new(1.0, 0.0) / three_zn;
                    }
                }
                GroundingConfig::SolidlyGrounded | GroundingConfig::EffectivelyGrounded => {
                    // No additional modification needed
                }
            }
        }

        dense_invert(&y0)
    }

    // ── Peak and thermal factors ──────────────────────────────────────────

    /// Compute IEC 60909 peak factor κ from R/X ratio at the fault point.
    ///
    /// Formula: κ = 1.02 + 0.98·exp(−3·R/X)  (IEC 60909-0 Equation 29)
    ///
    /// Valid range: 1.02 (large R/X, resistive network) to 2.0 (pure reactive, R/X → 0).
    pub fn compute_kappa(r_x_ratio: f64) -> f64 {
        let rx = r_x_ratio.max(0.0); // R/X must be non-negative
        1.02 + 0.98 * (-3.0 * rx).exp()
    }

    /// Compute thermal factors m and n for the thermal equivalent current.
    ///
    /// Returns `(m, n)` where:
    /// - `m` — AC component Joule integral factor (≈ 1.0 for tc > ~0.5 s)
    /// - `n` — DC component Joule integral factor
    ///
    /// IEC 60909 Annex A equations:
    /// ```text
    ///   m = (e^{4·f·tc·R/X·ln(κ−1.02)/0.98} − 1) / (2·f·tc·ln(κ−1.02)/0.98)
    ///   n = 1  (AC component, no decay for first approximation)
    /// ```
    /// Simplified approximation used here (conservative, valid for industrial SC):
    /// ```text
    ///   m = (1/4f·tc·R/X) * (exp(4*f*tc*ln(κ-1.02)/0.98) - 1)   [IEC 60909 Fig A.1]
    ///   n ≈ 1  (for tc ≤ 1 s with X/R ≤ 50)
    /// ```
    pub fn compute_thermal_factor(tc_s: f64, r_x_ratio: f64) -> (f64, f64) {
        let f = 50.0; // nominal frequency (Hz) — IEC uses 50 Hz
        let rx = r_x_ratio.max(1e-6);
        // κ from the given R/X
        let kappa = Self::compute_kappa(rx);
        let kappa_term = (kappa - 1.02).max(1e-6) / 0.98;

        // m factor (DC component decay integral)
        let exponent = 4.0 * f * tc_s * kappa_term.ln();
        let m = if exponent.abs() < 1e-9 {
            // L'Hôpital: limit = 1
            1.0
        } else {
            (exponent.exp() - 1.0) / exponent
        };

        // n factor (AC component integral): 1.0 for no AC decay
        // For generators with subtransient time constant ≫ tc, n ≈ 1.
        let n = 1.0;

        (m.max(0.0), n)
    }

    // ── Main computation ──────────────────────────────────────────────────

    /// Compute the complete IEC 60909 short-circuit result at the fault bus.
    ///
    /// Builds Z1, Z2, Z0 matrices, applies the equivalent voltage source
    /// formula for all four fault types, and computes peak / thermal currents.
    ///
    /// # Arguments
    /// - `network` — power network topology and parameters
    /// - `input`   — fault specification (bus, type, impedance, voltage)
    ///
    /// # Returns
    /// Full [`ShortCircuitResult`] with all IEC 60909 quantities.
    pub fn compute(
        &self,
        network: &PowerNetwork,
        input: &ShortCircuitInput,
    ) -> Result<ShortCircuitResult> {
        self.compute_with_grounding(network, input, &[])
    }

    /// Compute with explicit grounding configuration for zero-sequence network.
    pub fn compute_with_grounding(
        &self,
        network: &PowerNetwork,
        input: &ShortCircuitInput,
        grounding: &[GroundingConfig],
    ) -> Result<ShortCircuitResult> {
        let n = network.bus_count();
        let f = input.fault_bus;

        if f >= n {
            return Err(OxiGridError::InvalidParameter(format!(
                "fault_bus {f} out of range {n}"
            )));
        }

        // Build all three sequence Z-bus matrices
        let z1 = self.build_z1_matrix(network)?;
        let z2 = self.build_z2_matrix(network)?;
        let z0 = self.build_z0_matrix(network, grounding)?;

        // Sequence Thevenin impedances at fault bus
        let z1ff = z1[f][f];
        let z2ff = z2[f][f];
        let z0ff = z0[f][f];

        // IEC 60909 driving voltage: U_eq = c * V_n (line-to-neutral in pu)
        let v_eq = Complex64::new(self.c_factor * input.pre_fault_voltage_pu, 0.0);
        let zf = input.fault_impedance;

        // ── 3-phase SC current I"k3 (IEC 60909 Eq. 24) ──────────────────
        // I"k3 = c * V_n / (√3 * (Z1 + Zf))
        // In pu: I"k3_pu = V_eq / (Z1 + Zf), then multiply by √3 to get line current
        // (because V_eq is already line-to-neutral)
        let z_total_3ph = z1ff + zf;
        let i_k3_pu = if z_total_3ph.norm() < 1e-12 {
            0.0
        } else {
            (v_eq / z_total_3ph).norm()
        };

        // ── 2-phase SC current I"k2 (IEC 60909 Eq. 26) ──────────────────
        // I"k2 = √3 * c * V_n / (2 * |Z1 + Zf|)
        // In pu: I"k2_pu = (√3/2) * V_eq / |Z1 + Zf|  = (√3/2) * I"k3
        let z_total_2ph = z1ff + z2ff + zf;
        let i_k2_pu = if z_total_2ph.norm() < 1e-12 {
            0.0
        } else {
            (v_eq / z_total_2ph).norm() * 3.0_f64.sqrt()
        };

        // ── 1-phase (SLG) SC current I"k1 (IEC 60909 Eq. 28) ────────────
        // I"k1 = √3 * c * V_n / |Z1 + Z2 + Z0 + 3*Zf|
        let z_total_slg = z1ff + z2ff + z0ff + zf * 3.0;
        let i_k1_pu = if z_total_slg.norm() < 1e-12 {
            0.0
        } else {
            (v_eq * 3.0 / z_total_slg).norm()
        };

        // ── 2-phase-to-earth DLG current (IEC 60909 Eq. 30) ─────────────
        // Maximum phase current: I"kE2E1 at the faulted phase
        // Z_par = Z2 || Z0 = Z2*Z0 / (Z2+Z0)
        let z_sum_20 = z2ff + z0ff;
        let z_par = if z_sum_20.norm() < 1e-12 {
            Complex64::new(0.0, 0.0)
        } else {
            z2ff * z0ff / z_sum_20
        };
        let z_total_dlg = z1ff + z_par + zf;
        let i1_dlg_pu = if z_total_dlg.norm() < 1e-12 {
            Complex64::new(0.0, 0.0)
        } else {
            v_eq / z_total_dlg
        };
        // I2_dlg = -I1 * Z0 / (Z2 + Z0)
        let i2_dlg_pu = if z_sum_20.norm() < 1e-12 {
            Complex64::new(0.0, 0.0)
        } else {
            -i1_dlg_pu * z0ff / z_sum_20
        };
        // I0_dlg = -I1 * Z2 / (Z2 + Z0)
        let i0_dlg_pu = if z_sum_20.norm() < 1e-12 {
            Complex64::new(0.0, 0.0)
        } else {
            -i1_dlg_pu * z2ff / z_sum_20
        };
        // Phase currents via symmetrical component transformation
        let a = Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI / 3.0);
        let a2 = a * a;
        let ib_dlg = i0_dlg_pu + a2 * i1_dlg_pu + a * i2_dlg_pu;
        let ic_dlg = i0_dlg_pu + a * i1_dlg_pu + a2 * i2_dlg_pu;
        // Maximum of the two faulted phases
        let i_ke2e1_pu = ib_dlg.norm().max(ic_dlg.norm());

        // ── Base conversion ───────────────────────────────────────────────
        // Determine base voltage for this bus (Voltage newtype .0 gives raw f64 in kV)
        let bus_kv = {
            let bus = &network.buses[f];
            if bus.base_kv.0 > 1e-6 {
                bus.base_kv.0
            } else {
                1.0
            }
        };
        // I_base = S_base / (√3 * V_base)  [kA]
        let i_base_ka = network.base_mva / (3.0_f64.sqrt() * bus_kv);
        // Z_base = V_base² / S_base  [Ω]
        let z_base_ohm = (bus_kv * bus_kv) / network.base_mva;

        let i_k3_ka = i_k3_pu * i_base_ka;
        let i_k2_ka = i_k2_pu * i_base_ka;
        let i_k1_ka = i_k1_pu * i_base_ka;
        let i_k_e2e1_ka = i_ke2e1_pu * i_base_ka;

        // Short-circuit power S"k = √3 * V_n * I"k3  [MVA]
        let s_k_mva = 3.0_f64.sqrt() * bus_kv * i_k3_ka;

        // Thevenin impedance magnitude at fault bus [Ω]
        let z_k_ohm = z1ff.norm() * z_base_ohm;

        // R/X ratio at fault point
        let r_x_ratio = if z1ff.im.abs() < 1e-12 {
            10.0 // effectively resistive
        } else {
            (z1ff.re / z1ff.im).abs()
        };

        // ── Peak factor κ and peak current ───────────────────────────────
        let kappa = Self::compute_kappa(r_x_ratio);
        // ip = κ · √2 · I"k3  (IEC 60909 Eq. 52)
        let i_p_ka = kappa * 2.0_f64.sqrt() * i_k3_ka;

        // ── Thermal equivalent current ────────────────────────────────────
        // Ith = I"k · √(m + n)  (IEC 60909 Eq. 75)
        // where m accounts for DC aperiodic component, n for AC component.
        let tc = self.clearing_time_s;
        let (m, n_factor) = Self::compute_thermal_factor(tc, r_x_ratio);
        let i_th_ka = i_k3_ka * (m + n_factor).sqrt();
        let thermal_energy_ka2s = i_th_ka * i_th_ka * tc;

        // ── Breaking current ──────────────────────────────────────────────
        // For high-voltage circuit breakers: Ib ≈ μ · I"k  (μ ≤ 1.0)
        // μ factor from IEC 60909 Fig 13 (depends on X/R and contact parting time).
        // Conservative approximation: Ib = I"k3 (μ = 1 for large X/R).
        let i_b_ka = Some(i_k3_ka);

        Ok(ShortCircuitResult {
            fault_bus: f,
            fault_type: input.fault_type,
            i_k3_ka,
            i_k2_ka,
            i_k1_ka,
            i_k_e2e1_ka,
            s_k_mva,
            z_k_ohm,
            r_x_ratio,
            i_p_ka,
            kappa,
            i_th_ka,
            thermal_energy_ka2s,
            i_b_ka,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Induction motor short-circuit contribution
// ─────────────────────────────────────────────────────────────────────────────

/// Group of induction motors at a bus, modelled as a single equivalent machine.
///
/// During a system short-circuit, induction motors contribute to the fault
/// current because they are driven by their stored magnetic energy.
/// This contribution decays rapidly (time constant T'd ≈ 20–80 ms) because
/// induction motors have no independent excitation.
///
/// Model per IEC 60909-0 Clause 4.7.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InductionMotorGroup {
    /// Bus index (0-based) where the motor group is connected.
    pub bus: usize,
    /// Aggregate rated apparent power of the motor group \[MVA\].
    pub total_mva: f64,
    /// Rated line voltage of the motor terminals \[kV\].
    pub rated_voltage_kv: f64,
    /// Rated power factor (cos φ).  Default: 0.85.
    pub power_factor: f64,
    /// Rated shaft efficiency η.  Default: 0.92.
    pub efficiency: f64,
    /// Locked-rotor current ratio Ilr/In (ILR at starting).  Default: 5.5.
    pub locked_rotor_current: f64,
    /// Transient (subtransient) reactance X'd \[p.u. on motor base\].  Default: 0.20.
    pub x_d_prime: f64,
    /// Transient open-circuit time constant T'd \[s\].  Default: 0.040 s.
    pub t_d_prime_s: f64,
}

impl InductionMotorGroup {
    /// Create a new induction motor group with default electrical parameters.
    pub fn new(bus: usize, total_mva: f64, voltage_kv: f64) -> Self {
        Self {
            bus,
            total_mva,
            rated_voltage_kv: voltage_kv,
            power_factor: 0.85,
            efficiency: 0.92,
            locked_rotor_current: 5.5,
            x_d_prime: 0.20,
            t_d_prime_s: 0.040,
        }
    }

    /// Initial symmetrical SC contribution current at t = 0 \[kA RMS\].
    ///
    /// I_motor = LRC · I_rated   where   I_rated = S_rated / (√3 · V_rated)
    ///
    /// # Arguments
    /// - `bus_voltage_pu`  — pre-fault terminal voltage \[p.u.\]
    /// - `_base_mva`       — system MVA base (unused; motor current in physical units)
    /// - `_base_kv`        — system kV base at motor bus (unused; motor uses rated kV)
    pub fn sc_contribution_ka(&self, bus_voltage_pu: f64, _base_mva: f64, _base_kv: f64) -> f64 {
        if self.total_mva < 1e-9 || self.rated_voltage_kv < 1e-9 {
            return 0.0;
        }
        // Rated current of the motor group [kA]
        let i_rated_ka = self.total_mva / (3.0_f64.sqrt() * self.rated_voltage_kv);
        // Contribution at fault: proportional to terminal voltage
        // IEC 60909: I"motor = Ilr/In * In * (V_bus / V_rated)
        i_rated_ka * self.locked_rotor_current * bus_voltage_pu
    }

    /// Decaying SC contribution at time t \[s\] after fault inception \[kA\].
    ///
    /// Induction motor model: exponential decay toward zero (no sustained contribution).
    ///
    /// I(t) = I0 · exp(−t / T'd)
    ///
    /// where I0 is the initial contribution (t = 0).
    pub fn contribution_at_t(&self, t_s: f64, initial_ka: f64) -> f64 {
        if self.t_d_prime_s < 1e-9 {
            return 0.0;
        }
        initial_ka * (-t_s / self.t_d_prime_s).exp()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Synchronous motor short-circuit contribution
// ─────────────────────────────────────────────────────────────────────────────

/// Group of synchronous motors or condensers at a bus.
///
/// Synchronous machines contribute sustained fault current because they have
/// independent field excitation.  The transient current decays from the
/// subtransient value I" through the transient I' to the steady-state Id.
///
/// Model per Anderson "Analysis of Faulted Power Systems" Ch. 10.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SynchronousMotorGroup {
    /// Bus index (0-based) where the motor group is connected.
    pub bus: usize,
    /// Aggregate rated apparent power \[MVA\].
    pub total_mva: f64,
    /// Rated terminal voltage \[kV\].
    pub rated_voltage_kv: f64,
    /// Direct-axis subtransient reactance X"d \[p.u. on machine base\].  Default: 0.15.
    pub x_d_subtransient: f64,
    /// Direct-axis transient reactance X'd \[p.u. on machine base\].  Default: 0.25.
    pub x_d_transient: f64,
    /// Direct-axis subtransient time constant T"d \[s\].  Default: 0.015 s.
    pub t_d_subtransient_s: f64,
    /// Direct-axis transient time constant T'd \[s\].  Default: 0.80 s.
    pub t_d_transient_s: f64,
    /// Field voltage (no-load internal EMF) E'fd \[p.u.\].  Default: 1.05.
    pub e_fd: f64,
}

impl SynchronousMotorGroup {
    /// Create with default subtransient/transient parameters.
    pub fn new(bus: usize, total_mva: f64, voltage_kv: f64) -> Self {
        Self {
            bus,
            total_mva,
            rated_voltage_kv: voltage_kv,
            x_d_subtransient: 0.15,
            x_d_transient: 0.25,
            t_d_subtransient_s: 0.015,
            t_d_transient_s: 0.80,
            e_fd: 1.05,
        }
    }

    /// Base current of the machine group \[kA\].
    fn base_current_ka(&self) -> f64 {
        if self.rated_voltage_kv < 1e-9 {
            return 0.0;
        }
        self.total_mva / (3.0_f64.sqrt() * self.rated_voltage_kv)
    }

    /// Subtransient SC current I" \[kA\] at fault inception (t → 0⁺).
    ///
    /// I" = E"fd / X"d   in per-unit on machine base, then convert to kA.
    pub fn subtransient_sc_ka(&self, _base_mva: f64, _base_kv: f64) -> f64 {
        if self.x_d_subtransient < 1e-9 {
            return 0.0;
        }
        let i_pp_pu = self.e_fd / self.x_d_subtransient;
        i_pp_pu * self.base_current_ka()
    }

    /// Transient SC current I' \[kA\] at the transient time scale.
    fn transient_sc_ka(&self) -> f64 {
        if self.x_d_transient < 1e-9 {
            return 0.0;
        }
        let i_p_pu = self.e_fd / self.x_d_transient;
        i_p_pu * self.base_current_ka()
    }

    /// Steady-state SC current Id \[kA\] — sustained contribution with excitation.
    fn steady_state_sc_ka(&self) -> f64 {
        // Approximate: use synchronous reactance ≈ 1.0 to 2.0 pu
        // For simplicity model with X_d_synch ≈ 1.5 * X_d_transient
        let x_d_synch = (self.x_d_transient * 1.5).max(self.x_d_transient);
        if x_d_synch < 1e-9 {
            return 0.0;
        }
        let i_d_pu = self.e_fd / x_d_synch;
        i_d_pu * self.base_current_ka()
    }

    /// SC contribution at time t \[s\] after fault inception \[kA\].
    ///
    /// Classical two-component model:
    /// ```text
    ///   I(t) = (I" − I') · exp(−t/T"d) + (I' − Id) · exp(−t/T'd) + Id
    /// ```
    pub fn contribution_at_t(&self, t_s: f64) -> f64 {
        let i_pp = self.subtransient_sc_ka(0.0, 0.0); // base args unused
        let i_p = self.transient_sc_ka();
        let i_d = self.steady_state_sc_ka();

        let subtransient_decay = if self.t_d_subtransient_s > 1e-9 {
            (-t_s / self.t_d_subtransient_s).exp()
        } else {
            0.0
        };
        let transient_decay = if self.t_d_transient_s > 1e-9 {
            (-t_s / self.t_d_transient_s).exp()
        } else {
            0.0
        };

        (i_pp - i_p) * subtransient_decay + (i_p - i_d) * transient_decay + i_d
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Network-wide SC survey
// ─────────────────────────────────────────────────────────────────────────────

/// Compute short-circuit levels at every bus in the network.
///
/// Runs the IEC 60909 calculation for a bolted 3-phase fault at each bus,
/// optionally adding motor group contributions to each bus current.
///
/// # Arguments
/// - `network`       — power network
/// - `calculator`    — configured `Iec60909Calculator`
/// - `motor_groups`  — induction motor groups (may be empty)
///
/// # Returns
/// Vector of `(bus_index, ShortCircuitResult)` for all buses (0-based ordering).
#[allow(clippy::needless_range_loop)]
pub fn short_circuit_survey(
    network: &PowerNetwork,
    calculator: &Iec60909Calculator,
    motor_groups: &[InductionMotorGroup],
) -> Result<Vec<(usize, ShortCircuitResult)>> {
    let n = network.bus_count();
    if n == 0 {
        return Ok(Vec::new());
    }

    // Pre-build Z1 matrix once and reuse for all buses
    let z1 = calculator.build_z1_matrix(network)?;

    let mut results = Vec::with_capacity(n);

    for bus_idx in 0..n {
        let input = ShortCircuitInput::bolted_three_phase(bus_idx);
        let mut result = calculator.compute(network, &input)?;

        // Add motor group contribution at this bus
        let motor_contribution_ka: f64 = motor_groups
            .iter()
            .filter(|mg| mg.bus == bus_idx)
            .map(|mg| {
                // Use the Thevenin voltage at the bus (from Z1 diagonal)
                let z1ff = z1[bus_idx][bus_idx];
                // Driving point voltage ≈ c_factor * 1.0 pu
                let v_eq = calculator.c_factor;
                let i_k3_base = if z1ff.norm() > 1e-12 {
                    v_eq / z1ff.norm()
                } else {
                    0.0
                };
                // Convert pu to kA for voltage context
                let bus_kv = {
                    let b = &network.buses[bus_idx];
                    if b.base_kv.0 > 1e-6 {
                        b.base_kv.0
                    } else {
                        1.0
                    }
                };
                let i_base_ka = network.base_mva / (3.0_f64.sqrt() * bus_kv);
                let v_bus_pu = (i_k3_base * i_base_ka).clamp(0.0, 1.0);
                // Reuse i_base_ka as scale — motor contribution in kA
                mg.sc_contribution_ka(
                    v_bus_pu.max(0.9), // pre-fault ~ 1.0 pu
                    network.base_mva,
                    bus_kv,
                )
            })
            .sum();

        // Add motor contribution to SC currents (superposition)
        result.i_k3_ka += motor_contribution_ka;
        result.i_k2_ka += motor_contribution_ka * 3.0_f64.sqrt() / 2.0;
        result.i_k1_ka += motor_contribution_ka;
        // Recompute peak with updated current
        result.i_p_ka = result.kappa * 2.0_f64.sqrt() * result.i_k3_ka;
        // Recompute thermal
        let (m, n_f) = Iec60909Calculator::compute_thermal_factor(
            calculator.clearing_time_s,
            result.r_x_ratio,
        );
        result.i_th_ka = result.i_k3_ka * (m + n_f).sqrt();
        result.thermal_energy_ka2s = result.i_th_ka * result.i_th_ka * calculator.clearing_time_s;
        result.i_b_ka = Some(result.i_k3_ka);

        results.push((bus_idx, result));
    }

    Ok(results)
}

/// Find minimum and maximum 3-phase SC current levels across a survey.
///
/// # Returns
/// `(min_bus_idx, min_i_k3_ka, max_bus_idx, max_i_k3_ka)`
///
/// Returns `(0, 0.0, 0, 0.0)` for an empty survey.
pub fn sc_level_extremes(survey: &[(usize, ShortCircuitResult)]) -> (usize, f64, usize, f64) {
    if survey.is_empty() {
        return (0, 0.0, 0, 0.0);
    }

    let mut min_bus = survey[0].0;
    let mut min_ka = survey[0].1.i_k3_ka;
    let mut max_bus = survey[0].0;
    let mut max_ka = survey[0].1.i_k3_ka;

    for (bus_idx, result) in survey.iter() {
        let ka = result.i_k3_ka;
        if ka < min_ka {
            min_ka = ka;
            min_bus = *bus_idx;
        }
        if ka > max_ka {
            max_ka = ka;
            max_bus = *bus_idx;
        }
    }

    (min_bus, min_ka, max_bus, max_ka)
}

// ─────────────────────────────────────────────────────────────────────────────
// Relay verification
// ─────────────────────────────────────────────────────────────────────────────

/// Result of verifying relay settings against computed SC levels at one bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelayVerification {
    /// Index into the relay settings slice.
    pub relay_idx: usize,
    /// Bus index where the relay operates.
    pub bus: usize,
    /// Sensitivity check: relay pickup is below maximum SC current at bus.
    /// If `false`, the relay will not trip for a nearby fault (blind zone).
    pub max_reach_ok: bool,
    /// Selectivity check: relay pickup is above minimum far-end SC current.
    /// If `false`, the relay may over-reach into adjacent zones.
    pub min_reach_ok: bool,
    /// Overall coordination flag (both checks must pass).
    pub coordination_ok: bool,
    /// Textual notes for any failed checks.
    pub notes: Vec<String>,
}

/// Relay settings record for coordination verification.
///
/// Associates an overcurrent relay with the bus it protects and its
/// expected minimum SC current (far-end, for selectivity check).
#[derive(Debug, Clone)]
pub struct RelaySettings {
    /// Overcurrent relay parameters.
    pub relay: OcRelay,
    /// Bus index (0-based) this relay is protecting.
    pub bus: usize,
    /// Minimum SC current the relay must NOT trip for (far-end selectivity) \[kA\].
    /// Set to 0.0 to skip the min-reach check.
    pub min_reach_ka: f64,
}

/// Verify relay pickup settings against network-wide SC survey results.
///
/// For each relay setting, checks:
/// 1. **Sensitivity (max-reach)**: relay pickup × √3 × V_base < max SC current at bus.
///    The relay must see and trip for the maximum local fault current.
/// 2. **Selectivity (min-reach)**: if `min_reach_ka > 0`, ensures pickup × √3 × V_base
///    does not trip for far-end minimum SC currents.
///
/// # Arguments
/// - `survey`          — SC survey (bus index, result) pairs
/// - `relay_settings`  — relay configurations to verify
pub fn verify_relay_settings(
    survey: &[(usize, ShortCircuitResult)],
    relay_settings: &[RelaySettings],
) -> Vec<RelayVerification> {
    // Build bus → SC result lookup
    let mut sc_at_bus: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
    for (bus_idx, result) in survey {
        sc_at_bus.insert(*bus_idx, result.i_k3_ka);
    }

    relay_settings
        .iter()
        .enumerate()
        .map(|(relay_idx, rs)| {
            let i_pickup_ka = rs.relay.i_pickup; // already in kA (or same unit as SC)
            let max_sc_ka = sc_at_bus.get(&rs.bus).copied().unwrap_or(0.0);

            let mut notes = Vec::new();

            // Sensitivity: relay must see maximum SC at its own bus
            let max_reach_ok = i_pickup_ka < max_sc_ka;
            if !max_reach_ok {
                notes.push(format!(
                    "Relay {relay_idx} at bus {}: pickup {:.3} kA >= max SC {:.3} kA — relay is blind",
                    rs.bus, i_pickup_ka, max_sc_ka
                ));
            }

            // Selectivity: relay must not trip for minimum far-end SC
            let min_reach_ok = if rs.min_reach_ka > 1e-9 {
                // Relay should NOT trip for far-end min SC: pickup > min_reach
                let ok = i_pickup_ka > rs.min_reach_ka;
                if !ok {
                    notes.push(format!(
                        "Relay {relay_idx} at bus {}: pickup {:.3} kA < far-end min SC {:.3} kA — over-reach risk",
                        rs.bus, i_pickup_ka, rs.min_reach_ka
                    ));
                }
                ok
            } else {
                true // no far-end minimum specified
            };

            RelayVerification {
                relay_idx,
                bus: rs.bus,
                max_reach_ok,
                min_reach_ok,
                coordination_ok: max_reach_ok && min_reach_ok,
                notes,
            }
        })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// ANSI/IEEE C37.010 Calculator
// ─────────────────────────────────────────────────────────────────────────────

/// ANSI/IEEE C37.010-1999 short-circuit duty calculator.
///
/// Computes:
/// - **First-cycle (momentary) duty** — asymmetrical peak current for
///   circuit breaker close-and-latch (withstand) rating.
/// - **Interrupting duty** — asymmetrical RMS current at contact parting
///   time for circuit breaker interrupting rating.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnsiCalculator {
    /// Multiplying factor for first-cycle duty.
    ///
    /// ANSI/IEEE uses 1.6 for close-and-latch (peak) duty at 60 Hz for
    /// networks with X/R ≤ 25, per IEEE C37.010 Clause 5.7.
    pub multiplying_factor: f64,
}

impl Default for AnsiCalculator {
    fn default() -> Self {
        Self::new()
    }
}

impl AnsiCalculator {
    /// Create with default ANSI multiplying factor of 1.6 (IEEE C37.010 typical).
    pub fn new() -> Self {
        Self {
            multiplying_factor: 1.6,
        }
    }

    /// Create with a custom multiplying factor.
    pub fn with_factor(mf: f64) -> Self {
        Self {
            multiplying_factor: mf,
        }
    }

    /// First-cycle (momentary) duty \[kA asymmetrical peak\].
    ///
    /// Used for circuit breaker close-and-latch (withstand) capability.
    ///
    /// I_mom = MF · √2 · I_sym   (ANSI C37.010 Eq. 1)
    ///
    /// # Arguments
    /// - `z_source`      — Thevenin impedance at the fault point \[p.u.\]
    /// - `v_pre_fault`   — pre-fault voltage \[p.u.\]
    /// - `base_kv`       — base voltage at fault bus \[kV\]
    /// - `base_mva`      — system MVA base \[MVA\]
    pub fn momentary_duty(
        &self,
        z_source: Complex64,
        v_pre_fault: f64,
        base_kv: f64,
        base_mva: f64,
    ) -> f64 {
        if z_source.norm() < 1e-12 || base_kv < 1e-9 {
            return 0.0;
        }
        // Symmetrical RMS fault current
        let i_sym_pu = v_pre_fault / z_source.norm();
        let i_base_ka = base_mva / (3.0_f64.sqrt() * base_kv);
        let i_sym_ka = i_sym_pu * i_base_ka;

        // First-cycle peak = MF · √2 · I_sym
        self.multiplying_factor * 2.0_f64.sqrt() * i_sym_ka
    }

    /// Interrupting duty \[kA asymmetrical RMS\] at contact parting time.
    ///
    /// Per IEEE C37.010, the asymmetrical interrupting current at the
    /// contact parting time accounts for DC offset decay:
    ///
    /// ```text
    ///   I_asym = I_sym · √(1 + 2 · e^{−2ω·t·R/X})
    /// ```
    ///
    /// where ω = 2π·f and t is the contact parting time.
    ///
    /// # Arguments
    /// - `i_sym_ka`             — symmetrical RMS fault current \[kA\]
    /// - `r_x_ratio`            — R/X at the fault point
    /// - `contact_parting_time_s` — time from fault to contact separation \[s\]
    ///   (typical: 0.033 s for 2-cycle, 0.050 s for 3-cycle, 0.083 s for 5-cycle CB)
    pub fn interrupting_duty(
        &self,
        i_sym_ka: f64,
        r_x_ratio: f64,
        contact_parting_time_s: f64,
    ) -> f64 {
        if i_sym_ka < 0.0 {
            return 0.0;
        }
        let omega = 2.0 * std::f64::consts::PI * 60.0; // 60 Hz ANSI
        let rx = r_x_ratio.max(0.0);
        let dc_decay = (-2.0 * omega * contact_parting_time_s * rx).exp();
        // Asymmetrical RMS = I_sym · √(1 + 2·e^{-2ωt·R/X})
        let asym_factor = (1.0 + 2.0 * dc_decay).sqrt();
        i_sym_ka * asym_factor
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::PowerNetwork;
    use crate::units::Voltage;

    // ── Network builder helpers ──────────────────────────────────────────

    /// 2-bus network: bus 1 (slack, 110 kV) — branch (r=0.01, x=0.10) — bus 2 (PQ)
    fn two_bus_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        let mut b1 = Bus::new(1, BusType::Slack);
        b1.base_kv = Voltage(110.0);
        let mut b2 = Bus::new(2, BusType::PQ);
        b2.base_kv = Voltage(110.0);
        net.buses.push(b1);
        net.buses.push(b2);
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.10,
            b: 0.02,
            rate_a: 200.0,
            rate_b: 200.0,
            rate_c: 200.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net
    }

    /// 3-bus ring network for survey tests.
    fn three_bus_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        for id in 1..=3usize {
            let bt = if id == 1 { BusType::Slack } else { BusType::PQ };
            let mut b = Bus::new(id, bt);
            b.base_kv = Voltage(33.0);
            net.buses.push(b);
        }
        for (from, to, r, x) in [(1, 2, 0.02, 0.15), (2, 3, 0.03, 0.20), (1, 3, 0.01, 0.12)] {
            net.branches.push(Branch {
                from_bus: from,
                to_bus: to,
                r,
                x,
                b: 0.01,
                rate_a: 100.0,
                rate_b: 100.0,
                rate_c: 100.0,
                tap: 0.0,
                shift: 0.0,
                status: true,
            });
        }
        net
    }

    // ── IEC 60909 basic tests ─────────────────────────────────────────────

    #[test]
    fn test_iec60909_bolted_3phase_positive() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let input = ShortCircuitInput::bolted_three_phase(0);
        let result = calc.compute(&net, &input).expect("compute failed");
        assert!(result.i_k3_ka > 0.0, "3-phase SC current must be positive");
        assert!(result.s_k_mva > 0.0, "SC MVA must be positive");
        assert!(result.z_k_ohm > 0.0, "Z_k must be positive");
    }

    #[test]
    fn test_iec60909_c_factor_scales_current() {
        let net = two_bus_network();
        let calc_1 = Iec60909Calculator::new(1.0);
        let calc_11 = Iec60909Calculator::new(1.1);
        let input = ShortCircuitInput::bolted_three_phase(0);
        let r1 = calc_1.compute(&net, &input).expect("compute failed");
        let r11 = calc_11.compute(&net, &input).expect("compute failed");
        // Higher c-factor → higher SC current
        assert!(
            r11.i_k3_ka > r1.i_k3_ka,
            "c=1.1 should give higher current than c=1.0"
        );
        // Ratio should match c-factor ratio closely
        let ratio = r11.i_k3_ka / r1.i_k3_ka;
        assert!((ratio - 1.1).abs() < 0.01, "ratio={:.4}", ratio);
    }

    #[test]
    fn test_iec60909_all_fault_types_positive() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let input = ShortCircuitInput::bolted_three_phase(0);
        let result = calc.compute(&net, &input).expect("compute failed");
        assert!(result.i_k3_ka > 0.0);
        assert!(result.i_k2_ka > 0.0);
        assert!(result.i_k1_ka > 0.0);
    }

    #[test]
    fn test_iec60909_fault_impedance_reduces_current() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let input_bolted = ShortCircuitInput::bolted_three_phase(0);
        let input_zf = ShortCircuitInput {
            fault_bus: 0,
            fault_type: FaultType::ThreePhase,
            // Real (resistive) fault impedance — always increases |Z_total| and reduces current
            fault_impedance: Complex64::new(1.0, 0.0),
            pre_fault_voltage_pu: 1.0,
        };
        let r_bolted = calc.compute(&net, &input_bolted).expect("compute failed");
        let r_zf = calc.compute(&net, &input_zf).expect("compute failed");
        assert!(
            r_bolted.i_k3_ka > r_zf.i_k3_ka,
            "Bolted fault must exceed resistive-impedance fault: {:.6} vs {:.6}",
            r_bolted.i_k3_ka,
            r_zf.i_k3_ka
        );
    }

    // ── κ peak factor tests ───────────────────────────────────────────────

    #[test]
    fn test_iec60909_kappa_high_rx() {
        // R/X = 0 (pure reactive) → κ = 1.02 + 0.98 = 2.00
        let kappa = Iec60909Calculator::compute_kappa(0.0);
        assert!(
            (kappa - 2.0).abs() < 1e-6,
            "κ(R/X=0) should be 2.0, got {:.6}",
            kappa
        );
    }

    #[test]
    fn test_iec60909_kappa_low_rx() {
        // R/X = 10 → κ ≈ 1.02 + 0.98·exp(-30) ≈ 1.02
        let kappa = Iec60909Calculator::compute_kappa(10.0);
        assert!(
            (kappa - 1.02).abs() < 1e-4,
            "κ(R/X=10) should be ≈1.02, got {:.6}",
            kappa
        );
    }

    #[test]
    fn test_iec60909_kappa_range() {
        // κ must always be in [1.02, 2.0]
        for &rx in &[0.0, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0] {
            let k = Iec60909Calculator::compute_kappa(rx);
            assert!(
                (1.02..=2.001).contains(&k),
                "κ={:.4} out of [1.02, 2.0] for R/X={rx}",
                k
            );
        }
    }

    #[test]
    fn test_iec60909_peak_current_consistent() {
        // ip = κ · √2 · I"k3
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let result = calc
            .compute(&net, &ShortCircuitInput::bolted_three_phase(0))
            .unwrap();
        let expected_ip = result.kappa * 2.0_f64.sqrt() * result.i_k3_ka;
        assert!(
            (result.i_p_ka - expected_ip).abs() < 1e-9,
            "ip={:.6} expected={:.6}",
            result.i_p_ka,
            expected_ip
        );
    }

    // ── Z-matrix dimension tests ──────────────────────────────────────────

    #[test]
    fn test_z_matrix_dimensions_2bus() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let z1 = calc.build_z1_matrix(&net).expect("Z1 build failed");
        let z2 = calc.build_z2_matrix(&net).expect("Z2 build failed");
        let z0 = calc.build_z0_matrix(&net, &[]).expect("Z0 build failed");
        let n = net.bus_count();
        assert_eq!(z1.len(), n);
        assert_eq!(z1[0].len(), n);
        assert_eq!(z2.len(), n);
        assert_eq!(z2[0].len(), n);
        assert_eq!(z0.len(), n);
        assert_eq!(z0[0].len(), n);
    }

    #[test]
    fn test_z_matrix_dimensions_3bus() {
        let net = three_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let z1 = calc.build_z1_matrix(&net).expect("Z1 build failed");
        let n = net.bus_count();
        assert_eq!(z1.len(), n);
        for row in &z1 {
            assert_eq!(row.len(), n, "Z1 must be n×n");
        }
    }

    #[test]
    fn test_z1_z2_equal_balanced() {
        // For balanced networks, Z1 = Z2 (positive == negative sequence)
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let z1 = calc.build_z1_matrix(&net).unwrap();
        let z2 = calc.build_z2_matrix(&net).unwrap();
        let n = net.bus_count();
        for i in 0..n {
            for j in 0..n {
                assert!(
                    (z1[i][j] - z2[i][j]).norm() < 1e-10,
                    "Z1[{i},{j}] != Z2[{i},{j}]"
                );
            }
        }
    }

    #[test]
    fn test_z_matrix_symmetric() {
        // Z-bus should be symmetric for passive networks
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let z1 = calc.build_z1_matrix(&net).unwrap();
        let n = net.bus_count();
        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            for j in 0..n {
                assert!(
                    (z1[i][j] - z1[j][i]).norm() < 1e-8,
                    "Z1 not symmetric at [{i},{j}]"
                );
            }
        }
    }

    // ── Grounding configuration tests ─────────────────────────────────────

    #[test]
    fn test_grounding_solidly_zero_impedance() {
        let cfg = GroundingConfig::SolidlyGrounded;
        let z = cfg.impedance_pu(110.0, 100.0);
        assert!(z.norm() < 1e-12, "Solidly grounded impedance must be 0");
    }

    #[test]
    fn test_grounding_delta_infinite_impedance() {
        let cfg = GroundingConfig::UngroundedDelta;
        let z = cfg.impedance_pu(110.0, 100.0);
        assert!(z.re > 1e6, "Delta impedance must be very large");
    }

    #[test]
    fn test_grounding_impedance_conversion() {
        let cfg = GroundingConfig::Impedance {
            r_ohm: 10.0,
            x_ohm: 0.0,
        };
        let z = cfg.impedance_pu(110.0, 100.0);
        // Z_base = 110^2 / 100 = 121 Ω → Z_pu = 10/121 ≈ 0.0826
        let z_base = 110.0_f64.powi(2) / 100.0;
        let expected = 10.0 / z_base;
        assert!(
            (z.re - expected).abs() < 1e-8,
            "Z_pu.re={:.6} expected={:.6}",
            z.re,
            expected
        );
    }

    // ── Motor contribution tests ──────────────────────────────────────────

    #[test]
    fn test_motor_contribution_positive() {
        let mg = InductionMotorGroup::new(0, 10.0, 6.3);
        let contrib = mg.sc_contribution_ka(1.0, 100.0, 6.3);
        assert!(contrib > 0.0, "Motor SC contribution must be positive");
    }

    #[test]
    fn test_motor_contribution_zero_mva() {
        let mg = InductionMotorGroup::new(0, 0.0, 6.3);
        let contrib = mg.sc_contribution_ka(1.0, 100.0, 6.3);
        assert!(
            (contrib).abs() < 1e-12,
            "Zero-MVA motor should give zero contribution"
        );
    }

    #[test]
    fn test_motor_contribution_decays() {
        let mg = InductionMotorGroup::new(0, 10.0, 6.3);
        let i0 = mg.sc_contribution_ka(1.0, 100.0, 6.3);
        let i_t1 = mg.contribution_at_t(0.05, i0);
        let i_t2 = mg.contribution_at_t(0.10, i0);
        assert!(
            i_t1 < i0,
            "Contribution at t=50ms should be less than t=0: {i_t1:.4} < {i0:.4}"
        );
        assert!(
            i_t2 < i_t1,
            "Contribution at t=100ms should be less than t=50ms: {i_t2:.4} < {i_t1:.4}"
        );
    }

    #[test]
    fn test_motor_contribution_decays_to_near_zero() {
        let mg = InductionMotorGroup::new(0, 10.0, 6.3);
        let i0 = mg.sc_contribution_ka(1.0, 100.0, 6.3);
        // After 10 time constants, contribution should be very small
        let i_late = mg.contribution_at_t(mg.t_d_prime_s * 10.0, i0);
        assert!(
            i_late < i0 * 1e-3,
            "Motor contribution should nearly vanish at 10*T'd: {i_late:.6} vs {i0:.4}"
        );
    }

    #[test]
    fn test_synchronous_motor_subtransient_positive() {
        let mg = SynchronousMotorGroup::new(0, 50.0, 11.0);
        let i_pp = mg.subtransient_sc_ka(100.0, 11.0);
        assert!(
            i_pp > 0.0,
            "Synchronous motor subtransient contribution must be positive"
        );
    }

    #[test]
    fn test_synchronous_motor_subtransient_gt_transient() {
        // I" (subtransient) > I' (transient) because X"d < X'd
        let mg = SynchronousMotorGroup::new(0, 50.0, 11.0);
        let i_pp = mg.subtransient_sc_ka(100.0, 11.0);
        let i_p = mg.transient_sc_ka();
        assert!(
            i_pp > i_p,
            "Subtransient current {i_pp:.4} should exceed transient {i_p:.4}"
        );
    }

    #[test]
    fn test_synchronous_motor_contribution_at_t() {
        let mg = SynchronousMotorGroup::new(0, 50.0, 11.0);
        let i_0 = mg.contribution_at_t(0.0);
        let i_50ms = mg.contribution_at_t(0.05);
        let i_1s = mg.contribution_at_t(1.0);
        // t=0 is the largest (subtransient peak)
        assert!(i_0 > i_50ms, "t=0 must exceed t=50ms");
        // Sustained field excitation — not zero at t=1s
        assert!(
            i_1s > 0.0,
            "Synchronous motor has sustained contribution (I_d > 0)"
        );
    }

    // ── Network survey tests ──────────────────────────────────────────────

    #[test]
    fn test_sc_survey_all_buses() {
        let net = three_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).expect("survey failed");
        assert_eq!(
            survey.len(),
            net.bus_count(),
            "Survey must cover all buses: expected {}, got {}",
            net.bus_count(),
            survey.len()
        );
    }

    #[test]
    fn test_sc_survey_bus_indices_match() {
        let net = three_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).expect("survey failed");
        for (i, (bus_idx, _)) in survey.iter().enumerate() {
            assert_eq!(*bus_idx, i, "Survey bus index mismatch at position {i}");
        }
    }

    #[test]
    fn test_sc_survey_positive_currents() {
        let net = three_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).expect("survey failed");
        for (bus_idx, result) in &survey {
            assert!(
                result.i_k3_ka > 0.0,
                "Bus {bus_idx}: i_k3_ka must be positive, got {:.4}",
                result.i_k3_ka
            );
        }
    }

    #[test]
    fn test_sc_survey_with_motor_groups() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let motors = vec![InductionMotorGroup::new(0, 5.0, 110.0)];
        let survey_no_motor = short_circuit_survey(&net, &calc, &[]).unwrap();
        let survey_with_motor = short_circuit_survey(&net, &calc, &motors).unwrap();
        // Bus 0 with motor should have higher SC current
        assert!(
            survey_with_motor[0].1.i_k3_ka >= survey_no_motor[0].1.i_k3_ka,
            "Motor contribution should increase bus 0 SC current"
        );
    }

    // ── SC extremes tests ─────────────────────────────────────────────────

    #[test]
    fn test_sc_extremes_valid_ordering() {
        let net = three_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).expect("survey failed");
        let (min_bus, min_ka, max_bus, max_ka) = sc_level_extremes(&survey);
        assert!(
            min_ka <= max_ka,
            "min SC {min_ka:.4} must be <= max SC {max_ka:.4}"
        );
        assert!(min_bus < net.bus_count(), "min_bus index must be valid");
        assert!(max_bus < net.bus_count(), "max_bus index must be valid");
    }

    #[test]
    fn test_sc_extremes_single_entry() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).unwrap();
        let (min_b, min_ka, max_b, max_ka) = sc_level_extremes(&survey);
        // For a 2-bus symmetric network, min and max may differ slightly
        assert!(min_ka > 0.0);
        assert!(max_ka >= min_ka);
        assert!(min_b < 2);
        assert!(max_b < 2);
    }

    #[test]
    fn test_sc_extremes_empty() {
        let (mb, mk, xb, xk) = sc_level_extremes(&[]);
        assert_eq!(mb, 0);
        assert_eq!(xb, 0);
        assert!((mk).abs() < 1e-12);
        assert!((xk).abs() < 1e-12);
    }

    // ── Relay verification tests ──────────────────────────────────────────

    #[test]
    fn test_verify_relay_sensitive_relay() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).unwrap();
        let sc_ka = survey[0].1.i_k3_ka;

        use crate::protection::relay::{OcRelay, RelayCharacteristic};
        // Pickup << SC current → relay is sensitive (max_reach_ok = true)
        let relay_settings = vec![RelaySettings {
            relay: OcRelay::new(sc_ka * 0.1, 0.5, RelayCharacteristic::StandardInverse),
            bus: 0,
            min_reach_ka: 0.0,
        }];
        let verifications = verify_relay_settings(&survey, &relay_settings);
        assert_eq!(verifications.len(), 1);
        assert!(
            verifications[0].max_reach_ok,
            "Relay with low pickup should be sensitive"
        );
    }

    #[test]
    fn test_verify_relay_blind_relay() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let survey = short_circuit_survey(&net, &calc, &[]).unwrap();
        let sc_ka = survey[0].1.i_k3_ka;

        use crate::protection::relay::{OcRelay, RelayCharacteristic};
        // Pickup >> SC current → relay is blind (max_reach_ok = false)
        let relay_settings = vec![RelaySettings {
            relay: OcRelay::new(sc_ka * 10.0, 0.5, RelayCharacteristic::StandardInverse),
            bus: 0,
            min_reach_ka: 0.0,
        }];
        let verifications = verify_relay_settings(&survey, &relay_settings);
        assert!(
            !verifications[0].max_reach_ok,
            "Relay with very high pickup should be blind"
        );
    }

    // ── ANSI/IEEE calculator tests ────────────────────────────────────────

    #[test]
    fn test_ansi_momentary_duty_positive() {
        let calc = AnsiCalculator::new();
        let z = Complex64::new(0.01, 0.12);
        let duty = calc.momentary_duty(z, 1.0, 110.0, 100.0);
        assert!(duty > 0.0, "Momentary duty must be positive: {duty:.4}");
    }

    #[test]
    fn test_ansi_momentary_duty_zero_for_zero_impedance() {
        let calc = AnsiCalculator::new();
        let z = Complex64::new(0.0, 0.0);
        let duty = calc.momentary_duty(z, 1.0, 110.0, 100.0);
        assert!(
            (duty).abs() < 1e-12,
            "Zero impedance should return 0 (protect against inf)"
        );
    }

    #[test]
    fn test_ansi_interrupting_duty_positive() {
        let calc = AnsiCalculator::new();
        let duty = calc.interrupting_duty(10.0, 0.1, 0.05);
        assert!(duty > 0.0, "Interrupting duty must be positive");
    }

    #[test]
    fn test_ansi_interrupting_duty_decays_with_time() {
        let calc = AnsiCalculator::new();
        // Longer contact parting time → smaller DC offset → lower asymmetrical current
        let i_fast = calc.interrupting_duty(10.0, 5.0, 0.033);
        let i_slow = calc.interrupting_duty(10.0, 5.0, 0.083);
        assert!(
            i_fast >= i_slow,
            "Faster interruption should have higher asymmetry: {i_fast:.4} >= {i_slow:.4}"
        );
    }

    #[test]
    fn test_ansi_interrupting_duty_approaches_sym_at_large_t() {
        let calc = AnsiCalculator::new();
        // Very large contact parting time → DC offset decays → I_asym → I_sym
        let i_asym = calc.interrupting_duty(10.0, 0.0, 10.0); // R/X=0 but long time
                                                              // With R/X=0: asym_factor = sqrt(1 + 2*exp(-2*ω*10*0)) = sqrt(3) ≠ 1
                                                              // Test with large R/X instead
        let i_asym_rx = calc.interrupting_duty(10.0, 100.0, 1.0);
        // With very high R/X: DC decays very fast → I_asym ≈ I_sym = 10 kA
        assert!(
            (i_asym_rx - 10.0).abs() < 0.01,
            "High R/X should make asym ≈ sym: {i_asym_rx:.6}"
        );
        let _ = i_asym; // used in assertion context
    }

    #[test]
    fn test_ansi_interrupting_duty_negative_input() {
        let calc = AnsiCalculator::new();
        // Negative SC current → return 0
        let duty = calc.interrupting_duty(-5.0, 0.1, 0.05);
        assert!((duty).abs() < 1e-12, "Negative I_sym should yield 0");
    }

    #[test]
    fn test_ansi_default_factor() {
        let calc = AnsiCalculator::new();
        assert!((calc.multiplying_factor - 1.6).abs() < 1e-9);
    }

    // ── Thermal factor tests ──────────────────────────────────────────────

    #[test]
    fn test_thermal_factor_positive() {
        let (m, n) = Iec60909Calculator::compute_thermal_factor(0.1, 0.1);
        assert!(m >= 0.0, "m factor must be non-negative");
        assert!(n > 0.0, "n factor must be positive");
    }

    #[test]
    fn test_thermal_equivalent_current_positive() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let result = calc
            .compute(&net, &ShortCircuitInput::bolted_three_phase(0))
            .unwrap();
        assert!(
            result.i_th_ka > 0.0,
            "Thermal equivalent current must be positive"
        );
        assert!(
            result.thermal_energy_ka2s > 0.0,
            "Thermal energy must be positive"
        );
    }

    // ── Out-of-range bus test ─────────────────────────────────────────────

    #[test]
    fn test_sc_out_of_range_bus() {
        let net = two_bus_network();
        let calc = Iec60909Calculator::new(1.0);
        let input = ShortCircuitInput::bolted_three_phase(99);
        let result = calc.compute(&net, &input);
        assert!(result.is_err(), "Out-of-range bus should return error");
    }
}
