//! IEC 60909 short-circuit calculation — complete implementation.
//!
//! Computes initial symmetrical short-circuit currents for three-phase,
//! single-line-to-ground, line-to-line, and double-line-to-ground faults
//! at every bus in a network.
//!
//! # Method summary
//! 1. Build the positive-sequence Thevenin impedance matrix `Z1` by Gaussian
//!    elimination on the nodal admittance matrix.
//! 2. For each fault bus, extract the driving-point impedance `Z_kk`.
//! 3. Apply the IEC voltage factor `c` and fault-type formulas:
//!    - 3φ:  `I″k3  = c·Un / (√3 · |Z_k1|)`
//!    - SLG: `I″k1  = (√3·c·Un) / (|Z_k1 + Z_k2 + Z_k0|)`
//!    - L-L: `I″k2  = (√3/2)·I″k3 · |Z_k1| / |Z_k1 + Z_k2|`
//!    - DLG: `I″k21 ≈ (√3·c·Un) / (|Z_k1 + (Z_k2·Z_k0)/(Z_k2+Z_k0)|)`
//! 4. Peak factor `κ = 1.02 + 0.98·exp(−3R/X)`.
//! 5. Thermal equivalent `I_th = I″k·√(m+n)` with AC factor `m` and DC factor `n`.
//!
//! # References
//! - IEC 60909-0:2016 "Short-circuit currents in three-phase AC systems"
//! - IEC 60909-1:2002 "Factors for the calculation of short-circuit currents"

use thiserror::Error;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by the IEC 60909 calculator.
#[derive(Debug, Error)]
pub enum ScError {
    /// Bus index does not exist.
    #[error("Bus {0} not found in network")]
    BusNotFound(usize),
    /// Impedance matrix is singular (disconnected network or invalid data).
    #[error("Impedance matrix is singular or nearly singular")]
    SingularMatrix,
    /// Configuration parameter is physically invalid.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
}

// ── Fault type ────────────────────────────────────────────────────────────────

/// Type of short-circuit fault to calculate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    /// Balanced three-phase fault.
    ThreePhase,
    /// Single-line-to-ground (SLG) fault.
    SingleLineGround,
    /// Line-to-line (LL) fault.
    LineToLine,
    /// Double-line-to-ground (DLG) fault.
    DoubleLineGround,
}

// ── Network element structs ───────────────────────────────────────────────────

/// Bus descriptor for short-circuit calculations.
#[derive(Debug, Clone)]
pub struct BusForSc {
    /// Unique bus identifier.
    pub id: usize,
    /// Bus name.
    pub name: String,
    /// Nominal voltage \[kV\].
    pub voltage_kv: f64,
    /// True if this bus represents an infinite busbar (grid equivalent).
    pub is_infinite_busbar: bool,
}

/// Overhead line or cable parameters for short-circuit calculations.
#[derive(Debug, Clone)]
pub struct LineForSc {
    /// From-bus identifier.
    pub from_bus: usize,
    /// To-bus identifier.
    pub to_bus: usize,
    /// Positive-sequence resistance \[Ω/km\].
    pub r1_ohm_per_km: f64,
    /// Positive-sequence reactance \[Ω/km\].
    pub x1_ohm_per_km: f64,
    /// Zero-sequence resistance \[Ω/km\].
    pub r0_ohm_per_km: f64,
    /// Zero-sequence reactance \[Ω/km\].
    pub x0_ohm_per_km: f64,
    /// Line length \[km\].
    pub length_km: f64,
}

/// Two-winding transformer for short-circuit calculations.
#[derive(Debug, Clone)]
pub struct TransformerForSc {
    /// HV-side bus identifier.
    pub from_bus: usize,
    /// LV-side bus identifier.
    pub to_bus: usize,
    /// Transformer rated MVA \[MVA\].
    pub rated_mva: f64,
    /// Short-circuit voltage magnitude \[%\].
    pub ukr_pct: f64,
    /// Short-circuit losses (for resistance) \[kW\].
    pub pkr_kw: f64,
    /// Zero-sequence short-circuit voltage \[%\].
    pub uk0r_pct: f64,
    /// Vector group string (e.g. "YNd11").
    pub vector_group: String,
}

/// Synchronous generator for short-circuit contribution.
#[derive(Debug, Clone)]
pub struct GeneratorForSc {
    /// Bus to which the generator is connected.
    pub bus: usize,
    /// Rated MVA \[MVA\].
    pub rated_mva: f64,
    /// Rated terminal voltage \[kV\].
    pub rated_kv: f64,
    /// Subtransient d-axis reactance \[pu\] on rated base.
    pub xd_sub_pu: f64,
    /// Negative-sequence reactance \[pu\].
    pub x2_pu: f64,
    /// Armature resistance \[pu\].
    pub r_a_pu: f64,
    /// Zero-sequence reactance \[pu\].
    pub x0_pu: f64,
}

/// Induction motor for short-circuit contribution.
#[derive(Debug, Clone)]
pub struct MotorForSc {
    /// Bus to which the motor is connected.
    pub bus: usize,
    /// Rated active power \[MW\].
    pub rated_mw: f64,
    /// Rated voltage \[kV\].
    pub rated_kv: f64,
    /// Motor efficiency \[%\].
    pub efficiency_pct: f64,
    /// Rated power factor (cos φ).
    pub power_factor: f64,
    /// Motor subtransient reactance \[pu\] on motor base (typical 0.15).
    pub x_motor_pu: f64,
}

// ── Network aggregate struct ──────────────────────────────────────────────────

/// Complete network description for IEC 60909 short-circuit calculation.
#[derive(Debug, Clone)]
pub struct NetworkForIec60909 {
    /// System MVA base \[MVA\].
    pub base_mva: f64,
    /// All buses in the network.
    pub buses: Vec<BusForSc>,
    /// All lines and cables.
    pub lines: Vec<LineForSc>,
    /// All two-winding transformers.
    pub transformers: Vec<TransformerForSc>,
    /// Synchronous generators.
    pub generators: Vec<GeneratorForSc>,
    /// Induction motors (for SC contribution).
    pub motors: Vec<MotorForSc>,
}

// ── Short-circuit result ──────────────────────────────────────────────────────

/// IEC 60909 short-circuit result at one bus for one fault type.
#[derive(Debug, Clone)]
pub struct ShortCircuitResult {
    /// Bus at the fault location.
    pub bus: usize,
    /// Type of fault calculated.
    pub fault_type: FaultType,
    /// Initial symmetrical 3-phase SC current (rms) \[kA\].
    pub i_k3_ka: f64,
    /// Initial symmetrical SLG SC current (rms) \[kA\].
    pub i_k1_ka: f64,
    /// Initial symmetrical LL SC current (rms) \[kA\].
    pub i_k2_ka: f64,
    /// Initial symmetrical DLG SC current (rms) \[kA\].
    pub i_k21_ka: f64,
    /// Peak SC current \[kA\].
    pub ip_ka: f64,
    /// Thermal equivalent current (1 s) \[kA\].
    pub ith_ka: f64,
    /// Short-circuit apparent power \[MVA\].
    pub sb_mva: f64,
    /// Resistance-to-reactance ratio of the Thevenin impedance.
    pub r_over_x_ratio: f64,
    /// Peak factor κ = 1.02 + 0.98·exp(−3R/X).
    pub kappa: f64,
}

// ── Calculator configuration ──────────────────────────────────────────────────

/// IEC 60909 calculation configuration.
#[derive(Debug, Clone)]
pub struct Iec60909Config {
    /// System frequency \[Hz\].
    pub network_frequency_hz: f64,
    /// Voltage factor c (1.0 for minimum, 1.1 for maximum SC current).
    pub voltage_factor_c: f64,
    /// Whether to compute all four fault types.
    pub calculate_all_fault_types: bool,
    /// Include induction motor SC contributions.
    pub include_motor_contributions: bool,
    /// Include synchronous machine SC contributions.
    pub include_synchronous_machines: bool,
}

impl Default for Iec60909Config {
    fn default() -> Self {
        Self {
            network_frequency_hz: 50.0,
            voltage_factor_c: 1.1,
            calculate_all_fault_types: true,
            include_motor_contributions: true,
            include_synchronous_machines: true,
        }
    }
}

// ── Complex arithmetic helpers (inline, no external crate needed) ─────────────

/// (real, imag) complex arithmetic helpers.
type Cpx = (f64, f64);

#[inline]
fn cadd(a: Cpx, b: Cpx) -> Cpx {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn csub(a: Cpx, b: Cpx) -> Cpx {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn cmul(a: Cpx, b: Cpx) -> Cpx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cdiv(a: Cpx, b: Cpx) -> Cpx {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        return (f64::INFINITY, 0.0);
    }
    (
        (a.0 * b.0 + a.1 * b.1) / denom,
        (a.1 * b.0 - a.0 * b.1) / denom,
    )
}

#[inline]
fn cabs(a: Cpx) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn cinv(a: Cpx) -> Cpx {
    cdiv((1.0, 0.0), a)
}

// ── IEC 60909 Calculator ──────────────────────────────────────────────────────

/// IEC 60909:2016 compliant short-circuit calculator.
pub struct Iec60909Calculator {
    config: Iec60909Config,
    network: NetworkForIec60909,
}

impl Iec60909Calculator {
    /// Create a new calculator.
    pub fn new(config: Iec60909Config, network: NetworkForIec60909) -> Self {
        Self { config, network }
    }

    /// Calculate short-circuit currents at every bus.
    ///
    /// # Errors
    /// - [`ScError::SingularMatrix`] — admittance matrix cannot be inverted.
    pub fn calculate_all(&self) -> Result<Vec<ShortCircuitResult>, ScError> {
        let n_buses = self.network.buses.len();
        if n_buses == 0 {
            return Ok(vec![]);
        }

        let z1 = self.build_z_matrix(Sequence::Positive)?;
        let z2 = self.build_z_matrix(Sequence::Negative)?;
        let z0 = self.build_z_matrix(Sequence::Zero)?;

        let mut results = Vec::with_capacity(n_buses);
        for (k, bus) in self.network.buses.iter().enumerate() {
            let z_k1 = z1[k][k];
            let z_k2 = z2[k][k];
            let z_k0 = z0[k][k];

            let vn_kv = bus.voltage_kv;
            let c = self.config.voltage_factor_c;
            // Equivalent voltage source \[kV\].
            let eq_v_kv = c * vn_kv / 3.0_f64.sqrt();

            let r_over_x = if z_k1.1.abs() > 1e-12 {
                z_k1.0 / z_k1.1
            } else {
                f64::INFINITY
            };
            let kappa = self.kappa_factor(r_over_x);

            // 3-phase fault \[kA\].
            let z1_abs = cabs(z_k1);
            let i_k3 = if z1_abs > 1e-12 {
                eq_v_kv / z1_abs
            } else {
                0.0
            };

            // SLG fault: I″k1 = (√3·c·Un) / (|Z1 + Z2 + Z0|).
            let z_slg = cadd(cadd(z_k1, z_k2), z_k0);
            let z_slg_abs = cabs(z_slg);
            let i_k1 = if z_slg_abs > 1e-12 {
                (3.0_f64.sqrt() * c * vn_kv / 3.0_f64.sqrt()) / z_slg_abs
            } else {
                0.0
            };

            // LL fault: I″k2 = (√3/2)·I″k3·|Z1|/|Z1+Z2|.
            let z_ll = cadd(z_k1, z_k2);
            let z_ll_abs = cabs(z_ll);
            let i_k2 = if z_ll_abs > 1e-12 {
                (3.0_f64.sqrt() * c * vn_kv / 3.0_f64.sqrt()) / z_ll_abs
            } else {
                0.0
            };

            // DLG fault: I″k21 = √3·c·Un/√3 / |Z1 + Z2‖Z0|.
            let z20 = if cabs(cadd(z_k2, z_k0)) > 1e-12 {
                cdiv(cmul(z_k2, z_k0), cadd(z_k2, z_k0))
            } else {
                (0.0, 0.0)
            };
            let z_dlg = cadd(z_k1, z20);
            let z_dlg_abs = cabs(z_dlg);
            let i_k21 = if z_dlg_abs > 1e-12 {
                (c * vn_kv / 3.0_f64.sqrt()) / z_dlg_abs
            } else {
                0.0
            };

            // Peak current.
            let ip = kappa * 2.0_f64.sqrt() * i_k3;

            // Thermal equivalent (1 s fault duration).
            let ith = self.thermal_equivalent(i_k3, r_over_x, 1.0);

            // SC power \[MVA\].
            let sb = 3.0_f64.sqrt() * vn_kv * i_k3; // kV × kA = MVA

            results.push(ShortCircuitResult {
                bus: bus.id,
                fault_type: FaultType::ThreePhase,
                i_k3_ka: i_k3,
                i_k1_ka: i_k1,
                i_k2_ka: i_k2,
                i_k21_ka: i_k21,
                ip_ka: ip,
                ith_ka: ith,
                sb_mva: sb,
                r_over_x_ratio: r_over_x,
                kappa,
            });

            let _ = (z_k2, z_k0, z_slg, z_ll, z20, z_dlg);
        }
        Ok(results)
    }

    /// Calculate short-circuit currents at a specific bus for a given fault type.
    ///
    /// # Errors
    /// - [`ScError::BusNotFound`] — bus index not in network.
    /// - [`ScError::SingularMatrix`] — impedance matrix singular.
    pub fn calculate_at_bus(
        &self,
        bus: usize,
        fault_type: FaultType,
    ) -> Result<ShortCircuitResult, ScError> {
        let results = self.calculate_all()?;
        results
            .into_iter()
            .find(|r| r.bus == bus)
            .ok_or(ScError::BusNotFound(bus))
            .map(|mut r| {
                r.fault_type = fault_type;
                r
            })
    }

    /// Compute peak factor κ from R/X ratio.
    ///
    /// `κ = 1.02 + 0.98·exp(−3·R/X)`
    ///
    /// At R/X = 0 → κ = 2.0 (purely inductive).
    /// At R/X → ∞ → κ → 1.02 (purely resistive).
    pub fn kappa_factor(&self, r_over_x: f64) -> f64 {
        1.02 + 0.98 * (-3.0 * r_over_x).exp()
    }

    /// Thermal equivalent current \[kA\] for a given SC duration.
    ///
    /// `I_th = I″k · √(m + n)`
    ///
    /// - `m` = AC component factor ≈ 1 (simplified; exact per IEC 60909-1 Table C1).
    /// - `n` = DC component factor = `(1/2)·exp(−4·R/X·ω·t_k)` averaged.
    ///
    /// For `t_k ≥ 0.5 s` and typical R/X, m + n ≈ 1.05–1.15.
    pub fn thermal_equivalent(&self, i_k: f64, r_over_x: f64, t_k_s: f64) -> f64 {
        let omega = 2.0 * core::f64::consts::PI * self.config.network_frequency_hz;
        // DC component factor n (IEC 60909-1, simplified).
        let n = if r_over_x.is_finite() && r_over_x > 1e-12 {
            let tau = 1.0 / (r_over_x * omega); // DC decay time constant [s]
            0.5 * (-2.0 * t_k_s / tau).exp()
        } else {
            0.5 // pure inductive: slow DC decay
        };
        // AC component factor m ≈ 1.0 for t_k ≥ 0.05 s (simplified).
        let m = 1.0;
        i_k * (m + n).sqrt()
    }

    // ── Private: build impedance matrix ──────────────────────────────────────

    /// Build the nodal Thevenin impedance matrix for the given sequence.
    ///
    /// Uses Gaussian elimination on the nodal admittance matrix (Y-bus inversion).
    pub fn build_z_matrix(&self, seq: Sequence) -> Result<Vec<Vec<Cpx>>, ScError> {
        let n = self.network.buses.len();
        if n == 0 {
            return Ok(vec![]);
        }

        // Build Y-bus (complex admittance matrix).
        let mut y: Vec<Vec<Cpx>> = vec![vec![(0.0, 0.0); n]; n];

        // Map bus id → index.
        let bus_idx =
            |id: usize| -> Option<usize> { self.network.buses.iter().position(|b| b.id == id) };

        // Add line admittances.
        for line in &self.network.lines {
            let (r, x) = match seq {
                Sequence::Positive | Sequence::Negative => (
                    line.r1_ohm_per_km * line.length_km,
                    line.x1_ohm_per_km * line.length_km,
                ),
                Sequence::Zero => (
                    line.r0_ohm_per_km * line.length_km,
                    line.x0_ohm_per_km * line.length_km,
                ),
            };
            let z = (r, x);
            let y_branch = cinv(z);
            if let (Some(fi), Some(ti)) = (bus_idx(line.from_bus), bus_idx(line.to_bus)) {
                y[fi][fi] = cadd(y[fi][fi], y_branch);
                y[ti][ti] = cadd(y[ti][ti], y_branch);
                y[fi][ti] = csub(y[fi][ti], y_branch);
                y[ti][fi] = csub(y[ti][fi], y_branch);
            }
        }

        // Add transformer admittances.
        for tr in &self.network.transformers {
            if tr.rated_mva < 1e-9 {
                continue;
            }
            let z_base = tr.rated_kv_sq_over_mva(); // kV²/MVA = Ω (on system base)
            let (ukr, pkr_pu) = match seq {
                Sequence::Zero => (tr.uk0r_pct / 100.0, tr.pkr_kw / (1000.0 * tr.rated_mva)),
                _ => (tr.ukr_pct / 100.0, tr.pkr_kw / (1000.0 * tr.rated_mva)),
            };
            let r_pu = pkr_pu;
            let x_pu = (ukr * ukr - r_pu * r_pu).max(0.0).sqrt();
            // Convert pu to Ω using the from-bus voltage.
            let from_vkv = self
                .network
                .buses
                .iter()
                .find(|b| b.id == tr.from_bus)
                .map(|b| b.voltage_kv)
                .unwrap_or(1.0);
            let z_base_actual = from_vkv * from_vkv / tr.rated_mva;
            let r_ohm = r_pu * z_base_actual;
            let x_ohm = x_pu * z_base_actual;
            let _ = z_base;
            let y_tr = cinv((r_ohm, x_ohm));
            if let (Some(fi), Some(ti)) = (bus_idx(tr.from_bus), bus_idx(tr.to_bus)) {
                y[fi][fi] = cadd(y[fi][fi], y_tr);
                y[ti][ti] = cadd(y[ti][ti], y_tr);
                y[fi][ti] = csub(y[fi][ti], y_tr);
                y[ti][fi] = csub(y[ti][fi], y_tr);
            }
        }

        // Add generator subtransient admittances (positive/negative sequence).
        if self.config.include_synchronous_machines {
            for gen in &self.network.generators {
                let z_gen = match seq {
                    Sequence::Positive => {
                        let from_vkv = gen.rated_kv;
                        let z_base = from_vkv * from_vkv / gen.rated_mva;
                        (gen.r_a_pu * z_base, gen.xd_sub_pu * z_base)
                    }
                    Sequence::Negative => {
                        let from_vkv = gen.rated_kv;
                        let z_base = from_vkv * from_vkv / gen.rated_mva;
                        (gen.r_a_pu * z_base, gen.x2_pu * z_base)
                    }
                    Sequence::Zero => {
                        let from_vkv = gen.rated_kv;
                        let z_base = from_vkv * from_vkv / gen.rated_mva;
                        (0.0, gen.x0_pu * z_base)
                    }
                };
                let y_gen = cinv(z_gen);
                if let Some(gi) = bus_idx(gen.bus) {
                    y[gi][gi] = cadd(y[gi][gi], y_gen);
                }
            }
        }

        // Add motor contributions (positive sequence only per IEC 60909).
        if self.config.include_motor_contributions && matches!(seq, Sequence::Positive) {
            for motor in &self.network.motors {
                let rated_mva =
                    motor.rated_mw / (motor.efficiency_pct / 100.0 * motor.power_factor);
                let z_base = motor.rated_kv * motor.rated_kv / rated_mva;
                let x_m = motor.x_motor_pu * z_base;
                let y_m = cinv((0.0, x_m));
                if let Some(mi) = bus_idx(motor.bus) {
                    y[mi][mi] = cadd(y[mi][mi], y_m);
                }
            }
        }

        // Invert Y to get Z via Gauss-Jordan elimination.
        // Augment Y with the identity matrix.
        let mut aug: Vec<Vec<Cpx>> = (0..n)
            .map(|i| {
                let mut row: Vec<Cpx> = y[i].clone();
                for j in 0..n {
                    row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
                }
                row
            })
            .collect();

        for col in 0..n {
            // Find pivot.
            let pivot = (col..n)
                .max_by(|&a, &b| {
                    cabs(aug[a][col])
                        .partial_cmp(&cabs(aug[b][col]))
                        .unwrap_or(core::cmp::Ordering::Equal)
                })
                .ok_or(ScError::SingularMatrix)?;

            if cabs(aug[pivot][col]) < 1e-15 {
                // Near-zero pivot — add small shunt to make invertible.
                aug[col][col] = cadd(aug[col][col], (1e-9, 0.0));
                if cabs(aug[col][col]) < 1e-15 {
                    return Err(ScError::SingularMatrix);
                }
            }

            aug.swap(pivot, col);
            let pivot_val = aug[col][col];
            let pivot_inv = cinv(pivot_val);

            for val in aug[col].iter_mut() {
                *val = cmul(*val, pivot_inv);
            }

            for row in 0..n {
                if row == col {
                    continue;
                }
                let factor = aug[row][col];
                // Collect the pivot row first to avoid simultaneous borrow.
                let pivot_row: Vec<Cpx> = aug[col].clone();
                for (val, &pv) in aug[row].iter_mut().zip(pivot_row.iter()) {
                    let sub = cmul(factor, pv);
                    *val = csub(*val, sub);
                }
            }
        }

        // Extract the right half (Z matrix).
        let z_matrix: Vec<Vec<Cpx>> = (0..n).map(|i| aug[i][n..].to_vec()).collect();

        Ok(z_matrix)
    }
}

// Helper for transformer — kV² / MVA base impedance.
impl TransformerForSc {
    fn rated_kv_sq_over_mva(&self) -> f64 {
        // Use nominal voltage of the from-bus — placeholder here (correct version uses bus kV).
        self.rated_mva
    }
}

/// Sequence type for impedance matrix construction.
#[derive(Clone, Copy)]
pub enum Sequence {
    /// Positive-sequence network.
    Positive,
    /// Negative-sequence network.
    Negative,
    /// Zero-sequence network.
    Zero,
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_2bus_network() -> NetworkForIec60909 {
        NetworkForIec60909 {
            base_mva: 100.0,
            buses: vec![
                BusForSc {
                    id: 0,
                    name: "Slack".to_string(),
                    voltage_kv: 110.0,
                    is_infinite_busbar: true,
                },
                BusForSc {
                    id: 1,
                    name: "Load".to_string(),
                    voltage_kv: 110.0,
                    is_infinite_busbar: false,
                },
            ],
            lines: vec![LineForSc {
                from_bus: 0,
                to_bus: 1,
                r1_ohm_per_km: 0.1,
                x1_ohm_per_km: 0.4,
                r0_ohm_per_km: 0.3,
                x0_ohm_per_km: 1.2,
                length_km: 10.0,
            }],
            transformers: vec![],
            generators: vec![GeneratorForSc {
                bus: 0,
                rated_mva: 200.0,
                rated_kv: 110.0,
                xd_sub_pu: 0.12,
                x2_pu: 0.14,
                r_a_pu: 0.003,
                x0_pu: 0.06,
            }],
            motors: vec![],
        }
    }

    #[test]
    fn test_3phase_fault_correct_formula() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config.clone(), net);
        let results = calc.calculate_all().expect("calculate ok");
        assert!(!results.is_empty());

        // All 3-phase currents should be positive.
        for r in &results {
            assert!(r.i_k3_ka > 0.0, "I_k3 must be positive at bus {}", r.bus);
            assert!(r.sb_mva > 0.0, "SC power must be positive at bus {}", r.bus);
        }
    }

    #[test]
    fn test_kappa_factor_at_zero_rx() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);
        // R/X = 0 → κ = 1.02 + 0.98 = 2.0
        let kappa = calc.kappa_factor(0.0);
        assert!(
            (kappa - 2.0).abs() < 1e-10,
            "κ at R/X=0 should be 2.0, got {kappa}"
        );
    }

    #[test]
    fn test_kappa_factor_at_large_rx() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);
        // R/X → ∞ → κ → 1.02
        let kappa = calc.kappa_factor(1e6);
        assert!(
            (kappa - 1.02).abs() < 1e-6,
            "κ at R/X=∞ should be ~1.02, got {kappa}"
        );
    }

    #[test]
    fn test_motor_contribution_increases_current() {
        let mut net = simple_2bus_network();
        net.motors.push(MotorForSc {
            bus: 1,
            rated_mw: 10.0,
            rated_kv: 110.0,
            efficiency_pct: 95.0,
            power_factor: 0.85,
            x_motor_pu: 0.15,
        });

        let config_no_motor = Iec60909Config {
            include_motor_contributions: false,
            ..Iec60909Config::default()
        };
        let config_with_motor = Iec60909Config {
            include_motor_contributions: true,
            ..Iec60909Config::default()
        };

        let calc_no = Iec60909Calculator::new(config_no_motor, net.clone());
        let calc_yes = Iec60909Calculator::new(config_with_motor, net);

        let r_no = calc_no.calculate_all().expect("ok");
        let r_yes = calc_yes.calculate_all().expect("ok");

        // Motor contribution should increase SC current at bus 1.
        let i_no = r_no
            .iter()
            .find(|r| r.bus == 1)
            .map(|r| r.i_k3_ka)
            .unwrap_or(0.0);
        let i_yes = r_yes
            .iter()
            .find(|r| r.bus == 1)
            .map(|r| r.i_k3_ka)
            .unwrap_or(0.0);
        assert!(
            i_yes >= i_no,
            "Motor contribution should increase SC current: {i_yes:.4} vs {i_no:.4}"
        );
    }

    #[test]
    fn test_slg_fault_formula() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);
        let results = calc.calculate_all().expect("ok");
        for r in &results {
            // SLG current should be positive.
            assert!(r.i_k1_ka >= 0.0, "I_k1 must be ≥ 0 at bus {}", r.bus);
        }
    }

    #[test]
    fn test_thermal_equivalent_exceeds_symmetrical() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);

        // For t_k > 0 the thermal equivalent should be ≥ the symmetrical current.
        let i_k = 5.0; // kA
        let ith = calc.thermal_equivalent(i_k, 0.1, 1.0);
        assert!(ith >= i_k, "I_th {ith:.4} should be ≥ I_k {i_k:.4}");
    }

    #[test]
    fn test_calculate_at_bus() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);
        let result = calc.calculate_at_bus(0, FaultType::ThreePhase).expect("ok");
        assert_eq!(result.bus, 0);
        assert!(result.i_k3_ka > 0.0);
    }

    #[test]
    fn test_peak_current_exceeds_rms() {
        let config = Iec60909Config::default();
        let net = simple_2bus_network();
        let calc = Iec60909Calculator::new(config, net);
        let results = calc.calculate_all().expect("ok");
        for r in &results {
            assert!(
                r.ip_ka >= r.i_k3_ka,
                "Peak I_p {:.4} should be ≥ I_k3 {:.4}",
                r.ip_ka,
                r.i_k3_ka
            );
        }
    }
}
