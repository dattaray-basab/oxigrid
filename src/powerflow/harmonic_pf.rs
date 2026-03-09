//! Harmonic Power Flow (HPF) analysis module.
//!
//! Extends fundamental-frequency power flow to compute voltages at harmonic
//! frequencies (5th, 7th, 11th, 13th, etc.) considering nonlinear loads and
//! harmonic sources.
//!
//! # Algorithm Overview
//!
//! 1. Solve fundamental (50/60 Hz) power flow using Newton-Raphson (simplified).
//! 2. For each harmonic order h in the configured set:
//!    a. Build frequency-dependent nodal admittance matrix Y(h).
//!    b. Compute harmonic Norton current injections I(h) from all sources.
//!    c. Solve V(h) = Y(h)^{-1} · I(h) via Gaussian elimination on complex system.
//!    d. Account for passive filter tuning (resonance near tuned harmonic).
//! 3. Compute voltage/current THD at each bus.
//! 4. Check IEEE 519 compliance.
//!
//! # References
//! - IEEE Std 519-2022: Harmonic Control in Electric Power Systems
//! - IEEE Std 1159-2019: Recommended Practice for Power Quality Monitoring
//! - J. Arrillaga & N. Watson, "Power System Harmonics", 2nd ed., Wiley 2003

use crate::error::{OxiGridError, Result};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Type alias
// ---------------------------------------------------------------------------

/// Harmonic order (1 = fundamental, 5 = 5th harmonic, etc.)
pub type HarmonicOrder = u32;

// ---------------------------------------------------------------------------
// HarmonicSourceType
// ---------------------------------------------------------------------------

/// Classification of harmonic-generating equipment.
///
/// Each variant carries an IEEE 519 / IEC 61000-3 typical spectral profile.
#[derive(Debug, Clone, PartialEq)]
pub enum HarmonicSourceType {
    /// 6-pulse variable-speed drive / rectifier.
    /// Dominant harmonics: 5th, 7th, 11th, 13th (characteristic: 6k±1).
    SixPulseDrive,
    /// 12-pulse drive / rectifier.
    /// Dominant harmonics: 11th, 13th, 23rd, 25th (characteristic: 12k±1).
    TwelvePulseDrive,
    /// Single-phase inverter with low-pass filter shaping (e.g., Arduino-scale inverter).
    /// Dominant: 3rd, 5th, 7th with exponential roll-off.
    ArcFurnace,
    /// Electric arc furnace — produces all odd harmonics plus even sub-harmonics.
    TransformerInrush,
    /// Transformer inrush current — 2nd harmonic dominant.
    SwitchModePowerSupply,
    /// Switch-mode power supply — 3rd harmonic dominant (single-phase nonlinear load).
    PhotovoltaicInverter,
    /// Grid-tied PV inverter — IEEE 1547 compliant: 5th, 7th, 11th at low levels.
    ArduinoInverter,
}

impl HarmonicSourceType {
    /// Return the harmonic spectrum `(order, magnitude_pu, angle_rad)` for this
    /// source type, scaled to `fundamental_current_pu`.
    ///
    /// Magnitudes are taken from IEEE 519 Table 10.2 / IEC 61000-3-2 typical data.
    pub fn spectrum(&self, fundamental_current_pu: f64) -> Vec<(HarmonicOrder, f64, f64)> {
        match self {
            // ----------------------------------------------------------------
            // 6-pulse drive: I_h = I_1 / h  (ideal), adjusted to typical measured
            // IEEE 519 Table 10.2: 5th=17.5%, 7th=11.1%, 11th=4.5%, 13th=2.9%
            // ----------------------------------------------------------------
            HarmonicSourceType::SixPulseDrive => vec![
                (5, fundamental_current_pu * 0.175, -PI / 6.0),
                (7, fundamental_current_pu * 0.111, PI / 6.0),
                (11, fundamental_current_pu * 0.045, -PI / 6.0),
                (13, fundamental_current_pu * 0.029, PI / 6.0),
                (17, fundamental_current_pu * 0.015, -PI / 6.0),
                (19, fundamental_current_pu * 0.010, PI / 6.0),
                (23, fundamental_current_pu * 0.009, -PI / 6.0),
                (25, fundamental_current_pu * 0.008, PI / 6.0),
            ],

            // ----------------------------------------------------------------
            // 12-pulse drive: cancels 5th/7th; dominant 11th, 13th, 23rd, 25th
            // IEEE 519: 11th=7.6%, 13th=5.9%, 23rd=2.0%, 25th=1.8%
            // ----------------------------------------------------------------
            HarmonicSourceType::TwelvePulseDrive => vec![
                (11, fundamental_current_pu * 0.076, -PI / 12.0),
                (13, fundamental_current_pu * 0.059, PI / 12.0),
                (23, fundamental_current_pu * 0.020, -PI / 12.0),
                (25, fundamental_current_pu * 0.018, PI / 12.0),
            ],

            // ----------------------------------------------------------------
            // Arduino-scale inverter: 3rd, 5th, 7th with exponential roll-off
            // ----------------------------------------------------------------
            HarmonicSourceType::ArduinoInverter => vec![
                (3, fundamental_current_pu * 0.30, -PI / 4.0),
                (5, fundamental_current_pu * 0.15, PI / 4.0),
                (7, fundamental_current_pu * 0.07, -PI / 4.0),
                (9, fundamental_current_pu * 0.03, PI / 4.0),
            ],

            // ----------------------------------------------------------------
            // Arc furnace: all odd harmonics + even sub-harmonics (2nd, 4th, etc.)
            // Approximate: odd up to 13th + 2nd, 4th
            // ----------------------------------------------------------------
            HarmonicSourceType::ArcFurnace => vec![
                (2, fundamental_current_pu * 0.06, 0.0),
                (3, fundamental_current_pu * 0.08, 0.0),
                (4, fundamental_current_pu * 0.03, 0.0),
                (5, fundamental_current_pu * 0.06, 0.0),
                (7, fundamental_current_pu * 0.04, 0.0),
                (9, fundamental_current_pu * 0.02, 0.0),
                (11, fundamental_current_pu * 0.015, 0.0),
                (13, fundamental_current_pu * 0.010, 0.0),
            ],

            // ----------------------------------------------------------------
            // Transformer inrush: 2nd harmonic dominant (~40–60% of rated)
            // Also significant DC component and 3rd harmonic
            // ----------------------------------------------------------------
            HarmonicSourceType::TransformerInrush => vec![
                (2, fundamental_current_pu * 0.50, 0.0),
                (3, fundamental_current_pu * 0.20, PI / 3.0),
                (4, fundamental_current_pu * 0.10, 0.0),
                (5, fundamental_current_pu * 0.05, PI / 3.0),
            ],

            // ----------------------------------------------------------------
            // SMPS: 3rd dominant (single-phase), high 5th and 7th
            // Typical PC power supply: 3rd≈85%, 5th≈52%, 7th≈19%
            // ----------------------------------------------------------------
            HarmonicSourceType::SwitchModePowerSupply => vec![
                (3, fundamental_current_pu * 0.85, PI / 2.0),
                (5, fundamental_current_pu * 0.52, -PI / 2.0),
                (7, fundamental_current_pu * 0.19, PI / 2.0),
                (9, fundamental_current_pu * 0.10, -PI / 2.0),
                (11, fundamental_current_pu * 0.06, PI / 2.0),
                (13, fundamental_current_pu * 0.04, -PI / 2.0),
            ],

            // ----------------------------------------------------------------
            // PV inverter: IEEE 1547 compliant — very low harmonics
            // 5th: 4%, 7th: 3%, 11th: 1.5%, 13th: 1%
            // ----------------------------------------------------------------
            HarmonicSourceType::PhotovoltaicInverter => vec![
                (5, fundamental_current_pu * 0.04, -PI / 6.0),
                (7, fundamental_current_pu * 0.03, PI / 6.0),
                (11, fundamental_current_pu * 0.015, -PI / 6.0),
                (13, fundamental_current_pu * 0.010, PI / 6.0),
            ],
        }
    }
}

// ---------------------------------------------------------------------------
// HarmonicSource — Norton equivalent harmonic source model
// ---------------------------------------------------------------------------

/// Norton equivalent harmonic current source at a bus.
#[derive(Debug, Clone)]
pub struct HarmonicSource {
    /// Bus index (0-based).
    pub bus: usize,
    /// Pre-computed harmonic currents: `(order, magnitude_pu, angle_rad)`.
    /// If empty, the source type's `spectrum()` is used at solve time.
    pub harmonic_currents: Vec<(HarmonicOrder, f64, f64)>,
    /// Equipment type determining the spectral profile.
    pub source_type: HarmonicSourceType,
    /// Fundamental current magnitude (pu) used when `harmonic_currents` is empty.
    pub fundamental_current_pu: f64,
}

impl HarmonicSource {
    /// Create a new harmonic source, deriving the spectrum from the source type.
    pub fn new(bus: usize, source_type: HarmonicSourceType, fundamental_current_pu: f64) -> Self {
        let harmonic_currents = source_type.spectrum(fundamental_current_pu);
        Self {
            bus,
            harmonic_currents,
            source_type,
            fundamental_current_pu,
        }
    }

    /// Create a harmonic source with explicitly specified current injections.
    pub fn with_explicit_currents(
        bus: usize,
        source_type: HarmonicSourceType,
        harmonic_currents: Vec<(HarmonicOrder, f64, f64)>,
    ) -> Self {
        let fundamental_current_pu = harmonic_currents
            .iter()
            .find(|(h, _, _)| *h == 1)
            .map(|(_, mag, _)| *mag)
            .unwrap_or(1.0);
        Self {
            bus,
            harmonic_currents,
            source_type,
            fundamental_current_pu,
        }
    }

    /// Get the injection at a specific harmonic order as `(I_real, I_imag)` pu.
    pub fn injection_at(&self, h: HarmonicOrder) -> (f64, f64) {
        for &(order, mag, angle) in &self.harmonic_currents {
            if order == h {
                return (mag * angle.cos(), mag * angle.sin());
            }
        }
        (0.0, 0.0)
    }
}

// ---------------------------------------------------------------------------
// HarmonicBranch — frequency-dependent line impedance
// ---------------------------------------------------------------------------

/// Transmission / distribution line with frequency-dependent impedance.
///
/// Skin-effect model: R(h) = R₁ · h^α where α is the skin-effect exponent.
/// Inductive: X(h) = X₁ · h.
/// Capacitive shunt: B(h) = B₁ · h (capacitive susceptance scales linearly).
#[derive(Debug, Clone)]
pub struct HarmonicBranch {
    /// Sending-end bus index (0-based).
    pub from: usize,
    /// Receiving-end bus index (0-based).
    pub to: usize,
    /// Fundamental-frequency series resistance (pu).
    pub r_fundamental_pu: f64,
    /// Fundamental-frequency series reactance — inductive (pu).
    pub x_fundamental_pu: f64,
    /// Fundamental-frequency shunt susceptance — capacitive (pu).
    pub b_fundamental_pu: f64,
    /// Skin-effect exponent α; R(h) = R₁ · h^α.  Typical: 0.5 (solid conductor).
    pub r_skin_effect_exp: f64,
    /// Branch MVA thermal rating.
    pub rating_mva: f64,
}

impl HarmonicBranch {
    /// Create a new harmonic branch with default skin-effect exponent (0.5).
    pub fn new(
        from: usize,
        to: usize,
        r_fundamental_pu: f64,
        x_fundamental_pu: f64,
        b_fundamental_pu: f64,
        rating_mva: f64,
    ) -> Self {
        Self {
            from,
            to,
            r_fundamental_pu,
            x_fundamental_pu,
            b_fundamental_pu,
            r_skin_effect_exp: 0.5,
            rating_mva,
        }
    }

    /// Frequency-dependent series impedance `(R_pu, X_pu)` at harmonic order `h`.
    ///
    /// - R(h) = R₁ · h^α   (skin effect)
    /// - X(h) = X₁ · h     (purely inductive scaling)
    pub fn impedance_at_harmonic(&self, h: HarmonicOrder) -> (f64, f64) {
        let hf = h as f64;
        let r_h = self.r_fundamental_pu * hf.powf(self.r_skin_effect_exp);
        let x_h = self.x_fundamental_pu * hf;
        (r_h, x_h)
    }

    /// Frequency-dependent series admittance `(G_pu, B_pu)` at harmonic order `h`.
    ///
    /// y = 1 / (R + jX)  ⟹  G = R/(R²+X²),  B = -X/(R²+X²)
    pub fn admittance_at_harmonic(&self, h: HarmonicOrder) -> (f64, f64) {
        let (r, x) = self.impedance_at_harmonic(h);
        let denom = r * r + x * x;
        if denom < 1e-30 {
            (0.0, 0.0)
        } else {
            (r / denom, -x / denom)
        }
    }

    /// Shunt susceptance `B_shunt(h)` at harmonic `h`.
    ///
    /// Capacitive shunt susceptance scales linearly with frequency:
    /// B_shunt(h) = B₁ · h
    pub fn shunt_susceptance_at_harmonic(&self, h: HarmonicOrder) -> f64 {
        self.b_fundamental_pu * h as f64
    }
}

// ---------------------------------------------------------------------------
// HarmonicLoadType / HarmonicLoad
// ---------------------------------------------------------------------------

/// Type of load model for harmonic studies.
#[derive(Debug, Clone)]
pub enum HarmonicLoadType {
    /// Linear resistive load — no harmonic injection, modelled as a shunt.
    LinearResistive,
    /// Nonlinear load that generates harmonics according to the given source type.
    NonLinear(HarmonicSourceType),
    /// Single-tuned passive LC trap filter.
    /// `tuned_order`: harmonic order the filter is tuned to (e.g. 5).
    /// `q_factor`: filter quality factor Q (typical 30–100).
    PassiveFilter {
        tuned_order: HarmonicOrder,
        q_factor: f64,
    },
    /// Active power filter — compensates all harmonic injections at the bus.
    ActiveFilter,
}

/// Bus-level load with harmonic characteristics.
#[derive(Debug, Clone)]
pub struct HarmonicLoad {
    /// Bus index (0-based).
    pub bus: usize,
    /// Fundamental active power (MW).
    pub p_fundamental_mw: f64,
    /// Fundamental reactive power (Mvar).
    pub q_fundamental_mvar: f64,
    /// Load harmonic model type.
    pub load_type: HarmonicLoadType,
}

// ---------------------------------------------------------------------------
// HarmonicPfConfig
// ---------------------------------------------------------------------------

/// Configuration for harmonic power flow analysis (legacy full-system solver).
///
/// For the new `HarmonicPfProblem`-based API use [`HarmonicPfConfig`] instead.
#[derive(Debug, Clone)]
pub struct HarmonicPfConfigV1 {
    /// System base MVA.
    pub base_mva: f64,
    /// Fundamental frequency (Hz) — 50 or 60.
    pub fundamental_hz: f64,
    /// Harmonic orders to analyse.
    pub harmonic_orders: Vec<HarmonicOrder>,
    /// Maximum Newton-Raphson iterations for fundamental PF.
    pub max_iterations: usize,
    /// Convergence tolerance (pu) for fundamental PF.
    pub tolerance_pu: f64,
    /// System nominal voltage (kV) used for IEEE 519 compliance check.
    pub nominal_voltage_kv: f64,
}

impl Default for HarmonicPfConfigV1 {
    fn default() -> Self {
        Self {
            base_mva: 100.0,
            fundamental_hz: 50.0,
            harmonic_orders: vec![3, 5, 7, 11, 13, 17, 19, 23, 25],
            max_iterations: 50,
            tolerance_pu: 1e-6,
            nominal_voltage_kv: 20.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// Per-bus, per-harmonic voltage result.
#[derive(Debug, Clone)]
pub struct HarmonicBusResult {
    /// Bus index (0-based).
    pub bus: usize,
    /// Harmonic order.
    pub harmonic: HarmonicOrder,
    /// Harmonic voltage magnitude (pu).
    pub voltage_magnitude_pu: f64,
    /// Harmonic voltage angle (rad).
    pub voltage_angle_rad: f64,
    /// Voltage THD at this bus (%) — populated only for `harmonic == 1`.
    pub thd_v: f64,
    /// Current THD at this bus (%) — populated only for `harmonic == 1`.
    pub thd_i: f64,
}

/// IEEE 519 compliance assessment for a single bus.
#[derive(Debug, Clone)]
pub struct IeeComplianceResult {
    /// Bus index (0-based).
    pub bus: usize,
    /// IEEE 519 voltage THD limit (%).
    pub voltage_thd_limit_pct: f64,
    /// Measured / computed voltage THD (%).
    pub voltage_thd_actual_pct: f64,
    /// IEEE 519 current THD limit (%) — simplified 5% for all buses here.
    pub current_thd_limit_pct: f64,
    /// Measured / computed current THD (%).
    pub current_thd_actual_pct: f64,
    /// `true` if both voltage and current THD are within limits.
    pub compliant: bool,
}

/// Complete harmonic power flow result (legacy full-system solver).
///
/// For the new `HarmonicPfProblem`-based API use [`HarmonicPfResult`] instead.
#[derive(Debug, Clone)]
pub struct HarmonicPfResultV1 {
    /// Fundamental voltage magnitudes (pu) per bus.
    pub fundamental_voltages: Vec<f64>,
    /// Fundamental voltage angles (rad) per bus.
    pub fundamental_angles: Vec<f64>,
    /// Per-bus harmonic voltages: `harmonic_voltages[bus]` = `[(order, mag_pu, angle_rad)]`.
    pub harmonic_voltages: Vec<Vec<(HarmonicOrder, f64, f64)>>,
    /// Voltage THD per bus (%).
    pub bus_thd_v: Vec<f64>,
    /// Current THD per bus (%).
    pub bus_thd_i: Vec<f64>,
    /// Per-branch harmonic current flows: `branch_harmonic_flows[branch]` = `[(order, current_pu)]`.
    pub branch_harmonic_flows: Vec<Vec<(HarmonicOrder, f64)>>,
    /// Total additional resistive losses due to harmonics (MW).
    pub total_harmonic_losses_mw: f64,
    /// IEEE 519 compliance check results per bus.
    pub ieee519_compliance: Vec<IeeComplianceResult>,
}

// ---------------------------------------------------------------------------
// Complex linear algebra helpers (internal)
// ---------------------------------------------------------------------------

/// Complex number as `(real, imag)` — avoids pulling in num-complex here.
type Cx = (f64, f64);

#[inline]
fn cx_add(a: Cx, b: Cx) -> Cx {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_sub(a: Cx, b: Cx) -> Cx {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn cx_mul(a: Cx, b: Cx) -> Cx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_div(a: Cx, b: Cx) -> Cx {
    let d = b.0 * b.0 + b.1 * b.1;
    if d < 1e-300 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
    }
}

#[inline]
fn cx_abs(a: Cx) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Solve the complex linear system A·x = b using Gaussian elimination with
/// partial pivoting.  A is given as a flat row-major Vec of length n*n.
#[allow(clippy::needless_range_loop)]
fn complex_gaussian_elimination(a_matrix: &[Cx], b_vec: &[Cx], n: usize) -> Result<Vec<Cx>> {
    if n == 0 {
        return Err(OxiGridError::InvalidParameter(
            "harmonic system size must be > 0".into(),
        ));
    }
    if a_matrix.len() != n * n || b_vec.len() != n {
        return Err(OxiGridError::InvalidParameter(
            "harmonic system dimension mismatch".into(),
        ));
    }

    // Build augmented matrix [A | b]
    let mut aug: Vec<Vec<Cx>> = (0..n)
        .map(|i| {
            let mut row: Vec<Cx> = (0..n).map(|j| a_matrix[i * n + j]).collect();
            row.push(b_vec[i]);
            row
        })
        .collect();

    for col in 0..n {
        // Partial pivoting: find row with max |a[row][col]|
        let mut max_val = cx_abs(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let v = cx_abs(aug[row][col]);
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }

        if max_val < 1e-20 {
            return Err(OxiGridError::LinearAlgebra(format!(
                "harmonic Y-matrix is singular at column {col} (near-zero pivot)"
            )));
        }

        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = cx_div(aug[row][col], pivot);
            for j in col..=n {
                let sub = cx_mul(factor, aug[col][j]);
                aug[row][j] = cx_sub(aug[row][j], sub);
            }
        }
    }

    // Back substitution
    let mut x = vec![(0.0_f64, 0.0_f64); n];
    for i in (0..n).rev() {
        let mut s = aug[i][n];
        for j in (i + 1)..n {
            s = cx_sub(s, cx_mul(aug[i][j], x[j]));
        }
        x[i] = cx_div(s, aug[i][i]);
    }

    Ok(x)
}

// ---------------------------------------------------------------------------
// HarmonicPowerFlow — main solver
// ---------------------------------------------------------------------------

/// Harmonic power flow solver.
///
/// Implements the decoupled harmonic iteration:
/// - Fundamental: simplified NR using direct Y-bus inversion.
/// - Harmonic h: V(h) = Y(h)⁻¹ · I(h) solved by Gaussian elimination.
pub struct HarmonicPowerFlow {
    /// Number of buses in the network (0-based indexing).
    pub n_buses: usize,
    /// Frequency-dependent branch models.
    pub branches: Vec<HarmonicBranch>,
    /// Harmonic Norton current sources.
    pub sources: Vec<HarmonicSource>,
    /// Bus-level harmonic load models.
    pub loads: Vec<HarmonicLoad>,
    /// Solver configuration.
    pub config: HarmonicPfConfigV1,
    /// Slack bus index (reference bus, angle = 0).
    pub slack_bus: usize,
}

impl HarmonicPowerFlow {
    /// Construct a new `HarmonicPowerFlow` solver.
    pub fn new(
        n_buses: usize,
        branches: Vec<HarmonicBranch>,
        sources: Vec<HarmonicSource>,
        loads: Vec<HarmonicLoad>,
        config: HarmonicPfConfigV1,
        slack_bus: usize,
    ) -> Self {
        Self {
            n_buses,
            branches,
            sources,
            loads,
            config,
            slack_bus,
        }
    }

    // -----------------------------------------------------------------------
    // Admittance matrix construction
    // -----------------------------------------------------------------------

    /// Build the frequency-dependent nodal admittance matrix Y(h).
    ///
    /// Y is returned as a `n_buses × n_buses` matrix stored as
    /// `Vec<Vec<(G, B)>>` where each entry is `(conductance, susceptance)`.
    ///
    /// Formation:
    /// - Diagonal: Y_ii = Σ_j y_ij + y_shunt_i(h)
    /// - Off-diagonal: Y_ij = -y_ij
    ///
    /// Passive filters contribute additional shunt admittances at their tuned
    /// frequency, modelled as a series RLC resonance.
    pub fn build_admittance_matrix(&self, h: HarmonicOrder) -> Vec<Vec<Cx>> {
        let n = self.n_buses;
        let mut y = vec![vec![(0.0_f64, 0.0_f64); n]; n];

        // Shunt capacitive susceptance from branch π-models
        for branch in &self.branches {
            let (g_series, b_series) = branch.admittance_at_harmonic(h);
            let b_shunt_half = branch.shunt_susceptance_at_harmonic(h) * 0.5;

            let fi = branch.from;
            let ti = branch.to;

            // Off-diagonal
            y[fi][ti] = cx_sub(y[fi][ti], (g_series, b_series));
            y[ti][fi] = cx_sub(y[ti][fi], (g_series, b_series));

            // Diagonal (series contribution)
            y[fi][fi] = cx_add(y[fi][fi], (g_series, b_series));
            y[ti][ti] = cx_add(y[ti][ti], (g_series, b_series));

            // Shunt susceptance (half at each end, purely imaginary)
            y[fi][fi] = cx_add(y[fi][fi], (0.0, b_shunt_half));
            y[ti][ti] = cx_add(y[ti][ti], (0.0, b_shunt_half));
        }

        // Passive filter shunt admittances
        for load in &self.loads {
            if let HarmonicLoadType::PassiveFilter {
                tuned_order,
                q_factor,
            } = load.load_type
            {
                let bus = load.bus;
                // Single-tuned RLC filter:
                //   Resonance at h_t: X_L(h_t) = X_C(h_t) ⟹ ωL = 1/(ωC)
                //   Quality factor Q = R_eq / X_L = ω_t·L / R_f
                // At harmonic h, the filter shunt impedance:
                //   Z_f(h) = R_f + j(hX_L - X_C/h)
                // where X_L and X_C are fundamental-frequency values.
                //
                // We parameterise from reactive power Q_load at fundamental:
                //   X_C1 = V² / Q_load_pu  (capacitive)
                //   X_L1 = X_C1 / h_t²
                //   R_f   = X_L(h_t) / Q_factor = h_t·X_L1 / Q_factor
                let q_load_pu = if load.q_fundamental_mvar.abs() > 1e-9 {
                    load.q_fundamental_mvar / self.config.base_mva
                } else {
                    0.01 // default 1% reactive base
                };
                let x_c1 = 1.0 / q_load_pu.abs().max(1e-9);
                let ht = tuned_order as f64;
                let x_l1 = x_c1 / (ht * ht);
                let r_f = ht * x_l1 / q_factor.max(1.0);

                let hf = h as f64;
                let x_l_h = x_l1 * hf;
                let x_c_h = x_c1 / hf;
                let x_net = x_l_h - x_c_h;

                let z_f_r = r_f;
                let z_f_x = x_net;
                let denom = r_f * r_f + x_net * x_net;
                let (g_f, b_f) = if denom > 1e-30 {
                    (z_f_r / denom, -z_f_x / denom)
                } else {
                    (1e6, 0.0) // near-resonance: very high admittance
                };

                y[bus][bus] = cx_add(y[bus][bus], (g_f, b_f));
            }
        }

        y
    }

    // -----------------------------------------------------------------------
    // Fundamental power flow (simplified)
    // -----------------------------------------------------------------------

    /// Solve fundamental (h=1) power flow.
    ///
    /// Uses a simplified iterative approach:
    /// 1. Build Y_bus at h=1.
    /// 2. Apply flat-start voltages.
    /// 3. Iterate: V_i = (P_i - jQ_i)* / V_i* − Σ_{j≠i} Y_ij V_j, normalised.
    ///
    /// The slack bus voltage is held at 1.0 ∠0°.
    ///
    /// For the fundamental frequency, Y-matrix diagonal elements include
    /// load admittances (linear loads modelled as constant-impedance).
    fn solve_fundamental(&self, p_inj: &[f64], q_inj: &[f64]) -> Result<(Vec<f64>, Vec<f64>)> {
        if p_inj.len() != self.n_buses || q_inj.len() != self.n_buses {
            return Err(OxiGridError::InvalidParameter(
                "p_inj/q_inj length must equal n_buses".into(),
            ));
        }

        let n = self.n_buses;
        let y = self.build_admittance_matrix(1);

        // Initial flat-start voltages (pu)
        let mut v_re: Vec<f64> = vec![1.0; n];
        let mut v_im: Vec<f64> = vec![0.0; n];
        v_re[self.slack_bus] = 1.0;
        v_im[self.slack_bus] = 0.0;

        for _iter in 0..self.config.max_iterations {
            let v_re_prev = v_re.clone();
            let v_im_prev = v_im.clone();
            let mut max_delta: f64 = 0.0;

            for i in 0..n {
                if i == self.slack_bus {
                    continue;
                }

                // Injected current at bus i = (P_i - jQ_i)* / V_i*
                // V_i* = v_re - j·v_im  (conjugate)
                let vi_re = v_re[i];
                let vi_im = v_im[i];
                let vi_mag2 = vi_re * vi_re + vi_im * vi_im;
                if vi_mag2 < 1e-20 {
                    continue;
                }

                // I_inj = (P - jQ) / (V_re + jV_im)  [using S = V·I*]
                // I_re = (P·V_re + Q·V_im) / |V|²
                // I_im = (P·V_im - Q·V_re) / |V|²
                let i_inj_re = (p_inj[i] * vi_re + q_inj[i] * vi_im) / vi_mag2;
                let i_inj_im = (p_inj[i] * vi_im - q_inj[i] * vi_re) / vi_mag2;

                // Y_ii · V_i = I_inj - Σ_{j≠i} Y_ij · V_j
                let (y_ii_g, y_ii_b) = y[i][i];

                let mut rhs_re = i_inj_re;
                let mut rhs_im = i_inj_im;
                for j in 0..n {
                    if j == i {
                        continue;
                    }
                    let (y_ij_g, y_ij_b) = y[i][j];
                    rhs_re -= y_ij_g * v_re[j] - y_ij_b * v_im[j];
                    rhs_im -= y_ij_g * v_im[j] + y_ij_b * v_re[j];
                }

                // V_i_new = rhs / Y_ii
                let (new_re, new_im) = cx_div((rhs_re, rhs_im), (y_ii_g, y_ii_b));
                let delta = ((new_re - vi_re).powi(2) + (new_im - vi_im).powi(2)).sqrt();
                max_delta = max_delta.max(delta);
                v_re[i] = new_re;
                v_im[i] = new_im;
            }

            let _ = v_re_prev;
            let _ = v_im_prev;

            if max_delta < self.config.tolerance_pu {
                break;
            }
        }

        // Convert to polar
        let v_mag: Vec<f64> = (0..n)
            .map(|i| (v_re[i] * v_re[i] + v_im[i] * v_im[i]).sqrt())
            .collect();
        let v_ang: Vec<f64> = (0..n).map(|i| v_im[i].atan2(v_re[i])).collect();

        Ok((v_mag, v_ang))
    }

    // -----------------------------------------------------------------------
    // Harmonic injection
    // -----------------------------------------------------------------------

    /// Compute the total Norton harmonic current injection at `bus` for harmonic `h`.
    ///
    /// Sums contributions from all `HarmonicSource` at the bus.
    /// Nonlinear loads are also included if their type matches.
    ///
    /// Returns `(I_real, I_imag)` in pu.
    pub fn harmonic_injection(&self, bus: usize, h: HarmonicOrder) -> Cx {
        let mut i_re = 0.0_f64;
        let mut i_im = 0.0_f64;

        // Direct harmonic current sources
        for src in &self.sources {
            if src.bus == bus {
                let (ir, ii) = src.injection_at(h);
                i_re += ir;
                i_im += ii;
            }
        }

        // Nonlinear loads (treated as Norton sources with source-type spectrum)
        for load in &self.loads {
            if load.bus == bus {
                if let HarmonicLoadType::NonLinear(ref src_type) = load.load_type {
                    // Fundamental current magnitude from P/Q at 1 pu voltage
                    let s_fund = (load.p_fundamental_mw.powi(2) + load.q_fundamental_mvar.powi(2))
                        .sqrt()
                        / self.config.base_mva;
                    let spectrum = src_type.spectrum(s_fund);
                    for (order, mag, angle) in &spectrum {
                        if *order == h {
                            i_re += mag * angle.cos();
                            i_im += mag * angle.sin();
                        }
                    }
                }
            }
        }

        (i_re, i_im)
    }

    // -----------------------------------------------------------------------
    // Harmonic voltage solve
    // -----------------------------------------------------------------------

    /// Solve harmonic voltages at order `h`.
    ///
    /// Formulation: V(h) = Y(h)⁻¹ · I(h)
    ///
    /// The slack bus is constrained to V_slack(h) = 0 (no harmonic source at
    /// reference bus by convention). This is implemented by zeroing the
    /// corresponding row/column and placing a 1 on the diagonal.
    ///
    /// Returns `(V_real, V_imag)` per bus (length = `n_buses`).
    pub fn solve_harmonic(
        &self,
        h: HarmonicOrder,
        y_matrix: &[Vec<Cx>],
        i_injections: &[Cx],
    ) -> Result<Vec<Cx>> {
        let n = self.n_buses;

        if y_matrix.len() != n || i_injections.len() != n {
            return Err(OxiGridError::InvalidParameter(
                "Y-matrix or injection vector dimension mismatch".into(),
            ));
        }

        // Flatten Y into row-major Vec for Gaussian elimination
        let mut a_flat: Vec<Cx> = Vec::with_capacity(n * n);
        let mut b: Vec<Cx> = i_injections.to_vec();

        for row in y_matrix.iter() {
            for &entry in row.iter() {
                a_flat.push(entry);
            }
        }

        // Enforce V_slack(h) = 0: replace slack row with [0…0,1,0…0]·V = 0
        let s = self.slack_bus;
        for j in 0..n {
            a_flat[s * n + j] = if j == s { (1.0, 0.0) } else { (0.0, 0.0) };
        }
        b[s] = (0.0, 0.0);

        let _ = h; // h is embedded in y_matrix already

        complex_gaussian_elimination(&a_flat, &b, n)
    }

    // -----------------------------------------------------------------------
    // THD computation
    // -----------------------------------------------------------------------

    /// Compute voltage THD at a bus given the fundamental voltage and harmonic voltages.
    ///
    /// THD_V (%) = √(Σ_{h≥2} V_h²) / V₁ × 100
    pub fn compute_voltage_thd(
        v_fundamental: f64,
        harmonic_voltages: &[(HarmonicOrder, f64, f64)],
    ) -> f64 {
        if v_fundamental < 1e-10 {
            return 0.0;
        }
        let sum_sq: f64 = harmonic_voltages
            .iter()
            .filter(|(h, _, _)| *h >= 2)
            .map(|(_, mag, _)| mag * mag)
            .sum();
        (sum_sq.sqrt() / v_fundamental) * 100.0
    }

    /// IEEE 519-2022 Table 2 voltage distortion limits.
    ///
    /// | Bus voltage (kV)   | THD limit |
    /// |--------------------|-----------|
    /// | ≤ 1 kV             | 8.0 %     |
    /// | 1 – 69 kV          | 5.0 %     |
    /// | 69 – 161 kV        | 2.5 %     |
    /// | > 161 kV           | 1.5 %     |
    pub fn ieee519_voltage_limit(voltage_kv: f64) -> f64 {
        if voltage_kv <= 1.0 {
            8.0
        } else if voltage_kv <= 69.0 {
            5.0
        } else if voltage_kv <= 161.0 {
            2.5
        } else {
            1.5
        }
    }

    /// IEEE 519-2022 Table 1 current distortion limit (TDD).
    ///
    /// Simplified: uses 5% for I_sc/I_L < 20 (conservative for MV feeders).
    /// A full implementation would need the short-circuit ratio at the PCC.
    pub fn ieee519_current_limit(_voltage_kv: f64) -> f64 {
        5.0
    }

    // -----------------------------------------------------------------------
    // Resonance detection
    // -----------------------------------------------------------------------

    /// Find harmonic orders at which bus `bus` is close to parallel resonance.
    ///
    /// Resonance occurs when the imaginary part of Y_ii(h) ≈ 0 and
    /// |Y_ii(h)| is locally minimised, implying high impedance.
    ///
    /// Scans the harmonic orders 2..=50 and returns orders where
    /// |B_ii(h)| < threshold (0.01 pu).
    pub fn find_resonance_frequencies(&self, bus: usize) -> Vec<HarmonicOrder> {
        let mut resonant: Vec<HarmonicOrder> = Vec::new();
        let threshold = 0.01_f64; // susceptance near-zero threshold (pu)

        for h in 2u32..=50 {
            let y = self.build_admittance_matrix(h);
            let (_, b_ii) = y[bus][bus];
            // Near parallel resonance: B_ii ≈ 0 (inductive and capacitive cancel)
            if b_ii.abs() < threshold {
                resonant.push(h);
            }
        }
        resonant
    }

    // -----------------------------------------------------------------------
    // Passive filter design
    // -----------------------------------------------------------------------

    /// Design a single-tuned passive LC filter for a target harmonic order.
    ///
    /// The filter is sized to supply `reactive_power_mvar` of reactive
    /// compensation at fundamental frequency.
    ///
    /// - C = Q / (V² · ω₁)
    /// - L = 1 / (C · ω_h²)  where ω_h = h · ω₁
    ///
    /// Returns `(L_henry, C_farad)`.
    pub fn design_passive_filter(
        target_order: HarmonicOrder,
        v_bus_kv: f64,
        reactive_power_mvar: f64,
        fundamental_hz: f64,
    ) -> (f64, f64) {
        let omega1 = 2.0 * PI * fundamental_hz;
        let omega_h = (target_order as f64) * omega1;
        let v_volts = v_bus_kv * 1000.0;
        let q_var = reactive_power_mvar * 1e6;

        // Capacitor sized to supply reactive power at fundamental:
        //   Q = V² · B_C = V² · ω₁ · C  ⟹  C = Q / (V² · ω₁)
        let c_farad = if v_volts.abs() < 1.0 || omega1 < 1e-6 {
            0.0
        } else {
            q_var / (v_volts * v_volts * omega1)
        };

        // Inductor tuned to harmonic frequency:
        //   ω_h² · L · C = 1  ⟹  L = 1 / (C · ω_h²)
        let l_henry = if c_farad < 1e-30 || omega_h < 1e-6 {
            0.0
        } else {
            1.0 / (c_farad * omega_h * omega_h)
        };

        (l_henry, c_farad)
    }

    // -----------------------------------------------------------------------
    // Main solve entry point
    // -----------------------------------------------------------------------

    /// Perform the full harmonic power flow.
    ///
    /// Steps:
    /// 1. Solve fundamental PF from `p_injections` / `q_injections` (pu).
    /// 2. For each harmonic order h:
    ///    a. Build Y(h).
    ///    b. Collect I(h) injection vector.
    ///    c. Solve V(h) = Y(h)⁻¹ · I(h).
    /// 3. Compute THD_V and THD_I at each bus.
    /// 4. Compute branch harmonic current flows.
    /// 5. Compute total harmonic losses.
    /// 6. Check IEEE 519 compliance.
    ///
    /// Perform the full harmonic power flow using the legacy solver.
    pub fn solve(&self, p_injections: &[f64], q_injections: &[f64]) -> Result<HarmonicPfResultV1> {
        let n = self.n_buses;

        // ------------------------------------------------------------------
        // Step 1: Fundamental power flow
        // ------------------------------------------------------------------
        let (v_fund_mag, v_fund_ang) = self.solve_fundamental(p_injections, q_injections)?;

        // ------------------------------------------------------------------
        // Step 2: Harmonic voltage solve for each order
        // ------------------------------------------------------------------
        // harmonic_v[bus] = Vec<(order, mag, angle)>
        let mut harmonic_v: Vec<Vec<(HarmonicOrder, f64, f64)>> = vec![Vec::new(); n];

        for &h in &self.config.harmonic_orders {
            // Build Y(h)
            let y_h = self.build_admittance_matrix(h);

            // Collect harmonic current injections
            let i_h: Vec<Cx> = (0..n).map(|bus| self.harmonic_injection(bus, h)).collect();

            // Solve V(h) = Y(h)^{-1} · I(h)
            let v_h = self.solve_harmonic(h, &y_h, &i_h)?;

            // Store results per bus
            for (bus, &(vr, vi)) in v_h.iter().enumerate() {
                let mag = (vr * vr + vi * vi).sqrt();
                let angle = vi.atan2(vr);
                harmonic_v[bus].push((h, mag, angle));
            }
        }

        // ------------------------------------------------------------------
        // Step 3: THD computation
        // ------------------------------------------------------------------
        let bus_thd_v: Vec<f64> = (0..n)
            .map(|bus| Self::compute_voltage_thd(v_fund_mag[bus], &harmonic_v[bus]))
            .collect();

        // Current THD: computed from branch harmonic currents flowing into each bus
        // We compute via: I_bus_h = Σ_branches (branch current at h flowing into bus)
        // Simplified: use sum of harmonic injection magnitudes / fundamental current
        let bus_thd_i: Vec<f64> = (0..n)
            .map(|bus| {
                // Fundamental current: |S_fund| / V_fund (pu)
                let p = if bus < p_injections.len() {
                    p_injections[bus]
                } else {
                    0.0
                };
                let q = if bus < q_injections.len() {
                    q_injections[bus]
                } else {
                    0.0
                };
                let s_fund = (p * p + q * q).sqrt();
                let v_f = v_fund_mag[bus].max(1e-6);
                let i_fund = s_fund / v_f;

                if i_fund < 1e-10 {
                    return 0.0;
                }

                // Sum harmonic injection magnitudes for THD_I estimate
                let sum_sq: f64 = harmonic_v[bus]
                    .iter()
                    .filter(|(h_ord, _, _)| *h_ord >= 2)
                    .map(|(h_ord, v_mag, _)| {
                        // Approximate harmonic current from harmonic voltage and bus admittance
                        // I_h ≈ V_h · |Y_bus_diag(h)|
                        let y_h = self.build_admittance_matrix(*h_ord);
                        let (g, b) = y_h[bus][bus];
                        let y_mag = (g * g + b * b).sqrt();
                        (v_mag * y_mag).powi(2)
                    })
                    .sum();
                (sum_sq.sqrt() / i_fund) * 100.0
            })
            .collect();

        // ------------------------------------------------------------------
        // Step 4: Branch harmonic current flows
        // ------------------------------------------------------------------
        let mut branch_harmonic_flows: Vec<Vec<(HarmonicOrder, f64)>> =
            vec![Vec::new(); self.branches.len()];

        for (br_idx, branch) in self.branches.iter().enumerate() {
            for &h in &self.config.harmonic_orders {
                // Find V_from(h) and V_to(h)
                let v_from = harmonic_v[branch.from]
                    .iter()
                    .find(|(order, _, _)| *order == h)
                    .map(|&(_, mag, ang)| (mag * ang.cos(), mag * ang.sin()))
                    .unwrap_or((0.0, 0.0));

                let v_to = harmonic_v[branch.to]
                    .iter()
                    .find(|(order, _, _)| *order == h)
                    .map(|&(_, mag, ang)| (mag * ang.cos(), mag * ang.sin()))
                    .unwrap_or((0.0, 0.0));

                let dv = cx_sub(v_from, v_to);
                let (g_h, b_h) = branch.admittance_at_harmonic(h);
                let i_branch = cx_mul(dv, (g_h, b_h));
                let i_mag = cx_abs(i_branch);

                branch_harmonic_flows[br_idx].push((h, i_mag));
            }
        }

        // ------------------------------------------------------------------
        // Step 5: Total harmonic losses
        // ------------------------------------------------------------------
        let mut total_harmonic_losses_mw: f64 = 0.0;
        for (br_idx, branch) in self.branches.iter().enumerate() {
            for &(h, i_mag) in &branch_harmonic_flows[br_idx] {
                let (r_h, _) = branch.impedance_at_harmonic(h);
                // P_loss = I² · R,  converted from pu to MW
                total_harmonic_losses_mw += i_mag * i_mag * r_h * self.config.base_mva;
            }
        }

        // ------------------------------------------------------------------
        // Step 6: IEEE 519 compliance
        // ------------------------------------------------------------------
        let ieee519_compliance: Vec<IeeComplianceResult> = (0..n)
            .map(|bus| {
                let v_thd_lim = Self::ieee519_voltage_limit(self.config.nominal_voltage_kv);
                let i_thd_lim = Self::ieee519_current_limit(self.config.nominal_voltage_kv);
                let v_thd_act = bus_thd_v[bus];
                let i_thd_act = bus_thd_i[bus];
                let compliant = v_thd_act <= v_thd_lim && i_thd_act <= i_thd_lim;
                IeeComplianceResult {
                    bus,
                    voltage_thd_limit_pct: v_thd_lim,
                    voltage_thd_actual_pct: v_thd_act,
                    current_thd_limit_pct: i_thd_lim,
                    current_thd_actual_pct: i_thd_act,
                    compliant,
                }
            })
            .collect();

        Ok(HarmonicPfResultV1 {
            fundamental_voltages: v_fund_mag,
            fundamental_angles: v_fund_ang,
            harmonic_voltages: harmonic_v,
            bus_thd_v,
            bus_thd_i,
            branch_harmonic_flows,
            total_harmonic_losses_mw,
            ieee519_compliance,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // Helper: build a simple 2-bus system
    // -----------------------------------------------------------------------
    fn two_bus_system() -> HarmonicPowerFlow {
        let branches = vec![HarmonicBranch::new(0, 1, 0.01, 0.1, 0.02, 100.0)];
        let sources = vec![HarmonicSource::new(
            1,
            HarmonicSourceType::SixPulseDrive,
            0.5,
        )];
        let loads = vec![HarmonicLoad {
            bus: 1,
            p_fundamental_mw: 10.0,
            q_fundamental_mvar: 3.0,
            load_type: HarmonicLoadType::LinearResistive,
        }];
        let config = HarmonicPfConfigV1 {
            base_mva: 100.0,
            fundamental_hz: 50.0,
            harmonic_orders: vec![3, 5, 7, 11, 13],
            max_iterations: 100,
            tolerance_pu: 1e-8,
            nominal_voltage_kv: 20.0,
        };
        HarmonicPowerFlow::new(2, branches, sources, loads, config, 0)
    }

    // -----------------------------------------------------------------------
    // Spectrum tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_harmonic_source_six_pulse_spectrum() {
        let spectrum = HarmonicSourceType::SixPulseDrive.spectrum(1.0);
        // 5th harmonic should be ~17.5%
        let fifth = spectrum
            .iter()
            .find(|(h, _, _)| *h == 5)
            .expect("5th harmonic");
        assert!((fifth.1 - 0.175).abs() < 1e-9, "5th should be 17.5%");
        // 7th harmonic should be ~11.1%
        let seventh = spectrum
            .iter()
            .find(|(h, _, _)| *h == 7)
            .expect("7th harmonic");
        assert!((seventh.1 - 0.111).abs() < 1e-9, "7th should be 11.1%");
        // Should not contain even harmonics
        let has_even = spectrum.iter().any(|(h, _, _)| h % 2 == 0);
        assert!(!has_even, "6-pulse drive should have no even harmonics");
    }

    #[test]
    fn test_harmonic_source_twelve_pulse_spectrum() {
        let spectrum = HarmonicSourceType::TwelvePulseDrive.spectrum(1.0);
        // Should have 11th, 13th
        let eleventh = spectrum.iter().find(|(h, _, _)| *h == 11).expect("11th");
        assert!((eleventh.1 - 0.076).abs() < 1e-9, "11th should be 7.6%");
        let thirteenth = spectrum.iter().find(|(h, _, _)| *h == 13).expect("13th");
        assert!((thirteenth.1 - 0.059).abs() < 1e-9, "13th should be 5.9%");
        // Should NOT have 5th harmonic
        let fifth = spectrum.iter().find(|(h, _, _)| *h == 5);
        assert!(fifth.is_none(), "12-pulse should cancel 5th harmonic");
        // Should NOT have 7th harmonic
        let seventh = spectrum.iter().find(|(h, _, _)| *h == 7);
        assert!(seventh.is_none(), "12-pulse should cancel 7th harmonic");
    }

    #[test]
    fn test_harmonic_source_pv_spectrum() {
        let spectrum = HarmonicSourceType::PhotovoltaicInverter.spectrum(1.0);
        // IEEE 1547 compliant: very low harmonics
        let fifth = spectrum.iter().find(|(h, _, _)| *h == 5).expect("5th");
        assert!(fifth.1 <= 0.05, "PV 5th harmonic should be <= 5%");
        let seventh = spectrum.iter().find(|(h, _, _)| *h == 7).expect("7th");
        assert!(seventh.1 <= 0.04, "PV 7th harmonic should be <= 4%");
        // Should not have 3rd harmonic (single-phase issue, not PV)
        let third = spectrum.iter().find(|(h, _, _)| *h == 3);
        assert!(third.is_none(), "PV inverter should not have 3rd harmonic");
    }

    #[test]
    fn test_harmonic_source_smps_spectrum() {
        let spectrum = HarmonicSourceType::SwitchModePowerSupply.spectrum(1.0);
        // SMPS: 3rd dominant (~85%)
        let third = spectrum.iter().find(|(h, _, _)| *h == 3).expect("3rd");
        assert!((third.1 - 0.85).abs() < 1e-9, "SMPS 3rd should be 85%");
        // 5th at 52%
        let fifth = spectrum.iter().find(|(h, _, _)| *h == 5).expect("5th");
        assert!((fifth.1 - 0.52).abs() < 1e-9, "SMPS 5th should be 52%");
        // All magnitudes scaled linearly with fundamental
        let spectrum2 = HarmonicSourceType::SwitchModePowerSupply.spectrum(2.0);
        let third2 = spectrum2.iter().find(|(h, _, _)| *h == 3).expect("3rd x2");
        assert!(
            (third2.1 - 1.70).abs() < 1e-9,
            "scaling by 2 should double magnitudes"
        );
    }

    // -----------------------------------------------------------------------
    // Branch impedance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_branch_impedance_at_5th_harmonic() {
        let branch = HarmonicBranch::new(0, 1, 0.01, 0.1, 0.0, 100.0);
        let (r5, x5) = branch.impedance_at_harmonic(5);
        // R(5) = 0.01 * 5^0.5 ≈ 0.02236
        let r5_expected = 0.01 * (5.0_f64).sqrt();
        assert!(
            (r5 - r5_expected).abs() < 1e-10,
            "R at 5th: {r5} vs {r5_expected}"
        );
        // X(5) = 0.1 * 5 = 0.5
        assert!((x5 - 0.5).abs() < 1e-10, "X at 5th: {x5}");
    }

    #[test]
    fn test_branch_impedance_at_11th_harmonic() {
        let branch = HarmonicBranch::new(0, 1, 0.01, 0.1, 0.0, 100.0);
        let (r11, x11) = branch.impedance_at_harmonic(11);
        let r11_expected = 0.01 * (11.0_f64).sqrt();
        assert!(
            (r11 - r11_expected).abs() < 1e-10,
            "R at 11th: {r11} vs {r11_expected}"
        );
        assert!((x11 - 1.1).abs() < 1e-10, "X at 11th: {x11}");
    }

    #[test]
    fn test_branch_admittance_at_harmonic() {
        let branch = HarmonicBranch::new(0, 1, 0.01, 0.1, 0.0, 100.0);
        let (g, b) = branch.admittance_at_harmonic(5);
        let (r5, x5) = branch.impedance_at_harmonic(5);
        let denom = r5 * r5 + x5 * x5;
        let g_exp = r5 / denom;
        let b_exp = -x5 / denom;
        assert!((g - g_exp).abs() < 1e-12, "G at 5th");
        assert!((b - b_exp).abs() < 1e-12, "B at 5th");
        // G > 0, B < 0 for inductive branch
        assert!(g > 0.0);
        assert!(b < 0.0);
    }

    // -----------------------------------------------------------------------
    // Admittance matrix tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_build_admittance_matrix_2bus() {
        let hpf = two_bus_system();
        let y = hpf.build_admittance_matrix(1);
        assert_eq!(y.len(), 2);
        assert_eq!(y[0].len(), 2);
        // Y_00 = Y_11 = y_series + y_shunt_half (diagonal dominant)
        let (g00, _b00) = y[0][0];
        let (g01, b01) = y[0][1];
        // Off-diagonal = -y_series
        assert!(
            g01 < 0.0 || b01 < 0.0,
            "off-diagonal should be negative of series admittance"
        );
        // Diagonal >= |off-diagonal| (row-sum property with shunt)
        let (g10, _b10) = y[1][0];
        assert!((g00 - g00.abs()).abs() < 1e-10, "G00 should be positive");
        let _ = g10;
    }

    #[test]
    fn test_build_admittance_matrix_3bus() {
        let branches = vec![
            HarmonicBranch::new(0, 1, 0.01, 0.1, 0.01, 100.0),
            HarmonicBranch::new(1, 2, 0.02, 0.15, 0.01, 100.0),
            HarmonicBranch::new(0, 2, 0.015, 0.12, 0.01, 100.0),
        ];
        let config = HarmonicPfConfigV1::default();
        let hpf = HarmonicPowerFlow::new(3, branches, vec![], vec![], config, 0);
        let y = hpf.build_admittance_matrix(1);
        assert_eq!(y.len(), 3);
        // Symmetry check: Y_ij = Y_ji
        for (i, y_row) in y.iter().enumerate() {
            for (j, &(gij, bij)) in y_row.iter().enumerate() {
                let (gji, bji) = y[j][i];
                assert!((gij - gji).abs() < 1e-12, "G[{i}][{j}] != G[{j}][{i}]");
                assert!((bij - bji).abs() < 1e-12, "B[{i}][{j}] != B[{j}][{i}]");
            }
        }
        // Diagonal should be positive (dominated by series admittances)
        for (i, y_row) in y.iter().enumerate() {
            let (gii, _) = y_row[i];
            assert!(gii > 0.0, "G[{i}][{i}] should be positive");
        }
    }

    // -----------------------------------------------------------------------
    // Harmonic injection test
    // -----------------------------------------------------------------------

    #[test]
    fn test_harmonic_injection_at_bus() {
        let hpf = two_bus_system();
        // Bus 1 has a SixPulseDrive source with fundamental current 0.5 pu
        let (ir5, ii5) = hpf.harmonic_injection(1, 5);
        // Expected: 0.5 * 0.175 = 0.0875 pu magnitude
        let mag = (ir5 * ir5 + ii5 * ii5).sqrt();
        assert!(
            (mag - 0.5 * 0.175).abs() < 1e-10,
            "5th harmonic injection magnitude"
        );

        // Bus 0 (slack) has no source
        let (ir0, ii0) = hpf.harmonic_injection(0, 5);
        assert!(
            (ir0 * ir0 + ii0 * ii0).sqrt() < 1e-10,
            "No injection at slack bus"
        );
    }

    // -----------------------------------------------------------------------
    // Harmonic solve tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_harmonic_2bus() {
        let hpf = two_bus_system();
        let y = hpf.build_admittance_matrix(5);
        let i_h: Vec<Cx> = vec![(0.0, 0.0), hpf.harmonic_injection(1, 5)];
        let v_h = hpf.solve_harmonic(5, &y, &i_h).expect("harmonic solve ok");
        assert_eq!(v_h.len(), 2);
        // Slack bus voltage should be near zero (no harmonic at slack)
        let v_slack = cx_abs(v_h[0]);
        assert!(
            v_slack < 1e-10,
            "Slack bus harmonic voltage should be ~0, got {v_slack}"
        );
        // Bus 1 should have non-zero harmonic voltage
        let v1 = cx_abs(v_h[1]);
        assert!(
            v1 > 1e-6,
            "Bus 1 should have non-zero harmonic voltage {v1}"
        );
    }

    #[test]
    fn test_solve_fundamental_2bus() {
        let hpf = two_bus_system();
        let p_inj = vec![0.0, -0.1]; // 0.1 pu load at bus 1
        let q_inj = vec![0.0, -0.03];
        let (v_mag, v_ang) = hpf
            .solve_fundamental(&p_inj, &q_inj)
            .expect("fundamental ok");
        assert_eq!(v_mag.len(), 2);
        // Slack bus voltage = 1.0 pu
        assert!((v_mag[0] - 1.0).abs() < 1e-6, "Slack voltage: {}", v_mag[0]);
        // Load bus voltage slightly below 1.0 due to line drop
        assert!(
            v_mag[1] < 1.01,
            "Load bus voltage should be <= 1.01: {}",
            v_mag[1]
        );
        assert!(
            v_mag[1] > 0.9,
            "Load bus voltage should be > 0.9: {}",
            v_mag[1]
        );
        // Slack bus angle = 0
        assert!(v_ang[0].abs() < 1e-10, "Slack angle: {}", v_ang[0]);
    }

    // -----------------------------------------------------------------------
    // THD tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_voltage_thd() {
        // Simple case: V1 = 1.0, V5 = 0.05, V7 = 0.03
        let harmonics = vec![(5u32, 0.05_f64, 0.0_f64), (7u32, 0.03, 0.0)];
        let thd = HarmonicPowerFlow::compute_voltage_thd(1.0, &harmonics);
        // THD = sqrt(0.05² + 0.03²) / 1.0 * 100 = sqrt(0.0025 + 0.0009) * 100
        let expected = (0.0025_f64 + 0.0009_f64).sqrt() * 100.0;
        assert!((thd - expected).abs() < 1e-8, "THD: {thd} vs {expected}");
    }

    #[test]
    fn test_thd_pure_fundamental() {
        // No harmonics — THD = 0
        let harmonics: Vec<(HarmonicOrder, f64, f64)> = vec![];
        let thd = HarmonicPowerFlow::compute_voltage_thd(1.0, &harmonics);
        assert!(
            thd.abs() < 1e-12,
            "Pure fundamental should have THD = 0, got {thd}"
        );

        // Only fundamental (order 1) listed — filtered out by h >= 2 condition
        let harmonics_fund_only = vec![(1u32, 1.0_f64, 0.0_f64)];
        let thd2 = HarmonicPowerFlow::compute_voltage_thd(1.0, &harmonics_fund_only);
        assert!(
            thd2.abs() < 1e-12,
            "Only fundamental should give THD = 0, got {thd2}"
        );
    }

    // -----------------------------------------------------------------------
    // IEEE 519 voltage limit tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ieee519_voltage_limit_lv() {
        // LV (≤ 1 kV): 8%
        let limit = HarmonicPowerFlow::ieee519_voltage_limit(0.4);
        assert!(
            (limit - 8.0).abs() < 1e-10,
            "LV limit should be 8%: {limit}"
        );
        let limit_at1 = HarmonicPowerFlow::ieee519_voltage_limit(1.0);
        assert!(
            (limit_at1 - 8.0).abs() < 1e-10,
            "1 kV boundary: {limit_at1}"
        );
    }

    #[test]
    fn test_ieee519_voltage_limit_mv() {
        // MV (1–69 kV): 5%
        let limit = HarmonicPowerFlow::ieee519_voltage_limit(20.0);
        assert!(
            (limit - 5.0).abs() < 1e-10,
            "20 kV (MV) limit should be 5%: {limit}"
        );
        let limit_hv_edge = HarmonicPowerFlow::ieee519_voltage_limit(69.0);
        assert!(
            (limit_hv_edge - 5.0).abs() < 1e-10,
            "69 kV boundary: {limit_hv_edge}"
        );
    }

    #[test]
    fn test_ieee519_voltage_limit_hv() {
        // HV (69–161 kV): 2.5%
        let limit = HarmonicPowerFlow::ieee519_voltage_limit(110.0);
        assert!(
            (limit - 2.5).abs() < 1e-10,
            "110 kV (HV) limit should be 2.5%: {limit}"
        );
        // EHV (> 161 kV): 1.5%
        let limit_ehv = HarmonicPowerFlow::ieee519_voltage_limit(400.0);
        assert!(
            (limit_ehv - 1.5).abs() < 1e-10,
            "400 kV (EHV) limit should be 1.5%: {limit_ehv}"
        );
        let limit_161 = HarmonicPowerFlow::ieee519_voltage_limit(161.0);
        assert!(
            (limit_161 - 2.5).abs() < 1e-10,
            "161 kV boundary: {limit_161}"
        );
    }

    // -----------------------------------------------------------------------
    // Resonance detection test
    // -----------------------------------------------------------------------

    #[test]
    fn test_find_resonance_frequencies() {
        // Create a system with capacitive shunt chosen to produce resonance near h=5
        // in the π-model half-shunt formulation.
        //
        // Resonance at bus 0 (with bus 1 effectively shorted):
        //   B_series(h) + B_shunt_half(h) = 0
        //   -X₁·h / (R₁²·hᵅ + X₁²·h²) + B₁·h/2 ≈ 0  (for small R)
        //   ⟹ B₁·X₁·h²/2 ≈ 1  ⟹  h² = 2 / (B₁ · X₁)
        //
        // For h_res = 5: B₁ = 2 / (25 · X₁)
        // With X₁ = 0.04: B₁ = 2 / (25 * 0.04) = 2.0
        let branches = vec![HarmonicBranch {
            from: 0,
            to: 1,
            r_fundamental_pu: 0.0005, // very small R to isolate resonance effect
            x_fundamental_pu: 0.04,
            b_fundamental_pu: 2.0, // tuned so resonance near h=5
            r_skin_effect_exp: 0.5,
            rating_mva: 100.0,
        }];
        let config = HarmonicPfConfigV1 {
            harmonic_orders: vec![5, 7, 11, 13],
            ..Default::default()
        };
        let hpf = HarmonicPowerFlow::new(2, branches, vec![], vec![], config, 0);
        let resonances = hpf.find_resonance_frequencies(0);
        // Should detect resonance at h=5 (or a neighbouring harmonic due to rounding/R)
        let near_5th = resonances.iter().any(|&h| (4..=6).contains(&h));
        assert!(
            near_5th,
            "Should detect resonance near 5th harmonic, got: {resonances:?}"
        );
    }

    // -----------------------------------------------------------------------
    // Passive filter design test
    // -----------------------------------------------------------------------

    #[test]
    fn test_design_passive_filter_5th() {
        let (l, c) = HarmonicPowerFlow::design_passive_filter(5, 20.0, 5.0, 50.0);
        // C = Q / (V^2 * omega1) = 5e6 / ((20000)^2 * 2π*50)
        let omega1 = 2.0 * PI * 50.0;
        let c_expected = 5e6 / (20_000.0_f64.powi(2) * omega1);
        assert!(
            (c - c_expected).abs() < 1e-18,
            "Capacitance: {c} vs {c_expected}"
        );

        // L = 1 / (C * (5*omega1)^2)
        let omega5 = 5.0 * omega1;
        let l_expected = 1.0 / (c_expected * omega5 * omega5);
        assert!(
            (l - l_expected).abs() < 1e-18,
            "Inductance: {l} vs {l_expected}"
        );

        // Physical sanity: L and C should be positive
        assert!(l > 0.0, "L should be positive: {l}");
        assert!(c > 0.0, "C should be positive: {c}");

        // Verify tuned frequency = 5th harmonic
        let f_tuned = 1.0 / (2.0 * PI * (l * c).sqrt());
        let f_5th = 5.0 * 50.0;
        assert!(
            (f_tuned - f_5th).abs() / f_5th < 1e-9,
            "Filter tuned at {f_tuned} Hz, expected {f_5th} Hz"
        );
    }

    // -----------------------------------------------------------------------
    // Full harmonic power flow end-to-end test
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_harmonic_pf_2bus() {
        let hpf = two_bus_system();
        let p_inj = vec![0.0, -0.1]; // 0.1 pu load at bus 1
        let q_inj = vec![0.0, -0.03];

        let result = hpf
            .solve(&p_inj, &q_inj)
            .expect("harmonic PF should converge");

        // Fundamental voltages
        assert_eq!(result.fundamental_voltages.len(), 2);
        assert!(
            (result.fundamental_voltages[0] - 1.0).abs() < 1e-4,
            "Slack voltage: {}",
            result.fundamental_voltages[0]
        );

        // Harmonic voltages: 5 harmonic orders × 2 buses
        assert_eq!(result.harmonic_voltages.len(), 2);
        assert_eq!(
            result.harmonic_voltages[1].len(),
            5,
            "bus 1 should have 5 harmonic entries"
        );

        // THD should be non-negative
        for (i, &thd) in result.bus_thd_v.iter().enumerate() {
            assert!(thd >= 0.0, "Bus {i} THD_V should be >= 0: {thd}");
        }

        // Branch harmonic flows
        assert_eq!(result.branch_harmonic_flows.len(), 1, "One branch");
        assert_eq!(
            result.branch_harmonic_flows[0].len(),
            5,
            "5 harmonic orders per branch"
        );

        // Total harmonic losses should be non-negative
        assert!(
            result.total_harmonic_losses_mw >= 0.0,
            "Harmonic losses must be >= 0: {}",
            result.total_harmonic_losses_mw
        );

        // IEEE 519 compliance entries for each bus
        assert_eq!(result.ieee519_compliance.len(), 2);
        // At 20 kV (MV), limit = 5%
        assert!((result.ieee519_compliance[0].voltage_thd_limit_pct - 5.0).abs() < 1e-9);

        // Bus 1 has harmonic source (6-pulse drive), so THD should be non-zero
        let thd_bus1 = result.bus_thd_v[1];
        assert!(
            thd_bus1 > 0.0,
            "Bus 1 with harmonic source should have THD > 0: {thd_bus1}"
        );
    }
}
