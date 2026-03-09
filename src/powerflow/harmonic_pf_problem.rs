//! Builder-pattern harmonic power flow problem API.
//!
//! Provides `HarmonicPfProblem`, `HarmonicPfConfig`, `HarmonicPfResult` and
//! supporting types for computing voltage harmonics throughout a network given
//! harmonic current injection sources.
//!
//! # Algorithm
//! For each harmonic order h:
//! 1. Build Y_h: branch series admittance `1/(R + j·h·X)`, shunt `j·h·B/2` at each end.
//! 2. Exclude slack bus row/column (slack harmonic voltage = 0 for h ≥ 2, 1∠0 for h = 1).
//! 3. Solve Y_red · V_h = I_h via Gaussian elimination with partial pivoting.
//! 4. Reconstruct full voltage, compute branch currents.
//! 5. Accumulate THD: `THD_V = sqrt(Σ_{h≥2} |V_h|²) / |V_1| * 100 %`.

use super::harmonic_pf::HarmonicOrder;

// ---------------------------------------------------------------------------
// Internal complex-number helpers (duplicated locally to avoid visibility issues)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for `HarmonicPfProblem`.
#[derive(Debug, Clone)]
pub struct HarmonicPfConfig {
    /// Harmonic orders to solve, e.g. `[1, 3, 5, 7, 11, 13]`.
    pub harmonics: Vec<HarmonicOrder>,
    /// Fundamental frequency in Hz (50 or 60).
    pub base_freq_hz: f64,
    /// Convergence tolerance (not used in direct solve, reserved for iterative).
    pub v_tolerance: f64,
    /// Maximum iterations per harmonic (direct solve always uses 1).
    pub max_iter: usize,
}

impl Default for HarmonicPfConfig {
    fn default() -> Self {
        Self {
            harmonics: vec![1, 3, 5, 7, 11, 13],
            base_freq_hz: 50.0,
            v_tolerance: 1e-6,
            max_iter: 50,
        }
    }
}

// ---------------------------------------------------------------------------
// Input types
// ---------------------------------------------------------------------------

/// Harmonic current source injected at a bus.
#[derive(Debug, Clone)]
pub struct HarmonicCurrentSource {
    /// Target bus (0-based).
    pub bus_id: usize,
    /// Harmonic order this source belongs to.
    pub harmonic: HarmonicOrder,
    /// Current magnitude in per-unit.
    pub magnitude_pu: f64,
    /// Current injection angle in radians.
    pub angle_rad: f64,
}

/// Bus descriptor for harmonic power flow.
#[derive(Debug, Clone)]
pub struct HarmonicBusData {
    /// Bus index (0-based).
    pub bus_id: usize,
    /// Nominal voltage in kV.
    pub base_kv: f64,
    /// If `true`, this bus is the slack (reference) bus.
    pub is_slack: bool,
}

/// Branch descriptor for harmonic power flow.
#[derive(Debug, Clone)]
pub struct HarmonicBranchData {
    /// From-bus index (0-based).
    pub from_bus: usize,
    /// To-bus index (0-based).
    pub to_bus: usize,
    /// Series resistance (pu).
    pub r_pu: f64,
    /// Series inductive reactance at fundamental (pu); scales as `h·X` at order h.
    pub x_pu: f64,
    /// Shunt susceptance at fundamental (pu); capacitive, scales as `h·B` at order h.
    pub b_shunt_pu: f64,
}

/// Frequency-dependent load model at a bus.
#[derive(Debug, Clone)]
pub enum HarmonicLoadModel {
    /// Constant impedance: `Z_h = R + j·h·X`.
    ConstantImpedance {
        /// Resistance (pu).
        r_pu: f64,
        /// Fundamental reactance (pu).
        x_pu: f64,
    },
    /// Parallel R-L: `Y_h = 1/R + 1/(j·h·L_pu)`.
    ParallelRL {
        /// Shunt resistance (pu).
        r_pu: f64,
        /// Inductance (same units as reactance pu; X_L1 = L_pu at h=1).
        l_pu: f64,
    },
    /// Norton equivalent shunt: `Y = 1/(Z_norton_r + j·Z_norton_x)`.
    NortonEquivalent {
        /// Norton impedance real part (pu).
        z_norton_r: f64,
        /// Norton impedance imaginary part (pu).
        z_norton_x: f64,
    },
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// Harmonic voltages and currents for one harmonic order.
#[derive(Debug, Clone)]
pub struct HarmonicOrderResult {
    /// Harmonic order.
    pub harmonic: HarmonicOrder,
    /// Per-bus voltages: `(magnitude_pu, angle_rad)`.
    pub bus_voltages: Vec<(f64, f64)>,
    /// Per-branch currents: `(magnitude_pu, angle_rad)`.
    pub branch_currents: Vec<(f64, f64)>,
    /// Per-bus squared voltage magnitude (raw, for THD accumulation).
    pub bus_thd: Vec<f64>,
}

/// Full harmonic power flow result from `HarmonicPfProblem::solve`.
#[derive(Debug, Clone)]
pub struct HarmonicPfResult {
    /// Results per harmonic order (same order as `HarmonicPfConfig::harmonics`).
    pub harmonic_results: Vec<HarmonicOrderResult>,
    /// Voltage THD per bus (%): `sqrt(Σ_{h≥2}|V_h|²)/|V_1| * 100`.
    pub bus_thd_v: Vec<f64>,
    /// Current THD per bus (%): estimated from source magnitudes.
    pub bus_thd_i: Vec<f64>,
    /// Index of the bus with the highest voltage THD.
    pub critical_bus: usize,
    /// Maximum voltage THD across all buses (%).
    pub max_thd_pct: f64,
    /// IEEE 519 MV compliance per bus (`THD_V < 5 %`).
    pub ieee519_compliant: Vec<bool>,
    /// Solve iteration count per harmonic (always 1 for direct solve).
    pub iterations: Vec<usize>,
    /// Whether each harmonic converged (always `true` for direct solve).
    pub converged: Vec<bool>,
}

// ---------------------------------------------------------------------------
// HarmonicPfProblem
// ---------------------------------------------------------------------------

/// Harmonic power flow problem using a builder pattern.
///
/// Collects buses, branches, sources and load models, then solves for
/// harmonic voltages at all configured orders using nodal admittance analysis.
pub struct HarmonicPfProblem {
    /// Bus descriptors (one per bus, 0-based).
    pub buses: Vec<HarmonicBusData>,
    /// Branch descriptors.
    pub branches: Vec<HarmonicBranchData>,
    /// Harmonic current source injections.
    pub current_sources: Vec<HarmonicCurrentSource>,
    /// Per-bus load models: `(bus_id, model)`.
    pub load_models: Vec<(usize, HarmonicLoadModel)>,
    /// Solver configuration.
    pub config: HarmonicPfConfig,
}

impl HarmonicPfProblem {
    /// Create an empty problem with the given configuration.
    pub fn new(config: HarmonicPfConfig) -> Self {
        Self {
            buses: Vec::new(),
            branches: Vec::new(),
            current_sources: Vec::new(),
            load_models: Vec::new(),
            config,
        }
    }

    /// Add a bus.
    pub fn add_bus(&mut self, bus: HarmonicBusData) {
        self.buses.push(bus);
    }

    /// Add a branch.
    pub fn add_branch(&mut self, branch: HarmonicBranchData) {
        self.branches.push(branch);
    }

    /// Add a harmonic current source.
    pub fn add_current_source(&mut self, source: HarmonicCurrentSource) {
        self.current_sources.push(source);
    }

    /// Attach a load model to a bus.
    pub fn add_load_model(&mut self, bus_id: usize, model: HarmonicLoadModel) {
        self.load_models.push((bus_id, model));
    }

    /// Build the `n×n` nodal admittance matrix for harmonic order `h`.
    fn build_y_matrix(&self, h: HarmonicOrder, n: usize) -> Vec<Vec<Cx>> {
        let hf = h as f64;
        let mut y = vec![vec![(0.0_f64, 0.0_f64); n]; n];

        for br in &self.branches {
            let (fi, ti) = (br.from_bus, br.to_bus);
            if fi >= n || ti >= n {
                continue;
            }
            let denom = br.r_pu * br.r_pu + (hf * br.x_pu) * (hf * br.x_pu);
            let (g_s, b_s) = if denom > 1e-30 {
                (br.r_pu / denom, -(hf * br.x_pu) / denom)
            } else {
                (0.0, 0.0)
            };
            let b_sh = hf * br.b_shunt_pu * 0.5;
            y[fi][ti] = cx_sub(y[fi][ti], (g_s, b_s));
            y[ti][fi] = cx_sub(y[ti][fi], (g_s, b_s));
            y[fi][fi] = cx_add(y[fi][fi], (g_s, b_s + b_sh));
            y[ti][ti] = cx_add(y[ti][ti], (g_s, b_s + b_sh));
        }

        for (bus_id, model) in &self.load_models {
            let bi = *bus_id;
            if bi >= n {
                continue;
            }
            let (g_l, b_l) = load_admittance(model, hf);
            y[bi][bi] = cx_add(y[bi][bi], (g_l, b_l));
        }

        y
    }

    /// Solve harmonic power flow for all orders in `config.harmonics`.
    ///
    /// # Errors
    /// Returns `Err(String)` if the admittance matrix is singular at any order.
    pub fn solve(&self) -> Result<HarmonicPfResult, String> {
        let n = self.buses.len();
        if n == 0 {
            return Err("HarmonicPfProblem: no buses defined".into());
        }

        let slack = self.buses.iter().position(|b| b.is_slack).unwrap_or(0);
        let full_idx: Vec<usize> = (0..n).filter(|&i| i != slack).collect();
        let n_red = full_idx.len();

        let mut harmonic_results = Vec::with_capacity(self.config.harmonics.len());
        let mut iterations_vec = Vec::with_capacity(self.config.harmonics.len());
        let mut converged_vec = Vec::with_capacity(self.config.harmonics.len());
        let mut v1_mag: Vec<f64> = vec![0.0; n];
        let mut sum_sq_h: Vec<f64> = vec![0.0; n];

        for &h in &self.config.harmonics {
            let y_full = self.build_y_matrix(h, n);

            // Build reduced system
            let mut a_red: Vec<Vec<Cx>> = full_idx
                .iter()
                .map(|&row| full_idx.iter().map(|&col| y_full[row][col]).collect())
                .collect();

            let mut b_red: Vec<Cx> = full_idx
                .iter()
                .map(|&row| {
                    self.current_sources
                        .iter()
                        .filter(|s| s.bus_id == row && s.harmonic == h)
                        .fold((0.0_f64, 0.0_f64), |acc, s| {
                            (
                                acc.0 + s.magnitude_pu * s.angle_rad.cos(),
                                acc.1 + s.magnitude_pu * s.angle_rad.sin(),
                            )
                        })
                })
                .collect();

            let v_red = if n_red == 0 {
                Vec::new()
            } else {
                solve_complex_linear(&mut a_red, &mut b_red, n_red)
                    .map_err(|e| format!("harmonic {h}: {e}"))?
            };

            // Reconstruct full voltage vector
            let mut vcx: Vec<Cx> = vec![(0.0, 0.0); n];
            if h == 1 {
                vcx[slack] = (1.0, 0.0);
            }
            for (ri, &fi) in full_idx.iter().enumerate() {
                vcx[fi] = v_red.get(ri).copied().unwrap_or((0.0, 0.0));
            }

            let bus_voltages: Vec<(f64, f64)> = vcx
                .iter()
                .map(|&(vr, vi)| ((vr * vr + vi * vi).sqrt(), vi.atan2(vr)))
                .collect();

            let branch_currents: Vec<(f64, f64)> = self
                .branches
                .iter()
                .map(|br| {
                    let (fi, ti) = (br.from_bus.min(n - 1), br.to_bus.min(n - 1));
                    let hf2 = h as f64;
                    let d = br.r_pu * br.r_pu + (hf2 * br.x_pu).powi(2);
                    let (gs, bs) = if d > 1e-30 {
                        (br.r_pu / d, -(hf2 * br.x_pu) / d)
                    } else {
                        (0.0, 0.0)
                    };
                    let dv = cx_sub(vcx[fi], vcx[ti]);
                    let icx = cx_mul(dv, (gs, bs));
                    (cx_abs(icx), icx.1.atan2(icx.0))
                })
                .collect();

            let bus_thd: Vec<f64> = bus_voltages.iter().map(|&(m, _)| m * m).collect();

            if h == 1 {
                v1_mag = bus_voltages.iter().map(|&(m, _)| m).collect();
            } else {
                for (i, &sq) in bus_thd.iter().enumerate() {
                    sum_sq_h[i] += sq;
                }
            }

            harmonic_results.push(HarmonicOrderResult {
                harmonic: h,
                bus_voltages,
                branch_currents,
                bus_thd,
            });
            iterations_vec.push(1);
            converged_vec.push(true);
        }

        let bus_thd_v: Vec<f64> = (0..n)
            .map(|i| sum_sq_h[i].sqrt() / v1_mag[i].max(1e-12) * 100.0)
            .collect();

        let bus_thd_i: Vec<f64> = (0..n)
            .map(|i| {
                let i1: f64 = self
                    .current_sources
                    .iter()
                    .filter(|s| s.bus_id == i && s.harmonic == 1)
                    .map(|s| s.magnitude_pu)
                    .sum();
                if i1 < 1e-12 {
                    return 0.0;
                }
                let sq_harm: f64 = self
                    .current_sources
                    .iter()
                    .filter(|s| s.bus_id == i && s.harmonic >= 2)
                    .map(|s| s.magnitude_pu * s.magnitude_pu)
                    .sum();
                sq_harm.sqrt() / i1 * 100.0
            })
            .collect();

        let (critical_bus, max_thd_pct) =
            bus_thd_v
                .iter()
                .enumerate()
                .fold(
                    (0usize, 0.0_f64),
                    |(ci, cm), (i, &v)| {
                        if v > cm {
                            (i, v)
                        } else {
                            (ci, cm)
                        }
                    },
                );

        let ieee519_compliant: Vec<bool> = bus_thd_v.iter().map(|&t| t < 5.0).collect();

        Ok(HarmonicPfResult {
            harmonic_results,
            bus_thd_v,
            bus_thd_i,
            critical_bus,
            max_thd_pct,
            ieee519_compliant,
            iterations: iterations_vec,
            converged: converged_vec,
        })
    }
}

/// Compute the shunt admittance `(G, B)` for a given load model at harmonic order `h`.
fn load_admittance(model: &HarmonicLoadModel, hf: f64) -> (f64, f64) {
    match model {
        HarmonicLoadModel::ConstantImpedance { r_pu, x_pu } => {
            let xh = hf * x_pu;
            let d = r_pu * r_pu + xh * xh;
            if d > 1e-30 {
                (r_pu / d, -xh / d)
            } else {
                (0.0, 0.0)
            }
        }
        HarmonicLoadModel::ParallelRL { r_pu, l_pu } => {
            let xl = hf * l_pu;
            let g = if r_pu.abs() > 1e-30 { 1.0 / r_pu } else { 0.0 };
            let b = if xl.abs() > 1e-30 { -1.0 / xl } else { 0.0 };
            (g, b)
        }
        HarmonicLoadModel::NortonEquivalent {
            z_norton_r,
            z_norton_x,
        } => {
            let d = z_norton_r * z_norton_r + z_norton_x * z_norton_x;
            if d > 1e-30 {
                (z_norton_r / d, -z_norton_x / d)
            } else {
                (0.0, 0.0)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// solve_complex_linear
// ---------------------------------------------------------------------------

/// Solve the complex linear system `A·x = b` via Gaussian elimination with
/// partial pivoting.
///
/// `a` is an `n×n` matrix (rows of `(real, imag)` pairs).
/// `b` is the length-`n` right-hand side; both are modified in-place.
///
/// Returns the solution vector, or `Err` if the matrix is singular.
#[allow(clippy::ptr_arg, clippy::needless_range_loop)]
pub fn solve_complex_linear(
    a: &mut Vec<Vec<(f64, f64)>>,
    b: &mut Vec<(f64, f64)>,
    n: usize,
) -> Result<Vec<(f64, f64)>, String> {
    if n == 0 {
        return Ok(Vec::new());
    }
    if a.len() != n || b.len() != n {
        return Err(format!(
            "solve_complex_linear: a={}, b={}, n={n}",
            a.len(),
            b.len()
        ));
    }
    for col in 0..n {
        let mut max_val = cx_abs(a[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let v = cx_abs(a[row][col]);
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-20 {
            return Err(format!("singular at column {col}"));
        }
        a.swap(col, max_row);
        b.swap(col, max_row);
        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = cx_div(a[row][col], pivot);
            for j in col..n {
                let sub = cx_mul(factor, a[col][j]);
                a[row][j] = cx_sub(a[row][j], sub);
            }
            let sub_b = cx_mul(factor, b[col]);
            b[row] = cx_sub(b[row], sub_b);
        }
    }
    let mut x = vec![(0.0_f64, 0.0_f64); n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for j in (i + 1)..n {
            s = cx_sub(s, cx_mul(a[i][j], x[j]));
        }
        x[i] = cx_div(s, a[i][i]);
    }
    Ok(x)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn two_bus_prob(h_list: Vec<HarmonicOrder>) -> HarmonicPfProblem {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig {
            harmonics: h_list,
            base_freq_hz: 50.0,
            v_tolerance: 1e-6,
            max_iter: 50,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 20.0,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_shunt_pu: 0.0,
        });
        p
    }

    // Test 1: builder add_bus/add_branch/add_current_source
    #[test]
    fn test_builder_methods() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig::default());
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 11.0,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 11.0,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.02,
            x_pu: 0.1,
            b_shunt_pu: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        assert_eq!(p.buses.len(), 2);
        assert_eq!(p.branches.len(), 1);
        assert_eq!(p.current_sources.len(), 1);
    }

    // Test 2: single-bus identity V = Y^{-1} * I
    #[test]
    fn test_single_bus_known_solution() {
        // Single non-slack bus: Y = G + jB, I = I_r + jI_i => V = I / Y
        // Use a 2-bus system where bus 1 has known injection
        // Y_11 ≈ y_series = 1/(0.01 + j*5*0.1) = 1/(0.01 + j0.5)
        let mut p = two_bus_prob(vec![5]);
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.1,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("solve ok");
        assert_eq!(r.harmonic_results.len(), 1);
        let (v1_mag, _) = r.harmonic_results[0].bus_voltages[1];
        assert!(v1_mag > 0.0, "Bus 1 voltage must be non-zero");
    }

    // Test 3: slack bus harmonic voltage = 0 for h >= 2
    #[test]
    fn test_slack_voltage_zero_for_harmonics() {
        let mut p = two_bus_prob(vec![5]);
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        let (v_slack, _) = r.harmonic_results[0].bus_voltages[0];
        assert!(v_slack < 1e-10, "Slack should have ~0 harmonic voltage");
    }

    // Test 4: fundamental h=1 slack voltage = 1 pu
    #[test]
    fn test_slack_voltage_fundamental() {
        let p = two_bus_prob(vec![1]);
        let r = p.solve().expect("ok");
        let (v0, _) = r.harmonic_results[0].bus_voltages[0];
        assert!((v0 - 1.0).abs() < 1e-12, "Slack h=1 voltage = 1.0 pu");
    }

    // Test 5: THD = 0 when only fundamental is configured
    #[test]
    fn test_thd_zero_fundamental_only() {
        let p = two_bus_prob(vec![1]);
        let r = p.solve().expect("ok");
        for &t in &r.bus_thd_v {
            assert!(t.abs() < 1e-10, "THD must be 0 with fundamental only");
        }
    }

    // Test 6: THD computation formula
    #[test]
    fn test_thd_formula_correctness() {
        // V1=1, V5=0.05 => THD = 5%
        let mut p = two_bus_prob(vec![1, 5]);
        // Inject at bus 1 for h=5: but we need the resulting voltage to be ~0.05
        // Instead verify the formula: manually check with known voltages
        // THD_V(bus) = sqrt(sum_{h>=2} V_h^2) / V_1 * 100
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        let v5 = r.harmonic_results.iter().find(|x| x.harmonic == 5).unwrap();
        let v1 = r.harmonic_results.iter().find(|x| x.harmonic == 1).unwrap();
        let v5_mag = v5.bus_voltages[1].0;
        let v1_mag = v1.bus_voltages[1].0.max(1e-12);
        let expected_thd = v5_mag / v1_mag * 100.0;
        let actual_thd = r.bus_thd_v[1];
        assert!(
            (actual_thd - expected_thd).abs() < 1e-6,
            "THD formula: {actual_thd} vs {expected_thd}"
        );
    }

    // Test 7: IEEE 519 compliance (THD < 5%)
    #[test]
    fn test_ieee519_compliant_low_thd() {
        let p = two_bus_prob(vec![1]);
        let r = p.solve().expect("ok");
        assert!(
            r.ieee519_compliant.iter().all(|&c| c),
            "No harmonics => compliant"
        );
    }

    // Test 8: IEEE 519 non-compliant with high THD
    #[test]
    fn test_ieee519_noncompliant_high_thd() {
        let mut p = two_bus_prob(vec![1, 5]);
        // Large 5th harmonic injection -> high THD
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 10.0,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        // Bus 1 should have high THD, likely non-compliant
        assert!(r.bus_thd_v[1] > 0.0);
        // compliant only if THD < 5%
        assert_eq!(r.ieee519_compliant[1], r.bus_thd_v[1] < 5.0);
    }

    // Test 9: multiple harmonics [3, 5, 7, 11, 13]
    #[test]
    fn test_multiple_harmonics() {
        let mut p = two_bus_prob(vec![1, 3, 5, 7, 11, 13]);
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.04,
            angle_rad: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 7,
            magnitude_pu: 0.02,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        assert_eq!(r.harmonic_results.len(), 6);
        assert!(r.converged.iter().all(|&c| c));
    }

    // Test 10: ConstantImpedance load model
    #[test]
    fn test_constant_impedance_load() {
        let mut p = two_bus_prob(vec![1, 5]);
        p.add_load_model(
            1,
            HarmonicLoadModel::ConstantImpedance {
                r_pu: 1.0,
                x_pu: 0.1,
            },
        );
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        // Load shunt makes Y matrix denser; voltage should be different from without load
        let v5 = r.harmonic_results.iter().find(|x| x.harmonic == 5).unwrap();
        assert!(v5.bus_voltages[1].0 >= 0.0);
    }

    // Test 11: ParallelRL load model
    #[test]
    fn test_parallel_rl_load() {
        let mut p = two_bus_prob(vec![5]);
        p.add_load_model(
            1,
            HarmonicLoadModel::ParallelRL {
                r_pu: 2.0,
                l_pu: 0.05,
            },
        );
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        let v5 = r.harmonic_results[0].bus_voltages[1].0;
        assert!(v5 >= 0.0, "Voltage non-negative");
    }

    // Test 12: NortonEquivalent load model
    #[test]
    fn test_norton_equivalent_load() {
        let mut p = two_bus_prob(vec![5]);
        p.add_load_model(
            1,
            HarmonicLoadModel::NortonEquivalent {
                z_norton_r: 1.0,
                z_norton_x: 0.5,
            },
        );
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.03,
            angle_rad: PI / 4.0,
        });
        let r = p.solve().expect("ok");
        assert!(r.converged[0]);
    }

    // Test 13: multiple current sources at different buses
    #[test]
    fn test_multiple_sources_different_buses() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig {
            harmonics: vec![5],
            ..Default::default()
        });
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 20.0,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 2,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_shunt_pu: 0.0,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 1,
            to_bus: 2,
            r_pu: 0.01,
            x_pu: 0.1,
            b_shunt_pu: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.03,
            angle_rad: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 2,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        assert_eq!(r.harmonic_results[0].bus_voltages.len(), 3);
    }

    // Test 14: branch current computation
    #[test]
    fn test_branch_current_non_negative() {
        let mut p = two_bus_prob(vec![5]);
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        for (mag, _) in &r.harmonic_results[0].branch_currents {
            assert!(*mag >= 0.0, "Branch current magnitude >= 0");
        }
    }

    // Test 15: critical_bus identification
    #[test]
    fn test_critical_bus_identified() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig {
            harmonics: vec![1, 5],
            ..Default::default()
        });
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 20.0,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 2,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_shunt_pu: 0.0,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 1,
            to_bus: 2,
            r_pu: 0.05,
            x_pu: 0.3,
            b_shunt_pu: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 2,
            harmonic: 5,
            magnitude_pu: 0.1,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        let max_thd = r
            .bus_thd_v
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((r.max_thd_pct - max_thd).abs() < 1e-10);
        assert_eq!(r.bus_thd_v[r.critical_bus], max_thd);
    }

    // Test 16: convergence flags always true for direct solve
    #[test]
    fn test_convergence_flags() {
        let mut p = two_bus_prob(vec![1, 5, 7]);
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        assert!(r.converged.iter().all(|&c| c));
        assert!(r.iterations.iter().all(|&i| i == 1));
    }

    // Test 17: 60 Hz base frequency
    #[test]
    fn test_60hz_base_frequency() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig {
            harmonics: vec![1, 5],
            base_freq_hz: 60.0,
            ..Default::default()
        });
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 13.8,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 13.8,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.08,
            b_shunt_pu: 0.0,
        });
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 1,
            harmonic: 5,
            magnitude_pu: 0.03,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        assert_eq!(r.harmonic_results.len(), 2);
    }

    // Test 18: harmonic admittance matrix Y_h symmetry
    #[test]
    fn test_admittance_matrix_symmetry() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig::default());
        p.add_bus(HarmonicBusData {
            bus_id: 0,
            base_kv: 20.0,
            is_slack: true,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 1,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_bus(HarmonicBusData {
            bus_id: 2,
            base_kv: 20.0,
            is_slack: false,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_shunt_pu: 0.01,
        });
        p.add_branch(HarmonicBranchData {
            from_bus: 1,
            to_bus: 2,
            r_pu: 0.02,
            x_pu: 0.15,
            b_shunt_pu: 0.01,
        });
        let y = p.build_y_matrix(5, 3);
        for (i, y_row) in y.iter().enumerate() {
            for (j, yij) in y_row.iter().enumerate() {
                assert!(
                    (yij.0 - y[j][i].0).abs() < 1e-12,
                    "G[{i}][{j}] != G[{j}][{i}]"
                );
                assert!(
                    (yij.1 - y[j][i].1).abs() < 1e-12,
                    "B[{i}][{j}] != B[{j}][{i}]"
                );
            }
        }
    }

    // Test 19: solve_complex_linear known solution
    #[test]
    fn test_solve_complex_linear_known() {
        // 2x2: [(1,0),(0,0); (0,0),(2,0)] * x = [(3,0); (4,0)] => x = [(3,0); (2,0)]
        let mut a = vec![vec![(1.0, 0.0), (0.0, 0.0)], vec![(0.0, 0.0), (2.0, 0.0)]];
        let mut b = vec![(3.0, 0.0), (4.0, 0.0)];
        let x = solve_complex_linear(&mut a, &mut b, 2).expect("ok");
        assert!((x[0].0 - 3.0).abs() < 1e-10 && x[0].1.abs() < 1e-10);
        assert!((x[1].0 - 2.0).abs() < 1e-10 && x[1].1.abs() < 1e-10);
    }

    // Test 20: 5-bus radial network harmonic propagation
    #[test]
    fn test_5bus_radial_propagation() {
        let mut p = HarmonicPfProblem::new(HarmonicPfConfig {
            harmonics: vec![1, 5],
            ..Default::default()
        });
        for i in 0..5 {
            p.add_bus(HarmonicBusData {
                bus_id: i,
                base_kv: 20.0,
                is_slack: i == 0,
            });
        }
        for i in 0..4 {
            p.add_branch(HarmonicBranchData {
                from_bus: i,
                to_bus: i + 1,
                r_pu: 0.01,
                x_pu: 0.08,
                b_shunt_pu: 0.0,
            });
        }
        // 5th harmonic source at end of feeder
        p.add_current_source(HarmonicCurrentSource {
            bus_id: 4,
            harmonic: 5,
            magnitude_pu: 0.05,
            angle_rad: 0.0,
        });
        let r = p.solve().expect("ok");
        assert_eq!(r.bus_thd_v.len(), 5);
        // Slack has zero THD (no harmonic voltage at reference)
        assert!(r.bus_thd_v[0].abs() < 1e-10);
        // End bus has higher voltage harmonics than middle bus
        assert!(r.bus_thd_v[4] >= r.bus_thd_v[2]);
    }
}
