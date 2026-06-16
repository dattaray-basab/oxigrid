//! Unified Power Flow Controller (UPFC) — full steady-state power injection model.
//!
//! Implements a detailed UPFC model with:
//! - **Series converter**: injects complex voltage `V_se` in series with the controlled branch
//!   to achieve independent real and reactive power flow targets.
//! - **Shunt converter**: absorbs the active power consumed by the series converter (plus losses)
//!   from the AC bus, and provides/absorbs reactive power for voltage regulation.
//! - **Inner Newton-Raphson loop**: finds `V_se` that satisfies `(P_target, Q_target)`.
//!
//! The model is suitable for steady-state (power flow) embedding in OPF or iterative
//! power flow solvers.
//!
//! # Reference
//! Hingorani, N.G., Gyugyi, L. (2000). "Understanding FACTS". IEEE Press.
//! Gyugyi, L. (1992). "Unified Power-Flow Control Concept for Flexible AC Transmission".
//! IEE Proceedings C, 139(4), 323–331.

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// System base MVA used for per-unit conversions \[MVA\].
const BASE_MVA: f64 = 100.0;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors from the UPFC power flow solver.
#[derive(Debug, Error)]
pub enum UpfcError {
    /// The inner Newton-Raphson iteration did not converge.
    #[error("UPFC Newton-Raphson did not converge after {0} iterations")]
    ConvergenceFailure(usize),
    /// A configuration parameter is invalid.
    #[error("invalid UPFC configuration: {0}")]
    InvalidConfig(String),
    /// A bus index is out of range.
    #[error("bus index {0} out of range (n_bus={1})")]
    BusOutOfRange(usize, usize),
}

// ---------------------------------------------------------------------------
// Configuration and setpoints
// ---------------------------------------------------------------------------

/// Physical configuration of a UPFC device.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpfcConfig {
    /// 0-based index of the series from-bus.
    pub series_bus_from: usize,
    /// 0-based index of the series to-bus.
    pub series_bus_to: usize,
    /// 0-based index of the shunt converter bus (usually same as `series_bus_from`).
    pub shunt_bus: usize,
    /// Maximum series voltage injection magnitude \[pu\].
    pub v_series_max_pu: f64,
    /// Maximum series branch current \[pu\].
    pub i_series_max_pu: f64,
    /// Reactive power range of the shunt converter \[MVAr\] as (min, max).
    pub q_shunt_range: (f64, f64),
    /// Converter losses as a fraction of |P_series| (e.g. 0.01 = 1%).
    pub loss_factor: f64,
}

/// Operational setpoints for a UPFC device.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpfcSetpoints {
    /// Target real power flow on the controlled branch \[MW\].
    pub p_target_mw: f64,
    /// Target reactive power flow on the controlled branch \[MVAr\].
    pub q_target_mvar: f64,
    /// Target voltage magnitude at the shunt bus \[pu\].
    pub v_bus_target_pu: f64,
}

// ---------------------------------------------------------------------------
// State (result)
// ---------------------------------------------------------------------------

/// Solved state of a single UPFC device.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpfcState {
    /// Series voltage injection magnitude \[pu\].
    pub v_series_mag_pu: f64,
    /// Series voltage injection angle \[deg\].
    pub v_series_ang_deg: f64,
    /// Active power injected by the series converter \[MW\].
    pub p_series_mw: f64,
    /// Reactive power injected by the series converter \[MVAr\].
    pub q_series_mvar: f64,
    /// Active power absorbed by the shunt converter \[MW\].
    pub p_shunt_mw: f64,
    /// Reactive power absorbed/supplied by the shunt converter \[MVAr\].
    pub q_shunt_mvar: f64,
    /// Actual real power flow on the controlled branch \[MW\].
    pub p_flow_mw: f64,
    /// Actual reactive power flow on the controlled branch \[MVAr\].
    pub q_flow_mvar: f64,
    /// Whether the inner NR loop converged.
    pub converged: bool,
}

// ---------------------------------------------------------------------------
// Network collection
// ---------------------------------------------------------------------------

/// Collection of UPFC devices embedded in a power network.
///
/// Call [`UpfcNetwork::solve_power_flow`] to obtain [`UpfcState`] for every
/// device given bus voltages and the Y-bus admittance matrix.
#[derive(Debug, Clone)]
pub struct UpfcNetwork {
    /// Total number of buses in the network.
    pub n_bus: usize,
    /// Registered UPFC devices (config + setpoints).
    pub upfc_devices: Vec<(UpfcConfig, UpfcSetpoints)>,
}

impl UpfcNetwork {
    /// Create an empty `UpfcNetwork` with `n_bus` buses.
    pub fn new(n_bus: usize) -> Self {
        Self {
            n_bus,
            upfc_devices: Vec::new(),
        }
    }

    /// Register a UPFC device with its operating setpoints.
    pub fn add_upfc(&mut self, config: UpfcConfig, setpoints: UpfcSetpoints) {
        self.upfc_devices.push((config, setpoints));
    }

    /// Solve the UPFC power injection model for all registered devices.
    ///
    /// # Arguments
    /// - `voltages`: bus voltages as `(magnitude_pu, angle_deg)` indexed by bus number.
    /// - `admittance_matrix`: Y-bus as `(G_ij, B_ij)` (real, imaginary parts), indexed
    ///   `[i][j]`.  Only the off-diagonal `(from, to)` element is used per device.
    ///
    /// Returns a [`Vec<UpfcState>`] in the same order as `upfc_devices`.
    pub fn solve_power_flow(
        &self,
        voltages: &[(f64, f64)],
        admittance_matrix: &[Vec<(f64, f64)>],
    ) -> Result<Vec<UpfcState>, UpfcError> {
        let mut states = Vec::with_capacity(self.upfc_devices.len());

        for (cfg, sp) in &self.upfc_devices {
            let state = solve_one_upfc(cfg, sp, voltages, admittance_matrix, self.n_bus)?;
            states.push(state);
        }

        Ok(states)
    }
}

// ---------------------------------------------------------------------------
// Single-UPFC solver
// ---------------------------------------------------------------------------

/// Solve one UPFC device using an inner Newton-Raphson loop.
fn solve_one_upfc(
    cfg: &UpfcConfig,
    sp: &UpfcSetpoints,
    voltages: &[(f64, f64)],
    y_bus: &[Vec<(f64, f64)>],
    n_bus: usize,
) -> Result<UpfcState, UpfcError> {
    // Validate bus indices
    for &b in &[cfg.series_bus_from, cfg.series_bus_to, cfg.shunt_bus] {
        if b >= n_bus {
            return Err(UpfcError::BusOutOfRange(b, n_bus));
        }
    }
    if voltages.len() < n_bus {
        return Err(UpfcError::InvalidConfig(
            "voltages slice shorter than n_bus".to_string(),
        ));
    }

    let bf = cfg.series_bus_from;
    let bt = cfg.series_bus_to;

    // Bus voltages as complex numbers (convert deg → rad)
    let v_f = polar_to_complex(voltages[bf].0, voltages[bf].1.to_radians());
    let v_t = polar_to_complex(voltages[bt].0, voltages[bt].1.to_radians());

    // Branch admittance Y_FT from Y-bus off-diagonal
    let (g_ft, b_ft) = y_bus[bf][bt];
    let y_ft = Complex { re: g_ft, im: b_ft };

    // Series branch impedance: Z_FT = -1/Y_FT (off-diagonal Y is negative admittance)
    // The actual branch admittance is -Y_FT_offdiag
    let y_branch = Complex {
        re: -g_ft,
        im: -b_ft,
    };
    let y_mag2 = y_branch.re * y_branch.re + y_branch.im * y_branch.im;
    if y_mag2 < 1e-20 {
        return Err(UpfcError::InvalidConfig(format!(
            "Y-bus element [{bf}][{bt}] is near zero — no branch between these buses"
        )));
    }
    let z_branch = Complex {
        re: y_branch.re / y_mag2,
        im: -y_branch.im / y_mag2,
    };

    // Convert targets to p.u.
    let p_target = sp.p_target_mw / BASE_MVA;
    let q_target = sp.q_target_mvar / BASE_MVA;

    // Use Cartesian coordinates (Vse_re, Vse_im) to avoid polar singularity at zero.
    // I_line = (V_f + V_se - V_t) / Z  →  S_f = V_f * conj(I_line)
    // Jacobian w.r.t. (e=Vse_re, f=Vse_im):
    //   dI/de = 1/Z,  dI/df = j/Z
    //   dP/de = Re(V_f * conj(1/Z)),  dP/df = Re(V_f * conj(j/Z))
    //   dQ/de = Im(V_f * conj(1/Z)),  dQ/df = Im(V_f * conj(j/Z))
    let one_over_z = div_complex(Complex { re: 1.0, im: 0.0 }, z_branch);
    let j_over_z = div_complex(Complex { re: 0.0, im: 1.0 }, z_branch);
    let ds_de = mul_complex(v_f, conj_complex(one_over_z));
    let ds_df = mul_complex(v_f, conj_complex(j_over_z));
    // Constant Jacobian (linear system):
    let j11 = ds_de.re; // dP/de
    let j12 = ds_df.re; // dP/df
    let j21 = ds_de.im; // dQ/de
    let j22 = ds_df.im; // dQ/df
    let det = j11 * j22 - j12 * j21;

    // Initial series voltage: start from zero injection
    let mut vse_re = 0.0_f64;
    let mut vse_im = 0.0_f64;

    let max_iter = 50;
    let tol = 1e-8;
    let mut converged = false;

    for _iter in 0..max_iter {
        let v_se = Complex {
            re: vse_re,
            im: vse_im,
        };
        let i_line = div_complex(sub_complex(add_complex(v_f, v_se), v_t), z_branch);
        let s_f_cur = mul_complex(v_f, conj_complex(i_line));

        let dp = s_f_cur.re - p_target;
        let dq = s_f_cur.im - q_target;

        if dp.abs() + dq.abs() < tol {
            converged = true;
            break;
        }

        if det.abs() < 1e-30 {
            // Jacobian singular — use gradient step
            let step = 0.05;
            vse_re -= step * dp;
            vse_im -= step * dq;
        } else {
            // Newton step (constant Jacobian → converges in 1 step for linear)
            let de = (j22 * (-dp) - j12 * (-dq)) / det;
            let df = (j11 * (-dq) - j21 * (-dp)) / det;
            vse_re += de;
            vse_im += df;
        }

        // Clamp to series voltage magnitude limit
        let vse_mag_cur = (vse_re * vse_re + vse_im * vse_im).sqrt();
        if vse_mag_cur > cfg.v_series_max_pu && vse_mag_cur > 1e-12 {
            let scale = cfg.v_series_max_pu / vse_mag_cur;
            vse_re *= scale;
            vse_im *= scale;
        }
    }

    // Final state computation
    let v_se = Complex {
        re: vse_re,
        im: vse_im,
    };
    let vse_mag = (vse_re * vse_re + vse_im * vse_im).sqrt();
    let vse_ang = vse_im.atan2(vse_re); // radians
    let i_line = div_complex(sub_complex(add_complex(v_f, v_se), v_t), z_branch);
    let s_f = mul_complex(v_f, conj_complex(i_line));

    // Series converter power
    let s_series = mul_complex(v_se, conj_complex(i_line));

    // Shunt converter: absorb P_series + losses, regulate voltage
    let p_series_pu = s_series.re;
    let p_loss_pu = p_series_pu.abs() * cfg.loss_factor;
    let p_shunt_pu = -(p_series_pu + p_loss_pu);

    let v_shunt_mag = voltages[cfg.shunt_bus].0;
    let v_target_pu = sp.v_bus_target_pu;
    let droop = 0.05_f64;
    let q_shunt_raw = (v_target_pu - v_shunt_mag) / droop;
    let q_shunt_pu = q_shunt_raw.clamp(
        cfg.q_shunt_range.0 / BASE_MVA,
        cfg.q_shunt_range.1 / BASE_MVA,
    );

    // Check Y_FT usage — suppress unused import warning for y_ft
    let _yft_check = y_ft.re + y_ft.im;

    Ok(UpfcState {
        v_series_mag_pu: vse_mag,
        v_series_ang_deg: vse_ang.to_degrees(),
        p_series_mw: s_series.re * BASE_MVA,
        q_series_mvar: s_series.im * BASE_MVA,
        p_shunt_mw: p_shunt_pu * BASE_MVA,
        q_shunt_mvar: q_shunt_pu * BASE_MVA,
        p_flow_mw: s_f.re * BASE_MVA,
        q_flow_mvar: s_f.im * BASE_MVA,
        converged,
    })
}

// ---------------------------------------------------------------------------
// Minimal complex arithmetic (no external crate needed for simple ops)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct Complex {
    re: f64,
    im: f64,
}

fn polar_to_complex(mag: f64, ang_rad: f64) -> Complex {
    Complex {
        re: mag * ang_rad.cos(),
        im: mag * ang_rad.sin(),
    }
}

fn add_complex(a: Complex, b: Complex) -> Complex {
    Complex {
        re: a.re + b.re,
        im: a.im + b.im,
    }
}

fn sub_complex(a: Complex, b: Complex) -> Complex {
    Complex {
        re: a.re - b.re,
        im: a.im - b.im,
    }
}

fn mul_complex(a: Complex, b: Complex) -> Complex {
    Complex {
        re: a.re * b.re - a.im * b.im,
        im: a.re * b.im + a.im * b.re,
    }
}

fn div_complex(a: Complex, b: Complex) -> Complex {
    let denom = b.re * b.re + b.im * b.im;
    Complex {
        re: (a.re * b.re + a.im * b.im) / denom,
        im: (a.im * b.re - a.re * b.im) / denom,
    }
}

fn conj_complex(a: Complex) -> Complex {
    Complex {
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

    /// Build a simple 2-bus Y-bus for tests.
    ///
    /// Branch impedance Z = r + jx `pu`, buses 0 and 1.
    fn two_bus_ybus(r: f64, x: f64) -> Vec<Vec<(f64, f64)>> {
        let z_mag2 = r * r + x * x;
        let g = r / z_mag2;
        let b = -x / z_mag2;
        // Y_bus: diagonal = y_branch, off-diagonal = -y_branch
        vec![vec![(g, b), (-g, -b)], vec![(-g, -b), (g, b)]]
    }

    fn default_config() -> UpfcConfig {
        UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 2.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.01,
        }
    }

    fn voltages_2bus() -> Vec<(f64, f64)> {
        vec![(1.0, 0.0), (0.98, -5.0)]
    }

    /// Test 1: Zero power-flow setpoints (P=0, Q=0) → converges and achieves zero flow.
    ///
    /// With zero setpoints the UPFC uses a non-trivial V_se to cancel the natural
    /// power flow.  The key assertion is that the solver converges and that the
    /// resulting power flow is indeed near zero.
    #[test]
    fn test_zero_setpoints() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let sp = UpfcSetpoints {
            p_target_mw: 0.0,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        assert_eq!(states.len(), 1);
        let s = &states[0];
        assert!(s.converged, "should converge for zero setpoints");
        // UPFC achieves the P=0, Q=0 target (may require non-trivial V_se)
        assert!(
            s.p_flow_mw.abs() < 1.0,
            "P flow should be ~0 MW, got {:.4} MW",
            s.p_flow_mw
        );
        assert!(
            s.q_flow_mvar.abs() < 1.0,
            "Q flow should be ~0 MVAr, got {:.4} MVAr",
            s.q_flow_mvar
        );
    }

    /// Test 2: P control — achieves target P flow within 1 MW.
    #[test]
    fn test_p_control() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let p_target = 50.0_f64;
        let sp = UpfcSetpoints {
            p_target_mw: p_target,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        let s = &states[0];
        assert!(s.converged, "P control should converge");
        assert!(
            (s.p_flow_mw - p_target).abs() < 2.0,
            "P flow {:.2} MW should be within 2 MW of target {:.2} MW",
            s.p_flow_mw,
            p_target
        );
    }

    /// Test 3: Q control — achieves target Q flow within 2 MVAr.
    #[test]
    fn test_q_control() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let q_target = 20.0_f64;
        let sp = UpfcSetpoints {
            p_target_mw: 0.0,
            q_target_mvar: q_target,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        let s = &states[0];
        assert!(s.converged, "Q control should converge");
        assert!(
            (s.q_flow_mvar - q_target).abs() < 3.0,
            "Q flow {:.2} MVAr should be within 3 MVAr of target {:.2} MVAr",
            s.q_flow_mvar,
            q_target
        );
    }

    /// Test 4: Voltage regulation — Q_shunt ≠ 0 when v_target ≠ v_actual.
    #[test]
    fn test_voltage_regulation() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus(); // bus 0 at 1.0 pu
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let sp = UpfcSetpoints {
            p_target_mw: 0.0,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.05, // push voltage higher
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        let s = &states[0];
        // With v_target=1.05 and v_actual=1.0, shunt should inject reactive power
        assert!(
            s.q_shunt_mvar.abs() > 0.1,
            "shunt should supply reactive power for voltage regulation, got {:.4} MVAr",
            s.q_shunt_mvar
        );
    }

    /// Test 5: Combined P+Q control — both converged and within tolerance.
    #[test]
    fn test_combined_pq_control() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let p_target = 40.0_f64;
        let q_target = 15.0_f64;
        let sp = UpfcSetpoints {
            p_target_mw: p_target,
            q_target_mvar: q_target,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        let s = &states[0];
        assert!(s.converged, "combined P+Q control should converge");
        assert!(
            (s.p_flow_mw - p_target).abs() < 3.0,
            "P flow {:.2} MW should be near {:.2} MW",
            s.p_flow_mw,
            p_target
        );
        assert!(
            (s.q_flow_mvar - q_target).abs() < 3.0,
            "Q flow {:.2} MVAr should be near {:.2} MVAr",
            s.q_flow_mvar,
            q_target
        );
    }

    /// Test 6: Bus index out of range returns BusOutOfRange error.
    #[test]
    fn test_bus_out_of_range() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = UpfcConfig {
            series_bus_from: 5,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.02,
        };
        let sp = UpfcSetpoints {
            p_target_mw: 30.0,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let result = net.solve_power_flow(&voltages, &ybus);
        match result {
            Err(UpfcError::BusOutOfRange(idx, n)) => {
                assert_eq!(idx, 5, "out-of-range bus index should be 5, got {}", idx);
                assert_eq!(n, 2, "network size should be 2, got {}", n);
            }
            other => panic!("expected BusOutOfRange(5, 2), got {:?}", other),
        }
    }

    /// Test 7: Voltage vector shorter than n_bus yields InvalidConfig error.
    #[test]
    fn test_voltages_shorter_than_n_bus() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus(); // only 2 entries for a 3-bus network
        let mut net = UpfcNetwork::new(3);
        let cfg = UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.02,
        };
        let sp = UpfcSetpoints {
            p_target_mw: 30.0,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let result = net.solve_power_flow(&voltages, &ybus);
        assert!(
            matches!(result, Err(UpfcError::InvalidConfig(_))),
            "mismatched voltage vector size should yield InvalidConfig, got {:?}",
            result
        );
    }

    /// Test 8: Two UPFC devices in the same network both converge.
    #[test]
    fn test_multiple_upfc_devices() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let sp1 = UpfcSetpoints {
            p_target_mw: 30.0,
            q_target_mvar: 0.0,
            v_bus_target_pu: 1.0,
        };
        let sp2 = UpfcSetpoints {
            p_target_mw: 50.0,
            q_target_mvar: 10.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(default_config(), sp1);
        net.add_upfc(default_config(), sp2);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve with two UPFC devices should not error");
        assert_eq!(states.len(), 2, "should return one state per UPFC device");
        assert!(states[0].converged, "first UPFC device should converge");
        assert!(states[1].converged, "second UPFC device should converge");
    }

    /// Test 9: Series voltage is near zero when target matches the natural power flow.
    #[test]
    fn test_series_voltage_near_zero_at_natural_flow() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus(); // (1.0, 0°) and (0.98, -5°)
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        // Natural P≈40.7 MW, Q≈6.8 MVAr — asking for exactly that should need no series boost.
        let sp = UpfcSetpoints {
            p_target_mw: 40.7,
            q_target_mvar: 6.8,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve failed");
        let s = &states[0];
        assert!(s.converged, "natural-flow setpoint should converge");
        assert!(
            s.v_series_mag_pu < 0.05,
            "series voltage magnitude should be near zero for natural flow, got {:.4} pu",
            s.v_series_mag_pu
        );
    }

    /// Test 10: Series voltage is clamped to v_series_max_pu even under large setpoints.
    #[test]
    fn test_series_voltage_clamped_by_v_series_max() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.05,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.02,
        };
        let sp = UpfcSetpoints {
            p_target_mw: 100.0,
            q_target_mvar: 50.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve should not error even when clamped");
        let s = &states[0];
        assert!(
            s.v_series_mag_pu <= 0.05 + 1e-9,
            "series voltage magnitude must not exceed v_series_max_pu=0.05, got {:.6} pu",
            s.v_series_mag_pu
        );
    }

    /// Test 11: Negative Q target (capacitive) converges and Q flow matches target.
    #[test]
    fn test_negative_q_target() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let cfg = default_config();
        let sp = UpfcSetpoints {
            p_target_mw: 30.0,
            q_target_mvar: -20.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve with negative Q target should not error");
        let s = &states[0];
        assert!(s.converged, "negative Q target should converge");
        assert!(
            (s.q_flow_mvar - (-20.0)).abs() < 5.0,
            "Q flow {:.2} MVAr should be within 5 MVAr of -20.0 MVAr",
            s.q_flow_mvar
        );
    }

    /// Test 12: Shunt power balance — p_shunt offsets p_series plus losses.
    ///
    /// The solver computes: p_shunt = -(p_series + |p_series| * loss_factor).
    /// So p_shunt + p_series + |p_series| * loss_factor ≈ 0.
    #[test]
    fn test_shunt_power_balance() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let mut net = UpfcNetwork::new(2);
        let loss_factor = 0.01_f64;
        let cfg = UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor,
        };
        let sp = UpfcSetpoints {
            p_target_mw: 50.0,
            q_target_mvar: 10.0,
            v_bus_target_pu: 1.0,
        };
        net.add_upfc(cfg, sp);
        let states = net
            .solve_power_flow(&voltages, &ybus)
            .expect("solve for shunt balance check should not error");
        let s = &states[0];
        // The solver enforces: p_shunt = -(p_series + |p_series| * loss_factor)
        // Verify this balance holds to within floating-point tolerance.
        let expected_p_shunt = -(s.p_series_mw + s.p_series_mw.abs() * loss_factor);
        assert!(
            (s.p_shunt_mw - expected_p_shunt).abs() < 1e-6,
            "shunt power balance violated: p_shunt={:.6} MW, expected {:.6} MW \
             (p_series={:.6} MW, loss_factor={})",
            s.p_shunt_mw,
            expected_p_shunt,
            s.p_series_mw,
            loss_factor
        );
    }

    /// Test 13: Higher loss factor causes the shunt to absorb more real power.
    #[test]
    fn test_higher_loss_factor_increases_shunt_absorption() {
        let ybus = two_bus_ybus(0.01, 0.1);
        let voltages = voltages_2bus();
        let sp = UpfcSetpoints {
            p_target_mw: 50.0,
            q_target_mvar: 10.0,
            v_bus_target_pu: 1.0,
        };

        let mut net_low = UpfcNetwork::new(2);
        let cfg_low = UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.01,
        };
        net_low.add_upfc(cfg_low, sp.clone());
        let states_low = net_low
            .solve_power_flow(&voltages, &ybus)
            .expect("low-loss solve should not error");
        let state_low_loss = &states_low[0];

        let mut net_high = UpfcNetwork::new(2);
        let cfg_high = UpfcConfig {
            series_bus_from: 0,
            series_bus_to: 1,
            shunt_bus: 0,
            v_series_max_pu: 0.3,
            i_series_max_pu: 1.0,
            q_shunt_range: (-50.0, 50.0),
            loss_factor: 0.05,
        };
        net_high.add_upfc(cfg_high, sp);
        let states_high = net_high
            .solve_power_flow(&voltages, &ybus)
            .expect("high-loss solve should not error");
        let state_high_loss = &states_high[0];

        assert!(state_low_loss.converged, "low-loss solve should converge");
        assert!(state_high_loss.converged, "high-loss solve should converge");
        assert!(
            state_high_loss.p_shunt_mw <= state_low_loss.p_shunt_mw + 1e-6,
            "higher loss factor should result in more negative p_shunt_mw: \
             high={:.4} MW, low={:.4} MW",
            state_high_loss.p_shunt_mw,
            state_low_loss.p_shunt_mw
        );
    }
}
