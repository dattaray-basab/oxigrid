//! Harmonic Power Flow.
//!
//! Computes the steady-state harmonic voltages across all buses of a power network
//! due to non-linear (harmonic-current-injecting) loads and devices.
//!
//! # Method
//!
//! For each harmonic order `h` the harmonic admittance matrix `Y_h` is built from
//! the fundamental-frequency Y-bus, scaling branch parameters as:
//!
//! ```text
//! R_h = R_1          (resistance independent of frequency)
//! X_h = h · X_1     (inductive reactance scales linearly with h)
//! B_h = h · B_1     (shunt charging susceptance scales linearly with h)
//! ```
//!
//! The harmonic voltage vector is then:
//!
//! ```text
//! V_h = Z_h · I_h = Y_h⁻¹ · I_h
//! ```
//!
//! solved via dense Gaussian elimination (suitable for typical distribution networks
//! of up to a few hundred buses).
//!
//! Total Harmonic Distortion at each bus is:
//!
//! ```text
//! THD_i = 100 · √(Σ_{h≥2} |V_{h,i}|²) / |V_{1,i}|
//! ```
//!
//! where `V_{1,i}` is the fundamental-frequency voltage (assumed 1.0 pu flat start).
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Specification of the harmonic power flow problem.
#[derive(Debug, Clone)]
pub struct HarmonicPowerFlow {
    /// Fundamental frequency (Hz) (typically 50 or 60).
    pub fundamental_hz: f64,
    /// Harmonic orders to analyse, e.g. `[3, 5, 7, 11, 13]`.
    pub harmonics: Vec<usize>,
}

/// A harmonic current injection at one bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicInjection {
    /// 0-based internal bus index.
    pub bus: usize,
    /// Harmonic order (must match one entry in [`HarmonicPowerFlow::harmonics`]).
    pub harmonic_order: usize,
    /// Injection current magnitude [p.u.].
    pub current_magnitude_pu: f64,
    /// Injection current angle (rad).
    pub current_angle_rad: f64,
}

/// Results of a harmonic power flow solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicPfResult {
    /// Harmonic voltage phasors per bus per harmonic:
    /// `harmonic_voltages[bus_idx][h_idx]` is the complex voltage at bus `bus_idx`
    /// for the `h_idx`-th harmonic order in [`HarmonicPowerFlow::harmonics`].
    pub harmonic_voltages: Vec<Vec<Complex64>>,
    /// Total harmonic distortion at each bus [%].
    pub thd_per_bus: Vec<f64>,
    /// Individual harmonic voltage magnitudes as % of fundamental (assumed 1 pu):
    /// `individual_harmonics[bus_idx][h_idx]`.
    pub individual_harmonics: Vec<Vec<f64>>,
    /// Harmonic voltage magnitudes in per-unit:
    /// `voltage_harmonic_spectrum[bus_idx][h_idx]`.
    pub voltage_harmonic_spectrum: Vec<Vec<f64>>,
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

impl HarmonicPowerFlow {
    /// Create a harmonic power flow problem.
    pub fn new(fundamental_hz: f64, harmonics: Vec<usize>) -> Self {
        Self {
            fundamental_hz,
            harmonics,
        }
    }

    /// Solve the harmonic power flow.
    ///
    /// For each harmonic order `h`:
    /// 1. Build the harmonic admittance matrix `Y_h` (n×n complex, CSR-format dense).
    /// 2. Assemble the harmonic current injection vector `I_h` (n-element complex).
    /// 3. Solve `Y_h · V_h = I_h` via dense Gaussian elimination with partial pivoting.
    /// 4. Store `V_h` and compute per-bus THD.
    ///
    /// # Errors
    /// - [`OxiGridError::InvalidParameter`] for invalid bus indices or harmonic orders.
    /// - [`OxiGridError::LinearAlgebra`] if `Y_h` is singular for any harmonic.
    pub fn solve(
        &self,
        network: &PowerNetwork,
        injections: &[HarmonicInjection],
    ) -> Result<HarmonicPfResult> {
        let n_bus = network.bus_count();
        let n_harm = self.harmonics.len();

        if n_bus == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "Network has no buses".to_string(),
            ));
        }

        // Validate injections
        for inj in injections {
            if inj.bus >= n_bus {
                return Err(OxiGridError::InvalidParameter(format!(
                    "Injection bus index {} out of range (n_bus={n_bus})",
                    inj.bus
                )));
            }
            if !self.harmonics.contains(&inj.harmonic_order) {
                return Err(OxiGridError::InvalidParameter(format!(
                    "Harmonic order {} not in harmonics list {:?}",
                    inj.harmonic_order, self.harmonics
                )));
            }
        }

        // Pre-allocate result arrays
        // harmonic_voltages[bus][harm_idx]
        let mut harmonic_voltages: Vec<Vec<Complex64>> =
            vec![vec![Complex64::new(0.0, 0.0); n_harm]; n_bus];

        for (h_idx, &h) in self.harmonics.iter().enumerate() {
            // Build Y_h as a dense n×n matrix (stored row-major)
            let y_h = build_harmonic_y_dense(network, h)?;

            // Build I_h vector
            let mut i_h: Vec<Complex64> = vec![Complex64::new(0.0, 0.0); n_bus];
            for inj in injections {
                if inj.harmonic_order == h {
                    let i_phasor =
                        Complex64::from_polar(inj.current_magnitude_pu, inj.current_angle_rad);
                    i_h[inj.bus] += i_phasor;
                }
            }

            // Solve Y_h · V_h = I_h via Gaussian elimination
            let v_h = dense_solve(&y_h, &i_h, n_bus)?;

            for bus in 0..n_bus {
                harmonic_voltages[bus][h_idx] = v_h[bus];
            }
        }

        // Compute THD and per-harmonic spectra
        // THD referenced to flat-start fundamental voltage = 1.0 pu
        let v1_assumed = 1.0_f64;

        let mut thd_per_bus = vec![0.0_f64; n_bus];
        let mut individual_harmonics = vec![vec![0.0_f64; n_harm]; n_bus];
        let mut voltage_harmonic_spectrum = vec![vec![0.0_f64; n_harm]; n_bus];

        for bus in 0..n_bus {
            let mut sum_sq = 0.0_f64;
            for h_idx in 0..n_harm {
                let v_mag = harmonic_voltages[bus][h_idx].norm();
                voltage_harmonic_spectrum[bus][h_idx] = v_mag;
                let ihd = 100.0 * v_mag / v1_assumed;
                individual_harmonics[bus][h_idx] = ihd;
                sum_sq += v_mag * v_mag;
            }
            thd_per_bus[bus] = 100.0 * sum_sq.sqrt() / v1_assumed;
        }

        Ok(HarmonicPfResult {
            harmonic_voltages,
            thd_per_bus,
            individual_harmonics,
            voltage_harmonic_spectrum,
        })
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Build the harmonic admittance matrix for order `h` as a dense row-major array
/// of length n×n complex values.
///
/// Harmonic scaling rules:
/// - `Z_h = R + j·h·X`  (series branch impedance at harmonic h)
/// - `Y_h_series = 1/Z_h`
/// - `B_h_shunt = h · B1`  (half-line charging at each end)
/// - Bus shunt: `G_s + j·h·B_s`
fn build_harmonic_y_dense(network: &PowerNetwork, h: usize) -> Result<Vec<Complex64>> {
    let n = network.bus_count();
    let mut y = vec![Complex64::new(0.0, 0.0); n * n];

    let hf = h as f64;

    for branch in &network.branches {
        if !branch.status {
            continue;
        }
        let i = network.bus_index(branch.from_bus)?;
        let j = network.bus_index(branch.to_bus)?;

        // Harmonic series impedance
        let z_h = Complex64::new(branch.r, hf * branch.x);
        if z_h.norm_sqr() < 1e-30 {
            continue; // degenerate branch
        }
        let y_series = z_h.inv();

        // Harmonic shunt susceptance (half charging at each end)
        let b_shunt = hf * branch.b / 2.0;
        let y_shunt = Complex64::new(0.0, b_shunt);

        // Tap (treated as 1.0 for harmonics — transformer core is non-linear but
        // we use the nominal turns ratio for the harmonic impedance model)
        let tap = branch.tap_complex();
        let tap_conj = tap.conj();
        let tap_sq = tap.norm_sqr();

        // π-model contributions (same structure as fundamental Y-bus)
        let yii = y_series / tap_sq + y_shunt;
        let yjj = y_series + y_shunt;
        let yij = -y_series / tap_conj;
        let yji = -y_series / tap;

        y[i * n + i] += yii;
        y[j * n + j] += yjj;
        y[i * n + j] += yij;
        y[j * n + i] += yji;
    }

    // Bus shunt elements: G_s + j·h·B_s
    for (k, bus) in network.buses.iter().enumerate() {
        if bus.gs != 0.0 || bus.bs != 0.0 {
            let y_shunt = Complex64::new(bus.gs, hf * bus.bs) / network.base_mva;
            y[k * n + k] += y_shunt;
        }
    }

    Ok(y)
}

/// Dense Gaussian elimination with partial pivoting.
/// Solves `A · x = b` where A is given row-major (length n×n) and b is length n.
///
/// Returns the solution vector x.
fn dense_solve(a_flat: &[Complex64], b: &[Complex64], n: usize) -> Result<Vec<Complex64>> {
    // Build augmented matrix [A | b] for in-place elimination
    let mut mat: Vec<Vec<Complex64>> = (0..n)
        .map(|i| {
            let mut row: Vec<Complex64> = a_flat[i * n..(i + 1) * n].to_vec();
            row.push(b[i]);
            row
        })
        .collect();

    for col in 0..n {
        // Partial pivot: find row with max |a[row][col]|
        let pivot_row = (col..n)
            .max_by(|&r1, &r2| {
                mat[r1][col]
                    .norm()
                    .partial_cmp(&mat[r2][col].norm())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(col);

        if mat[pivot_row][col].norm() < 1e-30 {
            return Err(OxiGridError::LinearAlgebra(format!(
                "Harmonic Y-bus is singular at column {col} — check network connectivity"
            )));
        }

        mat.swap(col, pivot_row);

        let pivot = mat[col][col];
        // Scale pivot row (clippy-friendly: iterate with enumerate offset by col)
        let ncols = n + 1;
        {
            let row_copy: Vec<_> = mat[col][col..ncols].to_vec();
            for (ci, v) in row_copy.iter().enumerate() {
                mat[col][col + ci] = *v / pivot;
            }
        }

        // Eliminate column in all other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = mat[row][col];
            if factor.norm() < 1e-30 {
                continue;
            }
            let pivot_row_slice: Vec<_> = mat[col][col..ncols].to_vec();
            for (ci, &pv) in pivot_row_slice.iter().enumerate() {
                mat[row][col + ci] -= factor * pv;
            }
        }
    }

    // Extract solution
    Ok((0..n).map(|i| mat[i][n]).collect())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};
    use std::f64::consts::PI;

    /// Single-branch two-bus network for harmonic testing.
    fn make_two_bus_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));

        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.5,
            qg: 0.0,
            qmax: 100.0,
            qmin: -100.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 100.0,
            pmin: 0.0,
        });

        net
    }

    #[test]
    fn test_harmonic_pf_result_dimensions() {
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(60.0, vec![3, 5, 7]);
        let injections = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        let result = hpf.solve(&net, &injections).expect("should succeed");

        assert_eq!(result.harmonic_voltages.len(), 2, "one entry per bus");
        assert_eq!(
            result.harmonic_voltages[0].len(),
            3,
            "one entry per harmonic"
        );
        assert_eq!(result.thd_per_bus.len(), 2);
        assert_eq!(result.individual_harmonics.len(), 2);
        assert_eq!(result.voltage_harmonic_spectrum.len(), 2);
    }

    #[test]
    fn test_harmonic_pf_zero_injection_zero_voltage() {
        // No injections → all harmonic voltages should be (near) zero.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![3, 5, 7, 11, 13]);
        let result = hpf.solve(&net, &[]).expect("should succeed");

        for (bus, row) in result.harmonic_voltages.iter().enumerate() {
            for (h_idx, v) in row.iter().enumerate() {
                assert!(
                    v.norm() < 1e-10,
                    "bus={bus} h_idx={h_idx}: expected 0 voltage, got |V|={:.3e}",
                    v.norm()
                );
            }
        }
        for (i, &thd) in result.thd_per_bus.iter().enumerate() {
            assert!(
                thd < 1e-8,
                "bus {i}: THD={thd:.4e} should be ~0 with no injections"
            );
        }
    }

    #[test]
    fn test_harmonic_pf_nonzero_injection_nonzero_voltage() {
        // A non-zero 5th harmonic injection must produce non-zero 5th harmonic voltage.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(60.0, vec![5]);
        let injections = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.1,
            current_angle_rad: PI / 6.0,
        }];
        let result = hpf.solve(&net, &injections).expect("should succeed");

        // 5th harmonic voltage at bus 1 should be non-zero
        let v5_bus1 = result.harmonic_voltages[1][0].norm();
        assert!(
            v5_bus1 > 1e-8,
            "5th harmonic voltage at bus 1 should be non-zero, got {v5_bus1:.3e}"
        );

        // THD at bus 1 should be positive
        assert!(
            result.thd_per_bus[1] > 0.0,
            "THD at bus 1 should be positive"
        );
    }

    #[test]
    fn test_harmonic_pf_invalid_bus_index_errors() {
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![3]);
        let injections = vec![HarmonicInjection {
            bus: 99, // invalid
            harmonic_order: 3,
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        assert!(hpf.solve(&net, &injections).is_err());
    }

    #[test]
    fn test_harmonic_pf_higher_order_larger_impedance() {
        // At a higher harmonic order, the inductive reactance is larger, so for
        // the same current injection the voltage response should differ between
        // harmonic 3 and harmonic 13.  (Both should be non-zero; we verify they
        // are different.)
        let net = make_two_bus_network();

        let inj_magnitude = 0.05;
        let inj_angle = 0.0;

        let hpf3 = HarmonicPowerFlow::new(50.0, vec![3]);
        let inj3 = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 3,
            current_magnitude_pu: inj_magnitude,
            current_angle_rad: inj_angle,
        }];
        let res3 = hpf3.solve(&net, &inj3).expect("h=3 solve");

        let hpf13 = HarmonicPowerFlow::new(50.0, vec![13]);
        let inj13 = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 13,
            current_magnitude_pu: inj_magnitude,
            current_angle_rad: inj_angle,
        }];
        let res13 = hpf13.solve(&net, &inj13).expect("h=13 solve");

        let v3 = res3.harmonic_voltages[1][0].norm();
        let v13 = res13.harmonic_voltages[1][0].norm();

        // Different harmonics → different voltage magnitudes (X scales with h)
        let rel_diff = (v3 - v13).abs() / (v3 + v13 + 1e-15);
        assert!(
            rel_diff > 1e-3,
            "3rd and 13th harmonic voltages should differ (v3={v3:.4e}, v13={v13:.4e})"
        );
    }

    // --- 7 new tests ---

    #[test]
    fn test_harmonic_pf_invalid_harmonic_order_errors() {
        // An injection whose harmonic_order is not in the harmonics list must return Err.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![3, 5, 7]);
        let injections = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 9, // not in [3, 5, 7]
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        assert!(
            hpf.solve(&net, &injections).is_err(),
            "harmonic_order 9 not in [3,5,7] must return Err"
        );
    }

    #[test]
    fn test_harmonic_pf_multiple_injections_same_bus_sum() {
        // Two identical injections at the same bus and harmonic should be summed,
        // producing approximately 2x the voltage of a single injection.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![5]);

        let single_inj = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        let res_single = hpf
            .solve(&net, &single_inj)
            .expect("single injection solve");

        let double_inj = vec![
            HarmonicInjection {
                bus: 1,
                harmonic_order: 5,
                current_magnitude_pu: 0.05,
                current_angle_rad: 0.0,
            },
            HarmonicInjection {
                bus: 1,
                harmonic_order: 5,
                current_magnitude_pu: 0.05,
                current_angle_rad: 0.0,
            },
        ];
        let res_double = hpf
            .solve(&net, &double_inj)
            .expect("double injection solve");

        let v_single = res_single.harmonic_voltages[1][0].norm();
        let v_double = res_double.harmonic_voltages[1][0].norm();

        assert!(
            v_single > 1e-10,
            "single injection must produce non-zero voltage, got {v_single:.3e}"
        );
        let ratio = v_double / v_single;
        assert!(
            (ratio - 2.0).abs() < 0.01,
            "two identical injections at the same bus should double the voltage (ratio={ratio:.6}, expected 2.0)"
        );
    }

    #[test]
    fn test_individual_harmonics_equals_100x_spectrum() {
        // individual_harmonics[bus][h_idx] must equal 100 * voltage_harmonic_spectrum[bus][h_idx]
        // for every bus and every harmonic index.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(60.0, vec![5, 7]);
        let injections = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.08,
            current_angle_rad: 0.0,
        }];
        let result = hpf.solve(&net, &injections).expect("solve should succeed");

        for (bus, (ind_row, spec_row)) in result
            .individual_harmonics
            .iter()
            .zip(result.voltage_harmonic_spectrum.iter())
            .enumerate()
        {
            for (h_idx, (&ind, &spec)) in ind_row.iter().zip(spec_row.iter()).enumerate() {
                let expected = 100.0 * spec;
                assert!(
                    (ind - expected).abs() < 1e-10,
                    "bus={bus} h_idx={h_idx}: individual_harmonics={ind:.10} != 100*spectrum={expected:.10}"
                );
            }
        }
    }

    #[test]
    fn test_harmonic_pf_non_standard_frequency_succeeds() {
        // 400 Hz fundamental is unusual but the solver must not reject any positive f64.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(400.0, vec![3, 5]);
        let injections = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 3,
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        let result = hpf
            .solve(&net, &injections)
            .expect("400 Hz solve should succeed");

        assert_eq!(result.harmonic_voltages.len(), 2, "two buses");
        assert_eq!(
            result.harmonic_voltages[0].len(),
            2,
            "two harmonics in list"
        );
    }

    #[test]
    fn test_thd_non_negative() {
        // THD must be >= 0.0 for every bus even when there are no injections.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![3, 5, 7]);
        let result = hpf
            .solve(&net, &[])
            .expect("zero-injection solve should succeed");

        for (i, &thd) in result.thd_per_bus.iter().enumerate() {
            assert!(
                thd >= 0.0,
                "bus {i}: thd_per_bus must be non-negative, got {thd}"
            );
        }
    }

    #[test]
    fn test_injections_at_different_buses_both_produce_voltage() {
        // Injecting at bus 0 (h=3) and bus 1 (h=7) should both produce non-zero
        // harmonic voltages at the respective buses for the respective harmonics.
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![3, 7]);
        let injections = vec![
            HarmonicInjection {
                bus: 0,
                harmonic_order: 3,
                current_magnitude_pu: 0.05,
                current_angle_rad: 0.0,
            },
            HarmonicInjection {
                bus: 1,
                harmonic_order: 7,
                current_magnitude_pu: 0.05,
                current_angle_rad: PI / 4.0,
            },
        ];
        let result = hpf
            .solve(&net, &injections)
            .expect("two-bus two-harmonic solve");

        // h_idx=0 corresponds to harmonic order 3, injected at bus 0
        let v_bus0_h3 = result.harmonic_voltages[0][0].norm();
        assert!(
            v_bus0_h3 > 1e-10,
            "bus 0, h=3: expected non-zero voltage, got {v_bus0_h3:.3e}"
        );

        // h_idx=1 corresponds to harmonic order 7, injected at bus 1
        let v_bus1_h7 = result.harmonic_voltages[1][1].norm();
        assert!(
            v_bus1_h7 > 1e-10,
            "bus 1, h=7: expected non-zero voltage, got {v_bus1_h7:.3e}"
        );
    }

    #[test]
    fn test_harmonic_voltage_scales_linearly_with_current() {
        // The system is linear: doubling the injection current must double the
        // resulting harmonic voltage magnitude (within 1% relative tolerance).
        let net = make_two_bus_network();
        let hpf = HarmonicPowerFlow::new(50.0, vec![5]);

        let inj_single = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.05,
            current_angle_rad: 0.0,
        }];
        let res_single = hpf
            .solve(&net, &inj_single)
            .expect("single-magnitude solve");

        let inj_double = vec![HarmonicInjection {
            bus: 1,
            harmonic_order: 5,
            current_magnitude_pu: 0.10,
            current_angle_rad: 0.0,
        }];
        let res_double = hpf
            .solve(&net, &inj_double)
            .expect("double-magnitude solve");

        let v_single = res_single.harmonic_voltages[1][0].norm();
        let v_double = res_double.harmonic_voltages[1][0].norm();

        assert!(
            v_single > 1e-10,
            "single magnitude must produce non-zero voltage, got {v_single:.3e}"
        );
        let ratio = v_double / v_single;
        assert!(
            (ratio - 2.0).abs() < 0.01,
            "doubling injection current must double harmonic voltage (ratio={ratio:.6}, expected 2.0)"
        );
    }
}
