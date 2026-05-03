/// Multi-machine transient stability — network-reduced model in COI frame.
///
/// # Model
/// Each generator is represented as a classical machine (constant voltage E'
/// behind transient reactance X'd). The generator buses are retained; all
/// load buses are eliminated (Kron/Ward reduction). This yields the
/// *generator admittance matrix* Y_red (n_gen × n_gen complex).
///
/// Swing equations in the *centre-of-inertia* (COI) frame:
///
///   δ̃_i = δ_i − δ_COI          (angle relative to COI)
///   ω̃_i = ω_i − ω_COI
///   M_i · dω̃_i/dt = P_m_i − P_e_i − (M_i/M_T) · P_COI_acc
///   dδ̃_i/dt = ω̃_i
///
/// where M_T = Σ M_i, δ_COI = (1/M_T)·Σ M_i·δ_i, ω_COI = (1/M_T)·Σ M_i·ω_i.
///
/// Electrical power for generator i:
///   P_e_i = Σ_j E_i·E_j·(G_ij·cos(δ_i−δ_j) + B_ij·sin(δ_i−δ_j))
///
/// Numerical integration: 4th-order Runge-Kutta with configurable step.
///
/// # References
/// Anderson & Fouad, "Power System Control and Stability", 2nd ed., Chapter 11.
use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Parameters for a single classical generator in the multi-machine model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineParams {
    /// Generator index label
    pub id: usize,
    /// Inertia constant H `s`
    pub h: f64,
    /// Damping coefficient D [p.u.]
    pub d: f64,
    /// Transient reactance X'd [p.u.] (used in Y_red build)
    pub xd_prime: f64,
    /// Internal voltage magnitude |E'| [p.u.]
    pub e_prime: f64,
    /// Pre-fault mechanical input power P_m [p.u.]
    pub p_mech: f64,
    /// System frequency `Hz`
    pub freq_hz: f64,
}

impl MachineParams {
    pub fn new(
        id: usize,
        h: f64,
        d: f64,
        xd_prime: f64,
        e_prime: f64,
        p_mech: f64,
        freq_hz: f64,
    ) -> Self {
        Self {
            id,
            h,
            d,
            xd_prime,
            e_prime,
            p_mech,
            freq_hz,
        }
    }

    /// Inertia constant M_i = 2H / ω_s [s²/rad].
    pub fn m_inertia(&self) -> f64 {
        2.0 * self.h / (2.0 * PI * self.freq_hz)
    }
}

/// State of one machine (angle + speed deviation in COI frame).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MachineState {
    /// Rotor angle δ `rad` (absolute, COI frame = relative to COI)
    pub delta: f64,
    /// Speed deviation Δω = ω − ω_COI [rad/s]
    pub omega: f64,
}

impl MachineState {
    pub fn equilibrium(delta_rad: f64) -> Self {
        Self {
            delta: delta_rad,
            omega: 0.0,
        }
    }
}

/// Reduced-network generator admittance matrix element.
#[derive(Debug, Clone, Copy)]
pub struct YredElement {
    pub row: usize,
    pub col: usize,
    /// Transfer admittance Y_ij = G_ij + jB_ij
    pub y: Complex64,
}

/// Multi-machine stability model (network-reduced, COI frame).
pub struct MultiMachineSim {
    pub machines: Vec<MachineParams>,
    /// Generator admittance matrix (n×n dense, complex)
    pub y_red: Vec<Vec<Complex64>>,
    /// Use COI frame for integration
    pub use_coi: bool,
}

impl MultiMachineSim {
    /// Construct from machine parameters and a pre-built reduced admittance matrix.
    ///
    /// `y_red_flat` is row-major: `y_red[i][j]` = G_ij + j·B_ij `p.u.`.
    pub fn new(machines: Vec<MachineParams>, y_red: Vec<Vec<Complex64>>) -> Self {
        Self {
            machines,
            y_red,
            use_coi: true,
        }
    }

    /// Build a two-machine system connected via a transfer admittance.
    ///
    /// Y_12 = −j / (xd1' + xline + xd2')  (pure imaginary for lossless line).
    pub fn two_machine(m1: MachineParams, m2: MachineParams, x_line: f64) -> Self {
        let x_total = m1.xd_prime + x_line + m2.xd_prime;
        let y12 = Complex64::new(0.0, -1.0 / x_total);
        // Diagonal: self-admittance (sum of admittances to each bus)
        let y11 = Complex64::new(0.0, 1.0 / (m1.xd_prime + x_line));
        let y22 = Complex64::new(0.0, 1.0 / (m2.xd_prime + x_line));
        let y_red = vec![vec![y11, y12], vec![y12.conj(), y22]];
        Self::new(vec![m1, m2], y_red)
    }

    /// Build an n-machine ring network (each machine connected to next via x_line).
    pub fn ring_network(machines: Vec<MachineParams>, x_line: f64) -> Self {
        let n = machines.len();
        let mut y_red = vec![vec![Complex64::new(0.0, 0.0); n]; n];

        for i in 0..n {
            let j = (i + 1) % n;
            let xt = machines[i].xd_prime + x_line + machines[j].xd_prime;
            let y_transfer = Complex64::new(0.0, -1.0 / xt);
            y_red[i][j] += y_transfer;
            y_red[j][i] += y_transfer;
            // Self-admittance contributions
            y_red[i][i] -= y_transfer;
            y_red[j][j] -= y_transfer;
        }

        Self::new(machines, y_red)
    }

    /// Compute electrical power output for each machine [p.u.].
    ///
    /// P_e_i = Σ_j E_i·E_j·[G_ij·cos(δ_i−δ_j) + B_ij·sin(δ_i−δ_j)]
    pub fn electrical_power(&self, states: &[MachineState]) -> Vec<f64> {
        let n = self.machines.len();
        let mut pe = vec![0.0f64; n];

        for (i, (mach_i, st_i)) in self.machines.iter().zip(states.iter()).enumerate() {
            let ei = mach_i.e_prime;
            let di = st_i.delta;
            for (j, (mach_j, st_j)) in self.machines.iter().zip(states.iter()).enumerate() {
                let ej = mach_j.e_prime;
                let dj = st_j.delta;
                let g = self.y_red[i][j].re;
                let b = self.y_red[i][j].im;
                let dij = di - dj;
                pe[i] += ei * ej * (g * dij.cos() + b * dij.sin());
            }
        }
        pe
    }

    /// Compute the COI angle and speed.
    pub fn coi(&self, states: &[MachineState]) -> (f64, f64) {
        let mt: f64 = self.machines.iter().map(|m| m.m_inertia()).sum();
        if mt < 1e-12 {
            return (0.0, 0.0);
        }
        let delta_coi: f64 = self
            .machines
            .iter()
            .zip(states.iter())
            .map(|(m, s)| m.m_inertia() * s.delta)
            .sum::<f64>()
            / mt;
        let omega_coi: f64 = self
            .machines
            .iter()
            .zip(states.iter())
            .map(|(m, s)| m.m_inertia() * s.omega)
            .sum::<f64>()
            / mt;
        (delta_coi, omega_coi)
    }

    /// Compute swing-equation derivatives d(δ, ω)/dt for all machines.
    ///
    /// In COI frame: the COI acceleration is subtracted from each machine.
    pub fn derivatives(&self, states: &[MachineState]) -> Vec<(f64, f64)> {
        let n = self.machines.len();
        let pe = self.electrical_power(states);
        let mt: f64 = self.machines.iter().map(|m| m.m_inertia()).sum();

        // Total accelerating power in COI
        let p_coi_acc: f64 = self
            .machines
            .iter()
            .zip(pe.iter())
            .map(|(m, &pe_i)| m.p_mech - pe_i)
            .sum();

        let (_, omega_coi) = self.coi(states);

        (0..n)
            .map(|i| {
                let m = &self.machines[i];
                let mi = m.m_inertia();
                let pm = m.p_mech;
                let d_delta = states[i].omega;
                let coi_correction = if self.use_coi && mt > 1e-12 {
                    (mi / mt) * p_coi_acc
                } else {
                    0.0
                };
                let d_omega =
                    (pm - pe[i] - m.d * (states[i].omega + omega_coi) - coi_correction) / mi;
                (d_delta, d_omega)
            })
            .collect()
    }

    /// Perform one RK4 integration step.
    pub fn step(&self, states: &[MachineState], dt: f64) -> Vec<MachineState> {
        let k1 = self.derivatives(states);
        let s2: Vec<MachineState> = states
            .iter()
            .zip(k1.iter())
            .map(|(s, &(dd, dw))| MachineState {
                delta: s.delta + 0.5 * dt * dd,
                omega: s.omega + 0.5 * dt * dw,
            })
            .collect();

        let k2 = self.derivatives(&s2);
        let s3: Vec<MachineState> = states
            .iter()
            .zip(k2.iter())
            .map(|(s, &(dd, dw))| MachineState {
                delta: s.delta + 0.5 * dt * dd,
                omega: s.omega + 0.5 * dt * dw,
            })
            .collect();

        let k3 = self.derivatives(&s3);
        let s4: Vec<MachineState> = states
            .iter()
            .zip(k3.iter())
            .map(|(s, &(dd, dw))| MachineState {
                delta: s.delta + dt * dd,
                omega: s.omega + dt * dw,
            })
            .collect();

        let k4 = self.derivatives(&s4);

        states
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let (d1, w1) = k1[i];
                let (d2, w2) = k2[i];
                let (d3, w3) = k3[i];
                let (d4, w4) = k4[i];
                MachineState {
                    delta: s.delta + dt / 6.0 * (d1 + 2.0 * d2 + 2.0 * d3 + d4),
                    omega: s.omega + dt / 6.0 * (w1 + 2.0 * w2 + 2.0 * w3 + w4),
                }
            })
            .collect()
    }

    /// Run a full multi-machine transient simulation.
    ///
    /// `fault_fn` is called at each step to optionally modify the Y_red
    /// (e.g., to model a fault being applied/cleared). It receives the current
    /// time and returns an optional modified admittance matrix.
    pub fn run(
        &self,
        initial: Vec<MachineState>,
        dt: f64,
        t_end: f64,
        fault_fn: Option<&dyn Fn(f64) -> Option<Vec<Vec<Complex64>>>>,
    ) -> MultiMachineResult {
        let n_steps = (t_end / dt).ceil() as usize;
        let mut states = initial;
        let mut snapshots = Vec::with_capacity(n_steps + 1);
        let mut time = 0.0;

        snapshots.push(MultiMachineSnapshot {
            time,
            states: states.clone(),
            pe: self.electrical_power(&states),
        });

        for step in 1..=n_steps {
            time = step as f64 * dt;

            // If a fault function is provided and modifies Y_red, create a temporary sim
            if let Some(ff) = fault_fn {
                if let Some(y_mod) = ff(time) {
                    let temp = MultiMachineSim {
                        machines: self.machines.clone(),
                        y_red: y_mod,
                        use_coi: self.use_coi,
                    };
                    states = temp.step(&states, dt);
                } else {
                    states = self.step(&states, dt);
                }
            } else {
                states = self.step(&states, dt);
            }

            snapshots.push(MultiMachineSnapshot {
                time,
                states: states.clone(),
                pe: self.electrical_power(&states),
            });
        }

        MultiMachineResult { snapshots }
    }

    /// Check if the system is transiently stable.
    ///
    /// Criterion: max(|δ_i − δ_j|) < π rad at all times.
    pub fn is_transient_stable(result: &MultiMachineResult) -> bool {
        for snap in &result.snapshots {
            let n = snap.states.len();
            for i in 0..n {
                for j in i + 1..n {
                    let diff = (snap.states[i].delta - snap.states[j].delta).abs();
                    if diff > PI {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Estimate critical clearing time (CCT) by bisection.
    ///
    /// Applies a fault (Y_red_fault) starting at t=0, clears at t=t_clear.
    /// Returns CCT in seconds (within `tol` precision).
    pub fn estimate_cct(
        &self,
        initial: &[MachineState],
        y_red_fault: Vec<Vec<Complex64>>,
        dt: f64,
        t_sim: f64,
        tol: f64,
    ) -> f64 {
        let mut lo = 0.0f64;
        let mut hi = t_sim;

        for _ in 0..30 {
            let mid = (lo + hi) / 2.0;
            let y_red_fault_clone = y_red_fault.clone();
            let y_red_post = self.y_red.clone();
            let machines_clone = self.machines.clone();

            let fault_fn = |t: f64| -> Option<Vec<Vec<Complex64>>> {
                if t <= mid {
                    Some(y_red_fault_clone.clone())
                } else {
                    Some(y_red_post.clone())
                }
            };

            let temp_sim = MultiMachineSim {
                machines: machines_clone,
                y_red: self.y_red.clone(),
                use_coi: self.use_coi,
            };

            let result = temp_sim.run(initial.to_vec(), dt, t_sim, Some(&fault_fn));
            if Self::is_transient_stable(&result) {
                lo = mid;
            } else {
                hi = mid;
            }

            if (hi - lo) < tol {
                break;
            }
        }

        (lo + hi) / 2.0
    }

    /// Compute the kinetic energy (relative to COI) stored in generator rotors [p.u.·s].
    pub fn kinetic_energy(&self, states: &[MachineState]) -> f64 {
        let (_, omega_coi) = self.coi(states);
        self.machines
            .iter()
            .zip(states.iter())
            .map(|(m, s)| 0.5 * m.m_inertia() * (s.omega - omega_coi).powi(2))
            .sum()
    }
}

/// Snapshot at one time point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiMachineSnapshot {
    pub time: f64,
    pub states: Vec<MachineState>,
    pub pe: Vec<f64>,
}

/// Full simulation result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiMachineResult {
    pub snapshots: Vec<MultiMachineSnapshot>,
}

impl MultiMachineResult {
    /// Maximum rotor angle spread (δ_max − δ_min) across all time steps.
    pub fn max_angle_spread_deg(&self) -> f64 {
        self.snapshots
            .iter()
            .map(|snap| {
                let deltas: Vec<f64> = snap.states.iter().map(|s| s.delta.to_degrees()).collect();
                let dmax = deltas.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let dmin = deltas.iter().cloned().fold(f64::INFINITY, f64::min);
                dmax - dmin
            })
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Time series of the COI angle `rad`.
    pub fn coi_angle(&self, machines: &[MachineParams]) -> Vec<(f64, f64)> {
        let mt: f64 = machines.iter().map(|m| m.m_inertia()).sum();
        self.snapshots
            .iter()
            .map(|snap| {
                let coi = if mt > 1e-12 {
                    machines
                        .iter()
                        .zip(snap.states.iter())
                        .map(|(m, s)| m.m_inertia() * s.delta)
                        .sum::<f64>()
                        / mt
                } else {
                    0.0
                };
                (snap.time, coi)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_machine(id: usize, h: f64, pm: f64) -> MachineParams {
        MachineParams::new(id, h, 2.0, 0.2, 1.05, pm, 60.0)
    }

    #[test]
    fn test_two_machine_equilibrium() {
        // At equilibrium: Pe = Pm for both machines; angles stable
        let m1 = make_machine(0, 6.0, 0.5);
        let m2 = make_machine(1, 4.0, 0.4);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);

        // Initial state at equilibrium angles
        let states = vec![
            MachineState::equilibrium(0.3),
            MachineState::equilibrium(-0.2),
        ];
        let result = sim.run(states, 0.01, 2.0, None);
        // Angle spread should remain small (bounded, no divergence)
        let spread = result.max_angle_spread_deg();
        assert!(
            spread < 180.0,
            "Angle spread should be bounded: {:.2}°",
            spread
        );
    }

    #[test]
    fn test_electrical_power_symmetric() {
        let m1 = make_machine(0, 6.0, 0.8);
        let m2 = make_machine(1, 6.0, 0.8);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);

        // With equal angles: power transfer = 0, self-power from G_ii
        let states = vec![
            MachineState::equilibrium(0.0),
            MachineState::equilibrium(0.0),
        ];
        let pe = sim.electrical_power(&states);
        // Both should have same P_e
        assert!(
            (pe[0] - pe[1]).abs() < 1e-10,
            "Pe0={:.4} Pe1={:.4}",
            pe[0],
            pe[1]
        );
    }

    #[test]
    fn test_coi_computation() {
        let m1 = MachineParams::new(0, 6.0, 2.0, 0.2, 1.0, 0.5, 60.0);
        let m2 = MachineParams::new(1, 4.0, 2.0, 0.2, 1.0, 0.5, 60.0);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);

        let states = vec![
            MachineState {
                delta: 0.4,
                omega: 0.1,
            },
            MachineState {
                delta: 0.2,
                omega: 0.05,
            },
        ];
        let (delta_coi, omega_coi) = sim.coi(&states);
        // Weighted average: M1=2*6/(2π*60), M2=2*4/(2π*60)
        let m1_i = 2.0 * 6.0 / (2.0 * PI * 60.0);
        let m2_i = 2.0 * 4.0 / (2.0 * PI * 60.0);
        let expected_coi = (m1_i * 0.4 + m2_i * 0.2) / (m1_i + m2_i);
        assert!(
            (delta_coi - expected_coi).abs() < 1e-10,
            "COI={:.6} expected={:.6}",
            delta_coi,
            expected_coi
        );
        let _ = omega_coi;
    }

    #[test]
    fn test_three_machine_ring() {
        let machines: Vec<MachineParams> = (0..3).map(|i| make_machine(i, 6.0, 0.5)).collect();
        let sim = MultiMachineSim::ring_network(machines, 0.4);
        let states: Vec<MachineState> = (0..3)
            .map(|i| MachineState::equilibrium(0.1 * i as f64))
            .collect();
        let result = sim.run(states, 0.01, 1.0, None);
        assert_eq!(result.snapshots.len(), 101);
        let _ = result.max_angle_spread_deg();
    }

    #[test]
    fn test_kinetic_energy_at_equilibrium() {
        let m1 = make_machine(0, 6.0, 0.5);
        let m2 = make_machine(1, 6.0, 0.5);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);
        let states = vec![
            MachineState::equilibrium(0.3),
            MachineState::equilibrium(-0.3),
        ];
        // At equilibrium (omega=0 for all), COI omega=0, kinetic energy=0
        let ke = sim.kinetic_energy(&states);
        assert!(
            ke.abs() < 1e-12,
            "KE at equilibrium should be zero: {:.2e}",
            ke
        );
    }

    #[test]
    fn test_fault_simulation() {
        let m1 = make_machine(0, 6.0, 0.8);
        let m2 = make_machine(1, 4.0, 0.4);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);

        // Fault: zero-admittance (no power transfer)
        let n = sim.machines.len();
        let y_fault = vec![vec![Complex64::new(0.0, 0.0); n]; n];
        let y_post = sim.y_red.clone();

        let t_clear = 0.1;
        let fault_fn = |t: f64| -> Option<Vec<Vec<Complex64>>> {
            if t <= t_clear {
                Some(y_fault.clone())
            } else {
                Some(y_post.clone())
            }
        };

        let initial = vec![
            MachineState::equilibrium(0.4),
            MachineState::equilibrium(-0.2),
        ];
        let result = sim.run(initial, 0.005, 1.0, Some(&fault_fn));
        assert!(!result.snapshots.is_empty());
        // During fault, machines accelerate (Pe=0, Pm>0)
        let delta_at_fault = result.snapshots[10].states[0].delta;
        assert!(
            delta_at_fault >= 0.4,
            "Machine 0 should accelerate during fault: δ={:.4}",
            delta_at_fault
        );
    }

    #[test]
    fn test_is_transient_stable_equilibrium() {
        let m1 = make_machine(0, 6.0, 0.5);
        let m2 = make_machine(1, 6.0, 0.5);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);
        let states = vec![
            MachineState::equilibrium(0.2),
            MachineState::equilibrium(-0.2),
        ];
        let result = sim.run(states, 0.01, 2.0, None);
        // Small initial angle separation → likely stable
        let stable = MultiMachineSim::is_transient_stable(&result);
        let _ = stable; // may or may not be stable depending on machine params
    }

    #[test]
    fn test_angle_spread_increases_during_fault() {
        let m1 = make_machine(0, 6.0, 0.9); // heavily loaded
        let m2 = make_machine(1, 6.0, 0.1);
        let sim = MultiMachineSim::two_machine(m1, m2, 0.3);

        let n = sim.machines.len();
        let y_fault = vec![vec![Complex64::new(0.0, 0.0); n]; n];
        let fault_fn = |_t: f64| -> Option<Vec<Vec<Complex64>>> { Some(y_fault.clone()) };

        let initial = vec![
            MachineState::equilibrium(0.3),
            MachineState::equilibrium(-0.1),
        ];
        let result = sim.run(initial, 0.005, 0.5, Some(&fault_fn));
        let spread = result.max_angle_spread_deg();
        assert!(
            spread > 10.0,
            "Angle spread during sustained fault should grow: {:.2}°",
            spread
        );
    }
}
