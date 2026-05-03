/// Transient stability analysis — swing equation solver.
///
/// Models each synchronous generator as a classical machine (constant
/// voltage E' behind transient reactance X'd).
///
/// The swing equation for generator i:
///
///   M_i · d²δ_i/dt² = P_m_i − P_e_i − D_i · dω_i/dt
///
/// where:
///   M_i  = 2H_i / ω_s     [s²/rad]  (inertia constant)
///   H_i  = inertia constant `s`
///   δ_i  = rotor angle `rad`
///   ω_i  = rotor speed deviation [rad/s]
///   P_m_i = mechanical input power [p.u.]
///   P_e_i = electrical output power [p.u.]
///   D_i  = damping coefficient [p.u.]
///   ω_s  = synchronous speed = 2π·f₀ [rad/s]
///
/// Numerical integration uses the 4th-order Runge-Kutta method.
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Synchronous generator parameters for classical model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClassicalGen {
    /// Inertia constant H `s`
    pub h: f64,
    /// Damping coefficient D [p.u.]
    pub d: f64,
    /// Transient reactance X'd [p.u.]
    pub xd_prime: f64,
    /// Internal voltage magnitude |E'| [p.u.]
    pub e_prime: f64,
    /// Mechanical input power P_m [p.u.] (assumed constant)
    pub p_mech: f64,
}

impl ClassicalGen {
    /// A typical thermal unit (60 Hz).
    pub fn thermal_unit() -> Self {
        Self {
            h: 6.0,
            d: 2.0,
            xd_prime: 0.20,
            e_prime: 1.05,
            p_mech: 0.8,
        }
    }

    /// A typical hydro unit.
    pub fn hydro_unit() -> Self {
        Self {
            h: 3.0,
            d: 1.0,
            xd_prime: 0.30,
            e_prime: 1.02,
            p_mech: 0.6,
        }
    }

    /// Inertia constant M = 2H/ω_s [s²/rad].
    pub fn m(&self, freq_hz: f64) -> f64 {
        2.0 * self.h / (2.0 * PI * freq_hz)
    }
}

/// State of a single generator during transient simulation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GenState {
    /// Rotor angle `rad`
    pub delta: f64,
    /// Speed deviation dω = ω − ω_s [rad/s]
    pub omega: f64,
}

impl GenState {
    pub fn new(delta_rad: f64) -> Self {
        Self {
            delta: delta_rad,
            omega: 0.0,
        }
    }
}

/// Transient simulation event (e.g. fault on/off).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TransientEvent {
    /// Apply a three-phase fault at a bus.
    FaultOn {
        /// Time of fault application `s`.
        time: f64,
        /// Bus index (0-based) where fault is applied.
        bus: usize,
        /// Fault impedance [p.u.] (0.0 = bolted fault).
        fault_impedance: f64,
    },
    /// Clear (remove) a fault at a bus.
    FaultOff {
        /// Time of fault clearing `s`.
        time: f64,
        /// Bus index (0-based) where fault is cleared.
        bus: usize,
    },
}

/// Configuration for a transient stability simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransientConfig {
    /// Simulation end time `s`.
    pub t_end: f64,
    /// Time step `s`.
    pub dt: f64,
    /// System frequency `Hz`.
    pub freq_hz: f64,
    /// Scheduled events (faults, etc.).
    pub events: Vec<TransientEvent>,
}

impl Default for TransientConfig {
    fn default() -> Self {
        Self {
            t_end: 5.0,
            dt: 0.01,
            freq_hz: 60.0,
            events: Vec::new(),
        }
    }
}

/// Snapshot at one time step of a transient simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransientSnapshot {
    pub time: f64,
    pub gen_states: Vec<GenState>,
}

/// Network interface: compute P_e_i for each generator given their states.
///
/// For a 2-machine system (infinite bus), Pe = E'*V_inf/Xtot * sin(δ).
/// For multi-machine: Pe_i = Σ_j |E_i||E_j|*|Y_ij|*sin(δ_i - δ_j - α_ij).
pub type PeFunction = Box<dyn Fn(&[GenState]) -> Vec<f64>>;

/// Transient stability simulator.
pub struct TransientSim {
    pub generators: Vec<ClassicalGen>,
    pub freq_hz: f64,
    /// Computes electrical power for all generators
    pe_fn: PeFunction,
}

impl TransientSim {
    pub fn new(generators: Vec<ClassicalGen>, freq_hz: f64, pe_fn: PeFunction) -> Self {
        Self {
            generators,
            freq_hz,
            pe_fn,
        }
    }

    /// Create a single-machine-infinite-bus (SMIB) simulation.
    ///
    /// - `gen`   — generator parameters
    /// - `v_inf` — infinite bus voltage [p.u.]
    /// - `x_tot` — total reactance between E' and infinite bus [p.u.]
    pub fn smib(gen: ClassicalGen, v_inf: f64, x_tot: f64) -> Self {
        let e_prime = gen.e_prime;
        let pe_fn: PeFunction = Box::new(move |states: &[GenState]| {
            let delta = states[0].delta;
            vec![e_prime * v_inf / x_tot * delta.sin()]
        });
        Self::new(vec![gen], 60.0, pe_fn)
    }

    /// Compute the swing equation derivatives: [dδ/dt, dω/dt] for each generator.
    fn derivatives(&self, states: &[GenState]) -> Vec<(f64, f64)> {
        let pe = (self.pe_fn)(states);
        self.generators
            .iter()
            .zip(states.iter())
            .zip(pe.iter())
            .map(|((gen, st), &pe_i)| {
                let m = gen.m(self.freq_hz);
                let ddelta = st.omega;
                let domega = (gen.p_mech - pe_i - gen.d * st.omega) / m;
                (ddelta, domega)
            })
            .collect()
    }

    /// Advance the simulation by one step using 4th-order Runge-Kutta.
    pub fn step(&self, states: &[GenState], dt: f64) -> Vec<GenState> {
        let k1 = self.derivatives(states);
        let s2: Vec<GenState> = states
            .iter()
            .zip(k1.iter())
            .map(|(s, &(dd, dw))| GenState {
                delta: s.delta + dt / 2.0 * dd,
                omega: s.omega + dt / 2.0 * dw,
            })
            .collect();

        let k2 = self.derivatives(&s2);
        let s3: Vec<GenState> = states
            .iter()
            .zip(k2.iter())
            .map(|(s, &(dd, dw))| GenState {
                delta: s.delta + dt / 2.0 * dd,
                omega: s.omega + dt / 2.0 * dw,
            })
            .collect();

        let k3 = self.derivatives(&s3);
        let s4: Vec<GenState> = states
            .iter()
            .zip(k3.iter())
            .map(|(s, &(dd, dw))| GenState {
                delta: s.delta + dt * dd,
                omega: s.omega + dt * dw,
            })
            .collect();

        let k4 = self.derivatives(&s4);

        states
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let (dd1, dw1) = k1[i];
                let (dd2, dw2) = k2[i];
                let (dd3, dw3) = k3[i];
                let (dd4, dw4) = k4[i];
                GenState {
                    delta: s.delta + dt / 6.0 * (dd1 + 2.0 * dd2 + 2.0 * dd3 + dd4),
                    omega: s.omega + dt / 6.0 * (dw1 + 2.0 * dw2 + 2.0 * dw3 + dw4),
                }
            })
            .collect()
    }

    /// Run a full simulation for `t_end` seconds starting from `initial`.
    /// Returns snapshots at every `dt` step.
    pub fn run(&self, initial: Vec<GenState>, dt: f64, t_end: f64) -> Vec<TransientSnapshot> {
        let n_steps = (t_end / dt).ceil() as usize;
        let mut states = initial;
        let mut snapshots = Vec::with_capacity(n_steps + 1);

        snapshots.push(TransientSnapshot {
            time: 0.0,
            gen_states: states.clone(),
        });

        for k in 1..=n_steps {
            states = self.step(&states, dt);
            snapshots.push(TransientSnapshot {
                time: k as f64 * dt,
                gen_states: states.clone(),
            });
        }

        snapshots
    }
}

/// Small-signal stability: compute the A matrix linearised around the operating point.
///
/// For the classical model, the state is [Δδ, Δω].
/// A = [[0, 1], [−K_s/M, −D/M]]
/// where K_s = ∂P_e/∂δ evaluated at the equilibrium.
///
/// Returns (eigenvalue_real_1, eigenvalue_real_2) or complex pair if oscillatory.
pub fn smib_eigenvalues(gen: &ClassicalGen, pe_grad: f64, freq_hz: f64) -> (f64, f64, f64, f64) {
    // A matrix: [[0,1], [-K/M, -D/M]]
    let m = gen.m(freq_hz);
    let k = pe_grad;
    let d = gen.d;
    // Characteristic equation: λ² + (D/M)λ + K/M = 0
    let a1 = d / m;
    let a0 = k / m;
    let discriminant = a1 * a1 - 4.0 * a0;
    if discriminant >= 0.0 {
        let r1 = (-a1 + discriminant.sqrt()) / 2.0;
        let r2 = (-a1 - discriminant.sqrt()) / 2.0;
        (r1, 0.0, r2, 0.0)
    } else {
        let real = -a1 / 2.0;
        let imag = (-discriminant).sqrt() / 2.0;
        (real, imag, real, -imag)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(dead_code)]
    fn make_smib(delta_init_deg: f64, _fault_pe: f64) -> (TransientSim, Vec<GenState>) {
        let gen = ClassicalGen::thermal_unit();
        let v_inf = 1.0;
        let x_tot = gen.xd_prime + 0.3;
        let sim = TransientSim::smib(gen, v_inf, x_tot);
        let initial = vec![GenState::new(delta_init_deg.to_radians())];
        (sim, initial)
    }

    #[test]
    fn test_stable_smib_returns_to_equilibrium() {
        // Start slightly above equilibrium angle → should oscillate but stay stable
        let gen = ClassicalGen {
            h: 6.0,
            d: 5.0,
            ..ClassicalGen::thermal_unit()
        };
        let v_inf = 1.0;
        let x_tot = gen.xd_prime + 0.3;
        let e = gen.e_prime;
        let sim = TransientSim::smib(gen, v_inf, x_tot);

        // Equilibrium: Pe = Pm → delta_eq = asin(Pm * X / (E*V))
        let pm = sim.generators[0].p_mech;
        let delta_eq = (pm * x_tot / (e * v_inf)).asin();
        let initial = vec![GenState::new(delta_eq + 5.0_f64.to_radians())];

        let snaps = sim.run(initial, 0.01, 5.0);
        let last = snaps.last().unwrap();
        // With D=5, should converge close to equilibrium
        let delta_final = last.gen_states[0].delta;
        assert!(
            (delta_final - delta_eq).abs() < 0.3,
            "δ_final={:.4}° equilib={:.4}°",
            delta_final.to_degrees(),
            delta_eq.to_degrees()
        );
    }

    #[test]
    fn test_rotor_angle_increases_after_fault() {
        // During a fault Pe=0 → Pm > 0 → rotor accelerates → δ increases
        let gen = ClassicalGen::thermal_unit();
        let pe_fn: PeFunction = Box::new(|_states| vec![0.0]); // fault: Pe=0
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(20.0_f64.to_radians())];
        let snaps = sim.run(initial, 0.01, 0.5); // 500ms fault
        let delta0 = snaps[0].gen_states[0].delta;
        let delta_end = snaps.last().unwrap().gen_states[0].delta;
        assert!(
            delta_end > delta0,
            "δ should increase during fault: {:.2}° > {:.2}°",
            delta_end.to_degrees(),
            delta0.to_degrees()
        );
    }

    #[test]
    fn test_smib_eigenvalues_stable() {
        let gen = ClassicalGen::thermal_unit();
        // At equilibrium Pe_grad = E*V*cos(δ)/X > 0
        let pe_grad = 2.0;
        let (r1, i1, r2, _) = smib_eigenvalues(&gen, pe_grad, 60.0);
        // For D>0, K>0: real parts should be negative (stable)
        assert!(r1 <= 0.0, "r1={:.4}", r1);
        assert!(r2 <= 0.0, "r2={:.4}", r2);
        // Should be oscillatory (imaginary part > 0)
        assert!(i1 > 0.0 || r1 != r2, "expect oscillatory or overdamped");
    }

    #[test]
    fn test_rk4_conserves_angle_zero_torque() {
        // If Pm = Pe exactly, dω/dt = 0, ω=0 → δ should stay constant
        let gen = ClassicalGen {
            p_mech: 1.0,
            e_prime: 1.0,
            xd_prime: 0.5,
            h: 6.0,
            d: 0.0,
        };
        let pe_fn: PeFunction = Box::new(|_| vec![1.0]); // Pe = Pm always
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(0.5)];
        let snaps = sim.run(initial.clone(), 0.01, 1.0);
        let delta_final = snaps.last().unwrap().gen_states[0].delta;
        assert!(
            (delta_final - 0.5).abs() < 1e-6,
            "δ changed: {}",
            delta_final
        );
    }

    #[test]
    fn test_inertia_constant_m_is_2h_over_omega_s() {
        let gen = ClassicalGen::thermal_unit();
        let m = gen.m(60.0);
        let expected = 2.0 * 6.0 / (2.0 * PI * 60.0);
        assert!(
            (m - expected).abs() < 1e-10,
            "M = {:.6}, expected {:.6}",
            m,
            expected
        );
    }

    #[test]
    fn test_hydro_unit_has_lower_inertia_than_thermal() {
        let thermal = ClassicalGen::thermal_unit();
        let hydro = ClassicalGen::hydro_unit();
        assert!(
            hydro.h < thermal.h,
            "hydro H={} must be < thermal H={}",
            hydro.h,
            thermal.h
        );
    }

    #[test]
    fn test_smib_eigenvalues_real_parts_negative_for_positive_k() {
        // Standard result: D>0, K>0 → both eigenvalue real parts ≤ 0
        let gen = ClassicalGen::thermal_unit();
        let (r1, _i1, r2, _i2) = smib_eigenvalues(&gen, 2.5, 60.0);
        assert!(r1 <= 0.0, "λ1 real = {:.4}", r1);
        assert!(r2 <= 0.0, "λ2 real = {:.4}", r2);
    }

    #[test]
    fn test_run_snapshot_count_matches_n_steps() {
        let gen = ClassicalGen::thermal_unit();
        let pe_fn: PeFunction = Box::new(|_| vec![0.8]);
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(0.3)];
        let dt = 0.01;
        let t_end = 0.5;
        let snaps = sim.run(initial, dt, t_end);
        // expect ceil(t_end/dt) + 1 = 51 snapshots
        let n_expected = (t_end / dt).ceil() as usize + 1;
        assert_eq!(snaps.len(), n_expected, "got {} snapshots", snaps.len());
    }

    #[test]
    fn test_smib_two_machine_symmetry() {
        // Two identical machines with symmetric Pe function → angles stay equal
        let gen1 = ClassicalGen::thermal_unit();
        let gen2 = ClassicalGen::thermal_unit();
        let pe_fn: PeFunction = Box::new(|states: &[GenState]| {
            // Each machine sees half the synchronising power
            let delta1 = states[0].delta;
            let delta2 = states[1].delta;
            let pe = 0.8 * (delta1 - delta2).sin();
            vec![pe, -pe]
        });
        let sim = TransientSim::new(vec![gen1, gen2], 60.0, pe_fn);
        // Start with identical angles → no torque → angles stay equal
        let initial = vec![GenState::new(0.5), GenState::new(0.5)];
        let snaps = sim.run(initial, 0.01, 0.5);
        let last = snaps.last().expect("should have snapshots");
        let d1 = last.gen_states[0].delta;
        let d2 = last.gen_states[1].delta;
        assert!((d1 - d2).abs() < 1e-8, "Δδ = {:.4e}", (d1 - d2).abs());
    }
}
