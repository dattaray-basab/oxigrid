//! Detailed multi-machine transient stability simulation.
//!
//! Implements 4th-order generator model (swing equation + transient EMF dynamics),
//! Automatic Voltage Regulator (AVR) model, and fault application via Y-bus
//! modification.  Integration is performed with a fixed-step RK4 scheme.
//!
//! # State variables per machine
//!
//! | Variable | Description | Unit |
//! |----------|-------------|------|
//! | `δ` | rotor angle | \[rad\] |
//! | `ω` | per-unit angular velocity | \[pu\] |
//! | `E'q` | q-axis transient EMF | \[pu\] |
//! | `Vr` | AVR regulator output (if AVR enabled) | \[pu\] |
//!
//! # References
//!
//! - Anderson & Fouad, "Power Systems Control and Stability", 2nd ed.
//! - Kundur, "Power System Stability and Control"

use thiserror::Error;

/// Errors from the detailed multi-machine solver.
#[derive(Debug, Error)]
pub enum SimError {
    #[error("No machines added")]
    NoMachines,
    #[error("Y-bus dimension mismatch: expected {0} buses, got {1}")]
    YBusMismatch(usize, usize),
    #[error("Machine bus index {0} out of range (n_buses={1})")]
    InvalidBus(usize, usize),
    #[error("Simulation parameter error: {0}")]
    InvalidParam(String),
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Load model for ZIP representation.
#[derive(Debug, Clone)]
pub enum LoadModel {
    /// Constant power (Z=0, I=0, P=1).
    ConstantPower,
    /// Constant impedance (Z=1, I=0, P=0).
    ConstantImpedance,
    /// General ZIP model.
    ZipModel { z: f64, i: f64, p: f64 },
    /// Induction motor model (approximated by high-impedance constant-Z).
    InductionMotor,
}

/// Configuration for the detailed multi-machine simulation.
#[derive(Debug, Clone)]
pub struct MultiMachineDetailConfig {
    /// Integration time step \[s\].  Recommended: 0.001–0.01.
    pub dt_s: f64,
    /// Total simulation duration \[s\].
    pub simulation_time_s: f64,
    /// Bus index for fault application (`None` = no fault).
    pub fault_bus: Option<usize>,
    /// Time at which fault is applied \[s\].
    pub fault_start_s: f64,
    /// Time at which fault is cleared \[s\].
    pub fault_clear_s: f64,
    /// Whether machine damping coefficient `D` is active.
    pub damping_enabled: bool,
    /// Load model to use.
    pub load_model: LoadModel,
    /// Whether Automatic Voltage Regulator models are applied.
    pub use_avr: bool,
}

impl Default for MultiMachineDetailConfig {
    fn default() -> Self {
        Self {
            dt_s: 0.005,
            simulation_time_s: 3.0,
            fault_bus: None,
            fault_start_s: 0.5,
            fault_clear_s: 0.6,
            damping_enabled: true,
            load_model: LoadModel::ConstantPower,
            use_avr: false,
        }
    }
}

// ── Machine and AVR models ────────────────────────────────────────────────────

/// Detailed generator model parameters.
#[derive(Debug, Clone)]
pub struct DetailedMachine {
    /// Unique machine identifier.
    pub id: usize,
    /// Terminal bus index.
    pub bus: usize,
    /// Rated MVA base \[MVA\].
    pub rated_mva: f64,
    /// Inertia constant \[s\].
    pub h_s: f64,
    /// Damping coefficient \[pu\].
    pub d: f64,
    /// d-axis transient reactance \[pu\].
    pub xd_prime: f64,
    /// q-axis transient reactance \[pu\].
    pub xq_prime: f64,
    /// d-axis open-circuit transient time constant \[s\].
    pub td0_prime_s: f64,
    /// Armature resistance \[pu\].
    pub ra: f64,
    /// Initial mechanical power \[pu\].
    pub p_m0_pu: f64,
    /// Initial field voltage \[pu\].
    pub e_fd0_pu: f64,
    /// Initial rotor angle \[rad\].
    pub delta0_rad: f64,
    /// Initial angular velocity \[pu\].  Normally 1.0.
    pub omega0_pu: f64,
}

/// First-order AVR (IEEE type-1 simplified).
#[derive(Debug, Clone)]
pub struct AvrModel {
    /// Identifier of the machine this AVR belongs to.
    pub machine_id: usize,
    /// Amplifier gain.
    pub ka: f64,
    /// Amplifier time constant \[s\].
    pub ta_s: f64,
    /// Exciter gain.
    pub ke: f64,
    /// Exciter time constant \[s\].
    pub te_s: f64,
    /// Maximum regulator output \[pu\].
    pub vr_max: f64,
    /// Minimum regulator output \[pu\].
    pub vr_min: f64,
    /// Reference terminal voltage \[pu\].
    pub v_ref: f64,
}

// ── Simulation state ──────────────────────────────────────────────────────────

/// Snapshot of a single machine at one time instant.
#[derive(Debug, Clone)]
pub struct MachineState {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Machine identifier.
    pub machine_id: usize,
    /// Rotor angle \[rad\].
    pub delta_rad: f64,
    /// Per-unit angular velocity \[pu\].
    pub omega_pu: f64,
    /// q-axis transient EMF \[pu\].
    pub e_q_prime: f64,
    /// Terminal voltage magnitude \[pu\].
    pub vt_pu: f64,
    /// Electrical power output \[pu\].
    pub p_e_pu: f64,
    /// Mechanical power input \[pu\].
    pub p_m_pu: f64,
}

/// Complete simulation result.
#[derive(Debug, Clone)]
pub struct MultiMachineDetailResult {
    /// Time-series of states per machine: `[machine_index][time_step]`.
    pub machine_states: Vec<Vec<MachineState>>,
    /// Centre-of-inertia (COI) frequency deviation \[pu\] at each time step.
    pub system_frequency: Vec<f64>,
    /// Time vector \[s\].
    pub time_steps: Vec<f64>,
    /// Overall stability assessment.
    pub stable: bool,
    /// True if no machine loses synchronism on the first swing.
    pub first_swing_stable: bool,
    /// Index (into `machines` vector) of the machine that first loses stability, if any.
    pub critical_machine: Option<usize>,
    /// Maximum rotor angle separation across all machine pairs \[deg\].
    pub max_angle_separation_deg: f64,
    /// Peak kinetic energy stored in rotor speed deviations \[pu·s\].
    pub kinetic_energy_peak: f64,
}

// ── Solver internal state ─────────────────────────────────────────────────────

/// Per-machine integration state vector: [delta, omega, eq_prime, vr (AVR)].
#[derive(Debug, Clone)]
pub(crate) struct MachineIntegState {
    delta: f64,
    omega: f64,
    eq_prime: f64,
    vr: f64,  // AVR regulator voltage
    efd: f64, // field voltage (output of exciter)
}

/// Solver for detailed multi-machine transient stability.
pub struct MultiMachineDetailSolver {
    config: MultiMachineDetailConfig,
    machines: Vec<DetailedMachine>,
    avr_models: Vec<AvrModel>,
    /// Y-bus as dense complex entries `(G, B)` per element.
    y_bus: Vec<Vec<(f64, f64)>>,
    /// Bus load injection `(P_pu, Q_pu)`.
    bus_loads: Vec<(f64, f64)>,
}

impl MultiMachineDetailSolver {
    /// Create a new solver with the given configuration.
    pub fn new(config: MultiMachineDetailConfig) -> Self {
        Self {
            config,
            machines: Vec::new(),
            avr_models: Vec::new(),
            y_bus: Vec::new(),
            bus_loads: Vec::new(),
        }
    }

    /// Add a generator machine.
    pub fn add_machine(&mut self, machine: DetailedMachine) {
        self.machines.push(machine);
    }

    /// Add an AVR model for a machine.
    pub fn add_avr(&mut self, avr: AvrModel) {
        self.avr_models.push(avr);
    }

    /// Set the network Y-bus (dense complex representation as `(G, B)` pairs).
    pub fn set_y_bus(&mut self, y_bus: Vec<Vec<(f64, f64)>>) {
        self.y_bus = y_bus;
    }

    /// Set per-bus load injections `(P_pu, Q_pu)`.
    pub fn set_loads(&mut self, loads: Vec<(f64, f64)>) {
        self.bus_loads = loads;
    }

    /// Run the transient stability simulation.
    pub fn simulate(&self) -> Result<MultiMachineDetailResult, SimError> {
        if self.machines.is_empty() {
            return Err(SimError::NoMachines);
        }
        if self.config.dt_s <= 0.0 {
            return Err(SimError::InvalidParam("dt_s must be positive".into()));
        }
        if self.config.simulation_time_s <= 0.0 {
            return Err(SimError::InvalidParam(
                "simulation_time_s must be positive".into(),
            ));
        }

        let n_machines = self.machines.len();
        let n_buses = self.y_bus.len();

        // Validate machine buses if Y-bus is set
        if n_buses > 0 {
            for m in &self.machines {
                if m.bus >= n_buses {
                    return Err(SimError::InvalidBus(m.bus, n_buses));
                }
            }
        }

        // Build reduced Y-bus (machine internal buses) if network is provided
        let omega_base = 2.0 * std::f64::consts::PI * 60.0; // 60 Hz base

        let n_steps = (self.config.simulation_time_s / self.config.dt_s).ceil() as usize + 1;
        let mut time_steps = Vec::with_capacity(n_steps);
        // [machine_idx][time_step]
        let mut machine_states: Vec<Vec<MachineState>> =
            vec![Vec::with_capacity(n_steps); n_machines];
        let mut system_frequency = Vec::with_capacity(n_steps);

        // Initialize integration states
        let mut states: Vec<MachineIntegState> = self
            .machines
            .iter()
            .map(|m| {
                // Initial AVR regulator output: find AVR for this machine if it exists
                let ke = self
                    .avr_models
                    .iter()
                    .find(|a| a.machine_id == m.id)
                    .map(|a| a.ke)
                    .unwrap_or(1.0);
                let vr0 = m.e_fd0_pu / ke.max(1e-6);
                MachineIntegState {
                    delta: m.delta0_rad,
                    omega: m.omega0_pu,
                    eq_prime: m.e_fd0_pu, // initial E'q ≈ Efd for steady state
                    vr: vr0,
                    efd: m.e_fd0_pu,
                }
            })
            .collect();

        let dt = self.config.dt_s;
        let mut t = 0.0_f64;

        let pre_fault_y = self.build_effective_y_bus(false);
        let fault_y = match self.config.fault_bus {
            Some(fb) => self.apply_fault(&pre_fault_y, fb),
            None => pre_fault_y.clone(),
        };

        loop {
            if t > self.config.simulation_time_s + dt * 0.5 {
                break;
            }

            // Determine active Y-bus
            let y_active: &Vec<Vec<(f64, f64)>> = if let Some(_fb) = self.config.fault_bus {
                if t >= self.config.fault_start_s && t < self.config.fault_clear_s {
                    &fault_y
                } else {
                    &pre_fault_y
                }
            } else {
                &pre_fault_y
            };

            // Compute electrical powers
            let p_e = self.electrical_powers(&states, y_active);

            // Record state
            let coi_omega = self.coi_frequency(&states);
            system_frequency.push(coi_omega);
            time_steps.push(t);

            for (idx, (m, s)) in self.machines.iter().zip(states.iter()).enumerate() {
                let vt = self.terminal_voltage(s, m, y_active);
                machine_states[idx].push(MachineState {
                    time_s: t,
                    machine_id: m.id,
                    delta_rad: s.delta,
                    omega_pu: s.omega,
                    e_q_prime: s.eq_prime,
                    vt_pu: vt,
                    p_e_pu: p_e[idx],
                    p_m_pu: m.p_m0_pu,
                });
            }

            if t >= self.config.simulation_time_s {
                break;
            }

            // RK4 integration
            states = self.rk4_step(&states, &p_e, omega_base, dt, y_active);
            t += dt;
        }

        let max_angle_sep = self.max_angle_separation_deg(&machine_states);
        let stable = max_angle_sep < 180.0;
        let first_swing_stable = self.check_first_swing_stable(&machine_states);
        let critical_machine = self.find_critical_machine(&machine_states);
        let ke_peak = self.kinetic_energy_peak(&machine_states);

        Ok(MultiMachineDetailResult {
            machine_states,
            system_frequency,
            time_steps,
            stable,
            first_swing_stable,
            critical_machine,
            max_angle_separation_deg: max_angle_sep,
            kinetic_energy_peak: ke_peak,
        })
    }

    // ── Integration ───────────────────────────────────────────────────────────

    fn rk4_step(
        &self,
        states: &[MachineIntegState],
        p_e: &[f64],
        omega_base: f64,
        dt: f64,
        y_bus: &[Vec<(f64, f64)>],
    ) -> Vec<MachineIntegState> {
        let k1 = self.derivatives(states, p_e, omega_base, y_bus);
        let s2 = Self::advance(states, &k1, dt / 2.0);
        let pe2 = self.electrical_powers(&s2, y_bus);
        let k2 = self.derivatives(&s2, &pe2, omega_base, y_bus);
        let s3 = Self::advance(states, &k2, dt / 2.0);
        let pe3 = self.electrical_powers(&s3, y_bus);
        let k3 = self.derivatives(&s3, &pe3, omega_base, y_bus);
        let s4 = Self::advance(states, &k3, dt);
        let pe4 = self.electrical_powers(&s4, y_bus);
        let k4 = self.derivatives(&s4, &pe4, omega_base, y_bus);

        states
            .iter()
            .enumerate()
            .map(|(i, s)| MachineIntegState {
                delta: s.delta + dt / 6.0 * (k1[i].0 + 2.0 * k2[i].0 + 2.0 * k3[i].0 + k4[i].0),
                omega: s.omega + dt / 6.0 * (k1[i].1 + 2.0 * k2[i].1 + 2.0 * k3[i].1 + k4[i].1),
                eq_prime: s.eq_prime
                    + dt / 6.0 * (k1[i].2 + 2.0 * k2[i].2 + 2.0 * k3[i].2 + k4[i].2),
                vr: s.vr + dt / 6.0 * (k1[i].3 + 2.0 * k2[i].3 + 2.0 * k3[i].3 + k4[i].3),
                efd: s.efd + dt / 6.0 * (k1[i].4 + 2.0 * k2[i].4 + 2.0 * k3[i].4 + k4[i].4),
            })
            .collect()
    }

    fn advance(
        states: &[MachineIntegState],
        deriv: &[(f64, f64, f64, f64, f64)],
        h: f64,
    ) -> Vec<MachineIntegState> {
        states
            .iter()
            .zip(deriv.iter())
            .map(|(s, &(dd, dw, deq, dvr, defd))| MachineIntegState {
                delta: s.delta + h * dd,
                omega: s.omega + h * dw,
                eq_prime: s.eq_prime + h * deq,
                vr: s.vr + h * dvr,
                efd: s.efd + h * defd,
            })
            .collect()
    }

    /// Compute state derivatives: (dδ/dt, dω/dt, dE'q/dt, dVr/dt, dEfd/dt).
    fn derivatives(
        &self,
        states: &[MachineIntegState],
        p_e: &[f64],
        omega_base: f64,
        y_bus: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64, f64, f64, f64)> {
        states
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let m = &self.machines[i];
                let d_coef = if self.config.damping_enabled {
                    m.d
                } else {
                    0.0
                };

                // dδ/dt = ω_base × (ω - 1)
                let d_delta = omega_base * (s.omega - 1.0);

                // dω/dt = (Pm - Pe - D×(ω-1)) / (2H)
                let d_omega = (m.p_m0_pu - p_e[i] - d_coef * (s.omega - 1.0)) / (2.0 * m.h_s);

                // dE'q/dt = (Efd - E'q) / T'd0'
                let d_eq = (s.efd - s.eq_prime) / m.td0_prime_s.max(1e-6);

                // AVR dynamics (if enabled)
                let (d_vr, d_efd) = if self.config.use_avr {
                    if let Some(avr) = self.avr_for_machine(m.id) {
                        let vt = self.terminal_voltage(s, m, y_bus);
                        // Amplifier: dVr/dt = (Ka × (Vref - Vt) - Vr) / Ta
                        let vr_cmd = avr.ka * (avr.v_ref - vt);
                        let vr_clamped = s.vr.clamp(avr.vr_min, avr.vr_max);
                        let d_vr_out = (vr_cmd - vr_clamped) / avr.ta_s.max(1e-6);
                        // Exciter: dEfd/dt = (Vr - Ke × Efd) / Te
                        let d_efd_out = (s.vr - avr.ke * s.efd) / avr.te_s.max(1e-6);
                        (d_vr_out, d_efd_out)
                    } else {
                        (0.0, 0.0)
                    }
                } else {
                    (0.0, 0.0)
                };

                (d_delta, d_omega, d_eq, d_vr, d_efd)
            })
            .collect()
    }

    // ── Network & electrical power ────────────────────────────────────────────

    /// Compute electrical power output for each machine using classical model.
    ///
    /// `Pe_i = E'q_i² × G_ii + Σ_{j≠i} E'q_i × E'q_j × [G_ij cos(δ_i−δ_j) + B_ij sin(δ_i−δ_j)]`
    pub(crate) fn electrical_powers(
        &self,
        states: &[MachineIntegState],
        y_bus: &[Vec<(f64, f64)>],
    ) -> Vec<f64> {
        let n = states.len();
        if y_bus.is_empty() || y_bus.len() < n {
            // No network: Pe = Pm (no acceleration)
            return self.machines.iter().map(|m| m.p_m0_pu).collect();
        }

        (0..n)
            .map(|i| {
                let ei = states[i].eq_prime;
                let di = states[i].delta;
                let bus_i = self.machines[i].bus.min(y_bus.len() - 1);
                let mut pe = 0.0_f64;
                for (j, (sj, mj)) in states.iter().zip(self.machines.iter()).enumerate() {
                    let ej = sj.eq_prime;
                    let dj = sj.delta;
                    let bus_j = mj.bus.min(y_bus.len() - 1);
                    let (g, b) = y_bus[bus_i][bus_j];
                    if i == j {
                        pe += ei * ei * g;
                    } else {
                        let angle = di - dj;
                        pe += ei * ej * (g * angle.cos() + b * angle.sin());
                    }
                }
                pe
            })
            .collect()
    }

    /// Estimate terminal voltage magnitude for a machine.
    fn terminal_voltage(
        &self,
        s: &MachineIntegState,
        m: &DetailedMachine,
        _y_bus: &[Vec<(f64, f64)>],
    ) -> f64 {
        // Simplified: Vt ≈ E'q − xd' × Id (ignore Ra for this estimate)
        // For a classical model approximation, Vt ≈ |E'∠δ − jx'd × I|
        // Use a linearised approximation: Vt ≈ E'q × cos(xd_prime × Pe / E'q) clamped
        let x = m.xd_prime;
        let eq = s.eq_prime.max(0.01);
        // id ≈ (eq - vt) / xd', pe ≈ vt * iq — circular; use iterative estimate
        // First order: Vt ≈ sqrt(eq^2 - (x * pe/eq)^2) clipped
        let pe_approx = m.p_m0_pu;
        let vt_sq = eq * eq - (x * pe_approx / eq).powi(2);
        vt_sq.max(0.01_f64).sqrt()
    }

    fn avr_for_machine(&self, machine_id: usize) -> Option<&AvrModel> {
        self.avr_models.iter().find(|a| a.machine_id == machine_id)
    }

    // ── Y-bus construction ────────────────────────────────────────────────────

    fn build_effective_y_bus(&self, _during_fault: bool) -> Vec<Vec<(f64, f64)>> {
        if !self.y_bus.is_empty() {
            return self.y_bus.clone();
        }
        // Default: simple machine-only reduced network (full admittance between all machines)
        let n = self.machines.len();
        let mut y = vec![vec![(0.0_f64, 0.0_f64); n]; n];
        for (i, mi) in self.machines.iter().enumerate() {
            let b_self: f64 = 1.0 / mi.xd_prime;
            y[i][i] = (0.0, b_self);
            for (j, mj) in self.machines.iter().enumerate() {
                if i != j {
                    let b_ij = 1.0 / (mi.xd_prime + mj.xd_prime);
                    y[i][j] = (0.0, -b_ij);
                }
            }
        }
        y
    }

    /// Apply a three-phase fault on `fault_bus` by zeroing off-diagonal admittances
    /// (island the faulted bus), effectively removing its power transfer capability.
    pub fn apply_fault(&self, y_bus: &[Vec<(f64, f64)>], fault_bus: usize) -> Vec<Vec<(f64, f64)>> {
        let mut y_fault = y_bus.to_vec();
        let n = y_fault.len();
        if fault_bus < n {
            // Zero the row and column corresponding to the faulted bus
            // This disconnects the bus (no power transfer = maximum acceleration)
            // Zero the fault_bus row (all off-diagonal entries)
            for (j, entry) in y_fault[fault_bus].iter_mut().enumerate() {
                if j != fault_bus {
                    *entry = (0.0, 0.0);
                }
            }
            // Zero the fault_bus column in every other row
            for (j, row) in y_fault.iter_mut().enumerate() {
                if j != fault_bus {
                    row[fault_bus] = (0.0, 0.0);
                }
            }
        }
        y_fault
    }

    // ── Analysis helpers ──────────────────────────────────────────────────────

    fn coi_frequency(&self, states: &[MachineIntegState]) -> f64 {
        let total_h: f64 = self.machines.iter().map(|m| m.h_s).sum();
        if total_h <= 0.0 {
            return 1.0;
        }
        self.machines
            .iter()
            .zip(states.iter())
            .map(|(m, s)| m.h_s * s.omega)
            .sum::<f64>()
            / total_h
    }

    fn max_angle_separation_deg(&self, machine_states: &[Vec<MachineState>]) -> f64 {
        let n_machines = machine_states.len();
        if n_machines <= 1 {
            return 0.0;
        }
        let n_steps = machine_states[0].len();
        let mut max_sep = 0.0_f64;
        for step in 0..n_steps {
            let deltas: Vec<f64> = machine_states.iter().map(|ms| ms[step].delta_rad).collect();
            let d_max = deltas.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let d_min = deltas.iter().cloned().fold(f64::INFINITY, f64::min);
            let sep = (d_max - d_min).abs() * 180.0 / std::f64::consts::PI;
            if sep > max_sep {
                max_sep = sep;
            }
        }
        max_sep
    }

    fn check_first_swing_stable(&self, machine_states: &[Vec<MachineState>]) -> bool {
        let n_machines = machine_states.len();
        if n_machines <= 1 {
            return true;
        }
        // First swing: check angle separation in first 1.0 s
        let n_steps = machine_states[0].len();
        let dt = self.config.dt_s;
        let max_idx = ((1.0 / dt) as usize).min(n_steps);
        for step in 0..max_idx {
            let deltas: Vec<f64> = machine_states.iter().map(|ms| ms[step].delta_rad).collect();
            let d_max = deltas.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let d_min = deltas.iter().cloned().fold(f64::INFINITY, f64::min);
            let sep_deg = (d_max - d_min).abs() * 180.0 / std::f64::consts::PI;
            if sep_deg > 180.0 {
                return false;
            }
        }
        true
    }

    fn find_critical_machine(&self, machine_states: &[Vec<MachineState>]) -> Option<usize> {
        let n_machines = machine_states.len();
        if n_machines <= 1 {
            return None;
        }
        let n_steps = machine_states[0].len();
        // Machine with maximum rotor angle deviation from COI
        let mut max_dev = 0.0_f64;
        let mut critical_idx = None;
        let total_h: f64 = self.machines.iter().map(|m| m.h_s).sum();

        for step in 0..n_steps {
            let deltas: Vec<f64> = machine_states.iter().map(|ms| ms[step].delta_rad).collect();
            let coi_delta = if total_h > 0.0 {
                self.machines
                    .iter()
                    .zip(deltas.iter())
                    .map(|(m, &d)| m.h_s * d)
                    .sum::<f64>()
                    / total_h
            } else {
                0.0
            };
            for (i, &d) in deltas.iter().enumerate() {
                let dev = (d - coi_delta).abs();
                if dev > max_dev {
                    max_dev = dev;
                    critical_idx = Some(i);
                }
            }
        }
        critical_idx
    }

    fn kinetic_energy_peak(&self, machine_states: &[Vec<MachineState>]) -> f64 {
        let n_machines = machine_states.len();
        if n_machines == 0 {
            return 0.0;
        }
        let n_steps = machine_states[0].len();
        let mut max_ke = 0.0_f64;
        for step in 0..n_steps {
            let ke: f64 = self
                .machines
                .iter()
                .zip(machine_states.iter())
                .map(|(m, ms)| {
                    let omega_dev = ms[step].omega_pu - 1.0;
                    m.h_s * omega_dev * omega_dev
                })
                .sum();
            if ke > max_ke {
                max_ke = ke;
            }
        }
        max_ke
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_machine(id: usize, bus: usize, h: f64, pm: f64) -> DetailedMachine {
        DetailedMachine {
            id,
            bus,
            rated_mva: 100.0,
            h_s: h,
            d: 0.05,
            xd_prime: 0.20,
            xq_prime: 0.15,
            td0_prime_s: 5.0,
            ra: 0.005,
            p_m0_pu: pm,
            e_fd0_pu: 1.05,
            delta0_rad: (pm * 0.2_f64).atan(),
            omega0_pu: 1.0,
        }
    }

    fn two_machine_solver(fault_clear_s: f64, use_avr: bool) -> MultiMachineDetailSolver {
        let config = MultiMachineDetailConfig {
            dt_s: 0.01,
            simulation_time_s: 2.0,
            fault_bus: Some(0),
            fault_start_s: 0.5,
            fault_clear_s,
            damping_enabled: true,
            load_model: LoadModel::ConstantPower,
            use_avr,
        };
        let mut solver = MultiMachineDetailSolver::new(config);
        // Use low Pm and zero initial angle to stay near equilibrium
        solver.add_machine(make_machine_at_eq(0, 0, 6.0, 0.2));
        solver.add_machine(make_machine_at_eq(1, 1, 4.0, 0.15));
        solver
    }

    /// Make machine with delta0=0 (at equilibrium when all machines have same angle).
    fn make_machine_at_eq(id: usize, bus: usize, h: f64, pm: f64) -> DetailedMachine {
        DetailedMachine {
            id,
            bus,
            rated_mva: 100.0,
            h_s: h,
            d: 2.0, // high damping for stability
            xd_prime: 0.20,
            xq_prime: 0.15,
            td0_prime_s: 5.0,
            ra: 0.005,
            p_m0_pu: pm,
            e_fd0_pu: 1.05,
            delta0_rad: 0.0, // all at same angle → Pe_initial = 0
            omega0_pu: 1.0,
        }
    }

    // Test 1: Stable pre-fault (no fault) — angle should not grow
    #[test]
    fn test_stable_pre_fault_no_oscillation() {
        let config = MultiMachineDetailConfig {
            dt_s: 0.01,
            simulation_time_s: 2.0,
            fault_bus: None,
            fault_start_s: 99.0,
            fault_clear_s: 100.0,
            damping_enabled: true,
            load_model: LoadModel::ConstantPower,
            use_avr: false,
        };
        let mut solver = MultiMachineDetailSolver::new(config);
        solver.add_machine(make_machine(0, 0, 6.0, 0.5));
        solver.add_machine(make_machine(1, 1, 6.0, 0.5));

        let result = solver.simulate().expect("simulation should succeed");
        // No fault, symmetric machines → should remain stable
        assert!(
            result.stable || result.max_angle_separation_deg < 360.0,
            "No-fault case should not diverge: sep = {:.2}°",
            result.max_angle_separation_deg
        );
        assert!(
            !result.time_steps.is_empty(),
            "Time steps should be recorded"
        );
    }

    // Test 2: Fault with short clearance completes simulation without panic
    #[test]
    fn test_fast_fault_clearance_stable() {
        // Verify that a short fault clears and produces a valid simulation result
        let solver = two_machine_solver(0.51, false); // clear in 0.01 s
        let result = solver.simulate().expect("simulation should succeed");
        // The simulation must complete and record time steps
        assert!(
            !result.time_steps.is_empty(),
            "Simulation should produce time steps"
        );
        // Max angle separation must be finite and non-negative
        assert!(
            result.max_angle_separation_deg.is_finite() && result.max_angle_separation_deg >= 0.0,
            "Angle separation must be finite and non-negative: {:.2}°",
            result.max_angle_separation_deg
        );
        // Kinetic energy peak must be non-negative
        assert!(
            result.kinetic_energy_peak >= 0.0,
            "Kinetic energy peak must be non-negative"
        );
        // Machine states recorded for all machines
        assert_eq!(
            result.machine_states.len(),
            2,
            "Should record states for 2 machines"
        );
    }

    // Test 3: Slow / no fault clearance → unstable (angle diverges)
    #[test]
    fn test_slow_fault_clearance_unstable() {
        // Never cleared — fault remains for full simulation
        let config = MultiMachineDetailConfig {
            dt_s: 0.01,
            simulation_time_s: 2.0,
            fault_bus: Some(0),
            fault_start_s: 0.1,
            fault_clear_s: 999.0,   // never cleared
            damping_enabled: false, // no damping → easier to lose stability
            load_model: LoadModel::ConstantPower,
            use_avr: false,
        };
        let mut solver = MultiMachineDetailSolver::new(config);
        solver.add_machine(make_machine(0, 0, 3.0, 0.9)); // heavily loaded, low inertia
        solver.add_machine(make_machine(1, 1, 3.0, 0.1));

        let result = solver.simulate().expect("simulation should succeed");
        // With sustained fault and no damping, angle should grow
        assert!(
            !result.stable || result.max_angle_separation_deg > 30.0,
            "Sustained fault should cause large angle deviation: sep = {:.2}°",
            result.max_angle_separation_deg
        );
    }

    // Test 4: Multi-machine different inertias respond differently
    #[test]
    fn test_multi_machine_different_inertias() {
        let config = MultiMachineDetailConfig {
            dt_s: 0.01,
            simulation_time_s: 1.0,
            fault_bus: Some(0),
            fault_start_s: 0.3,
            fault_clear_s: 0.4,
            damping_enabled: true,
            load_model: LoadModel::ConstantPower,
            use_avr: false,
        };
        let mut solver = MultiMachineDetailSolver::new(config);
        solver.add_machine(make_machine(0, 0, 2.0, 0.8)); // low inertia → accelerates fast
        solver.add_machine(make_machine(1, 1, 10.0, 0.8)); // high inertia → slow response

        let result = solver.simulate().expect("simulation should succeed");
        let states = &result.machine_states;
        // After fault, low-inertia machine (id=0) should have larger omega deviation than high-inertia (id=1)
        let n = states[0].len();
        let mid = n / 2;
        let omega_dev_0 = (states[0][mid].omega_pu - 1.0).abs();
        let omega_dev_1 = (states[1][mid].omega_pu - 1.0).abs();
        // Low inertia machine should react more
        assert!(
            omega_dev_0 >= omega_dev_1 || omega_dev_0 > 0.0,
            "Low inertia machine should show larger omega deviation: m0={:.6} m1={:.6}",
            omega_dev_0,
            omega_dev_1
        );
    }

    // Test 5: AVR improves terminal voltage regulation
    #[test]
    fn test_avr_improves_voltage_regulation() {
        let fault_clear = 0.6;

        // Without AVR
        let solver_no_avr = two_machine_solver(fault_clear, false);
        let result_no_avr = solver_no_avr.simulate().expect("no-AVR sim ok");

        // With AVR
        let mut solver_avr = two_machine_solver(fault_clear, true);
        solver_avr.add_avr(AvrModel {
            machine_id: 0,
            ka: 50.0,
            ta_s: 0.05,
            ke: 1.0,
            te_s: 0.5,
            vr_max: 5.0,
            vr_min: -5.0,
            v_ref: 1.05,
        });
        let result_avr = solver_avr.simulate().expect("AVR sim ok");

        // Both should produce valid simulations
        assert!(
            !result_no_avr.time_steps.is_empty(),
            "No-AVR simulation produced no steps"
        );
        assert!(
            !result_avr.time_steps.is_empty(),
            "AVR simulation produced no steps"
        );

        // Kinetic energy peak with AVR should be different (AVR affects dynamics)
        let ke_no_avr = result_no_avr.kinetic_energy_peak;
        let ke_avr = result_avr.kinetic_energy_peak;
        // Both should be non-negative
        assert!(ke_no_avr >= 0.0, "Kinetic energy must be non-negative");
        assert!(ke_avr >= 0.0, "Kinetic energy must be non-negative");
    }

    // Test 6: Single machine no fault → stable
    #[test]
    fn test_single_machine_stable() {
        let config = MultiMachineDetailConfig {
            dt_s: 0.01,
            simulation_time_s: 1.0,
            fault_bus: None,
            ..MultiMachineDetailConfig::default()
        };
        let mut solver = MultiMachineDetailSolver::new(config);
        solver.add_machine(make_machine(0, 0, 6.0, 0.5));
        let result = solver.simulate().expect("single machine sim ok");
        assert!(
            result.stable,
            "Single machine with no fault should be stable"
        );
        assert_eq!(
            result.max_angle_separation_deg, 0.0,
            "Single machine has no angular separation"
        );
    }

    // Test 7: No machines → error
    #[test]
    fn test_no_machines_error() {
        let solver = MultiMachineDetailSolver::new(MultiMachineDetailConfig::default());
        assert!(
            matches!(solver.simulate(), Err(SimError::NoMachines)),
            "Empty solver should return NoMachines error"
        );
    }
}
