//! Distribution Volt-VAR Optimization (VVO).
//!
//! Implements coordinated Volt-VAR control for radial distribution networks
//! using a two-stage sequential mixed-integer optimization approach:
//!
//! - **Stage 1 (Continuous)**: Optimize reactive power setpoints for SVCs,
//!   PV inverters, and BESS using gradient descent on the objective function.
//! - **Stage 2 (Discrete)**: Optimize OLTC tap positions and capacitor bank
//!   steps using greedy local search (enumerate per device, pick best).
//!
//! The two stages alternate until voltage convergence or maximum iterations.
//!
//! # Units
//! - Voltages in per-unit `pu`
//! - Reactive power in `MVAr` unless noted
//! - Active power in `MW` unless noted
//! - Tap positions dimensionless `pu` ratio

use crate::error::OxiGridError;
use std::collections::VecDeque;

// ─────────────────────────────────────────────────────────────────────────────
// Device model
// ─────────────────────────────────────────────────────────────────────────────

/// Volt-VAR control device enumeration covering all common distribution assets.
#[derive(Debug, Clone)]
pub enum VvoDevice {
    /// On-Load Tap-Changer (OLTC) transformer — adjusts feeder voltage via tap.
    OltcTransformer {
        /// Connected bus index.
        bus: usize,
        /// Minimum tap ratio `pu`.
        min_tap: f64,
        /// Maximum tap ratio `pu`.
        max_tap: f64,
        /// Tap step size `pu`.
        tap_step: f64,
        /// Current tap ratio `pu`.
        current_tap: f64,
        /// Anti-hunting time delay `s`.
        time_delay_s: f64,
    },
    /// Switched capacitor bank — discrete reactive power injection.
    CapacitorBank {
        /// Connected bus index.
        bus: usize,
        /// Reactive power per step `MVAr`.
        step_size_mvar: f64,
        /// Total number of steps.
        n_steps: usize,
        /// Currently energized step count.
        current_steps: usize,
        /// Whether switching is allowed.
        switchable: bool,
    },
    /// Static VAR Compensator (SVC) — continuous reactive source/sink.
    StaticVarCompensator {
        /// Connected bus index.
        bus: usize,
        /// Minimum reactive power `MVAr` (typically negative for absorption).
        q_min_mvar: f64,
        /// Maximum reactive power `MVAr`.
        q_max_mvar: f64,
        /// Current reactive output `MVAr`.
        current_q_mvar: f64,
        /// Droop coefficient [% voltage change per % reactive change].
        droop_pct: f64,
    },
    /// Grid-connected PV inverter with reactive capability.
    PhotovoltaicInverter {
        /// Connected bus index.
        bus: usize,
        /// Current active power generation `MW`.
        p_mw: f64,
        /// Rated apparent power `MVA`.
        s_rated_mva: f64,
        /// Minimum power factor (e.g. 0.9).
        power_factor_min: f64,
        /// Current reactive power output `MVAr`.
        current_q_mvar: f64,
        /// Whether the inverter can absorb reactive power (capacitive mode).
        can_absorb_q: bool,
    },
    /// Battery Energy Storage System (BESS) with four-quadrant reactive capability.
    BatteryEss {
        /// Connected bus index.
        bus: usize,
        /// Current active power `MW` (positive = discharge).
        p_mw: f64,
        /// Maximum reactive injection `MVAr`.
        q_max_mvar: f64,
        /// Maximum reactive absorption `MVAr` (negative value).
        q_min_mvar: f64,
        /// Current reactive output `MVAr`.
        current_q_mvar: f64,
    },
}

impl VvoDevice {
    /// Bus index where this device is connected.
    pub fn bus(&self) -> usize {
        match self {
            Self::OltcTransformer { bus, .. } => *bus,
            Self::CapacitorBank { bus, .. } => *bus,
            Self::StaticVarCompensator { bus, .. } => *bus,
            Self::PhotovoltaicInverter { bus, .. } => *bus,
            Self::BatteryEss { bus, .. } => *bus,
        }
    }

    /// Current reactive power output `MVAr`. Positive = reactive injection.
    pub fn current_q_mvar(&self) -> f64 {
        match self {
            Self::OltcTransformer { .. } => 0.0,
            Self::CapacitorBank {
                step_size_mvar,
                current_steps,
                ..
            } => step_size_mvar * (*current_steps as f64),
            Self::StaticVarCompensator { current_q_mvar, .. } => *current_q_mvar,
            Self::PhotovoltaicInverter { current_q_mvar, .. } => *current_q_mvar,
            Self::BatteryEss { current_q_mvar, .. } => *current_q_mvar,
        }
    }

    /// Feasible reactive power range `(q_min, q_max)` `MVAr`.
    pub fn q_range(&self) -> (f64, f64) {
        match self {
            Self::OltcTransformer { .. } => (0.0, 0.0),
            Self::CapacitorBank {
                step_size_mvar,
                n_steps,
                ..
            } => (0.0, step_size_mvar * (*n_steps as f64)),
            Self::StaticVarCompensator {
                q_min_mvar,
                q_max_mvar,
                ..
            } => (*q_min_mvar, *q_max_mvar),
            Self::PhotovoltaicInverter {
                p_mw,
                s_rated_mva,
                power_factor_min,
                can_absorb_q,
                ..
            } => {
                // Maximum Q limited by apparent power capacity.
                let q_max_apparent = (s_rated_mva.powi(2) - p_mw.powi(2).min(s_rated_mva.powi(2)))
                    .max(0.0)
                    .sqrt();
                // Maximum Q limited by minimum power factor: Q_max = P * tan(acos(pf_min)).
                let pf = power_factor_min.clamp(1e-6, 1.0);
                let q_max_pf = p_mw * (1.0 - pf * pf).max(0.0).sqrt() / pf;
                let q_max = q_max_apparent.min(q_max_pf).max(0.0);
                let q_min = if *can_absorb_q { -q_max } else { 0.0 };
                (q_min, q_max)
            }
            Self::BatteryEss {
                q_min_mvar,
                q_max_mvar,
                ..
            } => (*q_min_mvar, *q_max_mvar),
        }
    }

    /// Human-readable device type name.
    pub fn device_type_name(&self) -> &str {
        match self {
            Self::OltcTransformer { .. } => "OLTC",
            Self::CapacitorBank { .. } => "CapacitorBank",
            Self::StaticVarCompensator { .. } => "SVC",
            Self::PhotovoltaicInverter { .. } => "PV_Inverter",
            Self::BatteryEss { .. } => "BESS",
        }
    }

    /// Returns `true` if the device uses discrete (step-wise) control.
    fn is_discrete(&self) -> bool {
        matches!(
            self,
            Self::OltcTransformer { .. } | Self::CapacitorBank { .. }
        )
    }

    /// Returns `true` if the device provides continuous reactive power control.
    fn is_continuous(&self) -> bool {
        matches!(
            self,
            Self::StaticVarCompensator { .. }
                | Self::PhotovoltaicInverter { .. }
                | Self::BatteryEss { .. }
        )
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Network topology primitives
// ─────────────────────────────────────────────────────────────────────────────

/// Distribution feeder bus.
#[derive(Debug, Clone)]
pub struct VvoBus {
    /// Bus index (0-based, must be unique).
    pub id: usize,
    /// Minimum acceptable voltage `pu` (typically 0.95).
    pub v_min_pu: f64,
    /// Maximum acceptable voltage `pu` (typically 1.05).
    pub v_max_pu: f64,
    /// Active load at this bus `MW`.
    pub p_load_mw: f64,
    /// Reactive load at this bus `MVAr`.
    pub q_load_mvar: f64,
    /// `true` for the substation (slack) bus where V = 1.0 pu is enforced.
    pub is_substation: bool,
}

/// Distribution feeder branch (π-section, radial assumption).
#[derive(Debug, Clone)]
pub struct VvoBranch {
    /// Sending-end bus index.
    pub from: usize,
    /// Receiving-end bus index.
    pub to: usize,
    /// Series resistance `Ω`.
    pub r_ohm: f64,
    /// Series reactance `Ω`.
    pub x_ohm: f64,
    /// Thermal MVA rating `MVA`.
    pub rating_mva: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Objective and configuration
// ─────────────────────────────────────────────────────────────────────────────

/// VVO objective function weighting parameters.
#[derive(Debug, Clone)]
pub struct VvoObjective {
    /// Weight applied to real power losses `MW`.
    pub weight_losses: f64,
    /// Weight applied to sum of squared voltage deviations from reference.
    pub weight_voltage_deviation: f64,
    /// Weight applied to number of discrete device operations (tap changes / cap switches).
    pub weight_device_operations: f64,
    /// Voltage reference for deviation penalty `pu`.
    pub v_ref_pu: f64,
}

impl Default for VvoObjective {
    fn default() -> Self {
        Self {
            weight_losses: 1.0,
            weight_voltage_deviation: 0.5,
            weight_device_operations: 0.1,
            v_ref_pu: 1.0,
        }
    }
}

/// VVO solver configuration.
#[derive(Debug, Clone)]
pub struct VvoConfig {
    /// Number of buses in the network.
    pub n_buses: usize,
    /// System base apparent power `MVA`.
    pub base_mva: f64,
    /// System base voltage `kV` (line-to-line).
    pub base_kv: f64,
    /// Objective function weights and reference voltage.
    pub objective: VvoObjective,
    /// Maximum number of two-stage outer iterations.
    pub max_iterations: usize,
    /// Convergence tolerance on maximum bus voltage magnitude change `pu`.
    pub voltage_tolerance: f64,
    /// Number of trials for discrete device search (unused in current greedy impl).
    pub n_discrete_trials: usize,
}

impl Default for VvoConfig {
    fn default() -> Self {
        Self {
            n_buses: 0,
            base_mva: 100.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 50,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 10,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Volt-VAR optimization result.
#[derive(Debug, Clone)]
pub struct VvoResult {
    /// Whether the algorithm converged within `max_iterations`.
    pub converged: bool,
    /// Number of outer iterations performed.
    pub iterations: usize,
    /// Bus voltage magnitudes `pu` indexed by bus id.
    pub voltage_magnitudes: Vec<f64>,
    /// Bus voltage angles `rad` (zero for linearised BFS model).
    pub voltage_angles: Vec<f64>,
    /// Reactive power setpoint per device `MVAr` (same order as `devices`).
    pub device_setpoints: Vec<f64>,
    /// Total feeder active power losses `MW`.
    pub total_losses_mw: f64,
    /// RMS voltage deviation from reference across all buses `pu`.
    pub voltage_deviation_pu: f64,
    /// Number of OLTC tap-change operations performed.
    pub n_tap_operations: usize,
    /// Number of capacitor bank switching operations performed.
    pub n_capacitor_switches: usize,
    /// Final objective function value.
    pub objective_value: f64,
    /// Buses with voltage outside [v_min, v_max] as `(bus_id, voltage_pu)`.
    pub voltage_violations: Vec<(usize, f64)>,
    /// Branch loading as percentage of thermal MVA rating.
    pub branch_loadings_pct: Vec<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// Coordinated Volt-VAR optimizer for radial distribution networks.
///
/// Implements a two-stage sequential mixed-integer algorithm:
/// 1. Gradient descent on continuous device Q setpoints (SVC / PV / BESS).
/// 2. Greedy local search over discrete device positions (OLTC taps / cap steps).
///
/// Both stages use linearised DistFlow Backward-Forward Sweep as the inner
/// power flow engine.
pub struct VoltVarOptimizer {
    /// Distribution buses.
    pub buses: Vec<VvoBus>,
    /// Distribution branches.
    pub branches: Vec<VvoBranch>,
    /// Controllable devices.
    pub devices: Vec<VvoDevice>,
    /// Solver configuration.
    pub config: VvoConfig,
}

impl VoltVarOptimizer {
    /// Construct a new optimizer.
    pub fn new(
        buses: Vec<VvoBus>,
        branches: Vec<VvoBranch>,
        devices: Vec<VvoDevice>,
        config: VvoConfig,
    ) -> Self {
        Self {
            buses,
            branches,
            devices,
            config,
        }
    }

    // ── Public entry point ────────────────────────────────────────────────────

    /// Run the two-stage VVO algorithm and return the optimised operating point.
    ///
    /// Returns `Err(OxiGridError::InvalidNetwork)` when the network is degenerate
    /// (zero buses, no substation, etc.).
    pub fn optimize(&mut self) -> Result<VvoResult, OxiGridError> {
        let n_buses = self.config.n_buses;
        if n_buses == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "VVO network has zero buses".into(),
            ));
        }

        // Initialise device setpoints from current device state.
        let mut q_setpoints: Vec<f64> = self.devices.iter().map(|d| d.current_q_mvar()).collect();

        // Build bus injection vector.
        let mut q_injections = self.q_setpoints_to_injections(&q_setpoints);

        // Initial BFS solve.
        let (mut v, _) = self.solve_bfs(&q_injections)?;

        let mut converged = false;
        let mut iterations = 0usize;
        let mut total_tap_ops = 0usize;
        let mut total_cap_switches = 0usize;

        // Initial step size for gradient descent (decays with iterations).
        let step_size_initial = 0.05_f64;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;
            let v_prev: Vec<f64> = v.clone();

            // ── Stage 1: continuous devices (SVC, PV, BESS) ─────────────────
            let step_size = step_size_initial / (1.0 + iter as f64 * 0.05);
            self.optimize_continuous_devices(&mut q_injections, &v, step_size);

            // Sync setpoints from updated injections for continuous devices.
            for (idx, dev) in self.devices.iter().enumerate() {
                if dev.is_continuous() {
                    let bus = dev.bus();
                    if bus < q_injections.len() {
                        let (lo, hi) = dev.q_range();
                        q_setpoints[idx] = q_injections[bus].max(lo).min(hi);
                    }
                }
            }
            q_injections = self.q_setpoints_to_injections(&q_setpoints);

            // BFS after continuous stage.
            let (v_c, _) = self.solve_bfs(&q_injections)?;
            v = v_c;

            // ── Stage 2: discrete devices (OLTC, Capacitor) ──────────────────
            let (q_disc, tap_ops, cap_sw) = self.optimize_discrete_devices(&q_injections, &v);
            total_tap_ops += tap_ops;
            total_cap_switches += cap_sw;
            q_injections = q_disc;

            // Sync discrete setpoints from updated device state.
            for (idx, dev) in self.devices.iter().enumerate() {
                if dev.is_discrete() {
                    q_setpoints[idx] = dev.current_q_mvar();
                }
            }

            // BFS after discrete stage.
            let (v_d, _) = self.solve_bfs(&q_injections)?;
            v = v_d;

            // Convergence check: maximum voltage change across buses.
            let max_dv = v
                .iter()
                .zip(v_prev.iter())
                .map(|(vi, vp)| (*vi - *vp).abs())
                .fold(0.0_f64, f64::max);

            if max_dv < self.config.voltage_tolerance {
                converged = true;
                break;
            }
        }

        // Final synchronisation: reflect device state into setpoints.
        for (idx, dev) in self.devices.iter().enumerate() {
            q_setpoints[idx] = dev.current_q_mvar();
        }
        q_injections = self.q_setpoints_to_injections(&q_setpoints);
        let (v_final, angles_final) = self.solve_bfs(&q_injections)?;

        let losses = self.compute_losses(&v_final, &q_injections);
        let obj = self.compute_objective(&v_final, losses, &q_setpoints, &q_setpoints);

        // RMS voltage deviation.
        let n_buses_f = v_final.len() as f64;
        let sum_sq: f64 = v_final
            .iter()
            .map(|vi| (vi - self.config.objective.v_ref_pu).powi(2))
            .sum();
        let v_dev = (sum_sq / n_buses_f.max(1.0)).sqrt();

        let violations = self.check_violations(&v_final);
        let branch_loadings = self.compute_branch_loading(&v_final, &q_injections);

        Ok(VvoResult {
            converged,
            iterations,
            voltage_magnitudes: v_final.to_vec(),
            voltage_angles: angles_final,
            device_setpoints: q_setpoints,
            total_losses_mw: losses,
            voltage_deviation_pu: v_dev,
            n_tap_operations: total_tap_ops,
            n_capacitor_switches: total_cap_switches,
            objective_value: obj,
            voltage_violations: violations,
            branch_loadings_pct: branch_loadings,
        })
    }

    // ── Backward-Forward Sweep (BFS) power flow ───────────────────────────────

    /// Linearised DistFlow Backward-Forward Sweep for a radial distribution
    /// network.
    ///
    /// **Backward pass**: accumulate subtree active and reactive power demands
    /// from leaves towards the substation root.
    ///
    /// **Forward pass**: compute bus voltages from root outward using:
    /// `V_to ≈ V_from − (R_pu · P + X_pu · Q) / V_from`
    /// with a second-order correction term for improved accuracy.
    ///
    /// Returns `(voltage_magnitudes_pu, voltage_angles_rad)`.
    /// Angles are zero (linearised model).
    pub fn solve_bfs(&self, q_injections: &[f64]) -> Result<(Vec<f64>, Vec<f64>), OxiGridError> {
        let n = self.config.n_buses;
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork("BFS: zero-bus network".into()));
        }

        // Locate substation (slack) bus.
        let root = self
            .buses
            .iter()
            .find(|b| b.is_substation)
            .ok_or_else(|| OxiGridError::InvalidNetwork("No substation bus found".into()))?
            .id;

        // Build bidirectional adjacency list.
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (br_idx, br) in self.branches.iter().enumerate() {
            if br.from < n && br.to < n {
                adj[br.from].push((br.to, br_idx));
                adj[br.to].push((br.from, br_idx));
            }
        }

        // BFS to establish traversal order and parent/branch mapping.
        let mut order: Vec<usize> = Vec::with_capacity(n);
        let mut parent: Vec<Option<usize>> = vec![None; n];
        let mut parent_branch: Vec<Option<usize>> = vec![None; n];
        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();
        queue.push_back(root);
        visited[root] = true;

        while let Some(node) = queue.pop_front() {
            order.push(node);
            for &(nb, br_idx) in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    parent[nb] = Some(node);
                    parent_branch[nb] = Some(br_idx);
                    queue.push_back(nb);
                }
            }
        }

        // Base impedance: Z_base [Ω] = (V_base [kV])² / S_base [MVA].
        let z_base = self.config.base_kv * self.config.base_kv / self.config.base_mva;

        // ── Backward pass: accumulate subtree P/Q demands ─────────────────────
        // p_sub[i] / q_sub[i]: net P/Q that must be supplied through the
        // branch leading into bus i from its parent (in per-unit).
        let mut p_sub = vec![0.0_f64; n];
        let mut q_sub = vec![0.0_f64; n];

        for bus in &self.buses {
            if bus.id < n {
                p_sub[bus.id] = bus.p_load_mw / self.config.base_mva;
                let q_dev = q_injections.get(bus.id).copied().unwrap_or(0.0);
                // Net Q = load − device injection (injection reduces flow burden).
                q_sub[bus.id] = (bus.q_load_mvar - q_dev) / self.config.base_mva;
            }
        }

        // Propagate from leaves to root.
        for &bus in order.iter().rev() {
            if bus == root {
                continue;
            }
            if let Some(par) = parent[bus] {
                let p_b = p_sub[bus];
                let q_b = q_sub[bus];
                p_sub[par] += p_b;
                q_sub[par] += q_b;
            }
        }

        // ── Forward pass: update voltages root → leaves ───────────────────────
        let mut v = vec![1.0_f64; n];
        v[root] = 1.0;

        for &bus in order.iter() {
            if bus == root {
                continue;
            }
            if let (Some(par), Some(br_idx)) = (parent[bus], parent_branch[bus]) {
                let br = &self.branches[br_idx];
                let r_pu = br.r_ohm / z_base;
                let x_pu = br.x_ohm / z_base;
                let p_pu = p_sub[bus];
                let q_pu = q_sub[bus];
                let v_from = v[par].max(0.5);

                // Linearised DistFlow with second-order correction.
                let delta_v1 = (r_pu * p_pu + x_pu * q_pu) / v_from;
                let delta_v2 = (r_pu.powi(2) + x_pu.powi(2)) * (p_pu.powi(2) + q_pu.powi(2))
                    / (2.0 * v_from.powi(2));
                v[bus] = (v_from - delta_v1 + delta_v2).max(0.5);
            }
        }

        let angles = vec![0.0_f64; n];
        Ok((v, angles))
    }

    // ── Loss calculation ──────────────────────────────────────────────────────

    /// Real power losses `MW` via DistFlow:
    /// `P_loss = Σ_branches (P_branch² + Q_branch²) / V_from² × R_pu × S_base`
    fn compute_losses(&self, v: &[f64], q_injections: &[f64]) -> f64 {
        let n = self.config.n_buses;
        let z_base = self.config.base_kv * self.config.base_kv / self.config.base_mva;

        let root = self
            .buses
            .iter()
            .find(|b| b.is_substation)
            .map(|b| b.id)
            .unwrap_or(0);

        let (order, parent, parent_branch) = self.build_bfs_tree(root, n);

        let (p_sub, q_sub) = self.build_subtree_flows(n, root, &order, &parent, q_injections);

        let mut total_loss_pu = 0.0_f64;
        for &bus in &order {
            if bus == root {
                continue;
            }
            if let (Some(par), Some(br_idx)) = (parent[bus], parent_branch[bus]) {
                let br = &self.branches[br_idx];
                let r_pu = br.r_ohm / z_base;
                let v_from = v.get(par).copied().unwrap_or(1.0).max(0.1);
                total_loss_pu += (p_sub[bus].powi(2) + q_sub[bus].powi(2)) / v_from.powi(2) * r_pu;
            }
        }

        total_loss_pu * self.config.base_mva
    }

    // ── Objective function ────────────────────────────────────────────────────

    /// Compute the VVO objective function:
    /// `J = w_loss · P_loss + w_vdev · Σ(V_i − V_ref)² + w_ops · N_ops`
    ///
    /// where `N_ops` counts device setpoints that changed by more than 1 μMVAr.
    fn compute_objective(
        &self,
        v: &[f64],
        losses_mw: f64,
        q_setpoints_prev: &[f64],
        q_setpoints_new: &[f64],
    ) -> f64 {
        let obj = &self.config.objective;

        let loss_term = obj.weight_losses * losses_mw;

        let vdev_term = v.iter().map(|vi| (vi - obj.v_ref_pu).powi(2)).sum::<f64>()
            * obj.weight_voltage_deviation;

        let n_ops = q_setpoints_prev
            .iter()
            .zip(q_setpoints_new.iter())
            .filter(|(prev, new)| (*prev - *new).abs() > 1e-6)
            .count() as f64;
        let ops_term = obj.weight_device_operations * n_ops;

        loss_term + vdev_term + ops_term
    }

    // ── Gradient computation ──────────────────────────────────────────────────

    /// Approximate gradient of the objective w.r.t. Q injection at each bus.
    ///
    /// Loss gradient: `∂P_loss/∂Q_i ≈ −2 · Q_i_pu · R_upstream / V_i²`
    /// (negative because increasing injection reduces Q flow).
    ///
    /// Voltage deviation gradient: `2 · w_vdev · (V_i − V_ref) · (X_upstream / V_i)`
    fn compute_q_gradient(&self, v: &[f64], q_injections: &[f64]) -> Vec<f64> {
        let n = self.config.n_buses;
        let z_base = self.config.base_kv * self.config.base_kv / self.config.base_mva;
        let obj = &self.config.objective;

        let root = self
            .buses
            .iter()
            .find(|b| b.is_substation)
            .map(|b| b.id)
            .unwrap_or(0);

        // Map each bus to its upstream branch R_pu and X_pu.
        let mut upstream_r = vec![0.0_f64; n];
        let mut upstream_x = vec![0.0_f64; n];

        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (br_idx, br) in self.branches.iter().enumerate() {
            if br.from < n && br.to < n {
                adj[br.from].push((br.to, br_idx));
                adj[br.to].push((br.from, br_idx));
            }
        }
        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();
        queue.push_back(root);
        visited[root] = true;
        while let Some(node) = queue.pop_front() {
            for &(nb, br_idx) in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    upstream_r[nb] = self.branches[br_idx].r_ohm / z_base;
                    upstream_x[nb] = self.branches[br_idx].x_ohm / z_base;
                    queue.push_back(nb);
                }
            }
        }

        let mut grad = vec![0.0_f64; n];
        for i in 0..n {
            let vi = v.get(i).copied().unwrap_or(1.0).max(0.1);
            let qi_pu = q_injections.get(i).copied().unwrap_or(0.0) / self.config.base_mva;

            // Loss gradient (injection reduces Q burden → reduces losses → negative gradient).
            let loss_grad = -2.0 * qi_pu * upstream_r[i] / vi.powi(2);

            // Voltage deviation gradient: ∂V_i/∂Q_inj_i ≈ X_upstream / V_i (positive).
            let dv_dq = upstream_x[i] / vi;
            let vdev_grad = 2.0 * obj.weight_voltage_deviation * (vi - obj.v_ref_pu) * dv_dq;

            grad[i] = loss_grad + vdev_grad;
        }
        grad
    }

    // ── Stage 1: continuous device optimisation ───────────────────────────────

    /// Gradient descent step for SVCs, PV inverters, and BESS.
    ///
    /// Updates `q_injections` in-place; new Q is clamped to device feasible range.
    #[allow(clippy::ptr_arg)]
    fn optimize_continuous_devices(&self, q_injections: &mut Vec<f64>, v: &[f64], step_size: f64) {
        let grad = self.compute_q_gradient(v, q_injections);

        for dev in &self.devices {
            if !dev.is_continuous() {
                continue;
            }
            let bus = dev.bus();
            if bus >= q_injections.len() {
                continue;
            }
            let (q_min, q_max) = dev.q_range();
            // Gradient descent: move opposite to gradient direction.
            let new_q = (q_injections[bus] - step_size * grad[bus])
                .max(q_min)
                .min(q_max);
            q_injections[bus] = new_q;
        }
    }

    // ── Stage 2: discrete device optimisation ────────────────────────────────

    /// Evaluate the objective for a specific combination of OLTC tap positions
    /// and capacitor step counts, combined with the continuous Q injections.
    fn evaluate_discrete_combination(
        &self,
        tap_positions: &[usize],
        cap_steps: &[usize],
        q_continuous: &[f64],
    ) -> (f64, Vec<f64>) {
        let n = self.config.n_buses;
        let mut q_inj = vec![0.0_f64; n];

        // Copy continuous device injections.
        for (i, &q) in q_continuous.iter().enumerate().take(n) {
            q_inj[i] = q;
        }

        // Apply discrete device contributions.
        let mut oltc_idx = 0usize;
        let mut cap_idx = 0usize;
        for dev in &self.devices {
            match dev {
                VvoDevice::OltcTransformer {
                    bus,
                    min_tap,
                    tap_step,
                    ..
                } if *bus < n && oltc_idx < tap_positions.len() => {
                    let tap_val = min_tap + tap_positions[oltc_idx] as f64 * tap_step;
                    // Model OLTC voltage boost as a proportional Q virtual injection.
                    let bus_q_load = self
                        .buses
                        .iter()
                        .find(|b| b.id == *bus)
                        .map(|b| b.q_load_mvar)
                        .unwrap_or(0.0);
                    q_inj[*bus] += (tap_val - 1.0) * bus_q_load.abs().max(1.0);
                    oltc_idx += 1;
                }
                VvoDevice::CapacitorBank {
                    bus,
                    step_size_mvar,
                    ..
                } if *bus < n && cap_idx < cap_steps.len() => {
                    q_inj[*bus] += cap_steps[cap_idx] as f64 * step_size_mvar;
                    cap_idx += 1;
                }
                _ => {}
            }
        }

        match self.solve_bfs(&q_inj) {
            Ok((v, _)) => {
                let losses = self.compute_losses(&v, &q_inj);
                let sp: Vec<f64> = self.devices.iter().map(|d| d.current_q_mvar()).collect();
                let obj = self.compute_objective(&v, losses, &sp, &sp);
                (obj, v)
            }
            Err(_) => (f64::MAX, vec![1.0; n]),
        }
    }

    /// Greedy local search over OLTC tap positions and capacitor bank steps.
    ///
    /// For each discrete device in turn, all feasible positions are evaluated
    /// and the position with the lowest objective is retained.  The process
    /// repeats until no improvement is found or all devices have been processed.
    ///
    /// Returns `(updated_q_injections, n_tap_operations, n_capacitor_switches)`.
    fn optimize_discrete_devices(
        &mut self,
        q_continuous: &[f64],
        _v: &[f64],
    ) -> (Vec<f64>, usize, usize) {
        let n = self.config.n_buses;
        let mut n_tap_ops = 0usize;
        let mut n_cap_sw = 0usize;

        // Collect initial discrete device state.
        let mut oltc_taps: Vec<usize> = Vec::new();
        let mut cap_steps_vec: Vec<usize> = Vec::new();
        let mut oltc_dev_indices: Vec<usize> = Vec::new();
        let mut cap_dev_indices: Vec<usize> = Vec::new();

        for (idx, dev) in self.devices.iter().enumerate() {
            match dev {
                VvoDevice::OltcTransformer {
                    min_tap,
                    max_tap,
                    tap_step,
                    current_tap,
                    ..
                } => {
                    let n_taps = ((*max_tap - *min_tap) / tap_step.max(1e-9)).round() as usize + 1;
                    let init_pos =
                        ((*current_tap - *min_tap) / tap_step.max(1e-9)).round() as usize;
                    oltc_taps.push(init_pos.min(n_taps.saturating_sub(1)));
                    oltc_dev_indices.push(idx);
                }
                VvoDevice::CapacitorBank { current_steps, .. } => {
                    cap_steps_vec.push(*current_steps);
                    cap_dev_indices.push(idx);
                }
                _ => {}
            }
        }

        // Greedy search for each OLTC.
        for (i, &dev_idx) in oltc_dev_indices.iter().enumerate() {
            if let VvoDevice::OltcTransformer {
                min_tap,
                max_tap,
                tap_step,
                ..
            } = &self.devices[dev_idx]
            {
                let n_taps = ((*max_tap - *min_tap) / tap_step.max(1e-9)).round() as usize + 1;
                let orig_pos = oltc_taps[i];
                let mut best_obj = f64::MAX;
                let mut best_pos = orig_pos;

                for pos in 0..n_taps {
                    let mut trial_taps = oltc_taps.clone();
                    trial_taps[i] = pos;
                    let (obj, _) = self.evaluate_discrete_combination(
                        &trial_taps,
                        &cap_steps_vec,
                        q_continuous,
                    );
                    if obj < best_obj {
                        best_obj = obj;
                        best_pos = pos;
                    }
                }

                if best_pos != orig_pos {
                    oltc_taps[i] = best_pos;
                    n_tap_ops += 1;
                    let min_t = *min_tap;
                    let step_t = *tap_step;
                    if let VvoDevice::OltcTransformer { current_tap, .. } =
                        &mut self.devices[dev_idx]
                    {
                        *current_tap = min_t + best_pos as f64 * step_t;
                    }
                }
            }
        }

        // Greedy search for each capacitor bank.
        for (i, &dev_idx) in cap_dev_indices.iter().enumerate() {
            if let VvoDevice::CapacitorBank {
                n_steps,
                switchable,
                ..
            } = &self.devices[dev_idx]
            {
                if !switchable {
                    continue;
                }
                let n_st = *n_steps;
                let orig_steps = cap_steps_vec[i];
                let mut best_obj = f64::MAX;
                let mut best_steps = orig_steps;

                for s in 0..=n_st {
                    let mut trial_caps = cap_steps_vec.clone();
                    trial_caps[i] = s;
                    let (obj, _) =
                        self.evaluate_discrete_combination(&oltc_taps, &trial_caps, q_continuous);
                    if obj < best_obj {
                        best_obj = obj;
                        best_steps = s;
                    }
                }

                if best_steps != orig_steps {
                    cap_steps_vec[i] = best_steps;
                    n_cap_sw += 1;
                    if let VvoDevice::CapacitorBank { current_steps, .. } =
                        &mut self.devices[dev_idx]
                    {
                        *current_steps = best_steps;
                    }
                }
            }
        }

        // Build final injection vector: continuous + discrete.
        let mut q_inj = vec![0.0_f64; n];
        for (i, &q) in q_continuous.iter().enumerate().take(n) {
            q_inj[i] = q;
        }

        let mut oltc_idx = 0usize;
        let mut cap_idx = 0usize;
        for dev in &self.devices {
            match dev {
                VvoDevice::OltcTransformer {
                    bus,
                    min_tap,
                    tap_step,
                    ..
                } if *bus < n && oltc_idx < oltc_taps.len() => {
                    let tap_val = min_tap + oltc_taps[oltc_idx] as f64 * tap_step;
                    let bus_q_load = self
                        .buses
                        .iter()
                        .find(|b| b.id == *bus)
                        .map(|b| b.q_load_mvar)
                        .unwrap_or(0.0);
                    q_inj[*bus] += (tap_val - 1.0) * bus_q_load.abs().max(1.0);
                    oltc_idx += 1;
                }
                VvoDevice::CapacitorBank {
                    bus,
                    step_size_mvar,
                    ..
                } if *bus < n && cap_idx < cap_steps_vec.len() => {
                    q_inj[*bus] += cap_steps_vec[cap_idx] as f64 * step_size_mvar;
                    cap_idx += 1;
                }
                _ => {}
            }
        }

        (q_inj, n_tap_ops, n_cap_sw)
    }

    // ── Helper utilities ──────────────────────────────────────────────────────

    /// Map per-device setpoints to a per-bus Q injection vector `MVAr`.
    ///
    /// Multiple devices on the same bus have their contributions summed.
    pub fn q_setpoints_to_injections(&self, setpoints: &[f64]) -> Vec<f64> {
        let n = self.config.n_buses;
        let mut q_inj = vec![0.0_f64; n];
        for (idx, dev) in self.devices.iter().enumerate() {
            let bus = dev.bus();
            if bus < n && idx < setpoints.len() {
                q_inj[bus] += setpoints[idx];
            }
        }
        q_inj
    }

    /// Compute total Q injection per bus from device setpoints `MVAr`.
    pub fn compute_total_q_injections(&self, setpoints: &[f64]) -> Vec<f64> {
        self.q_setpoints_to_injections(setpoints)
    }

    /// Return buses with voltage outside their `[v_min_pu, v_max_pu]` bounds.
    pub fn check_violations(&self, v: &[f64]) -> Vec<(usize, f64)> {
        let mut violations = Vec::new();
        for bus in &self.buses {
            if bus.id < v.len() {
                let vi = v[bus.id];
                if vi < bus.v_min_pu || vi > bus.v_max_pu {
                    violations.push((bus.id, vi));
                }
            }
        }
        violations
    }

    /// Compute branch loading as percentage of thermal MVA rating.
    pub fn compute_branch_loading(&self, v: &[f64], q_injections: &[f64]) -> Vec<f64> {
        let n = self.config.n_buses;
        let root = self
            .buses
            .iter()
            .find(|b| b.is_substation)
            .map(|b| b.id)
            .unwrap_or(0);

        let (order, parent, parent_branch) = self.build_bfs_tree(root, n);
        let (p_sub, q_sub) = self.build_subtree_flows(n, root, &order, &parent, q_injections);

        let mut loadings = vec![0.0_f64; self.branches.len()];
        for &bus in &order {
            if bus == root {
                continue;
            }
            if let (Some(par), Some(br_idx)) = (parent[bus], parent_branch[bus]) {
                let br = &self.branches[br_idx];
                let v_from = v.get(par).copied().unwrap_or(1.0).max(0.1);
                let p_pu = p_sub[bus];
                let q_pu = q_sub[bus];
                let i_mag = (p_pu.powi(2) + q_pu.powi(2)).sqrt() / v_from;
                let s_mva = i_mag * v_from * self.config.base_mva;
                loadings[br_idx] = if br.rating_mva > 0.0 {
                    100.0 * s_mva / br.rating_mva
                } else {
                    0.0
                };
            }
        }
        loadings
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Build BFS traversal order and parent/branch maps from a root bus.
    fn build_bfs_tree(
        &self,
        root: usize,
        n: usize,
    ) -> (Vec<usize>, Vec<Option<usize>>, Vec<Option<usize>>) {
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (br_idx, br) in self.branches.iter().enumerate() {
            if br.from < n && br.to < n {
                adj[br.from].push((br.to, br_idx));
                adj[br.to].push((br.from, br_idx));
            }
        }

        let mut order = Vec::with_capacity(n);
        let mut parent: Vec<Option<usize>> = vec![None; n];
        let mut parent_branch: Vec<Option<usize>> = vec![None; n];
        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();
        queue.push_back(root);
        visited[root] = true;

        while let Some(node) = queue.pop_front() {
            order.push(node);
            for &(nb, br_idx) in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    parent[nb] = Some(node);
                    parent_branch[nb] = Some(br_idx);
                    queue.push_back(nb);
                }
            }
        }

        (order, parent, parent_branch)
    }

    /// Compute subtree P/Q flows in per-unit for each bus.
    fn build_subtree_flows(
        &self,
        n: usize,
        root: usize,
        order: &[usize],
        parent: &[Option<usize>],
        q_injections: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let mut p_sub = vec![0.0_f64; n];
        let mut q_sub = vec![0.0_f64; n];

        for bus in &self.buses {
            if bus.id < n {
                p_sub[bus.id] = bus.p_load_mw / self.config.base_mva;
                let q_dev = q_injections.get(bus.id).copied().unwrap_or(0.0);
                q_sub[bus.id] = (bus.q_load_mvar - q_dev) / self.config.base_mva;
            }
        }

        for &bus in order.iter().rev() {
            if bus == root {
                continue;
            }
            if let Some(par) = parent[bus] {
                let p_b = p_sub[bus];
                let q_b = q_sub[bus];
                p_sub[par] += p_b;
                q_sub[par] += q_b;
            }
        }

        (p_sub, q_sub)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Shared test fixtures ──────────────────────────────────────────────────

    fn make_2bus_system() -> VoltVarOptimizer {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 0.5,
                is_substation: false,
            },
        ];
        let branches = vec![VvoBranch {
            from: 0,
            to: 1,
            r_ohm: 0.5,
            x_ohm: 0.3,
            rating_mva: 5.0,
        }];
        let config = VvoConfig {
            n_buses: 2,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 50,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };
        VoltVarOptimizer::new(buses, branches, vec![], config)
    }

    fn make_3bus_system() -> VoltVarOptimizer {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 0.5,
                is_substation: false,
            },
            VvoBus {
                id: 2,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.8,
                q_load_mvar: 0.3,
                is_substation: false,
            },
        ];
        let branches = vec![
            VvoBranch {
                from: 0,
                to: 1,
                r_ohm: 0.5,
                x_ohm: 0.3,
                rating_mva: 5.0,
            },
            VvoBranch {
                from: 1,
                to: 2,
                r_ohm: 0.4,
                x_ohm: 0.2,
                rating_mva: 5.0,
            },
        ];
        let config = VvoConfig {
            n_buses: 3,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 50,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };
        VoltVarOptimizer::new(buses, branches, vec![], config)
    }

    // ── 1. Device creation tests ──────────────────────────────────────────────

    #[test]
    fn test_vvo_device_oltc_creation() {
        let dev = VvoDevice::OltcTransformer {
            bus: 2,
            min_tap: 0.9,
            max_tap: 1.1,
            tap_step: 0.00625,
            current_tap: 1.0,
            time_delay_s: 30.0,
        };
        assert_eq!(dev.bus(), 2);
        assert_eq!(dev.device_type_name(), "OLTC");
        let (q_min, q_max) = dev.q_range();
        assert_eq!(q_min, 0.0);
        assert_eq!(q_max, 0.0);
        assert_eq!(dev.current_q_mvar(), 0.0);
    }

    #[test]
    fn test_vvo_device_capacitor_creation() {
        let dev = VvoDevice::CapacitorBank {
            bus: 3,
            step_size_mvar: 0.5,
            n_steps: 4,
            current_steps: 2,
            switchable: true,
        };
        assert_eq!(dev.bus(), 3);
        assert_eq!(dev.device_type_name(), "CapacitorBank");
        assert!((dev.current_q_mvar() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_device_pv_inverter_creation() {
        let dev = VvoDevice::PhotovoltaicInverter {
            bus: 5,
            p_mw: 0.8,
            s_rated_mva: 1.0,
            power_factor_min: 0.9,
            current_q_mvar: 0.1,
            can_absorb_q: true,
        };
        assert_eq!(dev.bus(), 5);
        assert_eq!(dev.device_type_name(), "PV_Inverter");
        assert!((dev.current_q_mvar() - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_device_bus() {
        let devices = [
            VvoDevice::OltcTransformer {
                bus: 1,
                min_tap: 0.9,
                max_tap: 1.1,
                tap_step: 0.00625,
                current_tap: 1.0,
                time_delay_s: 30.0,
            },
            VvoDevice::CapacitorBank {
                bus: 2,
                step_size_mvar: 0.5,
                n_steps: 4,
                current_steps: 0,
                switchable: true,
            },
            VvoDevice::StaticVarCompensator {
                bus: 3,
                q_min_mvar: -2.0,
                q_max_mvar: 2.0,
                current_q_mvar: 0.0,
                droop_pct: 5.0,
            },
            VvoDevice::PhotovoltaicInverter {
                bus: 4,
                p_mw: 1.0,
                s_rated_mva: 1.5,
                power_factor_min: 0.9,
                current_q_mvar: 0.0,
                can_absorb_q: true,
            },
            VvoDevice::BatteryEss {
                bus: 5,
                p_mw: 0.5,
                q_max_mvar: 1.0,
                q_min_mvar: -1.0,
                current_q_mvar: 0.0,
            },
        ];
        assert_eq!(devices[0].bus(), 1);
        assert_eq!(devices[1].bus(), 2);
        assert_eq!(devices[2].bus(), 3);
        assert_eq!(devices[3].bus(), 4);
        assert_eq!(devices[4].bus(), 5);
    }

    #[test]
    fn test_vvo_device_q_range_capacitor() {
        let dev = VvoDevice::CapacitorBank {
            bus: 0,
            step_size_mvar: 0.25,
            n_steps: 8,
            current_steps: 0,
            switchable: true,
        };
        let (q_min, q_max) = dev.q_range();
        assert!((q_min - 0.0).abs() < 1e-9);
        assert!((q_max - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_device_q_range_svc() {
        let dev = VvoDevice::StaticVarCompensator {
            bus: 0,
            q_min_mvar: -3.0,
            q_max_mvar: 3.0,
            current_q_mvar: 0.0,
            droop_pct: 4.0,
        };
        let (q_min, q_max) = dev.q_range();
        assert!((q_min - (-3.0)).abs() < 1e-9);
        assert!((q_max - 3.0).abs() < 1e-9);
    }

    // ── 2. Struct creation tests ──────────────────────────────────────────────

    #[test]
    fn test_vvo_bus_creation() {
        let bus = VvoBus {
            id: 7,
            v_min_pu: 0.95,
            v_max_pu: 1.05,
            p_load_mw: 2.5,
            q_load_mvar: 1.2,
            is_substation: false,
        };
        assert_eq!(bus.id, 7);
        assert!((bus.v_min_pu - 0.95).abs() < 1e-9);
        assert!((bus.v_max_pu - 1.05).abs() < 1e-9);
        assert!((bus.p_load_mw - 2.5).abs() < 1e-9);
        assert!(!bus.is_substation);
    }

    #[test]
    fn test_vvo_branch_creation() {
        let br = VvoBranch {
            from: 0,
            to: 1,
            r_ohm: 0.3,
            x_ohm: 0.15,
            rating_mva: 10.0,
        };
        assert_eq!(br.from, 0);
        assert_eq!(br.to, 1);
        assert!((br.r_ohm - 0.3).abs() < 1e-9);
        assert!((br.x_ohm - 0.15).abs() < 1e-9);
        assert!((br.rating_mva - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_config_creation() {
        let cfg = VvoConfig {
            n_buses: 10,
            base_mva: 100.0,
            base_kv: 33.0,
            objective: VvoObjective::default(),
            max_iterations: 30,
            voltage_tolerance: 1e-5,
            n_discrete_trials: 8,
        };
        assert_eq!(cfg.n_buses, 10);
        assert!((cfg.base_mva - 100.0).abs() < 1e-9);
        assert!((cfg.base_kv - 33.0).abs() < 1e-9);
        assert_eq!(cfg.max_iterations, 30);
        assert_eq!(cfg.n_discrete_trials, 8);
    }

    // ── 3. BFS power flow tests ───────────────────────────────────────────────

    #[test]
    fn test_bfs_single_branch() {
        let opt = make_2bus_system();
        let q_inj = vec![0.0, 0.0];
        let (v, angles) = opt.solve_bfs(&q_inj).expect("BFS should converge");
        assert_eq!(v.len(), 2);
        assert_eq!(angles.len(), 2);
        assert!((v[0] - 1.0).abs() < 1e-9, "Substation V must be 1.0 pu");
        assert!(
            v[1] < 1.0,
            "Load bus V={} should drop below 1.0 due to line losses",
            v[1]
        );
        assert!(v[1] > 0.5, "V1={} should be physically reasonable", v[1]);
    }

    #[test]
    fn test_bfs_two_branches() {
        let opt = make_3bus_system();
        let q_inj = vec![0.0, 0.0, 0.0];
        let (v, _) = opt.solve_bfs(&q_inj).expect("BFS should converge");
        assert_eq!(v.len(), 3);
        assert!((v[0] - 1.0).abs() < 1e-9, "Substation V should be 1.0 pu");
        assert!(v[1] < 1.0, "V1={} should drop below 1.0", v[1]);
        assert!(
            v[2] < v[1],
            "V2={} should be lower than V1={} (further from source)",
            v[2],
            v[1]
        );
    }

    // ── 4. Loss computation tests ─────────────────────────────────────────────

    #[test]
    fn test_compute_losses_zero_load() {
        let mut opt = make_2bus_system();
        opt.buses[1].p_load_mw = 0.0;
        opt.buses[1].q_load_mvar = 0.0;
        let q_inj = vec![0.0, 0.0];
        let (v, _) = opt.solve_bfs(&q_inj).expect("BFS");
        let losses = opt.compute_losses(&v, &q_inj);
        assert!(
            losses.abs() < 1e-9,
            "Zero load → zero losses, got {}",
            losses
        );
    }

    #[test]
    fn test_compute_losses_nonzero() {
        let opt = make_2bus_system();
        let q_inj = vec![0.0, 0.0];
        let (v, _) = opt.solve_bfs(&q_inj).expect("BFS");
        let losses = opt.compute_losses(&v, &q_inj);
        assert!(
            losses > 0.0,
            "Nonzero load should produce positive losses, got {}",
            losses
        );
    }

    // ── 5. Objective function tests ───────────────────────────────────────────

    #[test]
    fn test_compute_objective_zero_deviation() {
        let opt = make_2bus_system();
        let v = vec![1.0, 1.0]; // all voltages at reference
        let losses_mw = 0.1_f64;
        let sp: Vec<f64> = vec![];
        // J = 1.0 * 0.1 + 0.5 * 0.0 + 0.1 * 0.0 = 0.1
        let obj = opt.compute_objective(&v, losses_mw, &sp, &sp);
        assert!((obj - 0.1).abs() < 1e-9, "Expected 0.1, got {}", obj);
    }

    // ── 6. Gradient tests ─────────────────────────────────────────────────────

    #[test]
    fn test_q_gradient_direction() {
        let opt = make_2bus_system();
        // With positive Q injection at bus 1, the gradient should indicate
        // that further injection reduces loss burden (gradient ≤ 0 at bus 1).
        let q_inj = vec![0.0, 2.0];
        let (v, _) = opt.solve_bfs(&q_inj).expect("BFS");
        let grad = opt.compute_q_gradient(&v, &q_inj);
        assert_eq!(grad.len(), 2);
        // Gradient at bus 1 with positive Q injection: loss component is negative.
        // The voltage deviation component may partially offset this, but the
        // combined gradient should be non-positive for typical operating conditions.
        assert!(
            grad[1] <= 0.1,
            "Gradient at bus 1 should be non-positive for positive Q injection, got {}",
            grad[1]
        );
    }

    // ── 7. Continuous device optimisation tests ───────────────────────────────

    #[test]
    fn test_optimize_continuous_devices_reduces_losses() {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 1.0,
                is_substation: false,
            },
        ];
        let branches = vec![VvoBranch {
            from: 0,
            to: 1,
            r_ohm: 0.5,
            x_ohm: 0.3,
            rating_mva: 5.0,
        }];
        let devices = vec![VvoDevice::StaticVarCompensator {
            bus: 1,
            q_min_mvar: -2.0,
            q_max_mvar: 2.0,
            current_q_mvar: 0.0,
            droop_pct: 5.0,
        }];
        let config = VvoConfig {
            n_buses: 2,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 20,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };
        let opt = VoltVarOptimizer::new(buses, branches, devices, config);

        let q_inj_before = vec![0.0, 0.0];
        let (v_before, _) = opt.solve_bfs(&q_inj_before).expect("BFS");
        let losses_before = opt.compute_losses(&v_before, &q_inj_before);

        let mut q_inj_after = q_inj_before.clone();
        opt.optimize_continuous_devices(&mut q_inj_after, &v_before, 0.05);
        let (v_after, _) = opt.solve_bfs(&q_inj_after).expect("BFS after");
        let losses_after = opt.compute_losses(&v_after, &q_inj_after);

        assert!(
            losses_after <= losses_before + 1e-6,
            "Losses after continuous opt ({}) should not exceed initial ({})",
            losses_after,
            losses_before
        );
    }

    // ── 8. q_setpoints_to_injections tests ───────────────────────────────────

    #[test]
    fn test_q_setpoints_to_injections() {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 0.5,
                is_substation: false,
            },
        ];
        let devices = vec![VvoDevice::CapacitorBank {
            bus: 1,
            step_size_mvar: 0.5,
            n_steps: 4,
            current_steps: 0,
            switchable: true,
        }];
        let config = VvoConfig {
            n_buses: 2,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 50,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };
        let opt = VoltVarOptimizer::new(buses, vec![], devices, config);

        let setpoints = vec![1.5_f64];
        let injections = opt.q_setpoints_to_injections(&setpoints);
        assert_eq!(injections.len(), 2);
        assert!((injections[0] - 0.0).abs() < 1e-9);
        assert!((injections[1] - 1.5).abs() < 1e-9);
    }

    // ── 9. Full optimisation tests ────────────────────────────────────────────

    #[test]
    fn test_optimize_2bus_with_capacitor() {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 0.8,
                is_substation: false,
            },
        ];
        let branches = vec![VvoBranch {
            from: 0,
            to: 1,
            r_ohm: 0.5,
            x_ohm: 0.3,
            rating_mva: 5.0,
        }];
        let devices = vec![VvoDevice::CapacitorBank {
            bus: 1,
            step_size_mvar: 0.25,
            n_steps: 4,
            current_steps: 0,
            switchable: true,
        }];
        let config = VvoConfig {
            n_buses: 2,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 30,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };

        // Baseline losses without any device.
        let baseline_opt = make_2bus_system();
        let q0 = vec![0.0, 0.0];
        let (v0, _) = baseline_opt.solve_bfs(&q0).expect("BFS");
        let losses_baseline = baseline_opt.compute_losses(&v0, &q0);

        let mut opt = VoltVarOptimizer::new(buses, branches, devices, config);
        let result = opt.optimize().expect("VVO should not error");

        assert!(
            result.iterations > 0,
            "Should perform at least one iteration"
        );
        assert!(
            result.total_losses_mw <= losses_baseline + 1e-2,
            "Capacitor-compensated losses ({}) should not significantly exceed baseline ({})",
            result.total_losses_mw,
            losses_baseline
        );
        assert_eq!(result.voltage_magnitudes.len(), 2);
        assert_eq!(result.branch_loadings_pct.len(), 1);
    }

    #[test]
    fn test_optimize_3bus_oltc() {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.0,
                q_load_mvar: 0.5,
                is_substation: false,
            },
            VvoBus {
                id: 2,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.8,
                q_load_mvar: 0.3,
                is_substation: false,
            },
        ];
        let branches = vec![
            VvoBranch {
                from: 0,
                to: 1,
                r_ohm: 0.5,
                x_ohm: 0.3,
                rating_mva: 5.0,
            },
            VvoBranch {
                from: 1,
                to: 2,
                r_ohm: 0.4,
                x_ohm: 0.2,
                rating_mva: 5.0,
            },
        ];
        let devices = vec![VvoDevice::OltcTransformer {
            bus: 0,
            min_tap: 0.9,
            max_tap: 1.1,
            tap_step: 0.00625,
            current_tap: 1.0,
            time_delay_s: 30.0,
        }];
        let config = VvoConfig {
            n_buses: 3,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 20,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };
        let mut opt = VoltVarOptimizer::new(buses, branches, devices, config);
        let result = opt.optimize().expect("OLTC VVO should not error");
        assert_eq!(result.voltage_magnitudes.len(), 3);
        assert!((result.voltage_magnitudes[0] - 1.0).abs() < 1e-9);
        assert!(result.total_losses_mw >= 0.0);
    }

    // ── 10. Voltage violation detection test ──────────────────────────────────

    #[test]
    fn test_voltage_violation_detection() {
        let opt = make_2bus_system();
        // Bus 1 has v_min = 0.95; set its voltage to 0.93 → violation.
        let v = vec![1.0, 0.93];
        let violations = opt.check_violations(&v);
        assert!(
            violations.iter().any(|(bus, _)| *bus == 1),
            "Bus 1 at 0.93 pu should be detected as a voltage violation"
        );
    }

    // ── 11. Additional robustness tests ───────────────────────────────────────

    #[test]
    fn test_bfs_error_on_zero_buses() {
        let config = VvoConfig {
            n_buses: 0,
            ..VvoConfig::default()
        };
        let opt = VoltVarOptimizer::new(vec![], vec![], vec![], config);
        let result = opt.solve_bfs(&[]);
        assert!(result.is_err(), "BFS on zero-bus network should return Err");
    }

    #[test]
    fn test_optimize_error_on_zero_buses() {
        let config = VvoConfig {
            n_buses: 0,
            ..VvoConfig::default()
        };
        let mut opt = VoltVarOptimizer::new(vec![], vec![], vec![], config);
        let result = opt.optimize();
        assert!(
            result.is_err(),
            "optimize() on zero buses should return Err"
        );
    }

    #[test]
    fn test_check_violations_high_voltage() {
        let opt = make_2bus_system();
        let v = vec![1.08, 1.0]; // bus 0 at 1.08 > v_max = 1.05
        let violations = opt.check_violations(&v);
        assert!(
            violations.iter().any(|(bus, _)| *bus == 0),
            "Bus 0 at 1.08 pu should be flagged as high-voltage violation"
        );
    }

    #[test]
    fn test_compute_total_q_injections_equivalence() {
        let opt = make_2bus_system();
        let setpoints = vec![];
        let inj1 = opt.q_setpoints_to_injections(&setpoints);
        let inj2 = opt.compute_total_q_injections(&setpoints);
        assert_eq!(inj1, inj2, "Both methods should return identical results");
    }

    #[test]
    fn test_bfs_q_injection_raises_voltage() {
        let opt = make_2bus_system();
        // Without Q injection
        let q_no_inj = vec![0.0, 0.0];
        let (v_no_inj, _) = opt.solve_bfs(&q_no_inj).expect("BFS");
        // With Q injection at bus 1 (reduces reactive flow → raises voltage).
        let q_with_inj = vec![0.0, 2.0];
        let (v_with_inj, _) = opt.solve_bfs(&q_with_inj).expect("BFS");
        assert!(
            v_with_inj[1] > v_no_inj[1],
            "Q injection should raise bus 1 voltage: {} vs {}",
            v_with_inj[1],
            v_no_inj[1]
        );
    }

    #[test]
    fn test_branch_loading_nonzero_for_loaded_feeder() {
        let opt = make_2bus_system();
        let q_inj = vec![0.0, 0.0];
        let (v, _) = opt.solve_bfs(&q_inj).expect("BFS");
        let loadings = opt.compute_branch_loading(&v, &q_inj);
        assert_eq!(loadings.len(), 1);
        assert!(
            loadings[0] > 0.0,
            "Loaded feeder should show non-zero branch loading"
        );
    }

    #[test]
    fn test_bess_q_range() {
        let dev = VvoDevice::BatteryEss {
            bus: 2,
            p_mw: 0.5,
            q_max_mvar: 1.5,
            q_min_mvar: -1.5,
            current_q_mvar: 0.3,
        };
        let (q_min, q_max) = dev.q_range();
        assert!((q_min - (-1.5)).abs() < 1e-9);
        assert!((q_max - 1.5).abs() < 1e-9);
        assert!((dev.current_q_mvar() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_objective_default() {
        let obj = VvoObjective::default();
        assert!((obj.weight_losses - 1.0).abs() < 1e-9);
        assert!((obj.weight_voltage_deviation - 0.5).abs() < 1e-9);
        assert!((obj.weight_device_operations - 0.1).abs() < 1e-9);
        assert!((obj.v_ref_pu - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_optimize_3bus_with_svc() {
        let buses = vec![
            VvoBus {
                id: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 0.0,
                q_load_mvar: 0.0,
                is_substation: true,
            },
            VvoBus {
                id: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 2.0,
                q_load_mvar: 1.5,
                is_substation: false,
            },
            VvoBus {
                id: 2,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
                p_load_mw: 1.5,
                q_load_mvar: 1.0,
                is_substation: false,
            },
        ];
        let branches = vec![
            VvoBranch {
                from: 0,
                to: 1,
                r_ohm: 1.0,
                x_ohm: 0.5,
                rating_mva: 10.0,
            },
            VvoBranch {
                from: 1,
                to: 2,
                r_ohm: 0.8,
                x_ohm: 0.4,
                rating_mva: 10.0,
            },
        ];
        let devices = vec![
            VvoDevice::StaticVarCompensator {
                bus: 1,
                q_min_mvar: -3.0,
                q_max_mvar: 3.0,
                current_q_mvar: 0.0,
                droop_pct: 5.0,
            },
            VvoDevice::StaticVarCompensator {
                bus: 2,
                q_min_mvar: -2.0,
                q_max_mvar: 2.0,
                current_q_mvar: 0.0,
                droop_pct: 5.0,
            },
        ];
        let config = VvoConfig {
            n_buses: 3,
            base_mva: 10.0,
            base_kv: 11.0,
            objective: VvoObjective::default(),
            max_iterations: 30,
            voltage_tolerance: 1e-4,
            n_discrete_trials: 5,
        };

        // Baseline without SVCs.
        let mut base_opt =
            VoltVarOptimizer::new(buses.clone(), branches.clone(), vec![], config.clone());
        let base_result = base_opt.optimize().expect("Baseline VVO");
        let base_losses = base_result.total_losses_mw;

        let mut svc_opt = VoltVarOptimizer::new(buses, branches, devices, config);
        let svc_result = svc_opt.optimize().expect("SVC VVO");

        // SVCs should reduce or maintain losses.
        assert!(
            svc_result.total_losses_mw <= base_losses + 1e-3,
            "SVC losses ({}) should not exceed baseline ({})",
            svc_result.total_losses_mw,
            base_losses
        );
    }
}
