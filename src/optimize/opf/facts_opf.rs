//! FACTS-Augmented DC Optimal Power Flow.
//!
//! Extends the standard DC-OPF with Flexible AC Transmission System (FACTS) devices:
//! - **STATCOM**: reactive power injection at buses (voltage magnitude support)
//! - **SVC**: variable shunt susceptance at buses (reactive compensation)
//! - **TCSC**: thyristor-controlled series compensator (variable branch reactance)
//!
//! # Algorithm
//! 1. Run standard DC-OPF (lambda-iteration) for real power dispatch.
//! 2. Assess branch flow overloads from the DC solution.
//! 3. For TCSC branches: reduce reactance to relieve overloads.
//! 4. For STATCOM buses: inject reactive power to pull flat-start voltages toward 1.0 pu.
//! 5. For SVC buses: set variable susceptance to correct residual voltage deviation.
//! 6. Estimate corrected voltage magnitudes via linearised ΔV ≈ ΔQ / B_self.
//!
//! The DC-OPF portion is exact under the DC approximation; the reactive/voltage
//! corrections are first-order estimates suitable for planning studies.
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{solve_dc_opf, GenCost};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Public configuration
// ---------------------------------------------------------------------------

/// Configuration for FACTS-augmented DC-OPF.
#[derive(Debug, Clone)]
pub struct FactsOpfConfig {
    /// Bus indices (0-based internal) with STATCOM devices.
    pub statcoms: Vec<usize>,
    /// Bus indices (0-based internal) with SVC devices.
    pub svcs: Vec<usize>,
    /// Branch indices (0-based) with TCSC devices.
    pub tcsc_branches: Vec<usize>,
    /// Cost of reactive dispatch ($/MVAr/h) — added to total cost. Default 0.01.
    pub q_cost: f64,
    /// Maximum reactive injection per STATCOM device (MVAr).
    pub statcom_q_max: f64,
    /// Maximum variable susceptance magnitude per SVC [p.u.].
    pub svc_b_max: f64,
    /// Maximum fractional reactance change per TCSC (e.g. 0.2 = ±20% of nominal).
    pub tcsc_delta_x_max: f64,
}

impl Default for FactsOpfConfig {
    fn default() -> Self {
        Self {
            statcoms: Vec::new(),
            svcs: Vec::new(),
            tcsc_branches: Vec::new(),
            q_cost: 0.01,
            statcom_q_max: 50.0,
            svc_b_max: 0.5,
            tcsc_delta_x_max: 0.2,
        }
    }
}

// ---------------------------------------------------------------------------
// Result type
// ---------------------------------------------------------------------------

/// Result of a FACTS-augmented DC-OPF solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FactsOpfResult {
    /// Real power dispatch per generator (MW) (same order as network.generators).
    pub gen_dispatch: Vec<f64>,
    /// Reactive injection per STATCOM device (MVAr) (positive = capacitive injection).
    pub statcom_q: Vec<f64>,
    /// Variable susceptance per SVC device [p.u.] (positive = capacitive).
    pub svc_b: Vec<f64>,
    /// Effective series reactance per TCSC branch [p.u.] (after TCSC adjustment).
    pub tcsc_x: Vec<f64>,
    /// Total cost [$/h] — generation cost plus reactive correction cost.
    pub total_cost: f64,
    /// Estimated bus voltage magnitudes [p.u.] after reactive correction.
    pub voltage_magnitudes: Vec<f64>,
    /// True if the DC power flow converged.
    pub converged: bool,
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Solve the FACTS-augmented DC-OPF for `network`.
///
/// `gen_costs` must match `network.generators` in order.
///
/// # Errors
/// - [`OxiGridError::InvalidParameter`] for out-of-range bus/branch indices
///   or if the underlying DC-OPF fails.
pub fn solve_facts_opf(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    config: &FactsOpfConfig,
) -> Result<FactsOpfResult> {
    let n_bus = network.buses.len();
    let n_branch = network.branches.len();

    // --- Validate device placement indices ---------------------------------
    for &bi in &config.statcoms {
        if bi >= n_bus {
            return Err(OxiGridError::InvalidParameter(format!(
                "STATCOM bus index {bi} out of range (n_bus={n_bus})"
            )));
        }
    }
    for &bi in &config.svcs {
        if bi >= n_bus {
            return Err(OxiGridError::InvalidParameter(format!(
                "SVC bus index {bi} out of range (n_bus={n_bus})"
            )));
        }
    }
    for &br in &config.tcsc_branches {
        if br >= n_branch {
            return Err(OxiGridError::InvalidParameter(format!(
                "TCSC branch index {br} out of range (n_branch={n_branch})"
            )));
        }
    }

    // --- Step 1: DC-OPF for real power dispatch ----------------------------
    let dc_result = solve_dc_opf(network, gen_costs)?;

    // --- Step 2: TCSC — reactance adjustment for overload relief -----------
    // For each TCSC branch: if |flow| > rate_a, reduce reactance by up to
    // tcsc_delta_x_max fraction to lower impedance and thus redistribute flow.
    let mut tcsc_x: Vec<f64> = config
        .tcsc_branches
        .iter()
        .map(|&br| network.branches[br].x)
        .collect();

    for (k, &br_idx) in config.tcsc_branches.iter().enumerate() {
        let branch = &network.branches[br_idx];
        let flow_mw = dc_result
            .branch_flows_mw
            .get(br_idx)
            .copied()
            .unwrap_or(0.0);
        let rate_a = branch.rate_a;
        // Only act if line is actually rated (non-zero limit) and overloaded
        if rate_a > 1e-6 && flow_mw.abs() > rate_a {
            let overload_ratio = (flow_mw.abs() - rate_a) / rate_a;
            // Scale the reactance reduction by overload severity, clamp to max
            let delta_frac =
                (overload_ratio * config.tcsc_delta_x_max).min(config.tcsc_delta_x_max);
            // Minimum reactance = 10 % of nominal to avoid singularity
            tcsc_x[k] = (branch.x * (1.0 - delta_frac)).max(branch.x * 0.1);
        }
        // Not overloaded → nominal reactance (already initialised above)
    }

    // --- Step 3: Self-susceptance estimate per bus (for voltage correction) --
    // B_self(i) ≈ Σ_{branches incident to i} 1/x_branch
    // Used for the linearised voltage correction ΔV ≈ ΔQ / (V · B_self).
    let mut b_self: Vec<f64> = vec![0.0; n_bus];
    for branch in &network.branches {
        if !branch.status {
            continue;
        }
        let i = network.bus_index(branch.from_bus).unwrap_or(usize::MAX);
        let j = network.bus_index(branch.to_bus).unwrap_or(usize::MAX);
        let b_series = if branch.x.abs() > 1e-12 {
            1.0 / branch.x
        } else {
            0.0
        };
        if i < n_bus {
            b_self[i] += b_series;
        }
        if j < n_bus {
            b_self[j] += b_series;
        }
    }

    // Flat-start voltages: all 1.0 pu
    let mut v_mag: Vec<f64> = vec![1.0; n_bus];

    // --- Step 4: STATCOM — reactive injection to push V toward 1.0 pu ------
    // Q_inject = (1.0 - V) · B_self · base_mva  (first-order linearisation)
    // ΔV = Q_inject / (V · B_self · base_mva)  ≈  1.0 - V
    let mut statcom_q: Vec<f64> = Vec::with_capacity(config.statcoms.len());
    for &bus_idx in &config.statcoms {
        let v_dev = 1.0 - v_mag[bus_idx]; // positive → under-voltage
        let q_needed = v_dev * b_self[bus_idx] * network.base_mva;
        let q_inject = q_needed.clamp(-config.statcom_q_max, config.statcom_q_max);
        statcom_q.push(q_inject);
        // Apply corrected voltage estimate
        let dv = if b_self[bus_idx] > 1e-12 {
            q_inject / (network.base_mva * b_self[bus_idx])
        } else {
            0.0
        };
        v_mag[bus_idx] = (v_mag[bus_idx] + dv).clamp(0.9, 1.1);
    }

    // --- Step 5: SVC — variable susceptance to correct residual deviation ---
    // B_svc such that Q = V² · B_svc · base_mva ≈ B_svc · base_mva
    // ΔV ≈ B_svc / B_self
    let mut svc_b: Vec<f64> = Vec::with_capacity(config.svcs.len());
    for &bus_idx in &config.svcs {
        let v_dev = 1.0 - v_mag[bus_idx];
        let b_needed = v_dev * b_self[bus_idx];
        let b_svc = b_needed.clamp(-config.svc_b_max, config.svc_b_max);
        svc_b.push(b_svc);
        let dv = if b_self[bus_idx] > 1e-12 {
            b_svc / b_self[bus_idx]
        } else {
            0.0
        };
        v_mag[bus_idx] = (v_mag[bus_idx] + dv).clamp(0.9, 1.1);
    }

    // --- Step 6: Compute FACTS reactive cost and total cost ----------------
    let q_cost_total: f64 = statcom_q
        .iter()
        .map(|q| config.q_cost * q.abs())
        .sum::<f64>()
        + svc_b
            .iter()
            .map(|b| config.q_cost * b.abs() * network.base_mva)
            .sum::<f64>();

    Ok(FactsOpfResult {
        gen_dispatch: dc_result.p_gen_mw,
        statcom_q,
        svc_b,
        tcsc_x,
        total_cost: dc_result.total_cost + q_cost_total,
        voltage_magnitudes: v_mag,
        converged: true,
    })
}

// ===========================================================================
// Self-contained FACTS-OPF problem (FactsOpfProblem API)
// ===========================================================================

// ─── Enumerations ─────────────────────────────────────────────────────────────

/// Type of FACTS (Flexible AC Transmission System) device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FactsDeviceType {
    /// Thyristor-Controlled Series Compensator — varies series reactance.
    Tcsc,
    /// Static Var Compensator — shunt reactive power injection.
    Svc,
    /// Unified Power Flow Controller — combined series + shunt control.
    Upfc,
    /// Static Synchronous Compensator — voltage-source shunt device.
    Statcom,
    /// Static Synchronous Series Compensator — voltage-source series device.
    Sssc,
    /// Interline Power Flow Controller — controls multiple transmission lines.
    Ipfc,
}

/// Primary control objective of a FACTS device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FactsControlObjective {
    /// Relieve thermal overloads on transmission corridors.
    CongestionRelief,
    /// Minimise I²R losses across the network.
    LossMinimization,
    /// Regulate bus voltage magnitude.
    VoltageSupport,
    /// Track a pre-specified inter-area power schedule.
    PowerScheduling,
    /// Damp inter-area or local oscillation modes.
    OscillationDamping,
}

// ─── FACTS Device ─────────────────────────────────────────────────────────────

/// A single FACTS device with location, ratings, and control settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FactsDevice {
    /// Unique device identifier (0-indexed).
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Physical device type.
    pub device_type: FactsDeviceType,
    /// From-bus index (0-indexed).
    pub from_bus: usize,
    /// To-bus index (0-indexed).
    pub to_bus: usize,
    /// Apparent power rating [MVA].
    pub rating_mva: f64,
    /// Minimum reactance setting [pu].  Capacitive compensation → negative.
    pub x_min_pu: f64,
    /// Maximum reactance setting [pu].
    pub x_max_pu: f64,
    /// Current reactance setting [pu].
    pub current_x_pu: f64,
    /// Primary control objective.
    pub control_objective: FactsControlObjective,
    /// Capital cost [M USD].
    pub capital_cost_musd: f64,
    /// Operating cost [USD/h].
    pub operating_cost_usd_per_h: f64,
}

impl FactsDevice {
    /// Create a new FACTS device with sensible defaults.
    pub fn new_device(
        id: usize,
        name: impl Into<String>,
        device_type: FactsDeviceType,
        from_bus: usize,
        to_bus: usize,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            device_type,
            from_bus,
            to_bus,
            rating_mva: 100.0,
            x_min_pu: -0.5,
            x_max_pu: 0.0,
            current_x_pu: 0.0,
            control_objective: FactsControlObjective::CongestionRelief,
            capital_cost_musd: 0.0,
            operating_cost_usd_per_h: 0.0,
        }
    }

    /// Clamp `x` to the device's [x_min, x_max] range.
    #[inline]
    pub fn clamp_x(&self, x: f64) -> f64 {
        x.max(self.x_min_pu).min(self.x_max_pu)
    }

    /// Effective susceptance contributed by this device as a series element.
    /// For non-series devices (SVC, STATCOM) this is 0.
    pub fn series_susceptance_delta(&self) -> f64 {
        match self.device_type {
            FactsDeviceType::Tcsc
            | FactsDeviceType::Sssc
            | FactsDeviceType::Upfc
            | FactsDeviceType::Ipfc => {
                // current_x_pu is a correction to branch reactance; the delta
                // susceptance is handled in DcBranchData::effective_b
                0.0
            }
            _ => 0.0,
        }
    }
}

// ─── DC Bus / Branch / Generator ──────────────────────────────────────────────

/// DC bus data for the self-contained FACTS-OPF problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcBusData {
    /// Bus index (0-indexed).
    pub id: usize,
    /// Active generation [MW].
    pub p_gen_mw: f64,
    /// Active load [MW].
    pub p_load_mw: f64,
    /// Shunt susceptance [pu].
    pub b_shunt_pu: f64,
    /// True if this is the reference (slack) bus.
    pub is_slack: bool,
    /// Computed voltage angle [rad] (output).
    pub angle_rad: f64,
}

impl DcBusData {
    /// Construct a load bus (no generation, no shunt).
    pub fn load_bus(id: usize, p_load_mw: f64) -> Self {
        Self {
            id,
            p_gen_mw: 0.0,
            p_load_mw,
            b_shunt_pu: 0.0,
            is_slack: false,
            angle_rad: 0.0,
        }
    }

    /// Construct a generator bus.
    pub fn gen_bus(id: usize, p_gen_mw: f64, p_load_mw: f64) -> Self {
        Self {
            id,
            p_gen_mw,
            p_load_mw,
            b_shunt_pu: 0.0,
            is_slack: false,
            angle_rad: 0.0,
        }
    }

    /// Construct the slack bus.
    pub fn slack_bus(id: usize) -> Self {
        Self {
            id,
            p_gen_mw: 0.0,
            p_load_mw: 0.0,
            b_shunt_pu: 0.0,
            is_slack: true,
            angle_rad: 0.0,
        }
    }
}

/// DC branch data for the self-contained FACTS-OPF problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcBranchData {
    /// Branch index (0-indexed).
    pub id: usize,
    /// From-bus index.
    pub from_bus: usize,
    /// To-bus index.
    pub to_bus: usize,
    /// Series reactance [pu].
    pub x_pu: f64,
    /// Series susceptance [pu] = 1 / x_pu.
    pub b_pu: f64,
    /// Thermal rating [MW].
    pub rating_mw: f64,
    /// Optional FACTS device ID installed on this branch.
    pub facts_device_id: Option<usize>,
}

impl DcBranchData {
    /// Construct a branch from reactance and rating; susceptance is derived.
    pub fn new_branch(
        id: usize,
        from_bus: usize,
        to_bus: usize,
        x_pu: f64,
        rating_mw: f64,
    ) -> Self {
        let b_pu = if x_pu.abs() > 1e-12 { 1.0 / x_pu } else { 1e6 };
        Self {
            id,
            from_bus,
            to_bus,
            x_pu,
            b_pu,
            rating_mw,
            facts_device_id: None,
        }
    }

    /// Effective susceptance including any installed FACTS device.
    pub fn effective_b(&self, devices: &[FactsDevice]) -> f64 {
        if let Some(dev_id) = self.facts_device_id {
            if let Some(dev) = devices.iter().find(|d| d.id == dev_id) {
                let x_eff = self.x_pu + dev.current_x_pu;
                return if x_eff.abs() > 1e-12 {
                    1.0 / x_eff
                } else {
                    1e6
                };
            }
        }
        self.b_pu
    }
}

/// Generator data for economic dispatch within the self-contained FACTS-OPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorData {
    /// Generator index (0-indexed).
    pub id: usize,
    /// Bus to which this generator is connected.
    pub bus_id: usize,
    /// Minimum active output [MW].
    pub p_min_mw: f64,
    /// Maximum active output [MW].
    pub p_max_mw: f64,
    /// No-load (constant) cost coefficient [USD/h].
    pub cost_a_usd_per_h: f64,
    /// Linear cost coefficient [USD/MWh].
    pub cost_b_usd_per_mwh: f64,
    /// Quadratic cost coefficient [USD/MW²h].
    pub cost_c_usd_per_mw2h: f64,
}

impl GeneratorData {
    /// Total cost at dispatch level `p` [USD/h].
    pub fn total_cost_at(&self, p: f64) -> f64 {
        self.cost_a_usd_per_h + self.cost_b_usd_per_mwh * p + self.cost_c_usd_per_mw2h * p * p
    }

    /// Marginal cost at dispatch level `p` [USD/MWh].
    pub fn marginal_cost_at(&self, p: f64) -> f64 {
        self.cost_b_usd_per_mwh + 2.0 * self.cost_c_usd_per_mw2h * p
    }

    /// Optimal dispatch at system lambda (unconstrained then clamped).
    pub fn optimal_dispatch_at(&self, lambda: f64) -> f64 {
        let p = if self.cost_c_usd_per_mw2h.abs() > 1e-12 {
            (lambda - self.cost_b_usd_per_mwh) / (2.0 * self.cost_c_usd_per_mw2h)
        } else if lambda >= self.cost_b_usd_per_mwh {
            self.p_max_mw
        } else {
            self.p_min_mw
        };
        p.max(self.p_min_mw).min(self.p_max_mw)
    }
}

// ─── FACTS-OPF Result ─────────────────────────────────────────────────────────

/// Solution of a self-contained FACTS-OPF problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FactsOpfProblemResult {
    /// Optimal dispatch per generator: `(gen_id, MW)`.
    pub generator_dispatch: Vec<(usize, f64)>,
    /// Optimal FACTS settings: `(device_id, x_pu)`.
    pub facts_settings: Vec<(usize, f64)>,
    /// Voltage angles per bus [rad].
    pub bus_angles_rad: Vec<f64>,
    /// Branch active power flows [MW].
    pub branch_flows_mw: Vec<f64>,
    /// Branch loading as percentage of rating [%].
    pub branch_loading_pct: Vec<f64>,
    /// Total generation cost [USD/h].
    pub total_generation_cost_usd: f64,
    /// Total FACTS operating cost [USD/h].
    pub total_facts_cost_usd: f64,
    /// Number of branches exceeding 95 % of thermal rating.
    pub n_congested_branches: usize,
    /// Total network losses [MW] (0 for lossless DC approximation).
    pub losses_mw: f64,
    /// Whether the FACTS iteration converged.
    pub converged: bool,
    /// Number of outer FACTS iterations performed.
    pub iterations: usize,
}

// ─── Main Problem Struct ───────────────────────────────────────────────────────

/// Self-contained FACTS-Augmented DC Optimal Power Flow problem.
///
/// Builds its own DC network representation and iteratively optimises generator
/// dispatch and FACTS device settings for congestion management.
///
/// # Example
/// ```no_run
/// # use oxigrid::optimize::opf::facts_opf::{
/// #     FactsOpfProblem, DcBusData, DcBranchData, GeneratorData
/// # };
/// let buses = vec![DcBusData::slack_bus(0), DcBusData::load_bus(1, 100.0)];
/// let branches = vec![DcBranchData::new_branch(0, 0, 1, 0.1, 150.0)];
/// let gens = vec![GeneratorData {
///     id: 0, bus_id: 0,
///     p_min_mw: 0.0, p_max_mw: 200.0,
///     cost_a_usd_per_h: 0.0, cost_b_usd_per_mwh: 20.0,
///     cost_c_usd_per_mw2h: 0.01,
/// }];
/// let problem = FactsOpfProblem::new(buses, branches, gens);
/// let result = problem.solve_base_opf();
/// ```
#[derive(Debug, Clone)]
pub struct FactsOpfProblem {
    /// Bus data (must include exactly one slack bus at index 0 in typical use).
    pub buses: Vec<DcBusData>,
    /// Branch data.
    pub branches: Vec<DcBranchData>,
    /// Generator data.
    pub generators: Vec<GeneratorData>,
    /// Installed FACTS devices.
    pub facts_devices: Vec<FactsDevice>,
    /// System base MVA (default 100.0).
    pub base_mva: f64,
    /// Maximum outer FACTS iterations (default 50).
    pub max_iterations: usize,
}

impl FactsOpfProblem {
    /// Create a new FACTS-OPF problem from network data.
    pub fn new(
        buses: Vec<DcBusData>,
        branches: Vec<DcBranchData>,
        generators: Vec<GeneratorData>,
    ) -> Self {
        Self {
            buses,
            branches,
            generators,
            facts_devices: Vec::new(),
            base_mva: 100.0,
            max_iterations: 50,
        }
    }

    /// Add a FACTS device and wire it to the matching branch (by from/to bus).
    pub fn add_facts_device(&mut self, device: FactsDevice) {
        for branch in &mut self.branches {
            let matches = (branch.from_bus == device.from_bus && branch.to_bus == device.to_bus)
                || (branch.from_bus == device.to_bus && branch.to_bus == device.from_bus);
            if matches {
                branch.facts_device_id = Some(device.id);
                break;
            }
        }
        self.facts_devices.push(device);
    }

    // ── B-matrix ──────────────────────────────────────────────────────────────

    /// Build the DC B-matrix (n × n nodal susceptance matrix).
    ///
    /// `B[i][i] = Σ b_ij` (sum of susceptances incident to bus *i*).
    /// `B[i][j] = -b_ij` for i ≠ j.
    ///
    /// FACTS devices modify the effective branch susceptance via `current_x_pu`.
    pub fn build_b_matrix(&self) -> Vec<Vec<f64>> {
        let n = self.buses.len();
        let mut b = vec![vec![0.0_f64; n]; n];

        for branch in &self.branches {
            let bij = branch.effective_b(&self.facts_devices);
            let i = branch.from_bus;
            let j = branch.to_bus;
            if i < n && j < n {
                b[i][i] += bij;
                b[j][j] += bij;
                b[i][j] -= bij;
                b[j][i] -= bij;
            }
        }
        // Add shunt susceptances on diagonal
        for bus in &self.buses {
            if bus.id < n {
                b[bus.id][bus.id] += bus.b_shunt_pu;
            }
        }
        b
    }

    // ── DC power flow ─────────────────────────────────────────────────────────

    /// Solve the DC power flow via Gaussian elimination with partial pivoting.
    ///
    /// The slack bus (index 0) angle is fixed to 0.  Returns the full angle
    /// vector [rad].
    ///
    /// * `b_matrix`    — Full n × n B-matrix (from [`Self::build_b_matrix`]).
    /// * `p_injections`— Net power injection per bus [pu] = (P_gen − P_load) / base_mva.
    pub fn solve_dc_pf(b_matrix: &[Vec<f64>], p_injections: &[f64]) -> Vec<f64> {
        let n = b_matrix.len();
        if n == 0 {
            return Vec::new();
        }
        let slack_idx = 0_usize;
        let nr = n - 1;
        if nr == 0 {
            return vec![0.0; n];
        }
        let full_indices: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();
        let mut a = vec![vec![0.0_f64; nr]; nr];
        let mut rhs = vec![0.0_f64; nr];
        for (ri, &fi) in full_indices.iter().enumerate() {
            rhs[ri] = p_injections[fi];
            for (rj, &fj) in full_indices.iter().enumerate() {
                a[ri][rj] = b_matrix[fi][fj];
            }
        }
        let theta_reduced = facts_gaussian_elimination(&a, &rhs);
        let mut angles = vec![0.0_f64; n];
        for (ri, &fi) in full_indices.iter().enumerate() {
            angles[fi] = theta_reduced[ri];
        }
        angles
    }

    // ── Branch flows ──────────────────────────────────────────────────────────

    /// Compute branch active power flows from bus angles.
    ///
    /// `P_ij = (θ_i − θ_j) × b_ij_eff × base_mva`  [MW]
    pub fn compute_branch_flows(&self, angles: &[f64]) -> Vec<f64> {
        self.branches
            .iter()
            .map(|br| {
                let theta_i = angles.get(br.from_bus).copied().unwrap_or(0.0);
                let theta_j = angles.get(br.to_bus).copied().unwrap_or(0.0);
                let bij = br.effective_b(&self.facts_devices);
                (theta_i - theta_j) * bij * self.base_mva
            })
            .collect()
    }

    // ── PTDF matrix ───────────────────────────────────────────────────────────

    /// Compute Power Transfer Distribution Factor (PTDF) matrix.
    ///
    /// Dimensions: `n_branch × n_bus`.
    /// `PTDF[l][k]` = fraction of a 1 MW injection at bus *k* (withdrawn at
    /// slack) that flows on branch *l*.
    ///
    /// Computed via finite differences: inject +δ at bus k, solve PF, record flows.
    pub fn compute_ptdf_matrix(&self) -> Vec<Vec<f64>> {
        let n_bus = self.buses.len();
        let n_branch = self.branches.len();
        let delta = 1.0_f64; // 1 MW perturbation
        let delta_pu = delta / self.base_mva;

        let b_mat = self.build_b_matrix();
        let base_inj = vec![0.0_f64; n_bus];
        let base_angles = Self::solve_dc_pf(&b_mat, &base_inj);
        let base_flows = self.compute_branch_flows(&base_angles);

        let slack_idx = self.buses.iter().position(|b| b.is_slack).unwrap_or(0);

        let mut ptdf = vec![vec![0.0_f64; n_bus]; n_branch];
        for k in 0..n_bus {
            if k == slack_idx {
                continue;
            }
            let mut inj = vec![0.0_f64; n_bus];
            inj[k] = delta_pu;
            inj[slack_idx] -= delta_pu;
            let angles = Self::solve_dc_pf(&b_mat, &inj);
            let flows = self.compute_branch_flows(&angles);
            for l in 0..n_branch {
                ptdf[l][k] = (flows[l] - base_flows[l]) / delta;
            }
        }
        ptdf
    }

    // ── OPF solvers ───────────────────────────────────────────────────────────

    /// Solve base DC-OPF without any FACTS adjustments.
    ///
    /// Uses lambda-iteration (equal-incremental-cost) for economic dispatch,
    /// then runs the resulting DC power flow.
    pub fn solve_base_opf(&self) -> FactsOpfProblemResult {
        let dispatch = self.lambda_iteration_dispatch();
        self.build_result_from_dispatch(&dispatch, 0, true)
    }

    /// Solve FACTS-augmented DC-OPF.
    ///
    /// Iterates between lambda-iteration dispatch and sensitivity-based FACTS
    /// adjustment until congestion is relieved or `max_iterations` is reached.
    pub fn solve_facts_opf(&self) -> FactsOpfProblemResult {
        let mut problem = self.clone();
        let mut iter = 0_usize;
        let mut converged = false;

        let mut dispatch = problem.lambda_iteration_dispatch();
        let mut result = problem.build_result_from_dispatch(&dispatch, iter, false);

        while iter < problem.max_iterations {
            iter += 1;
            let changed = problem.adjust_facts_for_congestion(
                &result.branch_flows_mw.clone(),
                &result.bus_angles_rad.clone(),
            );
            dispatch = problem.lambda_iteration_dispatch();
            result = problem.build_result_from_dispatch(&dispatch, iter, false);
            if !changed || result.n_congested_branches == 0 {
                converged = true;
                break;
            }
        }

        result.converged = converged;
        result.iterations = iter;
        result.facts_settings = problem
            .facts_devices
            .iter()
            .map(|d| (d.id, d.current_x_pu))
            .collect();
        result.total_facts_cost_usd = problem
            .facts_devices
            .iter()
            .map(|d| d.operating_cost_usd_per_h)
            .sum();
        result
    }

    /// Adjust FACTS device settings to relieve congestion.
    ///
    /// For each congested branch (|P_flow| > 0.95 × rating) with a TCSC/SSSC/UPFC/IPFC:
    /// 1. Compute sensitivity `dP_branch / dX_facts` via finite differences.
    /// 2. Apply a gradient step: `ΔX = -overflow / sensitivity`.
    /// 3. Clamp the new setting to [x_min, x_max].
    ///
    /// Returns `true` if at least one device setting changed.
    pub fn adjust_facts_for_congestion(&mut self, flows: &[f64], _angles: &[f64]) -> bool {
        let threshold = 0.95_f64;
        let mut changed = false;

        let congested: Vec<(usize, f64, f64)> = self
            .branches
            .iter()
            .enumerate()
            .filter_map(|(idx, br)| {
                let flow = flows.get(idx).copied().unwrap_or(0.0);
                if br.rating_mw > 1e-6 && flow.abs() > threshold * br.rating_mw {
                    Some((idx, flow, br.rating_mw))
                } else {
                    None
                }
            })
            .collect();

        for (br_idx, flow, rating) in congested {
            let dev_id = match self.branches[br_idx].facts_device_id {
                Some(id) => id,
                None => continue,
            };
            let dev_type = self
                .facts_devices
                .iter()
                .find(|d| d.id == dev_id)
                .map(|d| d.device_type);
            match dev_type {
                Some(
                    FactsDeviceType::Tcsc
                    | FactsDeviceType::Sssc
                    | FactsDeviceType::Upfc
                    | FactsDeviceType::Ipfc,
                ) => {}
                _ => continue,
            }

            let sensitivity = self.compute_sensitivity_pf_to_facts(dev_id);
            let s_k = sensitivity.get(br_idx).copied().unwrap_or(0.0);
            if s_k.abs() < 1e-9 {
                continue;
            }
            let overflow = flow - flow.signum() * rating;
            let delta_x = -overflow / s_k;

            if let Some(dev) = self.facts_devices.iter_mut().find(|d| d.id == dev_id) {
                let new_x = dev.clamp_x(dev.current_x_pu + delta_x);
                if (new_x - dev.current_x_pu).abs() > 1e-9 {
                    dev.current_x_pu = new_x;
                    changed = true;
                }
            }
        }
        changed
    }

    /// Compute sensitivity of each branch flow to a FACTS device reactance setting.
    ///
    /// Returns `dP_branch[l] / dX_facts` [MW/pu] for each branch *l*,
    /// via forward finite differences with δX = 1×10⁻⁴ pu.
    pub fn compute_sensitivity_pf_to_facts(&self, device_id: usize) -> Vec<f64> {
        let n_branch = self.branches.len();
        let delta_x = 1e-4_f64;

        let b0 = self.build_b_matrix();
        let inj0 = self.net_injections_pu();
        let angles0 = Self::solve_dc_pf(&b0, &inj0);
        let flows0 = self.compute_branch_flows(&angles0);

        let mut perturbed = self.clone();
        let found = perturbed
            .facts_devices
            .iter_mut()
            .find(|d| d.id == device_id)
            .map(|d| {
                d.current_x_pu += delta_x;
            })
            .is_some();

        if !found {
            return vec![0.0; n_branch];
        }

        let b1 = perturbed.build_b_matrix();
        let inj1 = perturbed.net_injections_pu();
        let angles1 = Self::solve_dc_pf(&b1, &inj1);
        let flows1 = perturbed.compute_branch_flows(&angles1);

        flows1
            .iter()
            .zip(flows0.iter())
            .map(|(f1, f0)| (f1 - f0) / delta_x)
            .collect()
    }

    /// Compute total generation cost [USD/h] from a dispatch vector.
    pub fn compute_total_cost(dispatch: &[(usize, f64)], generators: &[GeneratorData]) -> f64 {
        dispatch
            .iter()
            .map(|&(gid, p)| {
                generators
                    .iter()
                    .find(|g| g.id == gid)
                    .map(|g| g.total_cost_at(p))
                    .unwrap_or(0.0)
            })
            .sum()
    }

    // ── Private helpers ────────────────────────────────────────────────────────

    /// Net power injection per bus [pu]: (P_gen − P_load) / base_mva.
    fn net_injections_pu(&self) -> Vec<f64> {
        self.buses
            .iter()
            .map(|b| (b.p_gen_mw - b.p_load_mw) / self.base_mva)
            .collect()
    }

    /// Lambda-iteration economic dispatch via bisection on system marginal price.
    fn lambda_iteration_dispatch(&self) -> Vec<(usize, f64)> {
        let total_load: f64 = self.buses.iter().map(|b| b.p_load_mw).sum();

        let lambda_min = self
            .generators
            .iter()
            .map(|g| g.cost_b_usd_per_mwh)
            .fold(f64::INFINITY, f64::min)
            .min(0.0);
        let lambda_max = self
            .generators
            .iter()
            .map(|g| g.marginal_cost_at(g.p_max_mw))
            .fold(f64::NEG_INFINITY, f64::max)
            + 200.0;

        let tol = 0.1_f64;
        let mut lo = lambda_min;
        let mut hi = lambda_max;
        let mut lambda = (lo + hi) / 2.0;

        for _ in 0..200 {
            let total_gen: f64 = self
                .generators
                .iter()
                .map(|g| g.optimal_dispatch_at(lambda))
                .sum();
            let bal = total_gen - total_load;
            if bal.abs() < tol {
                break;
            }
            if bal < 0.0 {
                lo = lambda;
            } else {
                hi = lambda;
            }
            lambda = (lo + hi) / 2.0;
        }

        self.generators
            .iter()
            .map(|g| (g.id, g.optimal_dispatch_at(lambda)))
            .collect()
    }

    /// Build a [`FactsOpfProblemResult`] from a dispatch vector.
    fn build_result_from_dispatch(
        &self,
        dispatch: &[(usize, f64)],
        iterations: usize,
        converged: bool,
    ) -> FactsOpfProblemResult {
        // Overlay generator dispatch onto buses
        let mut buses_updated = self.buses.clone();
        for bus in &mut buses_updated {
            bus.p_gen_mw = 0.0;
        }
        for &(gid, p) in dispatch {
            if let Some(gen) = self.generators.iter().find(|g| g.id == gid) {
                if gen.bus_id < buses_updated.len() {
                    buses_updated[gen.bus_id].p_gen_mw += p;
                }
            }
        }
        let inj: Vec<f64> = buses_updated
            .iter()
            .map(|b| (b.p_gen_mw - b.p_load_mw) / self.base_mva)
            .collect();

        let b_mat = self.build_b_matrix();
        let angles = Self::solve_dc_pf(&b_mat, &inj);
        let branch_flows = self.compute_branch_flows(&angles);

        let branch_loading_pct: Vec<f64> = branch_flows
            .iter()
            .zip(self.branches.iter())
            .map(|(&f, br)| {
                if br.rating_mw > 1e-6 {
                    f.abs() / br.rating_mw * 100.0
                } else {
                    0.0
                }
            })
            .collect();

        let n_congested = branch_loading_pct.iter().filter(|&&pct| pct > 95.0).count();

        let total_gen_cost = Self::compute_total_cost(dispatch, &self.generators);
        let facts_cost: f64 = self
            .facts_devices
            .iter()
            .map(|d| d.operating_cost_usd_per_h)
            .sum();
        let facts_settings: Vec<(usize, f64)> = self
            .facts_devices
            .iter()
            .map(|d| (d.id, d.current_x_pu))
            .collect();

        FactsOpfProblemResult {
            generator_dispatch: dispatch.to_vec(),
            facts_settings,
            bus_angles_rad: angles,
            branch_flows_mw: branch_flows,
            branch_loading_pct,
            total_generation_cost_usd: total_gen_cost,
            total_facts_cost_usd: facts_cost,
            n_congested_branches: n_congested,
            losses_mw: 0.0,
            converged,
            iterations,
        }
    }
}

// ─── Gaussian Elimination (module-private) ────────────────────────────────────

/// Solve A x = b via Gaussian elimination with partial pivoting.
/// Returns a zero vector for singular systems.
fn facts_gaussian_elimination(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    if n == 0 {
        return Vec::new();
    }
    let mut aug: Vec<Vec<f64>> = a
        .iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();

    #[allow(clippy::needless_range_loop)]
    for col in 0..n {
        // Partial pivot
        let (mut max_row, mut max_val) = (col, aug[col][col].abs());
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-14 {
            return vec![0.0; n];
        }
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for k in col..=n {
                let v = aug[col][k];
                aug[row][k] -= factor * v;
            }
        }
    }
    // Back-substitution
    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        let mut sum = aug[row][n];
        for k in (row + 1)..n {
            sum -= aug[row][k] * x[k];
        }
        let diag = aug[row][row];
        x[row] = if diag.abs() > 1e-14 { sum / diag } else { 0.0 };
    }
    x
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};

    /// Two-bus network: slack at bus 1, load at bus 2, two generators.
    fn make_two_bus_network(load_mw: f64) -> (PowerNetwork, Vec<GenCost>) {
        let mut net = PowerNetwork::new(100.0);

        net.buses.push(Bus::new(1, BusType::Slack));
        let mut b2 = Bus::new(2, BusType::PQ);
        b2.pd = crate::units::Power(load_mw);
        net.buses.push(b2);

        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 200.0,
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
        net.generators.push(Generator {
            bus_id: 2,
            pg: 0.2,
            qg: 0.0,
            qmax: 50.0,
            qmin: -50.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 50.0,
            pmin: 0.0,
        });

        let costs = vec![
            GenCost::quadratic(0.0, 20.0, 0.02, 0.0, 100.0),
            GenCost::quadratic(0.0, 25.0, 0.03, 0.0, 50.0),
        ];

        (net, costs)
    }

    #[test]
    fn test_facts_opf_no_devices_balances_load() {
        // Without any FACTS devices the result should mirror a plain DC-OPF.
        let (net, costs) = make_two_bus_network(60.0);
        let config = FactsOpfConfig {
            statcoms: vec![],
            svcs: vec![],
            tcsc_branches: vec![],
            ..Default::default()
        };
        let result = solve_facts_opf(&net, &costs, &config).expect("should succeed");
        assert!(result.converged);
        assert_eq!(result.gen_dispatch.len(), 2);
        let total_gen: f64 = result.gen_dispatch.iter().sum();
        assert!(
            (total_gen - 60.0).abs() < 2.0,
            "total_gen={total_gen:.3} should ≈ 60 MW"
        );
        assert!(result.total_cost > 0.0);
    }

    #[test]
    fn test_facts_opf_statcom_within_limits() {
        let (net, costs) = make_two_bus_network(60.0);
        let config = FactsOpfConfig {
            statcoms: vec![1], // bus index 1
            svcs: vec![],
            tcsc_branches: vec![],
            statcom_q_max: 30.0,
            ..Default::default()
        };
        let result = solve_facts_opf(&net, &costs, &config).expect("should succeed");
        assert_eq!(result.statcom_q.len(), 1);
        assert!(
            result.statcom_q[0].abs() <= 30.0 + 1e-9,
            "STATCOM Q={:.3} exceeds 30 MVAr limit",
            result.statcom_q[0]
        );
        // Post-correction voltage must stay in [0.9, 1.1]
        assert!(
            result.voltage_magnitudes[1] >= 0.9 && result.voltage_magnitudes[1] <= 1.1,
            "V[1]={:.4} out of [0.9, 1.1]",
            result.voltage_magnitudes[1]
        );
    }

    #[test]
    fn test_facts_opf_svc_within_limits() {
        let (net, costs) = make_two_bus_network(60.0);
        let config = FactsOpfConfig {
            statcoms: vec![],
            svcs: vec![0], // bus 0 (slack)
            tcsc_branches: vec![],
            svc_b_max: 0.25,
            ..Default::default()
        };
        let result = solve_facts_opf(&net, &costs, &config).expect("should succeed");
        assert_eq!(result.svc_b.len(), 1);
        assert!(
            result.svc_b[0].abs() <= 0.25 + 1e-9,
            "SVC B={:.4} exceeds 0.25 pu limit",
            result.svc_b[0]
        );
    }

    #[test]
    fn test_facts_opf_tcsc_reduces_overloaded_reactance() {
        // Single-generator, single-load network with a tight line rating.
        // The line carries the full load → overloaded → TCSC should reduce x.
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let mut b2 = Bus::new(2, BusType::PQ);
        b2.pd = crate::units::Power(40.0);
        net.buses.push(b2);

        // Branch with rate_a = 10 MW (will be overloaded by 40 MW load)
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 10.0, // tight limit → will be overloaded
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.4,
            qg: 0.0,
            qmax: 100.0,
            qmin: -100.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 100.0,
            pmin: 0.0,
        });

        let costs = vec![GenCost::linear(20.0, 0.0, 100.0)];
        let config = FactsOpfConfig {
            tcsc_branches: vec![0],
            tcsc_delta_x_max: 0.3,
            ..Default::default()
        };
        let result = solve_facts_opf(&net, &costs, &config).expect("should succeed");
        assert_eq!(result.tcsc_x.len(), 1);
        // Overloaded line → TCSC must reduce reactance below nominal 0.1
        assert!(
            result.tcsc_x[0] < 0.1 + 1e-9,
            "TCSC x={:.4} should be ≤ nominal 0.1 (overload case)",
            result.tcsc_x[0]
        );
    }

    #[test]
    fn test_facts_opf_invalid_statcom_index_errors() {
        let (net, costs) = make_two_bus_network(60.0);
        let config = FactsOpfConfig {
            statcoms: vec![99], // out of range
            ..Default::default()
        };
        let result = solve_facts_opf(&net, &costs, &config);
        assert!(
            result.is_err(),
            "Expected error for invalid STATCOM bus index"
        );
    }

    // =========================================================================
    // FactsOpfProblem self-contained API tests
    // =========================================================================

    fn make_3bus_problem() -> FactsOpfProblem {
        //  Bus 0 (slack, gen) ─branch0(x=0.1)─ Bus 1 (load 100 MW)
        //  Bus 0              ─branch1(x=0.2)─ Bus 2 (load 50 MW)
        //  Bus 1              ─branch2(x=0.15)─ Bus 2
        let buses = vec![
            DcBusData {
                id: 0,
                p_gen_mw: 150.0,
                p_load_mw: 0.0,
                b_shunt_pu: 0.0,
                is_slack: true,
                angle_rad: 0.0,
            },
            DcBusData::load_bus(1, 100.0),
            DcBusData::load_bus(2, 50.0),
        ];
        let branches = vec![
            DcBranchData::new_branch(0, 0, 1, 0.1, 200.0),
            DcBranchData::new_branch(1, 0, 2, 0.2, 150.0),
            DcBranchData::new_branch(2, 1, 2, 0.15, 100.0),
        ];
        let generators = vec![GeneratorData {
            id: 0,
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 300.0,
            cost_a_usd_per_h: 0.0,
            cost_b_usd_per_mwh: 20.0,
            cost_c_usd_per_mw2h: 0.01,
        }];
        FactsOpfProblem::new(buses, branches, generators)
    }

    fn make_2gen_problem() -> FactsOpfProblem {
        let buses = vec![
            DcBusData {
                id: 0,
                p_gen_mw: 0.0,
                p_load_mw: 0.0,
                b_shunt_pu: 0.0,
                is_slack: true,
                angle_rad: 0.0,
            },
            DcBusData::load_bus(1, 0.0),
            DcBusData::load_bus(2, 200.0),
        ];
        let branches = vec![
            DcBranchData::new_branch(0, 0, 2, 0.1, 250.0),
            DcBranchData::new_branch(1, 1, 2, 0.1, 250.0),
        ];
        let generators = vec![
            GeneratorData {
                id: 0,
                bus_id: 0,
                p_min_mw: 0.0,
                p_max_mw: 200.0,
                cost_a_usd_per_h: 0.0,
                cost_b_usd_per_mwh: 20.0,
                cost_c_usd_per_mw2h: 0.02,
            },
            GeneratorData {
                id: 1,
                bus_id: 1,
                p_min_mw: 0.0,
                p_max_mw: 200.0,
                cost_a_usd_per_h: 0.0,
                cost_b_usd_per_mwh: 25.0,
                cost_c_usd_per_mw2h: 0.02,
            },
        ];
        FactsOpfProblem::new(buses, branches, generators)
    }

    #[test]
    fn test_problem_b_matrix_diagonal() {
        let p = make_3bus_problem();
        let b = p.build_b_matrix();
        let expected_b00 = 1.0 / 0.1 + 1.0 / 0.2;
        assert!((b[0][0] - expected_b00).abs() < 1e-9, "B[0][0]={}", b[0][0]);
        let expected_b11 = 1.0 / 0.1 + 1.0 / 0.15;
        assert!((b[1][1] - expected_b11).abs() < 1e-9, "B[1][1]={}", b[1][1]);
    }

    #[test]
    fn test_problem_b_matrix_off_diagonal() {
        let p = make_3bus_problem();
        let b = p.build_b_matrix();
        assert!((b[0][1] + 10.0).abs() < 1e-9, "B[0][1]={}", b[0][1]);
        assert!((b[0][2] + 5.0).abs() < 1e-9, "B[0][2]={}", b[0][2]);
        assert!((b[1][2] + 1.0 / 0.15).abs() < 1e-9, "B[1][2]={}", b[1][2]);
    }

    #[test]
    fn test_problem_dc_pf_angle_slack_zero() {
        let p = make_3bus_problem();
        let b = p.build_b_matrix();
        let inj = vec![1.5_f64, -1.0, -0.5];
        let angles = FactsOpfProblem::solve_dc_pf(&b, &inj);
        assert_eq!(angles.len(), 3);
        assert!(angles[0].abs() < 1e-10, "Slack angle={}", angles[0]);
    }

    #[test]
    fn test_problem_dc_pf_3bus() {
        let p = make_3bus_problem();
        let b = p.build_b_matrix();
        let inj = vec![1.5_f64, -1.0, -0.5];
        let angles = FactsOpfProblem::solve_dc_pf(&b, &inj);
        // Verify B*θ ≈ inj for non-slack buses
        for i in 1..3 {
            let lhs: f64 = (0..3).map(|j| b[i][j] * angles[j]).sum();
            assert!(
                (lhs - inj[i]).abs() < 1e-6,
                "Bus {i}: B*θ={lhs} inj={}",
                inj[i]
            );
        }
    }

    #[test]
    fn test_problem_branch_flows_kirchhoff() {
        let p = make_3bus_problem();
        let b = p.build_b_matrix();
        let inj: Vec<f64> = p
            .buses
            .iter()
            .map(|bus| (bus.p_gen_mw - bus.p_load_mw) / 100.0)
            .collect();
        let angles = FactsOpfProblem::solve_dc_pf(&b, &inj);
        let flows = p.compute_branch_flows(&angles);
        assert_eq!(flows.len(), 3);
        // Kirchhoff at bus 1: flow_01 (in) - flow_12 (out) = P_load_1 = 100 MW
        let kf1 = flows[0] - flows[2];
        assert!((kf1 - 100.0).abs() < 1.0, "Kirchhoff bus 1 = {kf1}");
    }

    #[test]
    fn test_problem_base_opf_no_facts() {
        let p = make_3bus_problem();
        let result = p.solve_base_opf();
        assert_eq!(result.bus_angles_rad.len(), 3);
        assert_eq!(result.branch_flows_mw.len(), 3);
        assert!(!result.generator_dispatch.is_empty());
    }

    #[test]
    fn test_problem_base_opf_cost_positive() {
        let p = make_3bus_problem();
        let result = p.solve_base_opf();
        assert!(
            result.total_generation_cost_usd >= 0.0,
            "Cost={}",
            result.total_generation_cost_usd
        );
    }

    #[test]
    fn test_problem_facts_device_adds_reactance() {
        let mut p = make_3bus_problem();
        let b_base = p.build_b_matrix();

        let mut dev = FactsDevice::new_device(0, "TCSC-01", FactsDeviceType::Tcsc, 0, 1);
        dev.x_min_pu = -0.05;
        dev.x_max_pu = 0.0;
        dev.current_x_pu = -0.03;
        p.add_facts_device(dev);

        let b_facts = p.build_b_matrix();
        assert!(
            (b_facts[0][0] - b_base[0][0]).abs() > 1e-6,
            "B-matrix should change with TCSC installed"
        );
    }

    #[test]
    fn test_problem_tcsc_congestion_relief() {
        // 3-bus network with two parallel paths from bus 0 to bus 2.
        // TCSC on branch 0 (0→2) can redistribute flow to branch 1 (0→2 via bus 1).
        //  Bus 0 ─ branch0(x=0.1, tight) ─ Bus 2
        //  Bus 0 ─ branch2(x=0.2)         ─ Bus 1 ─ branch1(x=0.1) ─ Bus 2
        let buses = vec![
            DcBusData {
                id: 0,
                p_gen_mw: 150.0,
                p_load_mw: 0.0,
                b_shunt_pu: 0.0,
                is_slack: true,
                angle_rad: 0.0,
            },
            DcBusData::load_bus(1, 0.0),
            DcBusData::load_bus(2, 150.0),
        ];
        let branches = vec![
            DcBranchData::new_branch(0, 0, 2, 0.1, 100.0), // TCSC goes here
            DcBranchData::new_branch(1, 1, 2, 0.1, 200.0),
            DcBranchData::new_branch(2, 0, 1, 0.2, 200.0),
        ];
        let generators = vec![GeneratorData {
            id: 0,
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 300.0,
            cost_a_usd_per_h: 0.0,
            cost_b_usd_per_mwh: 20.0,
            cost_c_usd_per_mw2h: 0.0,
        }];
        let mut p = FactsOpfProblem::new(buses, branches, generators);

        let mut dev = FactsDevice::new_device(0, "TCSC", FactsDeviceType::Tcsc, 0, 2);
        dev.x_min_pu = -0.08;
        dev.x_max_pu = 0.0;
        dev.current_x_pu = 0.0;
        p.add_facts_device(dev);

        let base = p.solve_base_opf();
        // Branch 0 should be loaded (has lower impedance so takes more flow)
        assert!(
            base.branch_loading_pct[0] > 0.0,
            "Loading={}",
            base.branch_loading_pct[0]
        );
        // Sensitivity of branch 0 flow to TCSC on branch 0 must be non-zero
        // because there's a parallel path (branches 1+2) to carry the load
        let sens = p.compute_sensitivity_pf_to_facts(0);
        assert!(
            sens[0].abs() > 0.1,
            "Expected non-zero sensitivity, got sens[0]={}",
            sens[0]
        );
    }

    #[test]
    fn test_problem_facts_opf_converges() {
        let p = make_2gen_problem();
        let result = p.solve_facts_opf();
        assert!(result.converged, "Expected converged=true");
    }

    #[test]
    fn test_problem_facts_opf_vs_base() {
        let buses = vec![
            DcBusData {
                id: 0,
                p_gen_mw: 0.0,
                p_load_mw: 0.0,
                b_shunt_pu: 0.0,
                is_slack: true,
                angle_rad: 0.0,
            },
            DcBusData::load_bus(1, 0.0),
            DcBusData::load_bus(2, 150.0),
        ];
        let branches = vec![
            DcBranchData::new_branch(0, 0, 2, 0.05, 100.0),
            DcBranchData::new_branch(1, 1, 2, 0.1, 200.0),
            DcBranchData::new_branch(2, 0, 1, 0.2, 200.0),
        ];
        let generators = vec![
            GeneratorData {
                id: 0,
                bus_id: 0,
                p_min_mw: 0.0,
                p_max_mw: 200.0,
                cost_a_usd_per_h: 0.0,
                cost_b_usd_per_mwh: 20.0,
                cost_c_usd_per_mw2h: 0.01,
            },
            GeneratorData {
                id: 1,
                bus_id: 1,
                p_min_mw: 0.0,
                p_max_mw: 200.0,
                cost_a_usd_per_h: 0.0,
                cost_b_usd_per_mwh: 30.0,
                cost_c_usd_per_mw2h: 0.01,
            },
        ];
        let mut p_facts = FactsOpfProblem::new(buses.clone(), branches.clone(), generators.clone());
        let mut dev = FactsDevice::new_device(0, "TCSC", FactsDeviceType::Tcsc, 0, 2);
        dev.x_min_pu = -0.04;
        dev.x_max_pu = 0.0;
        dev.current_x_pu = 0.0;
        p_facts.add_facts_device(dev);

        let p_base = FactsOpfProblem::new(buses, branches, generators);
        let base_result = p_base.solve_base_opf();
        let facts_result = p_facts.solve_facts_opf();

        assert!(!base_result.branch_flows_mw.is_empty());
        assert!(!facts_result.branch_flows_mw.is_empty());
        assert!(
            facts_result.n_congested_branches <= base_result.n_congested_branches,
            "FACTS should not worsen congestion"
        );
    }

    #[test]
    fn test_problem_ptdf_matrix_shape() {
        let p = make_3bus_problem();
        let ptdf = p.compute_ptdf_matrix();
        assert_eq!(ptdf.len(), p.branches.len());
        for row in &ptdf {
            assert_eq!(row.len(), p.buses.len());
        }
    }

    #[test]
    fn test_problem_ptdf_row_sum_zero() {
        // For a radial (single-path) 2-bus DC network, the PTDF of the only
        // branch w.r.t. the load bus equals exactly 1.0 and the slack column
        // is 0.  For meshed networks, the slack column is 0 by construction.
        let buses = vec![
            DcBusData {
                id: 0,
                p_gen_mw: 100.0,
                p_load_mw: 0.0,
                b_shunt_pu: 0.0,
                is_slack: true,
                angle_rad: 0.0,
            },
            DcBusData::load_bus(1, 100.0),
        ];
        let branches = vec![DcBranchData::new_branch(0, 0, 1, 0.1, 200.0)];
        let generators = vec![GeneratorData {
            id: 0,
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            cost_a_usd_per_h: 0.0,
            cost_b_usd_per_mwh: 20.0,
            cost_c_usd_per_mw2h: 0.01,
        }];
        let p = FactsOpfProblem::new(buses, branches, generators);
        let ptdf = p.compute_ptdf_matrix();
        assert_eq!(ptdf.len(), 1);
        assert_eq!(ptdf[0].len(), 2);
        // Slack column (bus 0) is always 0
        assert!(ptdf[0][0].abs() < 1e-9, "Slack PTDF col={}", ptdf[0][0]);
        // Load bus column should be non-zero (= 1.0 for radial)
        assert!(
            (ptdf[0][1].abs() - 1.0).abs() < 1e-6,
            "Radial PTDF should be 1.0, got {}",
            ptdf[0][1]
        );
    }

    #[test]
    fn test_problem_sensitivity_finite_differences() {
        let mut p = make_3bus_problem();
        let mut dev = FactsDevice::new_device(0, "TCSC", FactsDeviceType::Tcsc, 0, 1);
        dev.x_min_pu = -0.09;
        dev.x_max_pu = 0.0;
        dev.current_x_pu = 0.0;
        p.add_facts_device(dev);

        let sens = p.compute_sensitivity_pf_to_facts(0);
        assert_eq!(sens.len(), p.branches.len());
        assert!(
            sens[0].abs() > 0.1,
            "Expected non-zero sensitivity, got {}",
            sens[0]
        );
    }

    #[test]
    fn test_problem_loading_pct_calculation() {
        let p = make_3bus_problem();
        let result = p.solve_base_opf();
        for (l, &pct) in result.branch_loading_pct.iter().enumerate() {
            let expected = result.branch_flows_mw[l].abs() / p.branches[l].rating_mw * 100.0;
            assert!(
                (pct - expected).abs() < 1e-9,
                "Branch {l}: pct={pct} expected={expected}"
            );
        }
    }

    #[test]
    fn test_problem_facts_cost_computation() {
        let mut p = make_3bus_problem();
        let mut dev = FactsDevice::new_device(0, "TCSC", FactsDeviceType::Tcsc, 0, 1);
        dev.operating_cost_usd_per_h = 50.0;
        p.add_facts_device(dev);
        let result = p.solve_facts_opf();
        assert!((result.total_facts_cost_usd - 50.0).abs() < 1e-9);
    }

    #[test]
    fn test_problem_lambda_iteration_balance() {
        let p = make_2gen_problem();
        let result = p.solve_base_opf();
        let total_gen: f64 = result.generator_dispatch.iter().map(|&(_, pw)| pw).sum();
        let total_load: f64 = p.buses.iter().map(|b| b.p_load_mw).sum();
        assert!(
            (total_gen - total_load).abs() < 1.0,
            "Balance error={}",
            total_gen - total_load
        );
    }

    #[test]
    fn test_problem_generator_bounds_respected() {
        let p = make_2gen_problem();
        let result = p.solve_base_opf();
        for &(gid, pw) in &result.generator_dispatch {
            if let Some(gen) = p.generators.iter().find(|g| g.id == gid) {
                assert!(pw >= gen.p_min_mw - 1e-6, "Gen {gid} below p_min");
                assert!(pw <= gen.p_max_mw + 1e-6, "Gen {gid} above p_max");
            }
        }
    }

    #[test]
    fn test_problem_facts_bounds_respected() {
        let mut p = make_3bus_problem();
        let mut dev = FactsDevice::new_device(0, "TCSC", FactsDeviceType::Tcsc, 0, 1);
        dev.x_min_pu = -0.05;
        dev.x_max_pu = 0.0;
        dev.current_x_pu = 0.0;
        p.add_facts_device(dev);

        let result = p.solve_facts_opf();
        for &(did, x) in &result.facts_settings {
            if let Some(dev) = p.facts_devices.iter().find(|d| d.id == did) {
                assert!(x >= dev.x_min_pu - 1e-9, "FACTS {did} below x_min");
                assert!(x <= dev.x_max_pu + 1e-9, "FACTS {did} above x_max");
            }
        }
    }

    #[test]
    fn test_problem_no_facts_devices() {
        let p = make_3bus_problem();
        let result = p.solve_facts_opf();
        assert!(result.facts_settings.is_empty());
        assert_eq!(result.total_facts_cost_usd, 0.0);
        assert!(result.converged);
    }

    #[test]
    fn test_problem_compute_total_cost_static() {
        let generators = vec![GeneratorData {
            id: 0,
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 300.0,
            cost_a_usd_per_h: 100.0,
            cost_b_usd_per_mwh: 20.0,
            cost_c_usd_per_mw2h: 0.01,
        }];
        let dispatch = vec![(0_usize, 100.0_f64)];
        // C = 100 + 20*100 + 0.01*10000 = 2200
        let cost = FactsOpfProblem::compute_total_cost(&dispatch, &generators);
        assert!((cost - 2200.0).abs() < 1e-6, "Cost={cost}");
    }

    #[test]
    fn test_problem_device_enum_variants() {
        // Verify all FactsDeviceType variants can be constructed and compared
        assert_ne!(FactsDeviceType::Tcsc, FactsDeviceType::Svc);
        assert_ne!(FactsDeviceType::Upfc, FactsDeviceType::Statcom);
        assert_ne!(FactsDeviceType::Sssc, FactsDeviceType::Ipfc);
        assert_eq!(FactsDeviceType::Tcsc, FactsDeviceType::Tcsc);
    }

    #[test]
    fn test_problem_control_objective_variants() {
        assert_ne!(
            FactsControlObjective::CongestionRelief,
            FactsControlObjective::LossMinimization
        );
        assert_ne!(
            FactsControlObjective::VoltageSupport,
            FactsControlObjective::PowerScheduling
        );
        assert_eq!(
            FactsControlObjective::OscillationDamping,
            FactsControlObjective::OscillationDamping
        );
    }
}
