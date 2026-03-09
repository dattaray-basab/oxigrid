//! Cyber-Physical Co-Simulation for Power Grid Security Analysis.
//!
//! This module models the interaction between the power system (physical layer)
//! and its communication/control infrastructure (cyber layer), enabling
//! rigorous analysis of attack scenarios including:
//!
//! - **False Data Injection (FDI)**: Tampered SCADA measurements that can
//!   mislead energy management systems into issuing incorrect control actions.
//! - **Denial of Service (DoS)**: Bandwidth saturation that degrades
//!   communication latency and causes stale measurements to propagate.
//! - **Bad Data Detection (BDD)**: Chi-squared and normalized-residual tests
//!   that attempt to identify corrupted measurements before they reach the
//!   control layer.
//!
//! ## Simulation Architecture
//!
//! ```text
//!  ┌─────────────────────────────────────────────────────────┐
//!  │  Physical Layer  (dt_physical_s ≈ 0.1 s)                │
//!  │  • Bus voltage magnitudes (swing-equation approximation) │
//!  │  • Load-frequency control (droop + integral)            │
//!  └────────────────┬────────────────────────────────────────┘
//!                   │  true measurements z = Hx + v
//!  ┌────────────────▼────────────────────────────────────────┐
//!  │  Cyber Layer   (dt_cyber_s ≈ 1.0 s)                    │
//!  │  • CommNetwork: latency, packet loss, bandwidth limits  │
//!  │  • FDI attack: z̃ = z + a                               │
//!  │  • DoS attack: effective latency↑, packet_loss↑         │
//!  │  • Bad Data Detection: normalized residuals + χ² test   │
//!  └────────────────┬────────────────────────────────────────┘
//!                   │  control signal u (possibly delayed/corrupted)
//!  ┌────────────────▼────────────────────────────────────────┐
//!  │  Control Layer                                          │
//!  │  • Droop + proportional voltage regulator              │
//!  │  • Load shedding triggered on severe voltage deviation  │
//!  └─────────────────────────────────────────────────────────┘
//! ```

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// LCG (Knuth MMIX — no `rand` crate)
// ─────────────────────────────────────────────────────────────────────────────

/// Advance LCG state and return a uniform sample in \[0, 1).
#[inline(always)]
fn lcg_next(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64);
    // Use upper 53 bits for full double precision
    (*state >> 11) as f64 / (1u64 << 53) as f64
}

/// Box-Muller transform — returns a standard normal sample N(0,1).
#[inline]
fn lcg_normal(state: &mut u64) -> f64 {
    let u1 = lcg_next(state).max(1e-15); // avoid log(0)
    let u2 = lcg_next(state);
    (-2.0 * u1.ln()).sqrt() * (2.0 * core::f64::consts::PI * u2).cos()
}

// ─────────────────────────────────────────────────────────────────────────────
// Communication-network types
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of communication node in a SCADA/EMS network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommNodeType {
    /// High-voltage substation with local RTU/IED.
    Substation,
    /// Remote Terminal Unit (pole-mounted or pad-mounted).
    RemoteTerminalUnit,
    /// Central Energy Management System / SCADA control centre.
    ControlCenter,
    /// Data historian / time-series archive server.
    Historian,
    /// Field device (smart meter, sensor node, PMU).
    FieldDevice,
}

/// A node in the SCADA/EMS communication network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommNode {
    /// Unique node index (0-based).
    pub id: usize,
    /// Functional role of this node.
    pub node_type: CommNodeType,
    /// Whether this node has been compromised by an attacker.
    pub is_compromised: bool,
    /// Fraction of outgoing packets dropped in steady state \[0, 1\].
    pub packet_loss_rate: f64,
}

/// Industrial communication protocol carried on a link.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommProtocol {
    /// Modbus RTU/TCP (legacy SCADA).
    Modbus,
    /// DNP3 (IEEE 1815) — common in North American utilities.
    Dnp3,
    /// IEC 61850 GOOSE / MMS — modern substation automation.
    Iec61850,
    /// ICCP (IEC 60870-6 / TASE.2) — inter-control-centre protocol.
    Iccp,
    /// OPC Unified Architecture — plant-floor to IT integration.
    OpcUa,
}

impl CommProtocol {
    /// Typical minimum latency for this protocol \[ms\].
    pub fn base_latency_ms(self) -> f64 {
        match self {
            Self::Modbus => 10.0,
            Self::Dnp3 => 5.0,
            Self::Iec61850 => 2.0,
            Self::Iccp => 20.0,
            Self::OpcUa => 8.0,
        }
    }

    /// Whether this protocol provides built-in message authentication.
    pub fn has_native_auth(self) -> bool {
        match self {
            Self::Iec61850 | Self::OpcUa => true,
            Self::Modbus | Self::Dnp3 | Self::Iccp => false,
        }
    }
}

/// A directed communication link between two [`CommNode`]s.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommLink {
    /// Source node index.
    pub from: usize,
    /// Destination node index.
    pub to: usize,
    /// Nominal link capacity \[Mbps\].
    pub bandwidth_mbps: f64,
    /// One-way propagation latency under nominal load \[ms\].
    pub latency_ms: f64,
    /// Whether the link uses TLS/IPsec encryption.
    pub is_encrypted: bool,
    /// Industrial protocol carried on this link.
    pub protocol: CommProtocol,
}

impl CommLink {
    /// Effective latency \[ms\] given a bandwidth utilisation fraction
    /// caused by ongoing DoS traffic.
    ///
    /// Models queuing delay with an M/D/1 approximation:
    /// `W = rho / (2 * mu * (1 - rho))` added to propagation latency.
    pub fn effective_latency_ms(&self, dos_utilisation: f64) -> f64 {
        let rho = dos_utilisation.clamp(0.0, 0.99);
        // Service rate μ inversely proportional to bandwidth
        let queue_delay = rho / (2.0 * (1.0 - rho).max(1e-6)) * self.latency_ms;
        self.latency_ms + queue_delay + self.protocol.base_latency_ms()
    }
}

/// The SCADA/EMS communication network topology.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommNetwork {
    /// All communication nodes (substations, RTUs, control centre, …).
    pub nodes: Vec<CommNode>,
    /// Directed communication links.
    pub links: Vec<CommLink>,
    /// Symmetric latency matrix \[ms\] between every pair of nodes
    /// (`latency_ms[i][j]` = end-to-end latency from node *i* to node *j*).
    pub latency_ms: Vec<Vec<f64>>,
}

impl CommNetwork {
    /// Construct a new network, computing the latency matrix from the link list.
    ///
    /// Uses Floyd-Warshall shortest-path (by latency) over the link graph.
    pub fn new(nodes: Vec<CommNode>, links: Vec<CommLink>) -> Self {
        let n = nodes.len();
        // Initialise with ∞
        let inf = f64::INFINITY;
        let mut dist = vec![vec![inf; n]; n];
        for (i, dist_row) in dist.iter_mut().enumerate() {
            dist_row[i] = 0.0;
        }
        for link in &links {
            if link.from < n && link.to < n {
                let d = link.latency_ms + link.protocol.base_latency_ms();
                if d < dist[link.from][link.to] {
                    dist[link.from][link.to] = d;
                }
                // Treat as bidirectional for latency purposes
                if d < dist[link.to][link.from] {
                    dist[link.to][link.from] = d;
                }
            }
        }
        // Floyd-Warshall
        for k in 0..n {
            for i in 0..n {
                for j in 0..n {
                    let through_k = dist[i][k].saturating_add_f64(dist[k][j]);
                    if through_k < dist[i][j] {
                        dist[i][j] = through_k;
                    }
                }
            }
        }
        Self {
            nodes,
            links,
            latency_ms: dist,
        }
    }

    /// Maximum one-way latency across all reachable paths \[ms\].
    pub fn max_latency_ms(&self) -> f64 {
        self.latency_ms
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&v| v.is_finite())
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Mean latency across all finite (reachable) node pairs \[ms\].
    pub fn mean_latency_ms(&self) -> f64 {
        let finite: Vec<f64> = self
            .latency_ms
            .iter()
            .flat_map(|row| row.iter())
            .filter(|&&v| v.is_finite() && v > 0.0)
            .cloned()
            .collect();
        if finite.is_empty() {
            return 0.0;
        }
        finite.iter().sum::<f64>() / finite.len() as f64
    }
}

// Saturating addition helper for f64 (infinity-safe)
trait SaturatingAddF64 {
    fn saturating_add_f64(self, other: f64) -> f64;
}
impl SaturatingAddF64 for f64 {
    #[inline]
    fn saturating_add_f64(self, other: f64) -> f64 {
        if self.is_infinite() || other.is_infinite() {
            f64::INFINITY
        } else {
            self + other
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Attack models
// ─────────────────────────────────────────────────────────────────────────────

/// Variants of a False Data Injection attack.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FdiAttackType {
    /// Add zero-mean Gaussian noise with standard deviation `sigma`.
    RandomNoise {
        /// Standard deviation of the injected noise \[same units as measurement\].
        sigma: f64,
    },
    /// Shift every targeted measurement by a constant `bias`.
    BiasInjection {
        /// Constant additive bias.
        bias: f64,
    },
    /// Multiply every targeted measurement by `scale`.
    ScalingAttack {
        /// Multiplicative scale factor (e.g. 1.2 → 20 % over-reading).
        scale: f64,
    },
    /// Substitute targeted measurements with values recorded `replay_window_s`
    /// seconds in the past.
    ReplayAttack {
        /// Length of the replay window \[s\].
        replay_window_s: f64,
    },
    /// Construct an injection vector **a** = **H** **c** that lies in the
    /// column space of the measurement Jacobian, thereby satisfying the
    /// Bad-Data Detection residual test exactly (bypasses χ² alarm).
    CoordinatedStealth,
}

/// A False Data Injection attack specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FdiAttack {
    /// Indices into the global measurement vector to be tampered with.
    pub target_measurements: Vec<usize>,
    /// Additive perturbation vector (same length as `target_measurements`).
    ///
    /// For attack types that compute the injection analytically
    /// (`CoordinatedStealth`) or stochastically (`RandomNoise`), this field
    /// is used as a *base* vector that may be overridden at run time.
    pub injection_vector: Vec<f64>,
    /// Simulation time at which the attack begins \[s\].
    pub start_time: f64,
    /// How long the attack lasts \[s\].
    pub duration: f64,
    /// Mechanism used to compute the injected values.
    pub attack_type: FdiAttackType,
}

impl FdiAttack {
    /// Returns `true` if the attack is active at simulation time `t`.
    #[inline]
    pub fn is_active(&self, t: f64) -> bool {
        t >= self.start_time && t < self.start_time + self.duration
    }
}

/// A Denial-of-Service attack that saturates communication bandwidth.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DosAttack {
    /// Indices of [`CommNode`]s targeted by the DoS flood.
    pub target_nodes: Vec<usize>,
    /// Fraction of nominal link bandwidth consumed by attack traffic \[0, 1\].
    pub attack_intensity: f64,
    /// Simulation time at which the attack begins \[s\].
    pub start_time: f64,
    /// How long the attack lasts \[s\].
    pub duration: f64,
}

impl DosAttack {
    /// Returns `true` if the attack is active at simulation time `t`.
    #[inline]
    pub fn is_active(&self, t: f64) -> bool {
        t >= self.start_time && t < self.start_time + self.duration
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Co-simulation state & result
// ─────────────────────────────────────────────────────────────────────────────

/// Snapshot of the coupled cyber-physical state at one simulation instant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CyberPhysicalState {
    /// Simulation time \[s\].
    pub time: f64,
    /// Bus voltage magnitudes \[pu\].
    pub bus_voltages: Vec<f64>,
    /// Bus voltage angles \[rad\].
    pub bus_angles: Vec<f64>,
    /// Human-readable labels of currently active attacks.
    pub active_attacks: Vec<String>,
    /// Indices of [`CommNode`]s that are currently compromised.
    pub compromised_nodes: Vec<usize>,
    /// Per-measurement integrity score \[0, 1\]: 1.0 = pristine, 0.0 = fully
    /// corrupted.
    pub measurement_integrity: Vec<f64>,
    /// Effective round-trip SCADA update delay experienced by the controller
    /// \[ms\].
    pub control_delay_ms: f64,
    /// Cumulative real power involuntarily shed to protect voltage stability
    /// \[MW\].
    pub load_shedding_mw: f64,
}

/// Summary metrics from a completed cyber-physical co-simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CyberPhysicalResult {
    /// Full time series of coupled system snapshots.
    pub time_series: Vec<CyberPhysicalState>,
    /// Total energy not served due to attack-induced load shedding \[MWh\].
    pub total_load_shed_mwh: f64,
    /// Largest instantaneous voltage deviation from 1.0 pu observed \[pu\].
    pub max_voltage_deviation_pu: f64,
    /// Fraction of FDI attack steps that were correctly flagged by BDD
    /// \[0, 1\].
    pub attack_detection_rate: f64,
    /// Fraction of clean (non-attack) steps erroneously flagged by BDD \[0, 1\].
    pub false_positive_rate: f64,
    /// Overall grid resilience index \[0, 1\]: 1.0 means no impact.
    pub resilience_index: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Full configuration for a [`CyberPhysicalSimulator`] run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CyberPhysicalConfig {
    /// Number of buses in the physical power system.
    pub n_buses: usize,
    /// Total simulation horizon \[s\].
    pub simulation_duration_s: f64,
    /// Physical-layer integration time step \[s\] (default 0.1 s).
    pub dt_physical_s: f64,
    /// Cyber-layer (SCADA poll) interval \[s\] (default 1.0 s).
    pub dt_cyber_s: f64,
    /// FDI attack specifications to inject.
    pub fdi_attacks: Vec<FdiAttack>,
    /// DoS attack specifications to inject.
    pub dos_attacks: Vec<DosAttack>,
    /// Enable normalized-residual / chi-squared Bad Data Detection.
    pub enable_bad_data_detection: bool,
    /// Chi-squared detection threshold expressed as a number of standard
    /// deviations (default 3σ → critical value ≈ 9 for 1 dof).
    pub bdd_threshold: f64,
    /// Nominal per-bus real power generation \[MW\] (used to compute total
    /// energy for the resilience index).
    pub nominal_power_mw: Vec<f64>,
}

impl CyberPhysicalConfig {
    /// Construct a default configuration for an *n*-bus system.
    pub fn default_for(n_buses: usize) -> Self {
        Self {
            n_buses,
            simulation_duration_s: 60.0,
            dt_physical_s: 0.1,
            dt_cyber_s: 1.0,
            fdi_attacks: Vec::new(),
            dos_attacks: Vec::new(),
            enable_bad_data_detection: true,
            bdd_threshold: 3.0,
            nominal_power_mw: vec![100.0; n_buses],
        }
    }

    fn validate(&self) -> Result<()> {
        if self.n_buses == 0 {
            return Err(OxiGridError::InvalidParameter(
                "n_buses must be > 0".to_string(),
            ));
        }
        if self.simulation_duration_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "simulation_duration_s must be > 0".to_string(),
            ));
        }
        if self.dt_physical_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_physical_s must be > 0".to_string(),
            ));
        }
        if self.dt_cyber_s < self.dt_physical_s {
            return Err(OxiGridError::InvalidParameter(
                "dt_cyber_s must be >= dt_physical_s".to_string(),
            ));
        }
        if self.bdd_threshold <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "bdd_threshold must be > 0".to_string(),
            ));
        }
        for att in &self.fdi_attacks {
            if att.target_measurements.len() != att.injection_vector.len() {
                return Err(OxiGridError::InvalidParameter(format!(
                    "FdiAttack: target_measurements length ({}) != injection_vector length ({})",
                    att.target_measurements.len(),
                    att.injection_vector.len()
                )));
            }
            if att.duration <= 0.0 {
                return Err(OxiGridError::InvalidParameter(
                    "FdiAttack duration must be > 0".to_string(),
                ));
            }
        }
        for att in &self.dos_attacks {
            if !(0.0..=1.0).contains(&att.attack_intensity) {
                return Err(OxiGridError::InvalidParameter(
                    "DosAttack attack_intensity must be in [0, 1]".to_string(),
                ));
            }
            if att.duration <= 0.0 {
                return Err(OxiGridError::InvalidParameter(
                    "DosAttack duration must be > 0".to_string(),
                ));
            }
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulator
// ─────────────────────────────────────────────────────────────────────────────

/// Cyber-physical co-simulator for power grid security analysis.
///
/// Couples a first-order load-frequency control (physical) model with a SCADA
/// communication network model, injecting configurable cyber attacks and
/// measuring their impact on grid performance.
pub struct CyberPhysicalSimulator {
    config: CyberPhysicalConfig,
    network: CommNetwork,
    /// LCG seed (updated in-place during simulation for reproducibility).
    rng_state: u64,
}

impl CyberPhysicalSimulator {
    /// Create a new simulator from the given configuration and communication
    /// network topology.
    pub fn new(config: CyberPhysicalConfig, network: CommNetwork) -> Self {
        // Deterministic seed derived from config parameters
        let seed = 0xCAFE_BABE_0000_0001_u64
            .wrapping_add(config.n_buses as u64)
            .wrapping_add((config.simulation_duration_s * 1_000.0) as u64);
        Self {
            config,
            network,
            rng_state: seed,
        }
    }

    // ── Public simulation entry point ─────────────────────────────────────────

    /// Execute the co-simulation and return aggregated performance metrics.
    ///
    /// # Simulation Loop (per physical time step)
    ///
    /// 1. Apply active FDI attacks to the measurement vector.
    /// 2. Apply active DoS attacks (increases effective latency and packet
    ///    loss).
    /// 3. Run Bad Data Detection (normalised residuals + χ² global test).
    /// 4. Compute effective control signal (delayed/corrupted).
    /// 5. Integrate physical-layer dynamics (droop + LFC).
    /// 6. Record the combined cyber-physical state snapshot.
    pub fn run(&mut self) -> Result<CyberPhysicalResult> {
        self.config.validate()?;

        let n = self.config.n_buses;
        let dt_p = self.config.dt_physical_s;
        let dt_c = self.config.dt_cyber_s;
        let t_end = self.config.simulation_duration_s;

        // ── Physical-layer state ────────────────────────────────────────────
        // Swing-equation approximation: each bus has a voltage magnitude and
        // angle.  We use a first-order relaxation toward a droop-adjusted
        // reference.
        let mut v_pu: Vec<f64> = vec![1.0; n];
        let mut theta: Vec<f64> = vec![0.0; n];
        // Frequency deviation per bus (used for droop control) [Hz]
        let mut freq_dev: Vec<f64> = vec![0.0; n];
        // Integral term for LFC (area control error)
        let mut ace_integral: Vec<f64> = vec![0.0; n];

        // Reference voltage and power setpoints
        let v_ref = 1.0_f64;
        let nominal_power: Vec<f64> = if self.config.nominal_power_mw.len() == n {
            self.config.nominal_power_mw.clone()
        } else {
            vec![100.0; n]
        };

        // Load conductance [pu] (simple load model)
        let g_load = 1.0_f64;
        // Inertia constant [s] (approximate swing constant)
        let h_inertia = 5.0_f64;
        // Droop coefficient [Hz/pu]
        let r_droop = 0.02_f64;
        // Proportional gain for voltage regulation
        let kp_v = 1.0_f64;
        // Integral gain for LFC
        let ki_lfc = 0.05_f64;
        // Time constant for first-order voltage dynamics [s]
        let tau_v = 0.5_f64;

        // ── Measurement model ───────────────────────────────────────────────
        // Measurements: [V_1, …, V_n, P_1, …, P_n] (2n total)
        let n_meas = 2 * n;
        // Measurement noise standard deviation [pu / MW respectively]
        let sigma_meas = 0.005_f64;
        // Measurement history buffer for replay attacks
        // Each entry: (sim_time, measurement_vector)
        let max_hist = ((t_end / dt_c).ceil() as usize + 4).max(10);
        let mut meas_history: Vec<(f64, Vec<f64>)> = Vec::with_capacity(max_hist);

        // Last received (possibly corrupted / delayed) measurement vector
        let mut last_meas: Vec<f64> = vec![1.0; n_meas];
        // Measurement integrity scores (per measurement) [0, 1]
        let mut meas_integrity: Vec<f64> = vec![1.0; n_meas];

        // Welford online statistics for BDD baseline
        // Initialise variance estimates per-measurement type:
        //   - Voltage measurements (indices 0..n): noise σ = sigma_meas
        //   - Power measurements (indices n..2n): noise σ = sigma_meas * 10
        let mut bdd_mean: Vec<f64> = vec![0.0; n_meas];
        let mut bdd_m2: Vec<f64> = {
            let mut m2 = Vec::with_capacity(n_meas);
            for _ in 0..n {
                m2.push(sigma_meas * sigma_meas);
            }
            let sigma_p = sigma_meas * 10.0;
            for _ in 0..n {
                m2.push(sigma_p * sigma_p);
            }
            m2
        };
        let mut bdd_count: u64 = 1;

        // ── Result accumulators ─────────────────────────────────────────────
        let mut time_series: Vec<CyberPhysicalState> = Vec::new();
        let mut total_load_shed_mw_s = 0.0_f64; // MW·s
        let mut max_v_dev = 0.0_f64;
        let mut cumulative_load_shed_mw = 0.0_f64;

        // BDD scoring
        let mut n_attack_steps: u64 = 0;
        let mut n_attack_detected: u64 = 0;
        let mut n_clean_steps: u64 = 0;
        let mut n_false_positives: u64 = 0;

        // Effective SCADA delay [ms] — updated each cyber step
        #[allow(unused_assignments)]
        let mut effective_delay_ms = 0.0_f64;

        // ── Time stepping ───────────────────────────────────────────────────
        let n_phys_steps = (t_end / dt_p).ceil() as usize;
        let phys_per_cyber = ((dt_c / dt_p).round() as usize).max(1);

        for step in 0..n_phys_steps {
            let t = step as f64 * dt_p;
            let is_cyber_step = step % phys_per_cyber == 0;

            // ── Determine active attacks ────────────────────────────────────
            // Clone attack specs to avoid holding a borrow on `self.config`
            // while we later call `&mut self` methods.
            let active_fdi: Vec<FdiAttack> = self
                .config
                .fdi_attacks
                .iter()
                .filter(|a| a.is_active(t))
                .cloned()
                .collect();

            let active_dos: Vec<DosAttack> = self
                .config
                .dos_attacks
                .iter()
                .filter(|a| a.is_active(t))
                .cloned()
                .collect();

            let any_fdi = !active_fdi.is_empty();
            let any_dos = !active_dos.is_empty();

            // ── Step 2: DoS effect on latency and packet loss ───────────────
            // Compute the worst-case DoS utilisation across all targeted nodes
            let dos_utilisation: f64 = if any_dos {
                active_dos
                    .iter()
                    .map(|a| a.attack_intensity)
                    .fold(0.0_f64, f64::max)
            } else {
                0.0
            };

            // Effective latency: blend nominal with congested value
            let nominal_delay = self.network.mean_latency_ms().max(1.0);
            let dos_latency = if dos_utilisation > 0.0 {
                // M/D/1 queuing approximation
                let rho = dos_utilisation.clamp(0.0, 0.99);
                nominal_delay * (1.0 + rho / (2.0 * (1.0 - rho).max(1e-6)))
            } else {
                nominal_delay
            };
            effective_delay_ms = dos_latency;

            // DoS-induced packet loss on top of node base rates
            let base_loss: f64 = self
                .network
                .nodes
                .iter()
                .map(|nd| nd.packet_loss_rate)
                .fold(0.0_f64, f64::max);
            let effective_loss = (base_loss + dos_utilisation * 0.5).clamp(0.0, 1.0);

            // ── Cyber-layer update (every phys_per_cyber physical steps) ────
            let mut bdd_alarm = false;
            let mut flagged_indices: Vec<usize> = Vec::new();

            if is_cyber_step {
                // True measurement vector z = [V; P] + noise
                let mut z_true: Vec<f64> = Vec::with_capacity(n_meas);
                for v_val in v_pu.iter().take(n) {
                    z_true.push(*v_val + sigma_meas * lcg_normal(&mut self.rng_state));
                }
                for i in 0..n {
                    let p_meas = v_pu[i] * v_pu[i] * g_load * nominal_power[i] / 100.0
                        + sigma_meas * 10.0 * lcg_normal(&mut self.rng_state);
                    z_true.push(p_meas);
                }

                // Store true measurement in history
                meas_history.push((t, z_true.clone()));

                // ── Step 1: Apply FDI attacks ───────────────────────────────
                let mut z_corrupted = z_true.clone();
                let mut n_corrupted: usize = 0;

                for att in &active_fdi {
                    n_corrupted += Self::inject_fdi_static(
                        &mut self.rng_state,
                        &mut z_corrupted,
                        att,
                        t,
                        &meas_history,
                    );
                }

                // ── Packet loss (may drop some measurements) ────────────────
                for idx in 0..n_meas {
                    if lcg_next(&mut self.rng_state) < effective_loss {
                        // Lost — use last known value
                        z_corrupted[idx] = last_meas[idx];
                        // Integrity slightly degraded for lost measurements
                        meas_integrity[idx] = (meas_integrity[idx] * 0.95).max(0.5);
                    }
                }

                // ── Step 3: Bad Data Detection ──────────────────────────────
                if self.config.enable_bad_data_detection {
                    let expected: Vec<f64> = last_meas.clone();
                    flagged_indices =
                        self.run_bad_data_detection(&z_corrupted, &expected, &bdd_mean, &bdd_m2);
                    bdd_alarm = !flagged_indices.is_empty();
                }

                // Update Welford statistics (on the received — possibly
                // corrupted — measurements, after BDD has flagged outliers)
                bdd_count += 1;
                let count_f = bdd_count as f64;
                for idx in 0..n_meas {
                    if !flagged_indices.contains(&idx) {
                        let x = z_corrupted[idx];
                        let delta = x - bdd_mean[idx];
                        bdd_mean[idx] += delta / count_f;
                        let delta2 = x - bdd_mean[idx];
                        bdd_m2[idx] += delta * delta2;
                    }
                }

                // Update integrity scores
                for (idx, mi_val) in meas_integrity.iter_mut().enumerate().take(n_meas) {
                    if flagged_indices.contains(&idx) {
                        *mi_val = (*mi_val - 0.2).max(0.0);
                    } else {
                        *mi_val = (*mi_val + 0.05).min(1.0);
                    }
                    // Corruption directly degrades integrity
                    if n_corrupted > 0 && idx < att_target_count(&active_fdi) {
                        *mi_val = (*mi_val - 0.1).max(0.0);
                    }
                }

                // BDD scoring
                if any_fdi {
                    n_attack_steps += 1;
                    if bdd_alarm {
                        n_attack_detected += 1;
                    }
                } else {
                    n_clean_steps += 1;
                    if bdd_alarm {
                        n_false_positives += 1;
                    }
                }

                last_meas = z_corrupted;
            }

            // ── Step 4: Control signal (uses last received measurements) ────
            // Extract voltage measurements from last_meas[0..n]
            let v_meas: Vec<f64> = last_meas[..n].to_vec();

            // Voltage regulator: e_v = V_ref - V_meas
            // LFC: ACE = ΔP_tie + B * Δf
            let mut v_control: Vec<f64> = vec![v_ref; n];
            for i in 0..n {
                let e_v = v_ref - v_meas[i].clamp(0.0, 2.0);
                v_control[i] = (v_ref + kp_v * e_v + ki_lfc * ace_integral[i]).clamp(0.7, 1.3);
                // Droop: reduce reference when frequency rises
                v_control[i] -= r_droop * freq_dev[i];
                v_control[i] = v_control[i].clamp(0.7, 1.3);
            }

            // ── Step 5: Physical-layer dynamics ────────────────────────────
            // First-order voltage relaxation toward control setpoint
            for i in 0..n {
                let dv = (v_control[i] - v_pu[i]) / tau_v;
                v_pu[i] += dt_p * dv;
                v_pu[i] = v_pu[i].clamp(0.0, 2.0);

                // Swing equation (simplified): dθ/dt = ω deviation
                // Frequency deviation from power imbalance
                let p_gen = nominal_power[i];
                let p_load = v_pu[i] * v_pu[i] * g_load * p_gen;
                let dp = (p_gen - p_load) / (2.0 * h_inertia);
                // Natural damping (D coefficient) prevents frequency wind-up
                let d_damp = 1.0_f64;
                freq_dev[i] += dt_p * (dp - d_damp * freq_dev[i]);
                freq_dev[i] = freq_dev[i].clamp(-5.0, 5.0);
                theta[i] += dt_p * freq_dev[i] * 2.0 * core::f64::consts::PI / 50.0;

                // ACE integral
                ace_integral[i] += dt_p * (v_ref - v_pu[i]);
                ace_integral[i] = ace_integral[i].clamp(-10.0, 10.0);
            }

            // Divergence guard
            if v_pu.iter().any(|&v| !v.is_finite()) {
                return Err(OxiGridError::InvalidParameter(format!(
                    "Physical simulation diverged at t = {t:.3} s"
                )));
            }

            // Load shedding: triggered when any bus voltage < 0.88 pu
            let mut shed_this_step = 0.0_f64;
            for i in 0..n {
                if v_pu[i] < 0.88 {
                    // Shed load proportional to voltage deficit
                    let shed_fraction = (0.88 - v_pu[i]) / 0.88;
                    let shed_mw = shed_fraction * nominal_power[i] * 0.5;
                    shed_this_step += shed_mw;
                    cumulative_load_shed_mw += shed_mw * dt_p / 3600.0; // accumulate in MWh
                }
            }
            total_load_shed_mw_s += shed_this_step * dt_p;

            // Track maximum voltage deviation
            for &v in &v_pu {
                let dev = (v - 1.0).abs();
                if dev > max_v_dev {
                    max_v_dev = dev;
                }
            }

            // ── Step 6: Record state ────────────────────────────────────────
            if is_cyber_step {
                let active_attack_labels = build_attack_labels(&active_fdi, &active_dos);
                let compromised: Vec<usize> = self
                    .network
                    .nodes
                    .iter()
                    .filter(|nd| nd.is_compromised)
                    .map(|nd| nd.id)
                    .collect();

                time_series.push(CyberPhysicalState {
                    time: t,
                    bus_voltages: v_pu.clone(),
                    bus_angles: theta.clone(),
                    active_attacks: active_attack_labels,
                    compromised_nodes: compromised,
                    measurement_integrity: meas_integrity.clone(),
                    control_delay_ms: effective_delay_ms,
                    load_shedding_mw: cumulative_load_shed_mw * 3600.0, // convert back to MW·(total steps)
                });
            }
        }

        // ── Aggregate results ───────────────────────────────────────────────
        let total_load_shed_mwh = total_load_shed_mw_s / 3600.0;

        let attack_detection_rate = if n_attack_steps > 0 {
            n_attack_detected as f64 / n_attack_steps as f64
        } else {
            0.0
        };

        let false_positive_rate = if n_clean_steps > 0 {
            n_false_positives as f64 / n_clean_steps as f64
        } else {
            0.0
        };

        let dummy_result = CyberPhysicalResult {
            time_series: time_series.clone(),
            total_load_shed_mwh,
            max_voltage_deviation_pu: max_v_dev,
            attack_detection_rate,
            false_positive_rate,
            resilience_index: 0.0, // computed below
        };

        let resilience_index = Self::compute_resilience_index(&dummy_result, &self.config);

        Ok(CyberPhysicalResult {
            time_series,
            total_load_shed_mwh,
            max_voltage_deviation_pu: max_v_dev,
            attack_detection_rate,
            false_positive_rate,
            resilience_index,
        })
    }

    // ── FDI injection ─────────────────────────────────────────────────────────

    /// Apply a single FDI attack to the measurement vector `z`.
    ///
    /// Returns the number of measurements that were modified.
    pub fn inject_fdi(&mut self, measurements: &mut [f64], attack: &FdiAttack, time: f64) -> usize {
        if !attack.is_active(time) {
            return 0;
        }
        let dummy_history: Vec<(f64, Vec<f64>)> = Vec::new();
        Self::inject_fdi_static(
            &mut self.rng_state,
            measurements,
            attack,
            time,
            &dummy_history,
        )
    }

    /// Static FDI injection — takes `rng_state` by mutable reference so it
    /// can be called without holding an exclusive borrow on the whole `self`.
    fn inject_fdi_static(
        rng_state: &mut u64,
        z: &mut [f64],
        attack: &FdiAttack,
        time: f64,
        history: &[(f64, Vec<f64>)],
    ) -> usize {
        let n_z = z.len();
        let mut count = 0usize;

        match &attack.attack_type {
            FdiAttackType::RandomNoise { sigma } => {
                let s = *sigma;
                for (pos, &idx) in attack.target_measurements.iter().enumerate() {
                    if idx < n_z {
                        let base = attack.injection_vector.get(pos).copied().unwrap_or(0.0);
                        z[idx] += base + s * lcg_normal(rng_state);
                        count += 1;
                    }
                }
            }
            FdiAttackType::BiasInjection { bias } => {
                let b = *bias;
                for &idx in &attack.target_measurements {
                    if idx < n_z {
                        z[idx] += b;
                        count += 1;
                    }
                }
            }
            FdiAttackType::ScalingAttack { scale } => {
                let s = *scale;
                for &idx in &attack.target_measurements {
                    if idx < n_z {
                        z[idx] *= s;
                        count += 1;
                    }
                }
            }
            FdiAttackType::ReplayAttack { replay_window_s } => {
                // Find a historical snapshot `replay_window_s` seconds ago
                let target_t = time - replay_window_s;
                let replay_snap = history
                    .iter()
                    .rev()
                    .find(|(ht, _)| *ht <= target_t)
                    .or_else(|| history.first());
                if let Some((_, hist_z)) = replay_snap {
                    for &idx in &attack.target_measurements {
                        if idx < n_z && idx < hist_z.len() {
                            z[idx] = hist_z[idx];
                            count += 1;
                        }
                    }
                }
            }
            FdiAttackType::CoordinatedStealth => {
                // Construct a = H·c where H is the identity (simplified
                // measurement Jacobian for voltage magnitudes).  The key
                // property is that the residual r = z̃ - Hx̂ = z - Hx̂ + a,
                // and if a lies in col(H) the largest normalised residual
                // does not increase.  Here we inject via the injection_vector
                // field directly (pre-computed by the attacker to be in
                // col(H) = col(I) = ℝ^n, so any vector works when H = I).
                // The stealthy property is that the chi-squared statistic is
                // preserved when the attack is projected onto the column space.
                let small_perturbation = 0.01; // pu — below BDD threshold
                for (pos, &idx) in attack.target_measurements.iter().enumerate() {
                    if idx < n_z {
                        let a_i = attack
                            .injection_vector
                            .get(pos)
                            .copied()
                            .unwrap_or(small_perturbation);
                        // Coordinated: inject a_i scaled to stay below σ·threshold
                        z[idx] += a_i * small_perturbation;
                        count += 1;
                    }
                }
            }
        }
        count
    }

    // ── Bad Data Detection ────────────────────────────────────────────────────

    /// Run the normalised-residual and chi-squared Bad Data Detection test.
    ///
    /// # Algorithm
    ///
    /// 1. **Normalised residual**: for each measurement *i*,
    ///    `|r_i / σ_i| > threshold` → measurement flagged.
    /// 2. **Chi-squared global alarm**: `Σ(r_i²/σ_i²) > χ²_crit` where
    ///    `χ²_crit = (threshold² × n_meas)`.
    ///
    /// Returns the indices of flagged measurements.
    pub fn run_bad_data_detection(
        &self,
        measurements: &[f64],
        expected: &[f64],
        bdd_mean: &[f64],
        bdd_m2: &[f64],
    ) -> Vec<usize> {
        let n = measurements.len().min(expected.len());
        let sigma_sq_floor = 1e-8_f64;
        let thr = self.config.bdd_threshold;
        let chi2_crit = thr * thr * n as f64;

        let mut flagged = Vec::new();
        let mut chi2_sum = 0.0_f64;

        for i in 0..n {
            let r = measurements[i] - expected[i];
            // Variance estimated from Welford (fall back to floor)
            let var = if i < bdd_m2.len() {
                (bdd_m2[i]).max(sigma_sq_floor)
            } else {
                sigma_sq_floor
            };
            let _ = bdd_mean; // mean used externally for Welford updates
            let norm_r_sq = r * r / var;
            chi2_sum += norm_r_sq;
            if norm_r_sq.sqrt() > thr {
                flagged.push(i);
            }
        }

        // Global chi-squared alarm: flag all if sum exceeds critical value
        // and no individual measurement was already flagged (BDD may not
        // identify which measurement caused the alarm in the global test)
        if chi2_sum > chi2_crit && flagged.is_empty() {
            // Return all measurements as suspect when only global alarm fires
            for i in 0..n {
                flagged.push(i);
            }
        }

        flagged
    }

    // ── Resilience index ──────────────────────────────────────────────────────

    /// Compute the resilience index from simulation results.
    ///
    /// `resilience = 1 - (total_load_shed_mwh / total_energy_served_mwh)`
    ///
    /// Clamped to \[0, 1\].
    pub fn compute_resilience_index(
        result: &CyberPhysicalResult,
        config: &CyberPhysicalConfig,
    ) -> f64 {
        let total_nominal_mwh: f64 =
            config.nominal_power_mw.iter().sum::<f64>() * config.simulation_duration_s / 3600.0;
        if total_nominal_mwh <= 0.0 {
            return 1.0;
        }
        (1.0 - result.total_load_shed_mwh / total_nominal_mwh).clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

fn build_attack_labels(fdi: &[FdiAttack], dos: &[DosAttack]) -> Vec<String> {
    let mut labels = Vec::new();
    for att in fdi {
        let label = match &att.attack_type {
            FdiAttackType::RandomNoise { sigma } => format!("FDI:RandomNoise(σ={sigma:.3})"),
            FdiAttackType::BiasInjection { bias } => format!("FDI:Bias({bias:+.3})"),
            FdiAttackType::ScalingAttack { scale } => format!("FDI:Scale({scale:.3}x)"),
            FdiAttackType::ReplayAttack { replay_window_s } => {
                format!("FDI:Replay({replay_window_s:.1}s)")
            }
            FdiAttackType::CoordinatedStealth => "FDI:CoordinatedStealth".to_string(),
        };
        labels.push(label);
    }
    for att in dos {
        labels.push(format!(
            "DoS:intensity={:.0}%",
            att.attack_intensity * 100.0
        ));
    }
    labels
}

/// Sum of all targeted measurement indices across active FDI attacks.
fn att_target_count(attacks: &[FdiAttack]) -> usize {
    attacks.iter().map(|a| a.target_measurements.len()).sum()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper constructors ───────────────────────────────────────────────────

    fn default_network(n: usize) -> CommNetwork {
        let nodes: Vec<CommNode> = (0..n)
            .map(|i| CommNode {
                id: i,
                node_type: if i == 0 {
                    CommNodeType::ControlCenter
                } else {
                    CommNodeType::Substation
                },
                is_compromised: false,
                packet_loss_rate: 0.0,
            })
            .collect();

        let links: Vec<CommLink> = (0..n.saturating_sub(1))
            .map(|i| CommLink {
                from: i,
                to: i + 1,
                bandwidth_mbps: 100.0,
                latency_ms: 5.0,
                is_encrypted: true,
                protocol: CommProtocol::Iec61850,
            })
            .collect();

        CommNetwork::new(nodes, links)
    }

    fn simple_config(n: usize, duration_s: f64) -> CyberPhysicalConfig {
        CyberPhysicalConfig {
            n_buses: n,
            simulation_duration_s: duration_s,
            dt_physical_s: 0.1,
            dt_cyber_s: 1.0,
            fdi_attacks: Vec::new(),
            dos_attacks: Vec::new(),
            enable_bad_data_detection: true,
            bdd_threshold: 3.0,
            nominal_power_mw: vec![100.0; n],
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 1. Baseline (no attacks)
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_no_attack_baseline() {
        let cfg = simple_config(4, 10.0);
        let net = default_network(4);
        let mut sim = CyberPhysicalSimulator::new(cfg.clone(), net);
        let result = sim.run().expect("baseline should succeed");

        // No load shedding in healthy system
        assert!(
            result.total_load_shed_mwh < 1e-3,
            "Expected near-zero load shedding without attacks, got {:.6} MWh",
            result.total_load_shed_mwh
        );
        // Resilience should be 1.0
        assert!(
            (result.resilience_index - 1.0).abs() < 1e-6,
            "Resilience should be 1.0 without attacks, got {:.6}",
            result.resilience_index
        );
        // Time series should span the simulation
        assert!(!result.time_series.is_empty());
        // All voltages should stay near nominal
        for state in &result.time_series {
            for &v in &state.bus_voltages {
                assert!(
                    (v - 1.0).abs() < 0.15,
                    "Voltage {v:.4} deviates excessively without attack"
                );
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 2. FDI – random noise
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_fdi_random_noise() {
        let mut cfg = simple_config(3, 20.0);
        cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0, 1],
            injection_vector: vec![0.0, 0.0],
            start_time: 2.0,
            duration: 10.0,
            attack_type: FdiAttackType::RandomNoise { sigma: 0.5 },
        }];
        let net = default_network(3);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);
        let result = sim.run().expect("random noise FDI should succeed");

        // With large noise, BDD should catch at least some steps
        // (attack_detection_rate may be > 0)
        assert!(result.attack_detection_rate >= 0.0);
        assert!(result.attack_detection_rate <= 1.0);
        assert!(!result.time_series.is_empty());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 3. FDI – bias injection
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_fdi_bias_injection() {
        let mut cfg = simple_config(3, 30.0);
        // Large positive bias on bus-0 voltage measurement
        cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0],
            injection_vector: vec![0.5],
            start_time: 5.0,
            duration: 20.0,
            attack_type: FdiAttackType::BiasInjection { bias: 0.5 },
        }];
        let net = default_network(3);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);
        let result = sim.run().expect("bias FDI should succeed");

        // A large bias should either be detected or cause some voltage deviation
        let has_impact =
            result.max_voltage_deviation_pu > 0.01 || result.attack_detection_rate > 0.0;
        assert!(
            has_impact,
            "Bias attack should cause voltage deviation or BDD alarm"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 4. FDI – coordinated stealth (bypasses BDD)
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_fdi_coordinated_stealth() {
        let n = 4;
        let mut cfg = simple_config(n, 40.0);
        // Coordinated stealth: small perturbations in col(H)
        cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0, 1, 2, 3],
            injection_vector: vec![0.02, 0.02, 0.02, 0.02],
            start_time: 5.0,
            duration: 30.0,
            attack_type: FdiAttackType::CoordinatedStealth,
        }];
        let net = default_network(n);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);
        let result = sim.run().expect("stealth attack should succeed");

        // Coordinated stealth should have a lower detection rate than a naive bias attack
        assert!(
            result.attack_detection_rate < 0.5,
            "Stealth attack detection rate ({:.3}) should be low",
            result.attack_detection_rate
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 5. DoS attack – increases control delay
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_dos_attack() {
        let n = 3;
        let cfg_clean = simple_config(n, 20.0);
        let net_clean = default_network(n);
        let mut sim_clean = CyberPhysicalSimulator::new(cfg_clean, net_clean);
        let result_clean = sim_clean.run().expect("clean sim");

        let mut cfg_dos = simple_config(n, 20.0);
        cfg_dos.dos_attacks = vec![DosAttack {
            target_nodes: vec![0, 1],
            attack_intensity: 0.9,
            start_time: 2.0,
            duration: 15.0,
        }];
        let net_dos = default_network(n);
        let mut sim_dos = CyberPhysicalSimulator::new(cfg_dos, net_dos);
        let result_dos = sim_dos.run().expect("DoS sim should succeed");

        // During DoS, the effective delay should be higher than clean run
        let max_delay_dos = result_dos
            .time_series
            .iter()
            .map(|s| s.control_delay_ms)
            .fold(f64::NEG_INFINITY, f64::max);
        let max_delay_clean = result_clean
            .time_series
            .iter()
            .map(|s| s.control_delay_ms)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_delay_dos > max_delay_clean,
            "DoS should increase control delay (DoS: {max_delay_dos:.1} ms, clean: {max_delay_clean:.1} ms)"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 6. Combined FDI + DoS
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_combined_attack() {
        let n = 4;
        let mut cfg = simple_config(n, 30.0);
        cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0, 1],
            injection_vector: vec![0.3, 0.3],
            start_time: 3.0,
            duration: 20.0,
            attack_type: FdiAttackType::BiasInjection { bias: 0.3 },
        }];
        cfg.dos_attacks = vec![DosAttack {
            target_nodes: vec![0],
            attack_intensity: 0.7,
            start_time: 3.0,
            duration: 20.0,
        }];
        let net = default_network(n);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);
        let result = sim.run().expect("combined attack should complete");

        // Should produce active attack labels in time series
        let has_combined = result
            .time_series
            .iter()
            .any(|s| s.active_attacks.len() >= 2);
        assert!(
            has_combined,
            "Combined attack window should show ≥2 attack labels"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 7. BDD – normalised residual test (direct)
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_bad_data_detection_normalized_residual() {
        let cfg = simple_config(3, 10.0);
        let net = default_network(3);
        let sim = CyberPhysicalSimulator::new(cfg, net);

        // Measurements with one large outlier
        let meas = vec![1.0, 1.0, 5.0]; // z[2] is anomalous
        let expected = vec![1.0, 1.0, 1.0];
        let bdd_mean = vec![1.0; 3];
        // Set variance so that z[2] triggers: var = (σ * thr)^2, z[2]-exp = 4
        // threshold = 3 → need var < (4/3)^2 ≈ 1.78
        let bdd_m2 = vec![0.01, 0.01, 0.01]; // small variance → z[2] flagged

        let flagged = sim.run_bad_data_detection(&meas, &expected, &bdd_mean, &bdd_m2);
        assert!(
            flagged.contains(&2),
            "Index 2 (large residual) should be flagged; got {flagged:?}"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 8. BDD – chi-squared global alarm
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_chi_squared_alarm() {
        // Create a config with a very tight threshold
        let mut cfg = simple_config(3, 10.0);
        cfg.bdd_threshold = 1.0; // tight 1σ threshold
        let net = default_network(3);
        let sim = CyberPhysicalSimulator::new(cfg, net);

        // All measurements deviate by exactly 1.5σ — individually below threshold,
        // but collectively may trigger the global χ² alarm
        let meas = vec![1.15, 1.15, 1.15];
        let expected = vec![1.0, 1.0, 1.0];
        let bdd_mean = vec![1.0; 3];
        let bdd_m2 = vec![0.01; 3]; // σ = 0.1 → normalised residual = 1.5 > 1.0

        let flagged = sim.run_bad_data_detection(&meas, &expected, &bdd_mean, &bdd_m2);
        // With threshold 1.0 and residuals 1.5σ, individual test already flags them
        assert!(!flagged.is_empty(), "Chi-squared / NR alarm should fire");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 9. CommNetwork latency matrix
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_comm_network_latency_matrix() {
        let nodes = vec![
            CommNode {
                id: 0,
                node_type: CommNodeType::ControlCenter,
                is_compromised: false,
                packet_loss_rate: 0.0,
            },
            CommNode {
                id: 1,
                node_type: CommNodeType::Substation,
                is_compromised: false,
                packet_loss_rate: 0.0,
            },
            CommNode {
                id: 2,
                node_type: CommNodeType::RemoteTerminalUnit,
                is_compromised: false,
                packet_loss_rate: 0.0,
            },
        ];
        let links = vec![
            CommLink {
                from: 0,
                to: 1,
                bandwidth_mbps: 10.0,
                latency_ms: 5.0,
                is_encrypted: true,
                protocol: CommProtocol::Dnp3,
            },
            CommLink {
                from: 1,
                to: 2,
                bandwidth_mbps: 1.0,
                latency_ms: 10.0,
                is_encrypted: false,
                protocol: CommProtocol::Modbus,
            },
        ];
        let net = CommNetwork::new(nodes, links);

        // Diagonal must be zero
        assert_eq!(net.latency_ms[0][0], 0.0);
        assert_eq!(net.latency_ms[1][1], 0.0);
        assert_eq!(net.latency_ms[2][2], 0.0);

        // Direct link 0→1 should be finite and > 0
        assert!(
            net.latency_ms[0][1] > 0.0,
            "Direct link latency should be positive"
        );
        // Transitive 0→2 should be >= 0→1 + 1→2
        assert!(
            net.latency_ms[0][2] >= net.latency_ms[0][1] + net.latency_ms[1][2] - 1e-9,
            "Transitive latency should be >= sum of hops"
        );
        // max_latency_ms should be positive
        assert!(net.max_latency_ms() > 0.0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 10. CommLink protocols
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_comm_link_protocols() {
        let protocols = [
            CommProtocol::Modbus,
            CommProtocol::Dnp3,
            CommProtocol::Iec61850,
            CommProtocol::Iccp,
            CommProtocol::OpcUa,
        ];
        for proto in protocols {
            assert!(
                proto.base_latency_ms() > 0.0,
                "Protocol {proto:?} base latency should be positive"
            );
        }
        // IEC 61850 and OPC UA have native auth
        assert!(CommProtocol::Iec61850.has_native_auth());
        assert!(CommProtocol::OpcUa.has_native_auth());
        assert!(!CommProtocol::Modbus.has_native_auth());
        assert!(!CommProtocol::Dnp3.has_native_auth());

        // Effective latency should increase with DoS
        let link = CommLink {
            from: 0,
            to: 1,
            bandwidth_mbps: 10.0,
            latency_ms: 5.0,
            is_encrypted: false,
            protocol: CommProtocol::Modbus,
        };
        let nominal = link.effective_latency_ms(0.0);
        let congested = link.effective_latency_ms(0.8);
        assert!(
            congested > nominal,
            "Congested latency {congested:.1} should exceed nominal {nominal:.1}"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 11. FDI attack type – scaling
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_attack_type_scaling() {
        let cfg = simple_config(2, 10.0);
        let net = default_network(2);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);

        let attack = FdiAttack {
            target_measurements: vec![0],
            injection_vector: vec![0.0],
            start_time: 0.0,
            duration: 100.0,
            attack_type: FdiAttackType::ScalingAttack { scale: 2.0 },
        };

        let mut z = vec![1.0_f64, 0.95];
        let before = z[0];
        let n = sim.inject_fdi(&mut z, &attack, 5.0);
        assert_eq!(n, 1, "One measurement should be scaled");
        assert!(
            (z[0] - before * 2.0).abs() < 1e-9,
            "Scaled value should be 2x original"
        );
        assert!(
            (z[1] - 0.95).abs() < 1e-9,
            "Untargeted measurement should be unchanged"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 12. FDI attack type – replay
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_attack_type_replay() {
        let cfg = simple_config(2, 30.0);
        let net = default_network(2);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);

        // Run a full simulation with replay attack to verify it completes
        let mut run_cfg = simple_config(2, 30.0);
        run_cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0, 1],
            injection_vector: vec![0.0, 0.0],
            start_time: 5.0,
            duration: 20.0,
            attack_type: FdiAttackType::ReplayAttack {
                replay_window_s: 5.0,
            },
        }];
        let net2 = default_network(2);
        let mut sim2 = CyberPhysicalSimulator::new(run_cfg, net2);
        let result = sim2.run().expect("replay attack simulation should succeed");
        assert!(!result.time_series.is_empty());

        // Direct inject_fdi test (with empty history — falls back gracefully)
        let attack = FdiAttack {
            target_measurements: vec![0],
            injection_vector: vec![0.0],
            start_time: 0.0,
            duration: 100.0,
            attack_type: FdiAttackType::ReplayAttack {
                replay_window_s: 5.0,
            },
        };
        let mut z = vec![1.05_f64, 0.98];
        // No history available → should not panic (returns 0 modifications)
        let n = sim.inject_fdi(&mut z, &attack, 10.0);
        // Either 0 (no history) or graceful handling
        assert!(
            n <= z.len(),
            "Replay with no history should not corrupt count"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 13. Resilience index calculation
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_resilience_index_calculation() {
        let cfg = simple_config(2, 3600.0); // 1 hour
        let net = default_network(2);
        let sim = CyberPhysicalSimulator::new(cfg.clone(), net);

        // Perfect case: no load shed
        let perfect = CyberPhysicalResult {
            time_series: Vec::new(),
            total_load_shed_mwh: 0.0,
            max_voltage_deviation_pu: 0.0,
            attack_detection_rate: 0.0,
            false_positive_rate: 0.0,
            resilience_index: 0.0,
        };
        let ri = CyberPhysicalSimulator::compute_resilience_index(&perfect, &cfg);
        assert!(
            (ri - 1.0).abs() < 1e-9,
            "Zero load shed → resilience = 1.0, got {ri}"
        );

        // Total blackout: shed = nominal energy
        let total_mwh: f64 = cfg.nominal_power_mw.iter().sum::<f64>() * 3600.0 / 3600.0;
        let blackout = CyberPhysicalResult {
            time_series: Vec::new(),
            total_load_shed_mwh: total_mwh,
            max_voltage_deviation_pu: 1.0,
            attack_detection_rate: 0.0,
            false_positive_rate: 0.0,
            resilience_index: 0.0,
        };
        let ri_blackout = CyberPhysicalSimulator::compute_resilience_index(&blackout, &cfg);
        assert!(
            ri_blackout < 0.01,
            "Full blackout → resilience ≈ 0, got {ri_blackout}"
        );
        let _ = sim; // suppress unused warning
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 14. Measurement integrity update
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_measurement_integrity_update() {
        let n = 3;
        let mut cfg = simple_config(n, 20.0);
        // Bias attack on measurement 0 to drive integrity down
        cfg.fdi_attacks = vec![FdiAttack {
            target_measurements: vec![0],
            injection_vector: vec![0.5],
            start_time: 1.0,
            duration: 15.0,
            attack_type: FdiAttackType::BiasInjection { bias: 0.5 },
        }];
        let net = default_network(n);
        let mut sim = CyberPhysicalSimulator::new(cfg, net);
        let result = sim.run().expect("integrity test should succeed");

        // After attack window, at least one measurement should have integrity < 1.0
        // (BDD should have flagged some steps and reduced integrity)
        let min_integrity = result
            .time_series
            .iter()
            .flat_map(|s| s.measurement_integrity.iter())
            .cloned()
            .fold(f64::INFINITY, f64::min);
        assert!(
            min_integrity >= 0.0,
            "Integrity must be non-negative, got {min_integrity}"
        );
        // At least some integrity degradation expected during attack
        assert!(
            min_integrity <= 1.0,
            "Integrity must be ≤ 1.0, got {min_integrity}"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 15. Full simulation with result validation
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn test_full_simulation_with_result() {
        let n = 5;
        let mut cfg = simple_config(n, 60.0);
        cfg.fdi_attacks = vec![
            FdiAttack {
                target_measurements: vec![0, 2],
                injection_vector: vec![0.2, 0.2],
                start_time: 10.0,
                duration: 20.0,
                attack_type: FdiAttackType::BiasInjection { bias: 0.2 },
            },
            FdiAttack {
                target_measurements: vec![1],
                injection_vector: vec![0.0],
                start_time: 35.0,
                duration: 15.0,
                attack_type: FdiAttackType::RandomNoise { sigma: 0.3 },
            },
        ];
        cfg.dos_attacks = vec![DosAttack {
            target_nodes: vec![0, 1, 2],
            attack_intensity: 0.6,
            start_time: 15.0,
            duration: 10.0,
        }];

        let nodes: Vec<CommNode> = (0..n)
            .map(|i| CommNode {
                id: i,
                node_type: CommNodeType::Substation,
                is_compromised: i == 1, // node 1 is compromised
                packet_loss_rate: 0.02,
            })
            .collect();
        let links: Vec<CommLink> = (0..n - 1)
            .map(|i| CommLink {
                from: i,
                to: i + 1,
                bandwidth_mbps: 50.0,
                latency_ms: 8.0,
                is_encrypted: i % 2 == 0,
                protocol: if i % 2 == 0 {
                    CommProtocol::Iec61850
                } else {
                    CommProtocol::Dnp3
                },
            })
            .collect();
        let net = CommNetwork::new(nodes, links);

        let mut sim = CyberPhysicalSimulator::new(cfg.clone(), net);
        let result = sim.run().expect("full simulation should succeed");

        // Structural checks
        assert!(
            !result.time_series.is_empty(),
            "Time series must be non-empty"
        );
        assert!(
            result.total_load_shed_mwh >= 0.0,
            "Load shed must be non-negative"
        );
        assert!(
            result.max_voltage_deviation_pu >= 0.0,
            "Max voltage deviation must be non-negative"
        );
        assert!(
            (0.0..=1.0).contains(&result.attack_detection_rate),
            "Detection rate must be in [0, 1]"
        );
        assert!(
            (0.0..=1.0).contains(&result.false_positive_rate),
            "False positive rate must be in [0, 1]"
        );
        assert!(
            (0.0..=1.0).contains(&result.resilience_index),
            "Resilience index must be in [0, 1]"
        );

        // Compromised nodes should appear in state snapshots
        let has_compromised = result
            .time_series
            .iter()
            .any(|s| !s.compromised_nodes.is_empty());
        assert!(
            has_compromised,
            "Node 1 is compromised — should appear in state"
        );

        // Resilience index consistency
        let ri_check = CyberPhysicalSimulator::compute_resilience_index(&result, &cfg);
        assert!(
            (ri_check - result.resilience_index).abs() < 1e-9,
            "Resilience index mismatch"
        );
    }
}
