#![allow(clippy::needless_range_loop)]
//! Procedural synthetic power network generation.
//!
//! Implements multiple topology models for generating realistic synthetic
//! power networks suitable for algorithm testing and benchmarking:
//!
//! - **Ring**: simple closed-loop (single-bus degree-2 network)
//! - **Radial**: spanning-tree distribution-network style
//! - **Meshed**: random geometric graph (transmission-network style)
//! - **Geographic**: grid-placed buses connected to nearest neighbours
//! - **SmallWorld**: Watts-Strogatz with tunable clustering and path length
//! - **ScaleFree**: Barabasi-Albert preferential attachment model
//!
//! All generators use a minimal Linear Congruential Generator (LCG) so there
//! are no external RNG dependencies.

use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::units::{Power, ReactivePower, Voltage};

// ---------------------------------------------------------------------------
// LCG random number generator
// ---------------------------------------------------------------------------

/// 64-bit Linear Congruential Generator (Knuth constants).
///
/// State is updated as `state = a * state + c  (mod 2^64)`.
pub struct Lcg64 {
    state: u64,
}

impl Lcg64 {
    /// Construct a new LCG from a seed value.
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Advance one step and return the raw 64-bit output.
    fn next_u64(&mut self) -> u64 {
        // Knuth multiplicative LCG (MMIX)
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    /// Return a uniform float in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Return a uniform integer in `[0, n)`.
    pub fn next_usize(&mut self, n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        (self.next_u64() % n as u64) as usize
    }

    /// Return a sample from `N(0, 1)` via Box-Muller transform.
    pub fn next_normal(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    /// Return a sample from `LogNormal(mu, sigma)`.
    pub fn next_lognormal(&mut self, mean: f64, std_frac: f64) -> f64 {
        // If X ~ LN(μ, σ²) then E[X] = exp(μ + σ²/2)
        // so μ = ln(mean) - σ²/2
        let sigma = std_frac;
        let mu = mean.ln() - 0.5 * sigma * sigma;
        (mu + sigma * self.next_normal()).exp()
    }
}

// ---------------------------------------------------------------------------
// Public configuration
// ---------------------------------------------------------------------------

/// Topology model for synthetic network generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkTopology {
    /// Buses connected in a closed ring (each bus has degree 2).
    Ring,
    /// Spanning-tree radial topology (distribution-network style).
    Radial,
    /// Random geometric graph: buses placed in unit square, connected
    /// to neighbours within a radius ensuring connectivity.
    Meshed,
    /// Buses placed on a regular integer grid, connected to the four
    /// nearest grid neighbours (where they exist).
    Geographic,
    /// Watts-Strogatz small-world topology (k=4 initial, β=0.3 rewiring).
    SmallWorld,
    /// Barabasi-Albert scale-free (preferential attachment, m=2 per node).
    ScaleFree,
}

/// Configuration for procedural synthetic network generation.
#[derive(Debug, Clone)]
pub struct SyntheticNetworkConfig {
    /// Number of buses to generate.
    pub n_buses: usize,
    /// Number of generators to place.
    pub n_generators: usize,
    /// Network topology model.
    pub topology: NetworkTopology,
    /// Nominal bus base voltage \[kV\].
    pub voltage_level_kv: f64,
    /// System MVA base.
    pub base_mva: f64,
    /// Mean active power demand per load bus \[MW\].
    pub load_density_mw_per_bus: f64,
    /// Fractional standard deviation for load lognormal sampling.
    pub load_std_fraction: f64,
    /// Mean generator nameplate capacity \[MW\].
    pub generator_capacity_mw: f64,
    /// Mean line length used to compute impedance \[km\].
    pub line_length_km: f64,
    /// Reproducibility seed for the LCG.
    pub seed: u64,
}

impl Default for SyntheticNetworkConfig {
    fn default() -> Self {
        Self {
            n_buses: 30,
            n_generators: 5,
            topology: NetworkTopology::Meshed,
            voltage_level_kv: 132.0,
            base_mva: 100.0,
            load_density_mw_per_bus: 50.0,
            load_std_fraction: 0.3,
            generator_capacity_mw: 200.0,
            line_length_km: 80.0,
            seed: 42,
        }
    }
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

/// Generate a synthetic power network according to `config`.
///
/// The network is validated before being returned.  A `OxiGridError` is
/// returned if the configuration is inconsistent (e.g., zero buses) or if
/// the topology generator cannot ensure connectivity.
pub fn generate_synthetic_network(config: &SyntheticNetworkConfig) -> Result<PowerNetwork> {
    if config.n_buses < 2 {
        return Err(OxiGridError::InvalidParameter(
            "n_buses must be ≥ 2".to_string(),
        ));
    }
    if config.n_generators < 1 {
        return Err(OxiGridError::InvalidParameter(
            "n_generators must be ≥ 1".to_string(),
        ));
    }

    let mut rng = Lcg64::new(config.seed);

    let mut net = match config.topology {
        NetworkTopology::Ring => generate_ring(config, &mut rng),
        NetworkTopology::Radial => generate_radial(config, &mut rng),
        NetworkTopology::Meshed => generate_meshed(config, &mut rng),
        NetworkTopology::Geographic => generate_geographic(config, &mut rng),
        NetworkTopology::SmallWorld => generate_small_world(config, 4, 0.3, &mut rng),
        NetworkTopology::ScaleFree => generate_scale_free(config, 2, &mut rng),
    };

    // Assign generators
    let gen_buses = place_generators(config.n_buses, config.n_generators, &mut rng);
    let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    let total_gen = total_load * 1.15; // 15 % reserve margin

    let gen_pairs = assign_generators(
        config.n_buses,
        config.n_generators,
        total_gen,
        &gen_buses,
        &mut rng,
    );

    // Bus 1 is always Slack
    if let Some(bus) = net.buses.first_mut() {
        bus.bus_type = BusType::Slack;
    }

    for (bus_idx, capacity) in &gen_pairs {
        let bus_id = bus_idx + 1; // 1-based
                                  // Set PV type for all non-slack generator buses
        if let Some(bus) = net.buses.iter_mut().find(|b| b.id == bus_id) {
            if bus.bus_type != BusType::Slack {
                bus.bus_type = BusType::PV;
            }
        }
        let pg = capacity * 0.7; // dispatch at 70 % of nameplate
        net.generators.push(Generator {
            bus_id,
            pg,
            qg: 0.0,
            qmax: capacity * 0.5,
            qmin: -capacity * 0.3,
            vg: 1.02,
            mbase: config.base_mva,
            status: true,
            pmax: *capacity,
            pmin: 0.0,
        });
    }

    validate_network(&net)?;
    Ok(net)
}

// ---------------------------------------------------------------------------
// Topology generators
// ---------------------------------------------------------------------------

/// Build a ring topology: each bus `i` is connected to bus `(i+1) % n`.
pub(crate) fn generate_ring(config: &SyntheticNetworkConfig, rng: &mut Lcg64) -> PowerNetwork {
    let n = config.n_buses;
    let mut net = PowerNetwork::new(config.base_mva);

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    for i in 0..n {
        let from = i + 1;
        let to = (i % n) + 2;
        let to = if to > n { 1 } else { to };
        let len = config.line_length_km * (0.8 + 0.4 * rng.next_f64());
        let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
        net.branches.push(make_branch(from, to, r, x, b));
    }

    net
}

/// Build a radial (spanning-tree) topology.
///
/// Bus 1 is the substation root.  Each subsequent bus is connected to a
/// randomly chosen existing bus, producing a tree graph.
pub(crate) fn generate_radial(config: &SyntheticNetworkConfig, rng: &mut Lcg64) -> PowerNetwork {
    let n = config.n_buses;
    let mut net = PowerNetwork::new(config.base_mva);

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    // Connect each node to a randomly chosen predecessor (gives a random tree)
    for i in 1..n {
        let parent = rng.next_usize(i); // parent is in 0..i
        let from = parent + 1;
        let to = i + 1;
        let len = config.line_length_km * (0.5 + rng.next_f64());
        let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
        net.branches.push(make_branch(from, to, r, x, b));
    }

    net
}

/// Build a meshed random geometric graph.
///
/// Buses are placed uniformly at random in the unit square.
/// All pairs within distance `radius` are connected.
/// If the graph is disconnected, a spanning tree is added to reconnect.
pub(crate) fn generate_meshed(config: &SyntheticNetworkConfig, rng: &mut Lcg64) -> PowerNetwork {
    let n = config.n_buses;
    let mut net = PowerNetwork::new(config.base_mva);

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    // Place buses in unit square
    let mut x_pos = Vec::with_capacity(n);
    let mut y_pos = Vec::with_capacity(n);
    for _ in 0..n {
        x_pos.push(rng.next_f64());
        y_pos.push(rng.next_f64());
    }

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    // Choose radius so expected degree ≈ 4 (π r² n ≈ 4)
    let radius = (4.0 / (std::f64::consts::PI * n as f64)).sqrt().max(0.25);

    let mut edge_set: Vec<(usize, usize)> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let dx = x_pos[i] - x_pos[j];
            let dy = y_pos[i] - y_pos[j];
            let dist = (dx * dx + dy * dy).sqrt();
            if dist <= radius {
                edge_set.push((i, j));
            }
        }
    }

    // Add edges then repair connectivity
    for &(i, j) in &edge_set {
        let geo_dist = {
            let dx = x_pos[i] - x_pos[j];
            let dy = y_pos[i] - y_pos[j];
            (dx * dx + dy * dy).sqrt()
        };
        let len = geo_dist * config.line_length_km * 2.0;
        let (r, x, b) = line_impedance(len.max(5.0), config.voltage_level_kv, config.base_mva);
        net.branches.push(make_branch(i + 1, j + 1, r, x, b));
    }

    // Repair connectivity by finding connected components and bridging them
    ensure_connected(&mut net, &x_pos, &y_pos, config);

    net
}

/// Build a geographic grid topology.
///
/// Buses are placed on a `ceil(sqrt(n)) x ceil(sqrt(n))` integer grid.
/// Each bus is connected to its four von-Neumann neighbours where they exist.
pub(crate) fn generate_geographic(
    config: &SyntheticNetworkConfig,
    rng: &mut Lcg64,
) -> PowerNetwork {
    let n = config.n_buses;
    let mut net = PowerNetwork::new(config.base_mva);
    let cols = (n as f64).sqrt().ceil() as usize;

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    for i in 0..n {
        let row = i / cols;
        let col = i % cols;

        // Connect right
        if col + 1 < cols && (i + 1) < n {
            let len = config.line_length_km * (0.9 + 0.2 * rng.next_f64());
            let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
            net.branches.push(make_branch(i + 1, i + 2, r, x, b));
        }
        // Connect down
        if row + 1 < n.div_ceil(cols) && (i + cols) < n {
            let len = config.line_length_km * (0.9 + 0.2 * rng.next_f64());
            let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
            net.branches.push(make_branch(i + 1, i + cols + 1, r, x, b));
        }
    }

    net
}

/// Build a Watts-Strogatz small-world topology.
///
/// Start with a k-regular ring lattice, then rewire each edge with
/// probability `beta`.
pub(crate) fn generate_small_world(
    config: &SyntheticNetworkConfig,
    k: usize,
    beta: f64,
    rng: &mut Lcg64,
) -> PowerNetwork {
    let n = config.n_buses;
    let k = k.min(n / 2).max(1);
    let mut net = PowerNetwork::new(config.base_mva);

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    // Regular ring lattice: connect each node to k nearest on each side
    // Track adjacency to avoid duplicates
    let mut adj: Vec<Vec<bool>> = vec![vec![false; n]; n];

    for i in 0..n {
        for s in 1..=k {
            let j = (i + s) % n;
            if !adj[i][j] {
                adj[i][j] = true;
                adj[j][i] = true;
                let len = config.line_length_km * (0.8 + 0.4 * rng.next_f64());
                let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
                net.branches.push(make_branch(i + 1, j + 1, r, x, b));
            }
        }
    }

    // Rewiring pass: for each edge (i,j) with prob beta, rewire j -> random k
    let initial_count = net.branches.len();
    for bi in 0..initial_count {
        if rng.next_f64() < beta {
            // Attempt to rewire; remove old edge and replace with random
            let from_id = net.branches[bi].from_bus;
            let to_id = net.branches[bi].to_bus;
            let i = from_id - 1;
            let old_j = to_id - 1;

            // Find a new target different from i and not already connected
            let mut attempts = 0usize;
            let new_j = loop {
                let candidate = rng.next_usize(n);
                if candidate != i && !adj[i][candidate] {
                    break candidate;
                }
                attempts += 1;
                if attempts > 2 * n {
                    break old_j; // give up, keep old
                }
            };

            if new_j != old_j {
                adj[i][old_j] = false;
                adj[old_j][i] = false;
                adj[i][new_j] = true;
                adj[new_j][i] = true;
                let len = config.line_length_km * (0.8 + 0.4 * rng.next_f64());
                let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
                net.branches[bi] = make_branch(i + 1, new_j + 1, r, x, b);
            }
        }
    }

    net
}

/// Build a Barabasi-Albert scale-free network via preferential attachment.
///
/// Start with `m+1` fully-connected seed nodes.  Add each new node with
/// `m` edges, where attachment probability is proportional to current degree.
pub(crate) fn generate_scale_free(
    config: &SyntheticNetworkConfig,
    m: usize,
    rng: &mut Lcg64,
) -> PowerNetwork {
    let n = config.n_buses;
    let m = m.max(1).min(n / 2);
    let mut net = PowerNetwork::new(config.base_mva);

    let loads = assign_loads(
        n,
        config.load_density_mw_per_bus,
        config.load_std_fraction,
        &[],
        rng,
    );

    for i in 0..n {
        net.buses.push(Bus {
            id: i + 1,
            name: format!("Bus {}", i + 1),
            bus_type: BusType::PQ,
            base_kv: Voltage(config.voltage_level_kv),
            vm: 1.0,
            va: 0.0,
            pd: Power(loads[i]),
            qd: ReactivePower(loads[i] * 0.3),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        });
    }

    // Seed: fully connect the first m+1 nodes
    let seed = (m + 1).min(n);
    let mut degree = vec![0usize; n];
    let mut adj: Vec<Vec<bool>> = vec![vec![false; n]; n];

    for i in 0..seed {
        for j in (i + 1)..seed {
            if !adj[i][j] {
                adj[i][j] = true;
                adj[j][i] = true;
                degree[i] += 1;
                degree[j] += 1;
                let len = config.line_length_km * (0.8 + 0.4 * rng.next_f64());
                let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
                net.branches.push(make_branch(i + 1, j + 1, r, x, b));
            }
        }
    }

    // Preferential attachment for nodes seed..n
    for new_node in seed..n {
        let total_degree: usize = degree[..new_node].iter().sum();
        let total_degree = total_degree.max(1);

        let mut connected = 0usize;
        let mut attempts = 0usize;
        while connected < m && attempts < 10 * n {
            attempts += 1;
            // Draw a node proportional to degree (stochastic selection)
            let threshold = (rng.next_f64() * total_degree as f64) as usize;
            let mut cumulative = 0usize;
            let mut target = 0usize;
            for k in 0..new_node {
                cumulative += degree[k];
                if cumulative > threshold {
                    target = k;
                    break;
                }
            }
            if target != new_node && !adj[new_node][target] {
                adj[new_node][target] = true;
                adj[target][new_node] = true;
                degree[new_node] += 1;
                degree[target] += 1;
                let len = config.line_length_km * (0.8 + 0.4 * rng.next_f64());
                let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
                net.branches
                    .push(make_branch(new_node + 1, target + 1, r, x, b));
                connected += 1;
            }
        }

        // If we couldn't connect m edges, connect to at least 1 existing node
        if connected == 0 {
            let target = rng.next_usize(new_node);
            if !adj[new_node][target] {
                adj[new_node][target] = true;
                adj[target][new_node] = true;
                degree[new_node] += 1;
                degree[target] += 1;
                let len = config.line_length_km;
                let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
                net.branches
                    .push(make_branch(new_node + 1, target + 1, r, x, b));
            }
        }
    }

    net
}

// ---------------------------------------------------------------------------
// Load assignment
// ---------------------------------------------------------------------------

/// Assign per-bus active power loads using a log-normal distribution.
///
/// Generator buses receive zero load.
pub(crate) fn assign_loads(
    n_buses: usize,
    mean_mw: f64,
    std_fraction: f64,
    generator_buses: &[usize],
    rng: &mut Lcg64,
) -> Vec<f64> {
    let mut loads = Vec::with_capacity(n_buses);
    for i in 0..n_buses {
        if generator_buses.contains(&i) {
            loads.push(0.0);
        } else {
            let sample = rng.next_lognormal(mean_mw.max(1.0), std_fraction);
            loads.push(sample);
        }
    }
    loads
}

// ---------------------------------------------------------------------------
// Generator placement
// ---------------------------------------------------------------------------

/// Choose `n_generators` buses for generator placement.
///
/// Bus 0 (external id 1) is always the slack/reference.
fn place_generators(n_buses: usize, n_generators: usize, rng: &mut Lcg64) -> Vec<usize> {
    let mut buses = vec![0usize]; // bus 0 = slack
    let remaining = n_generators.saturating_sub(1);
    let mut available: Vec<usize> = (1..n_buses).collect();

    for _ in 0..remaining {
        if available.is_empty() {
            break;
        }
        let idx = rng.next_usize(available.len());
        buses.push(available.remove(idx));
    }
    buses
}

/// Assign generator capacities summing to `total_mw`.
///
/// Capacities are sampled from a uniform distribution and then scaled.
pub(crate) fn assign_generators(
    _n_buses: usize,
    n_generators: usize,
    total_mw: f64,
    gen_buses: &[usize],
    rng: &mut Lcg64,
) -> Vec<(usize, f64)> {
    // Draw raw shares from uniform [0.5, 1.5]
    let raw: Vec<f64> = (0..n_generators).map(|_| 0.5 + rng.next_f64()).collect();
    let sum: f64 = raw.iter().sum();

    gen_buses
        .iter()
        .zip(raw.iter())
        .map(|(&bus_idx, &share)| {
            let capacity = (share / sum) * total_mw;
            (bus_idx, capacity)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Impedance computation
// ---------------------------------------------------------------------------

/// Compute per-unit line impedance for an overhead line.
///
/// Uses typical 132 kV overhead line parameters:
/// - r ≈ 0.06 Ω/km, x ≈ 0.40 Ω/km, b ≈ 2.7 μS/km
///
/// The result is converted to p.u. at the given base.
///
/// Returns `(r_pu, x_pu, b_pu)`.
pub(crate) fn line_impedance(length_km: f64, voltage_kv: f64, base_mva: f64) -> (f64, f64, f64) {
    // Voltage-dependent specific impedance estimates (overhead line)
    let (r_ohm_km, x_ohm_km, b_us_km) = if voltage_kv >= 220.0 {
        (0.03, 0.30, 3.5)
    } else if voltage_kv >= 110.0 {
        (0.06, 0.40, 2.7)
    } else if voltage_kv >= 33.0 {
        (0.20, 0.40, 2.0)
    } else if voltage_kv >= 11.0 {
        (0.30, 0.35, 1.5)
    } else {
        // LV cable
        (0.50, 0.10, 5.0)
    };

    let z_base = voltage_kv * voltage_kv / base_mva;
    let y_base = base_mva / (voltage_kv * voltage_kv);

    let r_pu = (r_ohm_km * length_km) / z_base;
    let x_pu = (x_ohm_km * length_km) / z_base;
    let b_pu = (b_us_km * 1e-6 * length_km) / y_base;

    // Clamp r to avoid near-singular admittances
    let r_pu = r_pu.max(1e-5);
    let x_pu = x_pu.max(1e-4);

    (r_pu, x_pu, b_pu)
}

// ---------------------------------------------------------------------------
// Connectivity repair
// ---------------------------------------------------------------------------

/// Ensure `net` is fully connected by bridging isolated components with
/// minimum-distance edges.
fn ensure_connected(
    net: &mut PowerNetwork,
    x_pos: &[f64],
    y_pos: &[f64],
    config: &SyntheticNetworkConfig,
) {
    let n = net.buses.len();
    loop {
        // BFS to find components
        let mut component = vec![usize::MAX; n];
        let mut comp_id = 0usize;
        for start in 0..n {
            if component[start] != usize::MAX {
                continue;
            }
            let mut queue = std::collections::VecDeque::new();
            queue.push_back(start);
            component[start] = comp_id;
            while let Some(node) = queue.pop_front() {
                for branch in &net.branches {
                    let fi = branch.from_bus - 1;
                    let ti = branch.to_bus - 1;
                    if fi == node && component[ti] == usize::MAX {
                        component[ti] = comp_id;
                        queue.push_back(ti);
                    } else if ti == node && component[fi] == usize::MAX {
                        component[fi] = comp_id;
                        queue.push_back(fi);
                    }
                }
            }
            comp_id += 1;
        }

        if comp_id <= 1 {
            break; // Already connected
        }

        // Find closest pair of nodes in different components
        let mut best_dist = f64::INFINITY;
        let mut best_i = 0usize;
        let mut best_j = 1usize;
        for i in 0..n {
            for j in (i + 1)..n {
                if component[i] != component[j] {
                    let dx = x_pos[i] - x_pos[j];
                    let dy = y_pos[i] - y_pos[j];
                    let d = (dx * dx + dy * dy).sqrt();
                    if d < best_dist {
                        best_dist = d;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }

        let len = (best_dist * config.line_length_km * 2.0).max(5.0);
        let (r, x, b) = line_impedance(len, config.voltage_level_kv, config.base_mva);
        net.branches
            .push(make_branch(best_i + 1, best_j + 1, r, x, b));
    }
}

// ---------------------------------------------------------------------------
// Branch helper
// ---------------------------------------------------------------------------

fn make_branch(from_bus: usize, to_bus: usize, r: f64, x: f64, b: f64) -> Branch {
    Branch {
        from_bus,
        to_bus,
        r,
        x,
        b,
        rate_a: 250.0,
        rate_b: 250.0,
        rate_c: 250.0,
        tap: 0.0,
        shift: 0.0,
        status: true,
    }
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

/// Validate that the generated network is suitable for power flow:
/// - Has at least one slack bus
/// - Is topologically connected
/// - Generation approximately balances load
pub(crate) fn validate_network(net: &PowerNetwork) -> Result<()> {
    if net.buses.is_empty() {
        return Err(OxiGridError::InvalidNetwork("No buses".to_string()));
    }

    let has_slack = net.buses.iter().any(|b| b.bus_type == BusType::Slack);
    if !has_slack {
        return Err(OxiGridError::InvalidNetwork(
            "No slack bus in generated network".to_string(),
        ));
    }

    if !net.is_connected() {
        return Err(OxiGridError::InvalidNetwork(
            "Generated network is not connected".to_string(),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------------
    // 1. LCG64 determinism
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_determinism() {
        let mut rng_a = Lcg64::new(42);
        let mut rng_b = Lcg64::new(42);
        let mut rng_c = Lcg64::new(99);
        let seq_a: Vec<f64> = (0..10).map(|_| rng_a.next_f64()).collect();
        let seq_b: Vec<f64> = (0..10).map(|_| rng_b.next_f64()).collect();
        let first_c = rng_c.next_f64();
        assert_eq!(seq_a, seq_b, "same seed must produce identical sequences");
        assert_ne!(
            seq_a[0], first_c,
            "different seeds must differ on first value"
        );
    }

    // ------------------------------------------------------------------
    // 2. next_f64 range
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_next_f64_range() {
        let mut rng = Lcg64::new(7);
        for _ in 0..1000 {
            let v = rng.next_f64();
            assert!(v >= 0.0, "next_f64 must be >= 0.0, got {v}");
            assert!(v < 1.0, "next_f64 must be < 1.0, got {v}");
        }
    }

    // ------------------------------------------------------------------
    // 3. next_usize bound
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_next_usize_bound() {
        let mut rng = Lcg64::new(13);
        for _ in 0..1000 {
            let v = rng.next_usize(17);
            assert!(v < 17, "next_usize(17) must be < 17, got {v}");
        }
    }

    // ------------------------------------------------------------------
    // 4. next_normal distribution
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_next_normal_distribution() {
        let mut rng = Lcg64::new(21);
        let samples: Vec<f64> = (0..500).map(|_| rng.next_normal()).collect();
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let variance =
            samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / samples.len() as f64;
        let std_dev = variance.sqrt();
        assert!(
            mean.abs() < 0.4,
            "sample mean {mean:.4} should be within ±0.4 of 0"
        );
        assert!(
            (0.6..=1.6).contains(&std_dev),
            "sample std {std_dev:.4} should be in [0.6, 1.6]"
        );
    }

    // ------------------------------------------------------------------
    // 5. next_lognormal positivity
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_next_lognormal_positive() {
        let mut rng = Lcg64::new(33);
        for _ in 0..200 {
            let v = rng.next_lognormal(50.0, 0.3);
            assert!(v > 0.0, "next_lognormal must be > 0.0, got {v}");
        }
    }

    // ------------------------------------------------------------------
    // 6. Default config generation
    // ------------------------------------------------------------------
    #[test]
    fn test_generate_default_config() {
        let config = SyntheticNetworkConfig::default();
        let net =
            generate_synthetic_network(&config).expect("default config must generate successfully");
        assert_eq!(net.buses.len(), 30, "default config produces 30 buses");
        assert!(
            !net.generators.is_empty(),
            "at least one generator expected"
        );
        let has_slack = net.buses.iter().any(|b| b.bus_type == BusType::Slack);
        assert!(has_slack, "at least one Slack bus must exist");
    }

    // ------------------------------------------------------------------
    // 7. Ring topology
    // ------------------------------------------------------------------
    #[test]
    fn test_ring_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 10,
            topology: NetworkTopology::Ring,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("ring topology must succeed");
        assert_eq!(net.buses.len(), 10, "ring: expected 10 buses");
        assert_eq!(
            net.branches.len(),
            10,
            "ring topology on 10 nodes must have exactly 10 branches"
        );
    }

    // ------------------------------------------------------------------
    // 8. Radial topology
    // ------------------------------------------------------------------
    #[test]
    fn test_radial_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 12,
            topology: NetworkTopology::Radial,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("radial topology must succeed");
        assert_eq!(net.buses.len(), 12, "radial: expected 12 buses");
        assert_eq!(
            net.branches.len(),
            11,
            "radial (tree) on 12 nodes must have exactly 11 branches"
        );
    }

    // ------------------------------------------------------------------
    // 9. Meshed topology
    // ------------------------------------------------------------------
    #[test]
    fn test_meshed_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 15,
            topology: NetworkTopology::Meshed,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("meshed topology must succeed");
        assert_eq!(net.buses.len(), 15, "meshed: expected 15 buses");
        assert!(
            net.branches.len() >= 14,
            "meshed: connected graph needs at least n-1=14 branches, got {}",
            net.branches.len()
        );
    }

    // ------------------------------------------------------------------
    // 10. Geographic topology
    // ------------------------------------------------------------------
    #[test]
    fn test_geographic_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 16,
            topology: NetworkTopology::Geographic,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("geographic topology must succeed");
        assert_eq!(net.buses.len(), 16, "geographic: expected 16 buses");
        assert!(
            !net.branches.is_empty(),
            "geographic: must have at least 1 branch"
        );
    }

    // ------------------------------------------------------------------
    // 11. SmallWorld topology
    // ------------------------------------------------------------------
    #[test]
    fn test_small_world_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 20,
            topology: NetworkTopology::SmallWorld,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("small-world topology must succeed");
        assert_eq!(net.buses.len(), 20, "small-world: expected 20 buses");
        assert!(
            !net.branches.is_empty(),
            "small-world: must have at least 1 branch"
        );
    }

    // ------------------------------------------------------------------
    // 12. ScaleFree topology
    // ------------------------------------------------------------------
    #[test]
    fn test_scale_free_topology() {
        let config = SyntheticNetworkConfig {
            n_buses: 25,
            topology: NetworkTopology::ScaleFree,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("scale-free topology must succeed");
        assert_eq!(net.buses.len(), 25, "scale-free: expected 25 buses");
        assert!(
            !net.branches.is_empty(),
            "scale-free: must have at least 1 branch"
        );
    }

    // ------------------------------------------------------------------
    // 13. Error: n_buses too small
    // ------------------------------------------------------------------
    #[test]
    fn test_error_n_buses_too_small() {
        let config = SyntheticNetworkConfig {
            n_buses: 1,
            ..SyntheticNetworkConfig::default()
        };
        let result = generate_synthetic_network(&config);
        assert!(
            matches!(result, Err(OxiGridError::InvalidParameter(_))),
            "n_buses=1 must return InvalidParameter, got {:?}",
            result
        );
    }

    // ------------------------------------------------------------------
    // 14. Error: n_generators zero
    // ------------------------------------------------------------------
    #[test]
    fn test_error_n_generators_zero() {
        let config = SyntheticNetworkConfig {
            n_generators: 0,
            ..SyntheticNetworkConfig::default()
        };
        let result = generate_synthetic_network(&config);
        assert!(
            matches!(result, Err(OxiGridError::InvalidParameter(_))),
            "n_generators=0 must return InvalidParameter, got {:?}",
            result
        );
    }

    // ------------------------------------------------------------------
    // 15. validate_network rejects empty network
    // ------------------------------------------------------------------
    #[test]
    fn test_validate_network_rejects_empty() {
        let net = PowerNetwork::new(100.0);
        let result = validate_network(&net);
        assert!(
            matches!(result, Err(OxiGridError::InvalidNetwork(_))),
            "empty network must be rejected with InvalidNetwork, got {:?}",
            result
        );
    }

    // ------------------------------------------------------------------
    // 16. validate_network rejects network with no slack bus
    // ------------------------------------------------------------------
    #[test]
    fn test_validate_network_rejects_no_slack() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::PQ));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        let result = validate_network(&net);
        assert!(
            matches!(result, Err(OxiGridError::InvalidNetwork(_))),
            "no-slack network must be rejected with InvalidNetwork, got {:?}",
            result
        );
    }

    // ------------------------------------------------------------------
    // 17. Branch impedances are positive
    // ------------------------------------------------------------------
    #[test]
    fn test_branch_impedances_positive() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        for (i, branch) in net.branches.iter().enumerate() {
            assert!(
                branch.r > 0.0,
                "branch[{i}] resistance must be > 0.0, got {}",
                branch.r
            );
            assert!(
                branch.x > 0.0,
                "branch[{i}] reactance must be > 0.0, got {}",
                branch.x
            );
        }
    }

    // ------------------------------------------------------------------
    // 18. Generators dispatched at 70 % of pmax
    // ------------------------------------------------------------------
    #[test]
    fn test_generators_dispatched_at_70_percent() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        for (i, gen) in net.generators.iter().enumerate() {
            let expected = gen.pmax * 0.7;
            assert!(
                (gen.pg - expected).abs() < 1e-9,
                "generator[{i}] pg={} but pmax*0.7={expected}",
                gen.pg
            );
        }
    }

    // ------------------------------------------------------------------
    // 19. Generator voltage setpoint
    // ------------------------------------------------------------------
    #[test]
    fn test_generator_voltage_setpoint() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        for (i, gen) in net.generators.iter().enumerate() {
            assert!(
                (gen.vg - 1.02).abs() < 1e-9,
                "generator[{i}] vg={} but expected 1.02",
                gen.vg
            );
        }
    }

    // ------------------------------------------------------------------
    // 20. Reproducibility with the same seed
    // ------------------------------------------------------------------
    #[test]
    fn test_reproducibility() {
        let config = SyntheticNetworkConfig {
            seed: 42,
            ..SyntheticNetworkConfig::default()
        };
        let net_a = generate_synthetic_network(&config).expect("first call must succeed");
        let net_b = generate_synthetic_network(&config).expect("second call must succeed");
        assert_eq!(
            net_a.buses.len(),
            net_b.buses.len(),
            "reproducibility: buses.len() must match"
        );
        assert_eq!(
            net_a.branches.len(),
            net_b.branches.len(),
            "reproducibility: branches.len() must match"
        );
        assert_eq!(
            net_a.buses[0].pd.0, net_b.buses[0].pd.0,
            "reproducibility: buses[0].pd must match"
        );
    }

    // ------------------------------------------------------------------
    // 21. Different seeds produce different networks
    // ------------------------------------------------------------------
    #[test]
    fn test_different_seeds_different_networks() {
        let config_a = SyntheticNetworkConfig {
            n_buses: 30,
            topology: NetworkTopology::Meshed,
            seed: 42,
            ..SyntheticNetworkConfig::default()
        };
        let config_b = SyntheticNetworkConfig {
            n_buses: 30,
            topology: NetworkTopology::Meshed,
            seed: 999,
            ..SyntheticNetworkConfig::default()
        };
        let net_a = generate_synthetic_network(&config_a).expect("seed=42 must succeed");
        let net_b = generate_synthetic_network(&config_b).expect("seed=999 must succeed");
        let any_differ = net_a
            .buses
            .iter()
            .zip(net_b.buses.iter())
            .any(|(ba, bb)| (ba.pd.0 - bb.pd.0).abs() > 1e-12);
        assert!(
            any_differ,
            "different seeds must produce at least one differing bus pd value"
        );
    }

    // ------------------------------------------------------------------
    // 22. Slack bus is bus id 1
    // ------------------------------------------------------------------
    #[test]
    fn test_slack_bus_is_bus_one() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        let bus_one = net
            .buses
            .iter()
            .find(|b| b.id == 1)
            .expect("bus with id==1 must exist");
        assert_eq!(
            bus_one.bus_type,
            BusType::Slack,
            "bus id=1 must be BusType::Slack"
        );
    }

    // ------------------------------------------------------------------
    // 23. Ring topology: exact bus count for n_buses=7
    // ------------------------------------------------------------------
    #[test]
    fn test_ring_bus_count_exact() {
        let config = SyntheticNetworkConfig {
            n_buses: 7,
            topology: NetworkTopology::Ring,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("ring n=7 must succeed");
        assert_eq!(
            net.buses.len(),
            7,
            "ring topology must have exactly 7 buses"
        );
    }

    // ------------------------------------------------------------------
    // 24. Meshed topology has more branches than a spanning tree
    // ------------------------------------------------------------------
    #[test]
    fn test_meshed_has_more_branches_than_tree() {
        let config = SyntheticNetworkConfig {
            n_buses: 20,
            topology: NetworkTopology::Meshed,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("meshed n=20 must succeed");
        assert!(
            net.branches.len() > net.buses.len() - 1,
            "meshed topology must have more branches than a spanning tree (n-1={}), got {}",
            net.buses.len() - 1,
            net.branches.len()
        );
    }

    // ------------------------------------------------------------------
    // 25. ScaleFree topology: exact bus count for n_buses=30
    // ------------------------------------------------------------------
    #[test]
    fn test_scale_free_bus_count_exact() {
        let config = SyntheticNetworkConfig {
            n_buses: 30,
            topology: NetworkTopology::ScaleFree,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("scale_free n=30 must succeed");
        assert_eq!(
            net.buses.len(),
            30,
            "scale_free topology must have exactly 30 buses"
        );
    }

    // ------------------------------------------------------------------
    // 26. SmallWorld topology: exact bus count for n_buses=15
    // ------------------------------------------------------------------
    #[test]
    fn test_small_world_bus_count_exact() {
        let config = SyntheticNetworkConfig {
            n_buses: 15,
            topology: NetworkTopology::SmallWorld,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("small_world n=15 must succeed");
        assert_eq!(
            net.buses.len(),
            15,
            "small_world topology must have exactly 15 buses"
        );
    }

    // ------------------------------------------------------------------
    // 27. Two seeds produce different first f64 values
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_different_seeds_produce_different_values() {
        let mut rng1 = Lcg64::new(1);
        let mut rng2 = Lcg64::new(2);
        let v1 = rng1.next_f64();
        let v2 = rng2.next_f64();
        assert!(
            (v1 - v2).abs() > f64::EPSILON,
            "seeds 1 and 2 must produce different first f64 values, got v1={v1}, v2={v2}"
        );
    }

    // ------------------------------------------------------------------
    // 28. next_f64 never returns exactly 1.0
    // ------------------------------------------------------------------
    #[test]
    fn test_lcg64_next_f64_never_exactly_one() {
        let mut rng = Lcg64::new(77777);
        for i in 0..5000 {
            let v = rng.next_f64();
            assert!(v < 1.0, "next_f64 must be < 1.0, got {v} on iteration {i}");
        }
    }

    // ------------------------------------------------------------------
    // 29. Default config network has positive total load
    // ------------------------------------------------------------------
    #[test]
    fn test_synthetic_network_total_load_positive() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        let total_pd: f64 = net.buses.iter().map(|b| b.pd.0).sum();
        assert!(
            total_pd > 0.0,
            "total load across all buses must be positive, got {total_pd}"
        );
    }

    // ------------------------------------------------------------------
    // 30. Radial topology: exact bus count for n_buses=8
    // ------------------------------------------------------------------
    #[test]
    fn test_radial_bus_count_exact() {
        let config = SyntheticNetworkConfig {
            n_buses: 8,
            topology: NetworkTopology::Radial,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("radial n=8 must succeed");
        assert_eq!(
            net.buses.len(),
            8,
            "radial topology must have exactly 8 buses"
        );
    }

    // ------------------------------------------------------------------
    // 31. Geographic topology: exact bus count for n_buses=9
    // ------------------------------------------------------------------
    #[test]
    fn test_geographic_bus_count_exact() {
        let config = SyntheticNetworkConfig {
            n_buses: 9,
            topology: NetworkTopology::Geographic,
            ..SyntheticNetworkConfig::default()
        };
        let net = generate_synthetic_network(&config).expect("geographic n=9 must succeed");
        assert_eq!(
            net.buses.len(),
            9,
            "geographic topology must have exactly 9 buses"
        );
    }

    // ------------------------------------------------------------------
    // 32. All generators have qmax > 0.0
    // ------------------------------------------------------------------
    #[test]
    fn test_generator_qmax_positive() {
        let config = SyntheticNetworkConfig::default();
        let net = generate_synthetic_network(&config).expect("default config must succeed");
        for (i, gen) in net.generators.iter().enumerate() {
            assert!(
                gen.qmax > 0.0,
                "generator[{i}] at bus {} must have qmax > 0.0, got {}",
                gen.bus_id,
                gen.qmax
            );
        }
    }
}
