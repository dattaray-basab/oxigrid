//! Comprehensive synthetic grid data generator for testing.
//!
//! Generates realistic synthetic power systems with buses, branches, generators,
//! loads, and renewables for any of six topology models.
//!
//! # Supported Topologies
//! - [`SyntheticTopology::Radial`] — tree structure (no cycles)
//! - [`SyntheticTopology::Ring`] — single closed ring (N buses, N branches)
//! - [`SyntheticTopology::Meshed`] — random geometric graph with multiple paths
//! - [`SyntheticTopology::SmallWorld`] — Watts-Strogatz clustering model
//! - [`SyntheticTopology::ScaleFree`] — Barabasi-Albert preferential attachment
//! - [`SyntheticTopology::Regional`] — geographically clustered sub-networks
//!
//! All random numbers use the Knuth LCG to avoid external RNG dependencies.
//!
//! # Example
//! ```rust,ignore
//! use oxigrid::testcases::synthetic_grid::{SyntheticGridConfig, SyntheticTopology,
//!     SyntheticGridGenerator};
//!
//! let config = SyntheticGridConfig {
//!     n_buses: 20,
//!     n_generators: 4,
//!     n_loads: 10,
//!     topology: SyntheticTopology::Meshed,
//!     voltage_level_kv: 110.0,
//!     base_mva: 100.0,
//!     seed: 42,
//!     add_renewables: true,
//!     renewable_fraction: 0.3,
//! };
//! let gen = SyntheticGridGenerator::new(config);
//! let grid = gen.generate().unwrap();
//! ```

use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors produced during grid generation.
#[derive(Debug, Error)]
pub enum GridGenError {
    /// The requested configuration is invalid.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// The generated grid failed validation.
    #[error("validation failed: {reasons:?}")]
    ValidationFailed { reasons: Vec<String> },
}

// ─────────────────────────────────────────────────────────────────────────────
// LCG (project-local copy to avoid cross-module borrowing issues)
// ─────────────────────────────────────────────────────────────────────────────

struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn next_usize(&mut self, n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        (self.next_u64() % n as u64) as usize
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Topology enum
// ─────────────────────────────────────────────────────────────────────────────

/// Grid topology model for the synthetic generator.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SyntheticTopology {
    /// Tree (spanning-tree) distribution style — no cycles.
    Radial,
    /// Single closed ring: N buses, N branches.
    Ring,
    /// Random geometric graph with multiple paths (transmission style).
    Meshed,
    /// Watts-Strogatz small-world model.
    SmallWorld,
    /// Barabasi-Albert preferential-attachment scale-free model.
    ScaleFree,
    /// Geographically clustered sub-networks.
    Regional {
        /// Number of geographic regions / clusters.
        n_regions: usize,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Top-level configuration for the synthetic grid generator.
#[derive(Debug, Clone)]
pub struct SyntheticGridConfig {
    /// Total number of buses.
    pub n_buses: usize,
    /// Number of generators to place.
    pub n_generators: usize,
    /// Number of loads to place.
    pub n_loads: usize,
    /// Network topology model.
    pub topology: SyntheticTopology,
    /// Nominal voltage level \[kV\].
    pub voltage_level_kv: f64,
    /// System base MVA.
    pub base_mva: f64,
    /// Random seed for reproducibility.
    pub seed: u64,
    /// Whether to add renewable generators.
    pub add_renewables: bool,
    /// Fraction of total generation capacity that is renewable.
    pub renewable_fraction: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid data structures
// ─────────────────────────────────────────────────────────────────────────────

/// A synthetic bus.
#[derive(Debug, Clone)]
pub struct SyntheticBus {
    /// Unique bus identifier (0-indexed).
    pub id: usize,
    /// Nominal voltage \[kV\].
    pub voltage_kv: f64,
    /// Longitude / X coordinate (normalised \[0, 1\]).
    pub x_coord: f64,
    /// Latitude / Y coordinate (normalised \[0, 1\]).
    pub y_coord: f64,
    /// Bus type string: `"PQ"`, `"PV"`, or `"Slack"`.
    pub bus_type: String,
}

/// A synthetic branch (transmission line or transformer).
#[derive(Debug, Clone)]
pub struct SyntheticBranch {
    /// From-bus identifier.
    pub from_bus: usize,
    /// To-bus identifier.
    pub to_bus: usize,
    /// Resistance \[pu\].
    pub r_pu: f64,
    /// Reactance \[pu\].
    pub x_pu: f64,
    /// Shunt susceptance \[pu\].
    pub b_pu: f64,
    /// MVA rating \[MW\].
    pub rating_mw: f64,
    /// Physical length \[km\].
    pub length_km: f64,
}

/// A synthetic conventional generator.
#[derive(Debug, Clone)]
pub struct SyntheticGenerator {
    /// Hosting bus.
    pub bus: usize,
    /// Maximum active power output \[MW\].
    pub p_max_mw: f64,
    /// Minimum active power output (must-run) \[MW\].
    pub p_min_mw: f64,
    /// Quadratic cost coefficient \[USD/MWh²\].
    pub cost_a: f64,
    /// Linear cost coefficient \[USD/MWh\].
    pub cost_b: f64,
    /// No-load (fixed) cost \[USD/h\].
    pub cost_c: f64,
    /// Inertia constant \[s\].
    pub h_s: f64,
    /// Generator technology type (`"Gas"`, `"Coal"`, `"Hydro"`, etc.).
    pub gen_type: String,
}

/// A synthetic load.
#[derive(Debug, Clone)]
pub struct SyntheticLoad {
    /// Hosting bus.
    pub bus: usize,
    /// Peak active power demand \[MW\].
    pub p_mw: f64,
    /// Peak reactive power demand \[MVAr\].
    pub q_mvar: f64,
    /// Load factor: peak / average (> 1).
    pub load_factor: f64,
    /// Normalised hourly annual profile (8760 values in \[0, 1\]).
    pub annual_profile: Vec<f64>,
}

/// A synthetic renewable generator.
#[derive(Debug, Clone)]
pub struct SyntheticRenewable {
    /// Hosting bus.
    pub bus: usize,
    /// Technology: `"Wind"` or `"Solar"`.
    pub technology: String,
    /// Nameplate capacity \[MW\].
    pub capacity_mw: f64,
    /// Normalised hourly capacity factors for one year (8760 values in \[0, 1\]).
    pub capacity_factors: Vec<f64>,
}

/// A fully generated synthetic grid.
#[derive(Debug, Clone)]
pub struct SyntheticGrid {
    /// All buses.
    pub buses: Vec<SyntheticBus>,
    /// All branches.
    pub branches: Vec<SyntheticBranch>,
    /// Conventional generators.
    pub generators: Vec<SyntheticGenerator>,
    /// Loads.
    pub loads: Vec<SyntheticLoad>,
    /// Renewable generators (empty if `add_renewables = false`).
    pub renewables: Vec<SyntheticRenewable>,
    /// Configuration used to generate this grid.
    pub config: SyntheticGridConfig,
}

// ─────────────────────────────────────────────────────────────────────────────
// Generator
// ─────────────────────────────────────────────────────────────────────────────

/// Synthetic grid generation engine.
pub struct SyntheticGridGenerator {
    config: SyntheticGridConfig,
}

impl SyntheticGridGenerator {
    /// Create a new generator with the given configuration.
    pub fn new(config: SyntheticGridConfig) -> Self {
        Self { config }
    }

    /// Generate a complete synthetic grid.
    ///
    /// # Steps
    /// 1. Generate bus coordinates (uniform or regional clustering).
    /// 2. Connect buses based on the chosen topology.
    /// 3. Assign line parameters based on distance and voltage level.
    /// 4. Place generators (large units near transmission hubs).
    /// 5. Place loads (proportional to bus degree).
    /// 6. Optionally add renewables.
    pub fn generate(&self) -> Result<SyntheticGrid, GridGenError> {
        let cfg = &self.config;
        if cfg.n_buses < 2 {
            return Err(GridGenError::InvalidConfig(
                "n_buses must be at least 2".to_string(),
            ));
        }
        if cfg.n_generators == 0 {
            return Err(GridGenError::InvalidConfig(
                "n_generators must be at least 1".to_string(),
            ));
        }

        let mut lcg = Lcg::new(cfg.seed);

        // Step 1: bus coordinates
        let buses = self.generate_buses(&mut lcg);

        // Step 2: topology-specific branch generation
        let branches = self.generate_branches(&buses, &mut lcg)?;

        // Compute bus degrees
        let mut degree = vec![0usize; buses.len()];
        for b in &branches {
            degree[b.from_bus] += 1;
            degree[b.to_bus] += 1;
        }

        // Step 3 (merged into generate_branches): line parameters already set.

        // Step 4: generators
        let generators = self.place_generators(&buses, &degree, &mut lcg);

        // Step 5: loads
        let loads = self.place_loads(&buses, &degree, &mut lcg);

        // Step 6: renewables
        let renewables = if cfg.add_renewables {
            self.place_renewables(&buses, &mut lcg)
        } else {
            Vec::new()
        };

        let grid = SyntheticGrid {
            buses,
            branches,
            generators,
            loads,
            renewables,
            config: cfg.clone(),
        };

        Ok(grid)
    }

    // ── Bus coordinate generation ────────────────────────────────────────────

    fn generate_buses(&self, lcg: &mut Lcg) -> Vec<SyntheticBus> {
        let n = self.config.n_buses;
        let kv = self.config.voltage_level_kv;

        match &self.config.topology {
            SyntheticTopology::Regional { n_regions } => {
                let n_r = (*n_regions).max(1);
                // Place cluster centres
                let centres: Vec<(f64, f64)> =
                    (0..n_r).map(|_| (lcg.next_f64(), lcg.next_f64())).collect();

                (0..n)
                    .map(|i| {
                        let c = &centres[i % n_r];
                        let dx = (lcg.next_f64() - 0.5) * 0.3;
                        let dy = (lcg.next_f64() - 0.5) * 0.3;
                        let bus_type = if i == 0 {
                            "Slack".to_string()
                        } else if i < self.config.n_generators {
                            "PV".to_string()
                        } else {
                            "PQ".to_string()
                        };
                        SyntheticBus {
                            id: i,
                            voltage_kv: kv,
                            x_coord: (c.0 + dx).clamp(0.0, 1.0),
                            y_coord: (c.1 + dy).clamp(0.0, 1.0),
                            bus_type,
                        }
                    })
                    .collect()
            }
            _ => (0..n)
                .map(|i| {
                    let bus_type = if i == 0 {
                        "Slack".to_string()
                    } else if i < self.config.n_generators {
                        "PV".to_string()
                    } else {
                        "PQ".to_string()
                    };
                    SyntheticBus {
                        id: i,
                        voltage_kv: kv,
                        x_coord: lcg.next_f64(),
                        y_coord: lcg.next_f64(),
                        bus_type,
                    }
                })
                .collect(),
        }
    }

    // ── Branch generation ────────────────────────────────────────────────────

    fn generate_branches(
        &self,
        buses: &[SyntheticBus],
        lcg: &mut Lcg,
    ) -> Result<Vec<SyntheticBranch>, GridGenError> {
        let n = buses.len();
        match &self.config.topology {
            SyntheticTopology::Radial => Ok(self.radial_branches(buses, lcg)),
            SyntheticTopology::Ring => Ok(self.ring_branches(buses, lcg)),
            SyntheticTopology::Meshed => Ok(self.meshed_branches(buses, lcg)),
            SyntheticTopology::SmallWorld => Ok(self.small_world_branches(buses, lcg)),
            SyntheticTopology::ScaleFree => Ok(self.scale_free_branches(buses, lcg)),
            SyntheticTopology::Regional { n_regions } => {
                Ok(self.regional_branches(buses, *n_regions, lcg))
            }
        }
        .map(|branches| {
            if branches.is_empty() && n > 1 {
                // Fallback: at least a chain
                (0..n - 1)
                    .map(|i| self.make_branch(i, i + 1, buses, lcg))
                    .collect()
            } else {
                branches
            }
        })
    }

    fn radial_branches(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticBranch> {
        let n = buses.len();
        // Prim-like spanning tree: greedily connect each new bus to a random existing one
        let mut branches = Vec::with_capacity(n - 1);
        for i in 1..n {
            let j = lcg.next_usize(i); // random existing bus
            branches.push(self.make_branch(i, j, buses, lcg));
        }
        branches
    }

    fn ring_branches(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticBranch> {
        let n = buses.len();
        // N branches forming a closed ring: 0-1, 1-2, ..., (N-1)-0
        (0..n)
            .map(|i| self.make_branch(i, (i + 1) % n, buses, lcg))
            .collect()
    }

    fn meshed_branches(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticBranch> {
        let n = buses.len();
        // Start with spanning tree (radial), then add random extra edges
        let mut branches = self.radial_branches(buses, lcg);
        let extra = (n as f64 * 0.4).ceil() as usize; // ~40% more branches
        let mut added: std::collections::HashSet<(usize, usize)> = branches
            .iter()
            .map(|b| {
                let a = b.from_bus.min(b.to_bus);
                let c = b.from_bus.max(b.to_bus);
                (a, c)
            })
            .collect();

        let mut attempts = 0usize;
        while branches.len() < n - 1 + extra && attempts < 5 * extra {
            let i = lcg.next_usize(n);
            let j = lcg.next_usize(n);
            if i != j {
                let key = (i.min(j), i.max(j));
                if added.insert(key) {
                    branches.push(self.make_branch(i, j, buses, lcg));
                }
            }
            attempts += 1;
        }
        branches
    }

    fn small_world_branches(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticBranch> {
        let n = buses.len();
        let k = 2; // each node initially connected to k nearest neighbours (ring)
        let mut added: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
        let mut branches = Vec::new();

        // Build initial ring-of-k
        for i in 0..n {
            for d in 1..=k {
                let j = (i + d) % n;
                let key = (i.min(j), i.max(j));
                if key.0 != key.1 && added.insert(key) {
                    branches.push(self.make_branch(i, j, buses, lcg));
                }
            }
        }

        // Rewire with probability p = 0.3
        let p_rewire = 0.3_f64;
        let n_rewire = (branches.len() as f64 * p_rewire) as usize;
        for _ in 0..n_rewire {
            if let Some(br) = branches.last_mut() {
                let new_to = lcg.next_usize(n);
                let key = (br.from_bus.min(new_to), br.from_bus.max(new_to));
                if br.from_bus != new_to && added.insert(key) {
                    br.to_bus = new_to;
                }
            }
        }

        branches
    }

    fn scale_free_branches(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticBranch> {
        let n = buses.len();
        if n < 3 {
            return self.radial_branches(buses, lcg);
        }
        // Barabasi-Albert: start with a triangle, then attach each new node
        // proportionally to degree.
        let mut degree = vec![0usize; n];
        let mut branches: Vec<SyntheticBranch> = Vec::new();
        let mut added: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();

        // Initial triangle
        for &(a, b) in &[(0usize, 1usize), (1, 2), (2, 0)] {
            if added.insert((a, b)) {
                branches.push(self.make_branch(a, b, buses, lcg));
                degree[a] += 1;
                degree[b] += 1;
            }
        }

        // Preferential attachment
        let m = 2usize; // edges per new node
        for new_node in 3..n {
            let total_deg: usize = degree[..new_node].iter().sum();
            if total_deg == 0 {
                continue;
            }
            let mut targets: std::collections::HashSet<usize> = std::collections::HashSet::new();
            let mut tries = 0usize;
            while targets.len() < m.min(new_node) && tries < 10 * n {
                let r = (lcg.next_u64() % total_deg as u64) as usize;
                let mut cumsum = 0usize;
                for (node, &deg) in degree[..new_node].iter().enumerate() {
                    cumsum += deg;
                    if cumsum > r {
                        targets.insert(node);
                        break;
                    }
                }
                tries += 1;
            }
            for tgt in targets {
                let key = (new_node.min(tgt), new_node.max(tgt));
                if added.insert(key) {
                    branches.push(self.make_branch(new_node, tgt, buses, lcg));
                    degree[new_node] += 1;
                    degree[tgt] += 1;
                }
            }
        }
        branches
    }

    fn regional_branches(
        &self,
        buses: &[SyntheticBus],
        n_regions: usize,
        lcg: &mut Lcg,
    ) -> Vec<SyntheticBranch> {
        let n = buses.len();
        let n_r = n_regions.max(1);
        let mut branches = Vec::new();
        let mut added: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();

        // Within-region: radial spanning tree per region
        for r in 0..n_r {
            let region_buses: Vec<usize> = (0..n).filter(|&i| i % n_r == r).collect();
            for idx in 1..region_buses.len() {
                let i = region_buses[idx];
                let j_pos = lcg.next_usize(idx);
                let j = region_buses[j_pos];
                let key = (i.min(j), i.max(j));
                if i != j && added.insert(key) {
                    branches.push(self.make_branch(i, j, buses, lcg));
                }
            }
        }

        // Cross-region ties: one tie per adjacent region pair
        for r in 0..n_r - 1 {
            let region_a: Vec<usize> = (0..n).filter(|&i| i % n_r == r).collect();
            let region_b: Vec<usize> = (0..n).filter(|&i| i % n_r == r + 1).collect();
            if !region_a.is_empty() && !region_b.is_empty() {
                let i = region_a[lcg.next_usize(region_a.len())];
                let j = region_b[lcg.next_usize(region_b.len())];
                let key = (i.min(j), i.max(j));
                if added.insert(key) {
                    branches.push(self.make_branch(i, j, buses, lcg));
                }
            }
        }

        // Ensure connectivity: if any bus is disconnected, chain it in
        let mut degree = vec![0usize; n];
        for b in &branches {
            degree[b.from_bus] += 1;
            degree[b.to_bus] += 1;
        }
        for (i, &deg) in degree.iter().enumerate() {
            if deg == 0 && i > 0 {
                let j = lcg.next_usize(i);
                let key = (i.min(j), i.max(j));
                if added.insert(key) {
                    branches.push(self.make_branch(i, j, buses, lcg));
                }
            }
        }

        branches
    }

    // ── Branch parameter helper ──────────────────────────────────────────────

    fn make_branch(
        &self,
        from: usize,
        to: usize,
        buses: &[SyntheticBus],
        lcg: &mut Lcg,
    ) -> SyntheticBranch {
        let kv = self.config.voltage_level_kv;
        let base_mva = self.config.base_mva;

        // Physical length based on Euclidean distance in normalised space
        let (dx, dy) = if from < buses.len() && to < buses.len() {
            (
                buses[from].x_coord - buses[to].x_coord,
                buses[from].y_coord - buses[to].y_coord,
            )
        } else {
            (0.1, 0.1)
        };
        // Scale to ~100 km grid
        let length_km = (dx * dx + dy * dy).sqrt() * 100.0 * (0.5 + lcg.next_f64() * 0.5);
        let length_km = length_km.max(1.0);

        // Per-unit parameters (typical 110 kV line: r ≈ 0.08 Ω/km, x ≈ 0.4 Ω/km)
        let z_base = kv * kv / base_mva;
        let r_per_km = 0.08 * (0.8 + lcg.next_f64() * 0.4);
        let x_per_km = 0.4 * (0.8 + lcg.next_f64() * 0.4);
        let b_per_km = 2.8e-6 * (0.8 + lcg.next_f64() * 0.4); // Siemens/km

        let r_pu = r_per_km * length_km / z_base;
        let x_pu = x_per_km * length_km / z_base;
        let b_pu = b_per_km * length_km * z_base; // total line charging

        // Rating: 2× natural load level scaled by voltage
        let rating_mw = kv / 110.0 * base_mva * (0.8 + lcg.next_f64() * 0.4);

        SyntheticBranch {
            from_bus: from,
            to_bus: to,
            r_pu: r_pu.max(1e-5),
            x_pu: x_pu.max(1e-4),
            b_pu,
            rating_mw,
            length_km,
        }
    }

    // ── Generator placement ──────────────────────────────────────────────────

    fn place_generators(
        &self,
        buses: &[SyntheticBus],
        degree: &[usize],
        lcg: &mut Lcg,
    ) -> Vec<SyntheticGenerator> {
        let n_gen = self.config.n_generators.min(buses.len());
        let types = ["Gas", "Coal", "Hydro", "Nuclear", "CCGT"];

        // Sort buses by degree descending; place generators at high-degree buses
        let mut sorted_buses: Vec<usize> = (0..buses.len()).collect();
        sorted_buses.sort_by(|&a, &b| degree[b].cmp(&degree[a]));

        (0..n_gen)
            .map(|i| {
                let bus = sorted_buses[i % sorted_buses.len()];
                let p_max = 100.0 + lcg.next_f64() * 400.0;
                let p_min = p_max * 0.1;
                let gen_type = types[lcg.next_usize(types.len())].to_string();
                let cost_b = match gen_type.as_str() {
                    "Nuclear" => 5.0 + lcg.next_f64() * 5.0,
                    "Hydro" => 3.0 + lcg.next_f64() * 10.0,
                    "Coal" => 20.0 + lcg.next_f64() * 15.0,
                    "Gas" | "CCGT" => 40.0 + lcg.next_f64() * 30.0,
                    _ => 30.0 + lcg.next_f64() * 20.0,
                };
                SyntheticGenerator {
                    bus,
                    p_max_mw: p_max,
                    p_min_mw: p_min,
                    cost_a: 0.002 + lcg.next_f64() * 0.003,
                    cost_b,
                    cost_c: 100.0 + lcg.next_f64() * 500.0,
                    h_s: 3.0 + lcg.next_f64() * 5.0,
                    gen_type,
                }
            })
            .collect()
    }

    // ── Load placement ───────────────────────────────────────────────────────

    fn place_loads(
        &self,
        buses: &[SyntheticBus],
        degree: &[usize],
        lcg: &mut Lcg,
    ) -> Vec<SyntheticLoad> {
        let n_loads = self.config.n_loads.min(buses.len());
        // Weight load assignment by bus degree
        let total_deg: usize = degree.iter().sum::<usize>().max(1);
        let mut cumsum = vec![0usize; buses.len() + 1];
        for (i, &d) in degree.iter().enumerate() {
            cumsum[i + 1] = cumsum[i] + d.max(1);
        }
        let total_cum = cumsum[buses.len()].max(1);

        let mut loads = Vec::with_capacity(n_loads);
        let mut assigned: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut attempts = 0;
        while loads.len() < n_loads && attempts < 5 * n_loads {
            let r = (lcg.next_u64() % total_cum as u64) as usize;
            let bus = cumsum
                .windows(2)
                .position(|w| w[0] <= r && r < w[1])
                .unwrap_or(0);
            if !assigned.insert(bus) {
                attempts += 1;
                continue;
            }
            let p_mw = 10.0 + lcg.next_f64() * 90.0;
            let q_mvar = p_mw * (0.2 + lcg.next_f64() * 0.3);
            let load_factor = 1.2 + lcg.next_f64() * 0.8;

            // Simple 8760-hour profile: sinusoidal daily + weekly variation
            let annual_profile: Vec<f64> = (0..8760)
                .map(|h| {
                    let daily = (2.0 * std::f64::consts::PI * (h % 24) as f64 / 24.0).sin();
                    (0.6 + 0.4 * (0.5 + 0.5 * daily)).clamp(0.0, 1.0)
                })
                .collect();

            loads.push(SyntheticLoad {
                bus,
                p_mw,
                q_mvar,
                load_factor,
                annual_profile,
            });
            attempts += 1;
            let _ = total_deg; // suppress unused warning
        }
        loads
    }

    // ── Renewable placement ──────────────────────────────────────────────────

    fn place_renewables(&self, buses: &[SyntheticBus], lcg: &mut Lcg) -> Vec<SyntheticRenewable> {
        let total_gen_cap: f64 = self.config.n_generators as f64 * 250.0; // rough estimate
        let ren_cap = total_gen_cap * self.config.renewable_fraction;
        let n_ren = ((ren_cap / 50.0) as usize).max(1).min(buses.len());

        (0..n_ren)
            .map(|i| {
                let bus = i % buses.len();
                let technology = if lcg.next_f64() > 0.5 {
                    "Wind".to_string()
                } else {
                    "Solar".to_string()
                };
                let capacity_mw = 20.0 + lcg.next_f64() * 80.0;

                // Simple capacity factor profiles
                let capacity_factors: Vec<f64> = (0..8760)
                    .map(|h| {
                        if technology == "Solar" {
                            // Solar: zero at night, peak midday
                            let hour_of_day = h % 24;
                            if !(6..=18).contains(&hour_of_day) {
                                0.0
                            } else {
                                let solar_angle =
                                    std::f64::consts::PI * (hour_of_day - 6) as f64 / 12.0;
                                solar_angle.sin() * (0.5 + lcg.next_f64() * 0.5)
                            }
                        } else {
                            // Wind: random with seasonal variation
                            let day = h / 24;
                            let seasonal =
                                0.3 + 0.2 * (2.0 * std::f64::consts::PI * day as f64 / 365.0).cos();
                            (seasonal + lcg.next_f64() * 0.4).clamp(0.0, 1.0)
                        }
                    })
                    .collect();

                SyntheticRenewable {
                    bus,
                    technology,
                    capacity_mw,
                    capacity_factors,
                }
            })
            .collect()
    }

    // ── Validation ───────────────────────────────────────────────────────────

    /// Validate a generated grid for connectivity and sufficiency.
    ///
    /// Returns `Ok(())` if valid, or `Err(Vec<String>)` listing all issues.
    pub fn validate(&self, grid: &SyntheticGrid) -> Result<(), Vec<String>> {
        let mut issues: Vec<String> = Vec::new();

        if grid.buses.is_empty() {
            issues.push("Grid has no buses".to_string());
        }
        if grid.branches.is_empty() {
            issues.push("Grid has no branches".to_string());
        }
        if grid.generators.is_empty() {
            issues.push("Grid has no generators".to_string());
        }

        // Check connectivity via BFS
        if !grid.buses.is_empty() && !grid.branches.is_empty() {
            let n = grid.buses.len();
            let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
            for b in &grid.branches {
                if b.from_bus < n && b.to_bus < n {
                    adj[b.from_bus].push(b.to_bus);
                    adj[b.to_bus].push(b.from_bus);
                }
            }
            let mut visited = vec![false; n];
            let mut queue = std::collections::VecDeque::new();
            queue.push_back(0usize);
            visited[0] = true;
            while let Some(node) = queue.pop_front() {
                for &next in &adj[node] {
                    if !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
            let n_disconnected = visited.iter().filter(|&&v| !v).count();
            if n_disconnected > 0 {
                issues.push(format!(
                    "{n_disconnected} buses are disconnected from the main network"
                ));
            }
        }

        // Check generation adequacy
        let total_gen: f64 = grid.generators.iter().map(|g| g.p_max_mw).sum();
        let total_load: f64 = grid.loads.iter().map(|l| l.p_mw).sum();
        if total_gen < total_load * 0.9 {
            issues.push(format!(
                "Insufficient generation: {total_gen:.1} MW < 90% of load {:.1} MW",
                total_load
            ));
        }

        if issues.is_empty() {
            Ok(())
        } else {
            Err(issues)
        }
    }

    // ── MATPOWER export ──────────────────────────────────────────────────────

    /// Export the grid to a MATPOWER case format string.
    pub fn to_matpower_format(grid: &SyntheticGrid) -> String {
        let mut s = String::new();
        s.push_str(&format!(
            "function mpc = synthetic_grid_{}\n",
            grid.config.n_buses
        ));
        s.push_str("mpc.version = '2';\n");
        s.push_str(&format!("mpc.baseMVA = {};\n\n", grid.config.base_mva));

        // Bus data
        s.push_str("mpc.bus = [\n");
        for bus in &grid.buses {
            let type_code = match bus.bus_type.as_str() {
                "Slack" => 3,
                "PV" => 2,
                _ => 1,
            };
            s.push_str(&format!(
                "  {} {} 0 0 0 0 1 1.0 0 {} {} {};\n",
                bus.id + 1,
                type_code,
                bus.voltage_kv,
                bus.voltage_kv * 0.95,
                bus.voltage_kv * 1.05
            ));
        }
        s.push_str("];\n\n");

        // Generator data
        s.push_str("mpc.gen = [\n");
        for g in &grid.generators {
            s.push_str(&format!(
                "  {} {:.1} 0 {} {} 1 1.0 1 {} {} 0 0 0 0 0 0 0 0 0 0 0;\n",
                g.bus + 1,
                g.p_max_mw * 0.5,
                g.p_max_mw,
                g.p_min_mw,
                g.p_max_mw,
                g.p_min_mw
            ));
        }
        s.push_str("];\n\n");

        // Branch data
        s.push_str("mpc.branch = [\n");
        for b in &grid.branches {
            s.push_str(&format!(
                "  {} {} {:.6} {:.6} {:.6} {} {} 0 0 0 1 -360 360;\n",
                b.from_bus + 1,
                b.to_bus + 1,
                b.r_pu,
                b.x_pu,
                b.b_pu,
                b.rating_mw,
                b.rating_mw
            ));
        }
        s.push_str("];\n\n");

        // Cost data
        s.push_str("mpc.gencost = [\n");
        for g in &grid.generators {
            s.push_str(&format!(
                "  2 0 0 3 {:.4} {:.4} {:.4};\n",
                g.cost_a, g.cost_b, g.cost_c
            ));
        }
        s.push_str("];\n");

        s
    }

    // ── Grid scaling ─────────────────────────────────────────────────────────

    /// Scale an existing grid by replicating it `scale_factor` times.
    ///
    /// Produces a larger grid by duplicating buses/branches with offset IDs.
    /// Tie branches connect corresponding slack buses across replicas.
    pub fn scale_grid(base: &SyntheticGrid, scale_factor: usize) -> SyntheticGrid {
        let n_base = base.buses.len();
        let n_total = n_base * scale_factor;
        let mut buses: Vec<SyntheticBus> = Vec::with_capacity(n_total);
        let mut branches: Vec<SyntheticBranch> = Vec::new();
        let mut generators: Vec<SyntheticGenerator> = Vec::new();
        let mut loads: Vec<SyntheticLoad> = Vec::new();
        let mut renewables: Vec<SyntheticRenewable> = Vec::new();

        for rep in 0..scale_factor {
            let offset = rep * n_base;
            for b in &base.buses {
                buses.push(SyntheticBus {
                    id: b.id + offset,
                    voltage_kv: b.voltage_kv,
                    x_coord: b.x_coord + rep as f64 * 1.1,
                    y_coord: b.y_coord,
                    bus_type: if b.id == 0 && rep == 0 {
                        "Slack".to_string()
                    } else if b.bus_type == "Slack" && rep > 0 {
                        "PV".to_string()
                    } else {
                        b.bus_type.clone()
                    },
                });
            }
            for br in &base.branches {
                branches.push(SyntheticBranch {
                    from_bus: br.from_bus + offset,
                    to_bus: br.to_bus + offset,
                    ..br.clone()
                });
            }
            for g in &base.generators {
                generators.push(SyntheticGenerator {
                    bus: g.bus + offset,
                    ..g.clone()
                });
            }
            for l in &base.loads {
                loads.push(SyntheticLoad {
                    bus: l.bus + offset,
                    ..l.clone()
                });
            }
            for r in &base.renewables {
                renewables.push(SyntheticRenewable {
                    bus: r.bus + offset,
                    ..r.clone()
                });
            }

            // Tie line between replicas at bus 0
            if rep > 0 {
                let prev_slack = (rep - 1) * n_base;
                let curr_slack = rep * n_base;
                branches.push(SyntheticBranch {
                    from_bus: prev_slack,
                    to_bus: curr_slack,
                    r_pu: 0.01,
                    x_pu: 0.05,
                    b_pu: 0.002,
                    rating_mw: base.config.base_mva,
                    length_km: 50.0,
                });
            }
        }

        let mut new_config = base.config.clone();
        new_config.n_buses = n_total;
        new_config.n_generators = generators.len();
        new_config.n_loads = loads.len();

        SyntheticGrid {
            buses,
            branches,
            generators,
            loads,
            renewables,
            config: new_config,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn base_config(n: usize, topo: SyntheticTopology) -> SyntheticGridConfig {
        SyntheticGridConfig {
            n_buses: n,
            n_generators: 2,
            n_loads: 4,
            topology: topo,
            voltage_level_kv: 110.0,
            base_mva: 100.0,
            seed: 42,
            add_renewables: false,
            renewable_fraction: 0.2,
        }
    }

    #[test]
    fn test_radial_tree_no_cycles() {
        let cfg = base_config(10, SyntheticTopology::Radial);
        let gen = SyntheticGridGenerator::new(cfg.clone());
        let grid = gen.generate().unwrap();

        // A spanning tree of N nodes has exactly N-1 branches
        assert_eq!(
            grid.branches.len(),
            cfg.n_buses - 1,
            "Radial grid should have N-1 branches"
        );
        assert_eq!(grid.buses.len(), cfg.n_buses);
    }

    #[test]
    fn test_ring_exactly_n_branches() {
        let n = 8;
        let cfg = base_config(n, SyntheticTopology::Ring);
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();

        assert_eq!(
            grid.branches.len(),
            n,
            "Ring topology should have exactly N branches"
        );
    }

    #[test]
    fn test_meshed_more_branches_than_buses() {
        let n = 12;
        let cfg = base_config(n, SyntheticTopology::Meshed);
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();

        assert!(
            grid.branches.len() > n,
            "Meshed grid should have more branches than buses: got {} branches for {} buses",
            grid.branches.len(),
            n
        );
    }

    #[test]
    fn test_validate_catches_disconnected_grid() {
        let cfg = base_config(6, SyntheticTopology::Radial);
        let gen = SyntheticGridGenerator::new(cfg.clone());
        let mut grid = gen.generate().unwrap();

        // Artificially disconnect by removing all branches
        grid.branches.clear();

        let result = gen.validate(&grid);
        assert!(
            result.is_err(),
            "Validate should catch a disconnected (no branches) grid"
        );
    }

    #[test]
    fn test_reproducible_same_seed() {
        let cfg = base_config(15, SyntheticTopology::SmallWorld);
        let gen = SyntheticGridGenerator::new(cfg.clone());
        let g1 = gen.generate().unwrap();
        let g2 = gen.generate().unwrap();

        assert_eq!(
            g1.branches.len(),
            g2.branches.len(),
            "Same seed should produce same number of branches"
        );
        for (b1, b2) in g1.branches.iter().zip(g2.branches.iter()) {
            assert_eq!(b1.from_bus, b2.from_bus);
            assert_eq!(b1.to_bus, b2.to_bus);
        }
    }

    #[test]
    fn test_scale_grid_doubles_buses() {
        let cfg = base_config(8, SyntheticTopology::Radial);
        let gen = SyntheticGridGenerator::new(cfg);
        let base = gen.generate().unwrap();
        let scaled = SyntheticGridGenerator::scale_grid(&base, 2);

        assert_eq!(scaled.buses.len(), base.buses.len() * 2);
        assert!(scaled.generators.len() >= base.generators.len() * 2);
    }

    #[test]
    fn test_renewables_added_when_configured() {
        let mut cfg = base_config(10, SyntheticTopology::Radial);
        cfg.add_renewables = true;
        cfg.renewable_fraction = 0.5;
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();
        assert!(
            !grid.renewables.is_empty(),
            "Renewables should be generated when add_renewables=true"
        );
    }

    #[test]
    fn test_matpower_export_contains_bus_section() {
        let cfg = base_config(5, SyntheticTopology::Ring);
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();
        let mp = SyntheticGridGenerator::to_matpower_format(&grid);
        assert!(
            mp.contains("mpc.bus"),
            "MATPOWER export should contain bus data"
        );
        assert!(
            mp.contains("mpc.branch"),
            "MATPOWER export should contain branch data"
        );
        assert!(
            mp.contains("mpc.gen"),
            "MATPOWER export should contain generator data"
        );
    }

    #[test]
    fn test_small_world_generates_valid_grid() {
        let cfg = base_config(20, SyntheticTopology::SmallWorld);
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();
        assert_eq!(grid.buses.len(), 20);
        assert!(!grid.branches.is_empty());
    }

    #[test]
    fn test_scale_free_generates_valid_grid() {
        let cfg = base_config(15, SyntheticTopology::ScaleFree);
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();
        assert_eq!(grid.buses.len(), 15);
        assert!(!grid.branches.is_empty());
    }

    #[test]
    fn test_regional_topology() {
        let mut cfg = base_config(18, SyntheticTopology::Regional { n_regions: 3 });
        cfg.n_buses = 18;
        let gen = SyntheticGridGenerator::new(cfg);
        let grid = gen.generate().unwrap();
        assert_eq!(grid.buses.len(), 18);
        assert!(!grid.branches.is_empty());
    }
}
