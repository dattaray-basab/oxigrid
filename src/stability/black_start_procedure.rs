//! Black Start Procedure Planning and Simulation
//!
//! This module implements a full black start (BS) capability assessment,
//! cranking-path discovery, restoration sequencing, and time-domain
//! simulation of the island-building process following a total blackout.
//!
//! # Key concepts
//! - **Black-start unit** — a generator that can self-energise without
//!   external grid power (hydro, diesel, battery-backed, etc.)
//! - **Cranking path** — the sequence of transmission branches that must
//!   be energised to deliver cranking power from a BS unit to a non-BS
//!   generator.
//! - **Power island** — an isolated sub-network that is being restored
//!   independently before merging with neighbouring islands.
//! - **Cold load pickup (CLPU)** — the inrush that occurs when load is
//!   restored after a cold outage; modelled as a multiplier on nominal MW.
//!
//! # Algorithm overview
//! 1. Identify all BS-capable generators.
//! 2. For each BS unit, BFS the branch graph to find shortest cranking
//!    paths to non-BS generators.
//! 3. Build a step-by-step restoration plan, respecting generation
//!    headroom and CLPU frequency constraints.
//! 4. Simulate execution of the plan, tracking frequency nadir and
//!    voltage violations at each load-pickup event.

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Black-start capability level of a generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BlackStartCapability {
    /// Generator cannot self-start (requires external cranking power).
    None,
    /// Can energise its own auxiliary loads for a limited hold time.
    Limited {
        /// Maximum MW the unit can supply during the limited period.
        startup_mw: f64,
    },
    /// Full black-start: can supply cranking power to remote generators.
    Full {
        /// Maximum MW available for cranking / initial energisation.
        startup_mw: f64,
        /// How long the unit can hold at startup output before needing load [min].
        hold_time_min: f64,
    },
}

/// Technology class of a generator, which determines BS eligibility.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GeneratorType {
    /// Hydro — typically excellent BS units.
    Hydro,
    /// Gas turbine — BS eligible only when fuel is available.
    Gas { fuel_available: bool },
    /// Nuclear — always requires external power; cannot BS.
    Nuclear,
    /// Diesel genset — BS if fuel level is adequate.
    Diesel { fuel_level_pct: f64 },
    /// Battery-backed thermal unit — instant response.
    BatteryBacked,
    /// Wind turbine with co-located battery — limited BS.
    WindWithBattery,
    /// PV solar — not BS capable on its own (no inertia, daylight-dependent).
    Solar,
}

/// Priority class for load restoration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum RestorePriority {
    /// Life-safety critical infrastructure.
    CriticalInfrastructure = 1,
    /// Police, fire, emergency dispatch.
    EmergencyServices = 2,
    /// Hospital and healthcare facilities.
    Hospitals = 3,
    /// Water treatment and wastewater.
    WaterWastewater = 4,
    /// Residential customers.
    Residential = 5,
    /// Industrial processes.
    Industrial = 6,
    /// Commercial (retail, offices, etc.).
    Commercial = 7,
}

/// Type of a black-start plan step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BlackStartStepType {
    /// Start a black-start-capable unit.
    StartBlackStartUnit { gen_id: usize },
    /// Close the breaker on a transmission branch.
    EnergizeLine { branch_id: usize },
    /// Synchronise a generator to an existing island.
    SynchronizeGenerator {
        gen_id: usize,
        from_island: usize,
        to_island: usize,
    },
    /// Pick up a load group (cold load pickup).
    PickupLoad {
        load_group_id: usize,
        amount_mw: f64,
    },
    /// Merge two power islands by closing a tie-line.
    IslandMerge { island_a: usize, island_b: usize },
    /// Restore system frequency toward nominal.
    FrequencyRestore { target_hz: f64 },
    /// Restore bus voltage toward nominal.
    VoltageRestore { target_pu: f64 },
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// A generator that participates in the black-start process.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackStartGenerator {
    /// Unique generator identifier.
    pub id: usize,
    /// Bus to which this generator is connected.
    pub bus: usize,
    /// Nameplate rating [MW].
    pub rated_mw: f64,
    /// Black-start capability level.
    pub capability: BlackStartCapability,
    /// Time from cold state to minimum stable output [min].
    pub startup_time_min: f64,
    /// Minimum stable generation [MW].
    pub min_stable_mw: f64,
    /// Maximum ramp rate [MW/min].
    pub max_ramp_mw_per_min: f64,
    /// Self-auxiliary power consumption [MW].
    pub aux_power_mw: f64,
    /// Technology type.
    pub technology: GeneratorType,
}

impl BlackStartGenerator {
    /// Returns `true` if this generator has full or limited BS capability
    /// *and* its technology type is operationally eligible.
    pub fn is_black_start_eligible(&self) -> bool {
        match &self.capability {
            BlackStartCapability::None => false,
            BlackStartCapability::Limited { .. } | BlackStartCapability::Full { .. } => {
                match &self.technology {
                    GeneratorType::Nuclear => false,
                    GeneratorType::Gas { fuel_available } => *fuel_available,
                    GeneratorType::Diesel { fuel_level_pct } => *fuel_level_pct > 10.0,
                    GeneratorType::Solar => false,
                    _ => true,
                }
            }
        }
    }

    /// Net exportable MW at startup (rated minus auxiliary load).
    pub fn net_startup_mw(&self) -> f64 {
        match &self.capability {
            BlackStartCapability::None => 0.0,
            BlackStartCapability::Limited { startup_mw } => {
                (startup_mw - self.aux_power_mw).max(0.0)
            }
            BlackStartCapability::Full { startup_mw, .. } => {
                (startup_mw - self.aux_power_mw).max(0.0)
            }
        }
    }
}

/// A load group whose pickup is managed during restoration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestorationLoad {
    /// Unique load identifier.
    pub id: usize,
    /// Bus to which this load is connected.
    pub bus: usize,
    /// Restoration priority class.
    pub priority: RestorePriority,
    /// Nominal active power [MW].
    pub p_mw: f64,
    /// Nominal reactive power [MVAR].
    pub q_mvar: f64,
    /// Cold-load-pickup multiplier (typically 1.5 – 3.0).
    pub cold_load_pickup_factor: f64,
    /// Duration of elevated demand during cold pickup [min].
    pub pickup_duration_min: f64,
    /// Minimum acceptable voltage during restoration [p.u.].
    pub min_voltage_pu: f64,
    /// `true` for hospitals, water plants, emergency services, etc.
    pub is_critical_infra: bool,
}

/// A cranking path through the network from a BS generator to another generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrankingPath {
    /// ID of the black-start generator supplying cranking power.
    pub from_gen: usize,
    /// ID of the generator to be cranked.
    pub to_gen: usize,
    /// Ordered sequence of branch indices to energise.
    pub branch_sequence: Vec<usize>,
    /// Total series impedance of the path [p.u.].
    pub path_impedance_pu: f64,
    /// Total capacitive charging MVAR of energised lines [MVAR].
    pub charging_mvar: f64,
    /// Approximate physical length [km].
    pub path_length_km: f64,
}

/// One step in the black-start restoration plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackStartStep {
    /// Sequential step identifier (0-based).
    pub step_id: usize,
    /// What action this step represents.
    pub step_type: BlackStartStepType,
    /// Planned start time relative to T=0 [min].
    pub start_time_min: f64,
    /// Estimated time to complete this step [min].
    pub estimated_duration_min: f64,
    /// Step IDs that must be fully completed before this step begins.
    pub prerequisites: Vec<usize>,
    /// Operating crew / control-room party responsible.
    pub responsible_party: String,
    /// Description of the verification check before proceeding.
    pub verification_check: String,
}

/// A self-contained energised sub-network during restoration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerIsland {
    /// Unique island identifier.
    pub id: usize,
    /// Buses that have been energised in this island.
    pub energized_buses: Vec<usize>,
    /// Generator IDs currently online in this island.
    pub online_generators: Vec<usize>,
    /// Total load connected [MW].
    pub connected_loads_mw: f64,
    /// Current frequency [Hz].
    pub frequency_hz: f64,
    /// Average voltage at the island centre [p.u.].
    pub voltage_center_pu: f64,
    /// Available headroom for additional load pickup [MW].
    pub headroom_mw: f64,
    /// `true` for the island anchored by the first BS unit (reference frame).
    pub is_reference_island: bool,
}

/// Complete black-start restoration plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackStartPlan {
    /// Ordered list of restoration steps.
    pub steps: Vec<BlackStartStep>,
    /// Cranking paths used during the plan.
    pub cranking_paths: Vec<CrankingPath>,
    /// Total estimated time from T=0 to full restoration [min].
    pub estimated_restoration_time_min: f64,
    /// Estimated time to restore all critical/priority loads [min].
    pub priority_load_restoration_min: f64,
    /// Total MW of load included in restoration plan.
    pub total_load_restored_mw: f64,
    /// Number of independent power islands formed.
    pub n_islands_formed: usize,
}

/// An event recorded during simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackStartEvent {
    /// Short classification tag (e.g. `"LINE_ENERGIZED"`, `"LOAD_PICKUP"`).
    pub event_type: String,
    /// Bus or generator that is the subject of this event.
    pub bus_or_gen_id: usize,
    /// Human-readable description.
    pub description: String,
    /// Whether the action succeeded (false = skipped/failed due to constraint).
    pub success: bool,
}

/// Full result of a black-start simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackStartSimulationResult {
    /// Chronological event log: (time_min, event).
    pub timeline: Vec<(f64, BlackStartEvent)>,
    /// Snapshot of all power islands at the end of simulation.
    pub islands: Vec<PowerIsland>,
    /// Fraction of total system load restored [%].
    pub restored_fraction_pct: f64,
    /// Whether all critical-infrastructure loads have been restored.
    pub critical_load_restored: bool,
    /// Time at which 100 % of planned load was restored [min].
    pub time_to_full_restoration_min: f64,
    /// Frequency violations: (time_min, observed_freq_hz).
    pub frequency_violations: Vec<(f64, f64)>,
    /// Voltage violations: (time_min, bus_id, V_pu).
    pub voltage_violations: Vec<(f64, usize, f64)>,
    /// Contribution to SAIDI from this blackout event [min].
    pub saidi_contribution_min: f64,
}

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------

/// Plans the optimal black-start restoration sequence for a power system.
pub struct BlackStartPlanner {
    /// All generators in the system (BS-capable and non-BS alike).
    pub generators: Vec<BlackStartGenerator>,
    /// All load groups to be restored.
    pub loads: Vec<RestorationLoad>,
    /// Total number of buses.
    pub n_buses: usize,
    /// Total number of branches.
    pub n_branches: usize,
    /// Branch endpoints: `branch_connectivity[i] = (from_bus, to_bus)`.
    pub branch_connectivity: Vec<(usize, usize)>,
    /// Branch series impedance: `branch_impedances[i] = (r_pu, x_pu)`.
    pub branch_impedances: Vec<(f64, f64)>,
    /// Line charging susceptance [MVAR at 1 p.u. voltage] per branch.
    pub branch_charging: Vec<f64>,
}

impl BlackStartPlanner {
    /// Construct a new planner.
    pub fn new(
        generators: Vec<BlackStartGenerator>,
        loads: Vec<RestorationLoad>,
        n_buses: usize,
        n_branches: usize,
        branch_connectivity: Vec<(usize, usize)>,
        branch_impedances: Vec<(f64, f64)>,
        branch_charging: Vec<f64>,
    ) -> Self {
        Self {
            generators,
            loads,
            n_buses,
            n_branches,
            branch_connectivity,
            branch_impedances,
            branch_charging,
        }
    }

    /// Return references to all generators that are eligible to black-start.
    pub fn identify_black_start_units(&self) -> Vec<&BlackStartGenerator> {
        self.generators
            .iter()
            .filter(|g| g.is_black_start_eligible())
            .collect()
    }

    /// BFS on the branch graph to find the shortest cranking path (fewest
    /// branch hops) between `from_gen_bus` and `to_gen_bus`.
    ///
    /// Returns `None` when the two buses are not connected.
    pub fn find_cranking_path(
        &self,
        from_gen_bus: usize,
        to_gen_bus: usize,
    ) -> Option<CrankingPath> {
        if from_gen_bus == to_gen_bus {
            return None;
        }
        if from_gen_bus >= self.n_buses || to_gen_bus >= self.n_buses {
            return None;
        }

        // Build adjacency list: bus -> Vec<(neighbour_bus, branch_index)>
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); self.n_buses];
        for (br_idx, &(a, b)) in self.branch_connectivity.iter().enumerate() {
            if a < self.n_buses && b < self.n_buses {
                adj[a].push((b, br_idx));
                adj[b].push((a, br_idx));
            }
        }

        // BFS state: (current_bus, path_of_branch_indices)
        let mut visited = vec![false; self.n_buses];
        let mut queue: VecDeque<(usize, Vec<usize>)> = VecDeque::new();
        visited[from_gen_bus] = true;
        queue.push_back((from_gen_bus, Vec::new()));

        while let Some((current_bus, path)) = queue.pop_front() {
            if current_bus == to_gen_bus {
                let mut total_r = 0.0_f64;
                let mut total_x = 0.0_f64;
                let mut total_b = 0.0_f64;
                let path_len_km = path.len() as f64 * 10.0; // 10 km per hop default
                for &br in &path {
                    let (r, x) = self.branch_impedances[br];
                    total_r += r;
                    total_x += x;
                    total_b += self.branch_charging[br];
                }
                let impedance_mag = (total_r * total_r + total_x * total_x).sqrt();
                return Some(CrankingPath {
                    from_gen: from_gen_bus,
                    to_gen: to_gen_bus,
                    branch_sequence: path,
                    path_impedance_pu: impedance_mag,
                    charging_mvar: total_b,
                    path_length_km: path_len_km,
                });
            }

            for &(next_bus, br_idx) in &adj[current_bus] {
                if !visited[next_bus] {
                    visited[next_bus] = true;
                    let mut new_path = path.clone();
                    new_path.push(br_idx);
                    queue.push_back((next_bus, new_path));
                }
            }
        }
        None
    }

    /// Estimate instantaneous cold-load-pickup demand for a load group.
    ///
    /// During the pickup interval the demand is elevated by
    /// `cold_load_pickup_factor`; it then linearly decays back to nominal.
    ///
    /// `time_since_outage_h` — hours since the load was de-energised.
    ///
    /// Returns the *current* demand [MW] immediately after energisation.
    pub fn estimate_cold_load_pickup(load: &RestorationLoad, time_since_outage_h: f64) -> f64 {
        // CLPU factor grows with outage duration (saturates at the configured
        // maximum after approximately 8 h).
        let duration_factor = (time_since_outage_h / 8.0).min(1.0);
        let effective_factor = 1.0 + (load.cold_load_pickup_factor - 1.0) * duration_factor;
        load.p_mw * effective_factor
    }

    /// Check whether picking up a load group is safe given current island state.
    ///
    /// The criterion is:
    ///   ΔP_pickup ≤ 0.5 · headroom_mw
    ///
    /// which ensures the frequency nadir stays within acceptable bounds
    /// (simplified inertia-based criterion).
    pub fn can_pickup_load(
        &self,
        load: &RestorationLoad,
        island: &PowerIsland,
        system_h_s: f64,
    ) -> bool {
        if island.headroom_mw <= 0.0 {
            return false;
        }
        // CLPU demand — use 8 h outage as conservative assumption
        let pickup_demand = Self::estimate_cold_load_pickup(load, 8.0);
        // Simple inertia-weighted criterion
        let inertia_limit = if system_h_s > 0.0 {
            island.headroom_mw * (system_h_s / 6.0).min(1.0)
        } else {
            island.headroom_mw * 0.5
        };
        pickup_demand <= inertia_limit
    }

    /// Estimate the inrush current and reactive charging load produced by
    /// energising a transmission branch.
    ///
    /// Returns `(inrush_ka, charging_mvar)`.
    ///
    /// # Method
    /// Worst-case inrush: I_inrush = V_nom / X_line (assumes base voltage).
    /// Charging MVAR: Q_C = V² · B_line.
    pub fn estimate_energization_impact(
        &self,
        branch_id: usize,
        island: &PowerIsland,
    ) -> (f64, f64) {
        if branch_id >= self.n_branches {
            return (0.0, 0.0);
        }
        let (_, x) = self.branch_impedances[branch_id];
        let b_pu = self.branch_charging[branch_id];
        let v = island.voltage_center_pu;

        // I_base = S_base / (sqrt(3) * V_base) = 100 MVA / (1.732 * 110 kV) ≈ 0.525 kA
        let i_base_ka = 0.525_f64;
        let inrush_pu = if x.abs() > 1e-9 { v / x } else { 0.0 };
        let inrush_ka = inrush_pu * i_base_ka;

        // Charging: Q = V² * B   (p.u. MVAR on 100 MVA base)
        let charging_mvar = v * v * b_pu * 100.0; // convert to MVAR

        (inrush_ka, charging_mvar)
    }

    /// Compute the total estimated duration of a plan from step timings.
    ///
    /// This accounts for parallel execution: a step can start as soon as
    /// all its prerequisites are done, and the plan finishes when all
    /// steps are done.
    fn estimate_plan_duration(steps: &[BlackStartStep]) -> f64 {
        if steps.is_empty() {
            return 0.0;
        }
        // Build finish-time map keyed by step_id
        let mut finish: HashMap<usize, f64> = HashMap::new();
        for step in steps {
            let earliest_start = step
                .prerequisites
                .iter()
                .filter_map(|&prereq_id| finish.get(&prereq_id).copied())
                .fold(step.start_time_min, f64::max);
            let fi = earliest_start + step.estimated_duration_min;
            finish.insert(step.step_id, fi);
        }
        finish.values().cloned().fold(0.0_f64, f64::max)
    }

    /// Generate a complete black-start restoration plan.
    ///
    /// # Algorithm
    /// 1. Identify BS-capable generators; error if none exist.
    /// 2. Select the reference BS unit (largest `rated_mw`).
    /// 3. For each remaining generator, find the shortest cranking path and
    ///    schedule a startup sequence.
    /// 4. Sort loads by priority and schedule load-pickup steps, checking
    ///    CLPU headroom constraints.
    /// 5. Estimate plan duration and critical-load restoration time.
    pub fn generate_plan(
        &self,
        time_since_outage_h: f64,
        frequency_min_hz: f64,
        frequency_max_hz: f64,
    ) -> Result<BlackStartPlan, OxiGridError> {
        let bs_units = self.identify_black_start_units();
        if bs_units.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "no black-start-capable generators found".into(),
            ));
        }
        if frequency_min_hz >= frequency_max_hz {
            return Err(OxiGridError::InvalidParameter(format!(
                "frequency band invalid: [{}, {}]",
                frequency_min_hz, frequency_max_hz
            )));
        }

        let mut steps: Vec<BlackStartStep> = Vec::new();
        let mut cranking_paths: Vec<CrankingPath> = Vec::new();
        let mut step_id = 0_usize;
        let mut time_cursor = 0.0_f64;
        let mut island_count = 0_usize;

        // Select reference (largest-capacity) BS unit
        let reference_gen = bs_units
            .iter()
            .max_by(|a, b| {
                a.rated_mw
                    .partial_cmp(&b.rated_mw)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
            .ok_or_else(|| OxiGridError::InvalidNetwork("no BS units".into()))?;

        // Step 0: start reference BS unit
        let ref_start_id = step_id;
        steps.push(BlackStartStep {
            step_id: ref_start_id,
            step_type: BlackStartStepType::StartBlackStartUnit {
                gen_id: reference_gen.id,
            },
            start_time_min: time_cursor,
            estimated_duration_min: reference_gen.startup_time_min,
            prerequisites: vec![],
            responsible_party: "System Operator".into(),
            verification_check: "Verify terminal voltage, speed within 0.5 % of nominal".into(),
        });
        time_cursor += reference_gen.startup_time_min;
        step_id += 1;
        island_count += 1;

        // Find cranking paths to non-BS generators and schedule their startup
        let non_bs_buses: Vec<(usize, usize, f64)> = self
            .generators
            .iter()
            .filter(|g| !g.is_black_start_eligible())
            .map(|g| (g.id, g.bus, g.startup_time_min))
            .collect();

        let mut gen_id_map: HashMap<usize, usize> = HashMap::new(); // gen_id -> step_id when online
        gen_id_map.insert(reference_gen.id, ref_start_id);

        for (gen_id, gen_bus, startup_time) in &non_bs_buses {
            if let Some(mut cp) = self.find_cranking_path(reference_gen.bus, *gen_bus) {
                cp.from_gen = reference_gen.id;
                cp.to_gen = *gen_id;

                let t_save = time_cursor;
                let mut last_prereq = ref_start_id;

                // Energise each branch along the cranking path
                for &br_id in &cp.branch_sequence.clone() {
                    let line_step_id = step_id;
                    steps.push(BlackStartStep {
                        step_id: line_step_id,
                        step_type: BlackStartStepType::EnergizeLine { branch_id: br_id },
                        start_time_min: time_cursor,
                        estimated_duration_min: 2.0,
                        prerequisites: vec![last_prereq],
                        responsible_party: "Field Crew".into(),
                        verification_check: "Confirm no fault current, voltage within limits"
                            .into(),
                    });
                    time_cursor += 2.0;
                    last_prereq = line_step_id;
                    step_id += 1;
                }

                // Synchronise and start target generator
                let sync_step_id = step_id;
                steps.push(BlackStartStep {
                    step_id: sync_step_id,
                    step_type: BlackStartStepType::SynchronizeGenerator {
                        gen_id: *gen_id,
                        from_island: 0,
                        to_island: island_count.saturating_sub(1),
                    },
                    start_time_min: time_cursor,
                    estimated_duration_min: startup_time + 5.0,
                    prerequisites: vec![last_prereq],
                    responsible_party: "Plant Operator".into(),
                    verification_check: "Verify sync check relay closed, breaker confirmed closed"
                        .into(),
                });
                gen_id_map.insert(*gen_id, sync_step_id);
                step_id += 1;
                cranking_paths.push(cp);

                // Restore time cursor for next parallel path
                time_cursor = t_save;
            }
        }

        // Advance cursor past all generator startup
        time_cursor = Self::estimate_plan_duration(&steps);

        // Sort loads by priority then schedule load pickups
        let mut sorted_loads: Vec<&RestorationLoad> = self.loads.iter().collect();
        sorted_loads.sort_by_key(|l| l.priority);

        let total_gen_mw: f64 = self
            .generators
            .iter()
            .filter(|g| gen_id_map.contains_key(&g.id))
            .map(|g| (g.rated_mw - g.aux_power_mw).max(0.0))
            .sum();

        let mut headroom_mw = total_gen_mw;
        let mut total_restored_mw = 0.0_f64;
        let mut priority_load_time = time_cursor;
        let mut priority_loads_done = false;

        let default_island = PowerIsland {
            id: 0,
            energized_buses: vec![],
            online_generators: vec![],
            connected_loads_mw: 0.0,
            frequency_hz: frequency_max_hz,
            voltage_center_pu: 1.0,
            headroom_mw,
            is_reference_island: true,
        };

        for load in &sorted_loads {
            let pickup_demand = Self::estimate_cold_load_pickup(load, time_since_outage_h);
            if pickup_demand > headroom_mw {
                continue;
            }
            let can_pick = self.can_pickup_load(load, &default_island, 6.0);
            if !can_pick && !load.is_critical_infra {
                continue;
            }
            steps.push(BlackStartStep {
                step_id,
                step_type: BlackStartStepType::PickupLoad {
                    load_group_id: load.id,
                    amount_mw: pickup_demand,
                },
                start_time_min: time_cursor,
                estimated_duration_min: load.pickup_duration_min.max(5.0),
                prerequisites: vec![],
                responsible_party: "Distribution Operator".into(),
                verification_check: "Verify bus voltage within limits, no protective relay pickup"
                    .into(),
            });
            time_cursor += load.pickup_duration_min.max(5.0);
            step_id += 1;

            headroom_mw -= pickup_demand;
            total_restored_mw += load.p_mw;

            if !priority_loads_done && load.priority >= RestorePriority::Residential {
                priority_load_time = time_cursor;
                priority_loads_done = true;
            }
        }

        // Island merge step (if multiple BS units exist)
        if bs_units.len() > 1 {
            steps.push(BlackStartStep {
                step_id,
                step_type: BlackStartStepType::IslandMerge {
                    island_a: 0,
                    island_b: 1,
                },
                start_time_min: time_cursor,
                estimated_duration_min: 10.0,
                prerequisites: vec![],
                responsible_party: "System Operator".into(),
                verification_check: "Verify phase angle difference < 10°, sync check relay enabled"
                    .into(),
            });
            time_cursor += 10.0;
            step_id += 1;
            island_count += 1;
        }

        let _ = step_id; // suppress unused warning
        let _ = time_cursor; // suppress unused warning

        let estimated_restoration_time_min = Self::estimate_plan_duration(&steps);

        Ok(BlackStartPlan {
            steps,
            cranking_paths,
            estimated_restoration_time_min,
            priority_load_restoration_min: priority_load_time,
            total_load_restored_mw: total_restored_mw,
            n_islands_formed: island_count,
        })
    }
}

// ---------------------------------------------------------------------------
// Simulator
// ---------------------------------------------------------------------------

/// Simulates the time-domain execution of a black-start plan.
pub struct BlackStartSimulator {
    /// The planner containing network and generator data.
    pub planner: BlackStartPlanner,
    /// Simulation time-step [min].
    pub dt_min: f64,
    /// Nominal system frequency [Hz].
    pub frequency_nominal_hz: f64,
    /// Aggregate system inertia constant H [s].
    pub frequency_h_constant_s: f64,
}

impl BlackStartSimulator {
    /// Construct a new simulator with the given planner and time-step.
    ///
    /// The inertia constant defaults to 6.0 s (typical for predominantly
    /// thermal systems).
    pub fn new(planner: BlackStartPlanner, dt_min: f64, frequency_nominal_hz: f64) -> Self {
        Self {
            planner,
            dt_min: dt_min.max(0.1),
            frequency_nominal_hz,
            frequency_h_constant_s: 6.0,
        }
    }

    /// Simulate frequency nadir following a load step of `delta_p_mw` MW.
    ///
    /// Uses the simplified swing-equation response:
    ///
    ///   Δf(t) = -ΔP / (2H) · t + (ΔP / (2H·T_gov)) · (1 - e^{-t/T_gov})
    ///
    /// where T_gov is the governor time constant.  The minimum is found
    /// numerically over a 30-second window (0.5 min).
    ///
    /// Returns the minimum (nadir) frequency [Hz].
    pub fn simulate_frequency_nadir(
        &self,
        delta_p_mw: f64,
        generation_mw: f64,
        h_constant_s: f64,
        t_governor_s: f64,
    ) -> f64 {
        if generation_mw <= 0.0 || h_constant_s <= 0.0 {
            return self.frequency_nominal_hz;
        }
        let delta_p_pu = delta_p_mw / generation_mw;
        let f0 = self.frequency_nominal_hz;
        let h = h_constant_s.max(0.1);
        let t_gov = t_governor_s.max(0.1);

        // Integrate swing equation over [0, 30] s using small time-steps.
        //
        // State: delta_omega [p.u.] = (f - f0) / f0
        // Swing: 2H * d(delta_omega)/dt = -delta_p_pu + p_gov - D * delta_omega
        // Governor: T_gov * dp_gov/dt + p_gov = R * (-delta_omega)
        //   where R = 1/droop = 20 for 5 % droop
        let n_steps = 600_usize; // 30 s at 0.05 s per step for stability
        let dt_s = 30.0 / n_steps as f64;
        let mut f_min = f0;
        let mut delta_omega = 0.0_f64; // per-unit frequency deviation
        let mut p_gov = 0.0_f64;
        let droop = 0.05_f64; // 5 % speed droop
        let gov_gain = 1.0 / droop; // governor gain
        let damping_d = 1.0_f64;

        for _ in 0..n_steps {
            // Governor: first-order response to speed deviation
            let p_gov_ref = gov_gain * (-delta_omega);
            let dp_gov_dt = (p_gov_ref - p_gov) / t_gov;
            p_gov += dp_gov_dt * dt_s;
            // Clamp governor output to [0, 1] p.u.
            p_gov = p_gov.clamp(0.0, 1.0);

            // Swing equation: 2H * d²(delta_omega)/dt² simplified to
            // 2H * d(delta_omega)/dt = net_power_pu
            // — integrated as first-order in d_omega_dt (velocity form)
            let net_pu = -delta_p_pu + p_gov - damping_d * delta_omega;
            let d_omega_dt = net_pu / (2.0 * h);
            delta_omega += d_omega_dt * dt_s;

            let f_now = f0 * (1.0 + delta_omega);
            if f_now < f_min {
                f_min = f_now;
            }
        }
        f_min
    }

    /// Execute the black-start plan and return a full simulation result.
    ///
    /// Steps are executed in order of their `start_time_min`.  At each
    /// step, load-pickup events trigger frequency-nadir checks; violations
    /// are recorded.  Voltage is modelled as a simple affine function of
    /// load-to-generation ratio.
    pub fn simulate(
        &mut self,
        plan: &BlackStartPlan,
        time_since_outage_h: f64,
    ) -> BlackStartSimulationResult {
        let mut timeline: Vec<(f64, BlackStartEvent)> = Vec::new();
        let mut frequency_violations: Vec<(f64, f64)> = Vec::new();
        let mut voltage_violations: Vec<(f64, usize, f64)> = Vec::new();

        let mut online_gen_mw = 0.0_f64;
        let mut online_gen_count = 0_usize;
        let mut total_load_restored_mw = 0.0_f64;
        let total_system_load_mw: f64 = self.planner.loads.iter().map(|l| l.p_mw).sum();
        let mut critical_loads_restored = 0_usize;
        let total_critical_loads = self
            .planner
            .loads
            .iter()
            .filter(|l| l.is_critical_infra)
            .count();
        let mut time_full_restore = plan.estimated_restoration_time_min;
        let mut full_restored = false;

        // Sort steps by start time for deterministic execution
        let mut ordered_steps: Vec<&BlackStartStep> = plan.steps.iter().collect();
        ordered_steps.sort_by(|a, b| {
            a.start_time_min
                .partial_cmp(&b.start_time_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let freq_min_hz = self.frequency_nominal_hz * 0.975; // 2.5 % under-freq threshold

        for step in &ordered_steps {
            let t = step.start_time_min;

            match &step.step_type {
                BlackStartStepType::StartBlackStartUnit { gen_id } => {
                    let mw = self
                        .planner
                        .generators
                        .iter()
                        .find(|g| g.id == *gen_id)
                        .map(|g| (g.rated_mw - g.aux_power_mw).max(0.0))
                        .unwrap_or(0.0);
                    online_gen_mw += mw;
                    online_gen_count += 1;
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "GEN_START".into(),
                            bus_or_gen_id: *gen_id,
                            description: format!(
                                "Black-start generator {} online, +{:.1} MW",
                                gen_id, mw
                            ),
                            success: true,
                        },
                    ));
                }

                BlackStartStepType::EnergizeLine { branch_id } => {
                    let island = PowerIsland {
                        id: 0,
                        energized_buses: vec![],
                        online_generators: vec![],
                        connected_loads_mw: total_load_restored_mw,
                        frequency_hz: self.frequency_nominal_hz,
                        voltage_center_pu: 1.0,
                        headroom_mw: online_gen_mw - total_load_restored_mw,
                        is_reference_island: true,
                    };
                    let (inrush_ka, q_charge) = self
                        .planner
                        .estimate_energization_impact(*branch_id, &island);
                    let v_rise = 1.0 + q_charge / (online_gen_mw.max(1.0) * 10.0);
                    let v_actual = v_rise.min(1.10);
                    if v_actual > 1.05 {
                        voltage_violations.push((t, *branch_id, v_actual));
                    }
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "LINE_ENERGIZED".into(),
                            bus_or_gen_id: *branch_id,
                            description: format!(
                                "Branch {} energised, inrush {:.3} kA, charging {:.2} MVAR",
                                branch_id, inrush_ka, q_charge
                            ),
                            success: true,
                        },
                    ));
                }

                BlackStartStepType::SynchronizeGenerator {
                    gen_id,
                    from_island: _,
                    to_island: _,
                } => {
                    let mw = self
                        .planner
                        .generators
                        .iter()
                        .find(|g| g.id == *gen_id)
                        .map(|g| (g.rated_mw - g.aux_power_mw).max(0.0))
                        .unwrap_or(0.0);
                    online_gen_mw += mw;
                    online_gen_count += 1;
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "GEN_SYNC".into(),
                            bus_or_gen_id: *gen_id,
                            description: format!(
                                "Generator {} synchronised, total online {:.1} MW",
                                gen_id, online_gen_mw
                            ),
                            success: true,
                        },
                    ));
                }

                BlackStartStepType::PickupLoad {
                    load_group_id,
                    amount_mw,
                } => {
                    let load_opt = self.planner.loads.iter().find(|l| l.id == *load_group_id);
                    let headroom = (online_gen_mw - total_load_restored_mw).max(0.0);

                    let clpu_demand = if let Some(load) = load_opt {
                        BlackStartPlanner::estimate_cold_load_pickup(load, time_since_outage_h)
                    } else {
                        *amount_mw
                    };

                    let success = clpu_demand <= headroom;
                    if success {
                        total_load_restored_mw += amount_mw;
                        if let Some(load) = load_opt {
                            if load.is_critical_infra {
                                critical_loads_restored += 1;
                            }
                            let load_ratio = total_load_restored_mw / online_gen_mw.max(1.0);
                            let v_sag = 1.0 - 0.1 * load_ratio;
                            if v_sag < load.min_voltage_pu {
                                voltage_violations.push((t, load.bus, v_sag));
                            }
                        }

                        let f_nadir = self.simulate_frequency_nadir(
                            clpu_demand,
                            online_gen_mw,
                            self.frequency_h_constant_s,
                            10.0,
                        );
                        if f_nadir < freq_min_hz {
                            frequency_violations.push((t, f_nadir));
                        }

                        timeline.push((
                            t,
                            BlackStartEvent {
                                event_type: "LOAD_PICKUP".into(),
                                bus_or_gen_id: *load_group_id,
                                description: format!(
                                    "Load group {} picked up {:.1} MW (CLPU {:.1} MW), f_nadir {:.3} Hz",
                                    load_group_id, amount_mw, clpu_demand, f_nadir
                                ),
                                success: true,
                            },
                        ));
                    } else {
                        timeline.push((
                            t,
                            BlackStartEvent {
                                event_type: "LOAD_PICKUP_DEFERRED".into(),
                                bus_or_gen_id: *load_group_id,
                                description: format!(
                                    "Load group {} deferred: CLPU {:.1} MW > headroom {:.1} MW",
                                    load_group_id, clpu_demand, headroom
                                ),
                                success: false,
                            },
                        ));
                    }
                }

                BlackStartStepType::IslandMerge { island_a, island_b } => {
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "ISLAND_MERGE".into(),
                            bus_or_gen_id: *island_a,
                            description: format!(
                                "Islands {} and {} merged, {} generators online",
                                island_a, island_b, online_gen_count
                            ),
                            success: true,
                        },
                    ));
                }

                BlackStartStepType::FrequencyRestore { target_hz } => {
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "FREQ_RESTORE".into(),
                            bus_or_gen_id: 0,
                            description: format!("AGC activated, targeting {:.2} Hz", target_hz),
                            success: true,
                        },
                    ));
                }

                BlackStartStepType::VoltageRestore { target_pu } => {
                    timeline.push((
                        t,
                        BlackStartEvent {
                            event_type: "VOLT_RESTORE".into(),
                            bus_or_gen_id: 0,
                            description: format!(
                                "AVR/OLTC adjustment, targeting {:.3} p.u.",
                                target_pu
                            ),
                            success: true,
                        },
                    ));
                }
            }

            if !full_restored && total_load_restored_mw >= total_system_load_mw * 0.99 {
                time_full_restore = t;
                full_restored = true;
            }
        }

        let restored_fraction_pct = if total_system_load_mw > 0.0 {
            (total_load_restored_mw / total_system_load_mw * 100.0).min(100.0)
        } else {
            100.0
        };

        let final_island = PowerIsland {
            id: 0,
            energized_buses: (0..self.planner.n_buses).collect(),
            online_generators: self.planner.generators.iter().map(|g| g.id).collect(),
            connected_loads_mw: total_load_restored_mw,
            frequency_hz: self.frequency_nominal_hz,
            voltage_center_pu: 1.0 - 0.02 * (total_load_restored_mw / online_gen_mw.max(1.0)),
            headroom_mw: (online_gen_mw - total_load_restored_mw).max(0.0),
            is_reference_island: true,
        };

        // SAIDI contribution [min]: simplification — plan duration × 1 MW-weighted customer
        let saidi_min = plan.estimated_restoration_time_min;

        BlackStartSimulationResult {
            timeline,
            islands: vec![final_island],
            restored_fraction_pct,
            critical_load_restored: critical_loads_restored >= total_critical_loads,
            time_to_full_restoration_min: time_full_restore,
            frequency_violations,
            voltage_violations,
            saidi_contribution_min: saidi_min,
        }
    }

    /// Compute restoration fraction from an event timeline.
    ///
    /// Counts successful `LOAD_PICKUP` events and sums their MW amounts
    /// relative to total planned load.
    pub fn compute_restoration_fraction(&self, timeline: &[(f64, BlackStartEvent)]) -> f64 {
        let total_mw: f64 = self.planner.loads.iter().map(|l| l.p_mw).sum();
        if total_mw <= 0.0 {
            return 100.0;
        }
        // Parse amount from description: "Load group N picked up X.Y MW ..."
        let restored_mw: f64 = timeline
            .iter()
            .filter(|(_, ev)| ev.event_type == "LOAD_PICKUP" && ev.success)
            .filter_map(|(_, ev)| {
                ev.description
                    .split_whitespace()
                    .collect::<Vec<_>>()
                    .windows(3)
                    .find(|w| w[2] == "MW")
                    .and_then(|w| w[1].parse::<f64>().ok())
            })
            .sum();
        (restored_mw / total_mw * 100.0).min(100.0)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- helpers -----------------------------------------------------------

    fn make_hydro_gen(id: usize, bus: usize, rated_mw: f64) -> BlackStartGenerator {
        BlackStartGenerator {
            id,
            bus,
            rated_mw,
            capability: BlackStartCapability::Full {
                startup_mw: rated_mw * 0.3,
                hold_time_min: 60.0,
            },
            startup_time_min: 20.0,
            min_stable_mw: rated_mw * 0.1,
            max_ramp_mw_per_min: rated_mw * 0.02,
            aux_power_mw: rated_mw * 0.02,
            technology: GeneratorType::Hydro,
        }
    }

    fn make_battery_gen(id: usize, bus: usize, rated_mw: f64) -> BlackStartGenerator {
        BlackStartGenerator {
            id,
            bus,
            rated_mw,
            capability: BlackStartCapability::Full {
                startup_mw: rated_mw,
                hold_time_min: 30.0,
            },
            startup_time_min: 1.0,
            min_stable_mw: 0.0,
            max_ramp_mw_per_min: rated_mw,
            aux_power_mw: 0.0,
            technology: GeneratorType::BatteryBacked,
        }
    }

    fn make_nuclear_gen(id: usize, bus: usize, rated_mw: f64) -> BlackStartGenerator {
        BlackStartGenerator {
            id,
            bus,
            rated_mw,
            capability: BlackStartCapability::None,
            startup_time_min: 600.0,
            min_stable_mw: rated_mw * 0.5,
            max_ramp_mw_per_min: 1.0,
            aux_power_mw: rated_mw * 0.05,
            technology: GeneratorType::Nuclear,
        }
    }

    fn make_gas_gen(id: usize, bus: usize, rated_mw: f64, fuel: bool) -> BlackStartGenerator {
        BlackStartGenerator {
            id,
            bus,
            rated_mw,
            capability: BlackStartCapability::Full {
                startup_mw: rated_mw * 0.2,
                hold_time_min: 45.0,
            },
            startup_time_min: 15.0,
            min_stable_mw: rated_mw * 0.15,
            max_ramp_mw_per_min: rated_mw * 0.05,
            aux_power_mw: rated_mw * 0.01,
            technology: GeneratorType::Gas {
                fuel_available: fuel,
            },
        }
    }

    fn make_load(
        id: usize,
        bus: usize,
        priority: RestorePriority,
        p_mw: f64,
        critical: bool,
    ) -> RestorationLoad {
        RestorationLoad {
            id,
            bus,
            priority,
            p_mw,
            q_mvar: p_mw * 0.3,
            cold_load_pickup_factor: 2.0,
            pickup_duration_min: 10.0,
            min_voltage_pu: 0.9,
            is_critical_infra: critical,
        }
    }

    /// Minimal 3-bus, 2-branch network: 0──0──1──1──2
    fn make_simple_planner(
        gens: Vec<BlackStartGenerator>,
        loads: Vec<RestorationLoad>,
    ) -> BlackStartPlanner {
        BlackStartPlanner::new(
            gens,
            loads,
            3,
            2,
            vec![(0, 1), (1, 2)],
            vec![(0.01, 0.1), (0.01, 0.1)],
            vec![0.05, 0.05],
        )
    }

    // ---- generator eligibility tests ---------------------------------------

    #[test]
    fn test_black_start_gen_hydro() {
        let g = make_hydro_gen(0, 0, 100.0);
        assert!(g.is_black_start_eligible());
        assert!(g.net_startup_mw() > 0.0);
    }

    #[test]
    fn test_black_start_gen_battery() {
        let g = make_battery_gen(1, 1, 50.0);
        assert!(g.is_black_start_eligible());
        // Battery has no aux load → net == rated
        assert!((g.net_startup_mw() - 50.0).abs() < 1e-9);
    }

    #[test]
    fn test_black_start_gen_nuclear_not_eligible() {
        let g = make_nuclear_gen(2, 2, 1000.0);
        assert!(!g.is_black_start_eligible());
        assert!(g.net_startup_mw() < 1e-9);
    }

    #[test]
    fn test_black_start_gen_gas_no_fuel() {
        let g = make_gas_gen(3, 0, 200.0, false);
        assert!(!g.is_black_start_eligible());
    }

    #[test]
    fn test_black_start_gen_gas_with_fuel() {
        let g = make_gas_gen(4, 0, 200.0, true);
        assert!(g.is_black_start_eligible());
    }

    // ---- load priority tests -----------------------------------------------

    #[test]
    fn test_restoration_load_priority() {
        let hospital = make_load(0, 0, RestorePriority::Hospitals, 5.0, true);
        let commercial = make_load(1, 1, RestorePriority::Commercial, 20.0, false);
        assert!(hospital.priority < commercial.priority);
    }

    #[test]
    fn test_priority_ordering() {
        assert!(RestorePriority::CriticalInfrastructure < RestorePriority::Residential);
        assert!(RestorePriority::EmergencyServices < RestorePriority::Industrial);
        assert!(RestorePriority::WaterWastewater < RestorePriority::Commercial);
    }

    // ---- identify BS units -------------------------------------------------

    #[test]
    fn test_identify_black_start_units() {
        let gens = vec![
            make_hydro_gen(0, 0, 100.0),
            make_nuclear_gen(1, 1, 500.0),
            make_battery_gen(2, 2, 20.0),
        ];
        let planner = make_simple_planner(gens, vec![]);
        let bs = planner.identify_black_start_units();
        assert_eq!(bs.len(), 2);
        let ids: Vec<usize> = bs.iter().map(|g| g.id).collect();
        assert!(ids.contains(&0));
        assert!(ids.contains(&2));
    }

    #[test]
    fn test_identify_no_black_start_units() {
        let gens = vec![
            make_nuclear_gen(0, 0, 500.0),
            make_gas_gen(1, 1, 200.0, false), // no fuel
        ];
        let planner = make_simple_planner(gens, vec![]);
        let bs = planner.identify_black_start_units();
        assert!(bs.is_empty());
    }

    // ---- cranking paths ----------------------------------------------------

    #[test]
    fn test_find_cranking_path_direct() {
        let planner = make_simple_planner(vec![], vec![]);
        let path = planner.find_cranking_path(0, 1);
        assert!(path.is_some());
        let cp = path.expect("path must exist");
        assert_eq!(cp.branch_sequence, vec![0]);
    }

    #[test]
    fn test_find_cranking_path_2hop() {
        let planner = make_simple_planner(vec![], vec![]);
        let path = planner.find_cranking_path(0, 2);
        assert!(path.is_some());
        let cp = path.expect("path must exist");
        assert_eq!(cp.branch_sequence.len(), 2);
    }

    #[test]
    fn test_find_cranking_path_no_path() {
        // Bus 2 is isolated (no branch connects to it)
        let planner = BlackStartPlanner::new(
            vec![],
            vec![],
            3,
            1,
            vec![(0, 1)],
            vec![(0.01, 0.1)],
            vec![0.05],
        );
        let path = planner.find_cranking_path(0, 2);
        assert!(path.is_none());
    }

    #[test]
    fn test_find_cranking_path_same_bus() {
        let planner = make_simple_planner(vec![], vec![]);
        assert!(planner.find_cranking_path(0, 0).is_none());
    }

    // ---- cold load pickup --------------------------------------------------

    #[test]
    fn test_cold_load_pickup_immediate() {
        let load = make_load(0, 0, RestorePriority::Residential, 100.0, false);
        // Immediately after outage (0 h) → factor = 1.0
        let demand = BlackStartPlanner::estimate_cold_load_pickup(&load, 0.0);
        assert!((demand - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_cold_load_pickup_after_recovery() {
        let load = make_load(0, 0, RestorePriority::Residential, 100.0, false);
        // After 8 h → full factor = 2.0 → 200 MW
        let demand = BlackStartPlanner::estimate_cold_load_pickup(&load, 8.0);
        assert!((demand - 200.0).abs() < 1e-9);
    }

    #[test]
    fn test_cold_load_pickup_partial() {
        let load = make_load(0, 0, RestorePriority::Residential, 100.0, false);
        // After 4 h → factor = 1 + (2-1)*0.5 = 1.5
        let demand = BlackStartPlanner::estimate_cold_load_pickup(&load, 4.0);
        assert!((demand - 150.0).abs() < 1e-9);
    }

    // ---- can_pickup_load ---------------------------------------------------

    #[test]
    fn test_can_pickup_load_within_headroom() {
        let planner = make_simple_planner(vec![], vec![]);
        let load = make_load(0, 0, RestorePriority::Residential, 10.0, false);
        let island = PowerIsland {
            id: 0,
            energized_buses: vec![0],
            online_generators: vec![],
            connected_loads_mw: 0.0,
            frequency_hz: 50.0,
            voltage_center_pu: 1.0,
            headroom_mw: 100.0,
            is_reference_island: true,
        };
        assert!(planner.can_pickup_load(&load, &island, 6.0));
    }

    #[test]
    fn test_can_pickup_load_exceeds_headroom() {
        let planner = make_simple_planner(vec![], vec![]);
        // CLPU at 8 h: 100 MW * 2.0 = 200 MW; headroom only 50 MW
        let load = make_load(0, 0, RestorePriority::Commercial, 100.0, false);
        let island = PowerIsland {
            id: 0,
            energized_buses: vec![0],
            online_generators: vec![],
            connected_loads_mw: 0.0,
            frequency_hz: 50.0,
            voltage_center_pu: 1.0,
            headroom_mw: 50.0,
            is_reference_island: true,
        };
        // CLPU = 200 MW > inertia_limit = min(headroom * h/6, headroom) = 50 MW
        assert!(!planner.can_pickup_load(&load, &island, 6.0));
    }

    // ---- energisation impact -----------------------------------------------

    #[test]
    fn test_energization_impact_charging() {
        let planner = make_simple_planner(vec![], vec![]);
        let island = PowerIsland {
            id: 0,
            energized_buses: vec![],
            online_generators: vec![],
            connected_loads_mw: 0.0,
            frequency_hz: 50.0,
            voltage_center_pu: 1.0,
            headroom_mw: 100.0,
            is_reference_island: true,
        };
        let (inrush_ka, charging_mvar) = planner.estimate_energization_impact(0, &island);
        // x = 0.1 p.u. → inrush_pu = 10 → inrush_ka = 10 * 0.525 = 5.25 kA
        assert!(inrush_ka > 0.0);
        // b = 0.05, v=1 → charging_mvar = 1*0.05*100 = 5 MVAR
        assert!((charging_mvar - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_energization_impact_out_of_bounds() {
        let planner = make_simple_planner(vec![], vec![]);
        let island = PowerIsland {
            id: 0,
            energized_buses: vec![],
            online_generators: vec![],
            connected_loads_mw: 0.0,
            frequency_hz: 50.0,
            voltage_center_pu: 1.0,
            headroom_mw: 0.0,
            is_reference_island: true,
        };
        let (ik, qc) = planner.estimate_energization_impact(999, &island);
        assert_eq!(ik, 0.0);
        assert_eq!(qc, 0.0);
    }

    // ---- generate_plan -----------------------------------------------------

    #[test]
    fn test_generate_plan_single_generator() {
        let gens = vec![make_hydro_gen(0, 0, 100.0)];
        let loads = vec![make_load(0, 1, RestorePriority::Hospitals, 30.0, true)];
        let planner = make_simple_planner(gens, loads);
        let plan = planner.generate_plan(4.0, 49.0, 51.0).expect("plan failed");
        assert!(!plan.steps.is_empty());
        assert!(plan.estimated_restoration_time_min > 0.0);
    }

    #[test]
    fn test_generate_plan_no_bs_units_error() {
        let gens = vec![make_nuclear_gen(0, 0, 500.0)];
        let planner = make_simple_planner(gens, vec![]);
        let result = planner.generate_plan(2.0, 49.0, 51.0);
        assert!(result.is_err());
    }

    #[test]
    fn test_generate_plan_two_generators() {
        let gens = vec![make_hydro_gen(0, 0, 100.0), make_gas_gen(1, 2, 80.0, true)];
        let loads = vec![
            make_load(0, 1, RestorePriority::CriticalInfrastructure, 5.0, true),
            make_load(1, 2, RestorePriority::Residential, 40.0, false),
        ];
        let planner = make_simple_planner(gens, loads);
        let plan = planner.generate_plan(6.0, 49.0, 51.0).expect("plan failed");
        assert!(plan.total_load_restored_mw > 0.0);
        assert!(plan.n_islands_formed >= 1);
    }

    #[test]
    fn test_plan_steps_sequential() {
        let gens = vec![make_hydro_gen(0, 0, 200.0)];
        let loads = vec![make_load(
            0,
            2,
            RestorePriority::EmergencyServices,
            20.0,
            true,
        )];
        let planner = make_simple_planner(gens, loads);
        let plan = planner.generate_plan(8.0, 49.0, 51.0).expect("plan failed");
        // Step IDs must be monotonically increasing 0, 1, 2, ...
        for (i, step) in plan.steps.iter().enumerate() {
            assert_eq!(step.step_id, i);
        }
    }

    // ---- simulator ---------------------------------------------------------

    #[test]
    fn test_simulate_frequency_nadir() {
        let planner = make_simple_planner(vec![], vec![]);
        let sim = BlackStartSimulator::new(planner, 0.5, 50.0);
        // Load step of 10 MW on 100 MW generation, H=6 s
        let f_nadir = sim.simulate_frequency_nadir(10.0, 100.0, 6.0, 10.0);
        assert!(f_nadir < 50.0);
        assert!(f_nadir > 48.0);
    }

    #[test]
    fn test_simulate_frequency_nadir_zero_generation() {
        let planner = make_simple_planner(vec![], vec![]);
        let sim = BlackStartSimulator::new(planner, 0.5, 50.0);
        // Degenerate guard: return nominal when generation = 0
        let f = sim.simulate_frequency_nadir(10.0, 0.0, 6.0, 10.0);
        assert_eq!(f, 50.0);
    }

    #[test]
    fn test_simulate_basic_restoration() {
        let gens = vec![make_hydro_gen(0, 0, 200.0)];
        let loads = vec![
            make_load(0, 1, RestorePriority::CriticalInfrastructure, 10.0, true),
            make_load(1, 2, RestorePriority::Residential, 30.0, false),
        ];
        let planner = make_simple_planner(gens.clone(), loads.clone());
        let plan = planner.generate_plan(4.0, 49.0, 51.0).expect("plan");
        let mut sim = BlackStartSimulator::new(make_simple_planner(gens, loads), 0.5, 50.0);
        let result = sim.simulate(&plan, 4.0);
        assert!(!result.timeline.is_empty());
        assert!(result.restored_fraction_pct >= 0.0);
        assert!(result.restored_fraction_pct <= 100.0);
    }

    #[test]
    fn test_restoration_fraction_computation() {
        let loads = vec![
            make_load(0, 0, RestorePriority::Residential, 50.0, false),
            make_load(1, 1, RestorePriority::Commercial, 50.0, false),
        ];
        let planner = make_simple_planner(vec![], loads);
        let sim = BlackStartSimulator::new(planner, 0.5, 50.0);

        let timeline = vec![(
            10.0,
            BlackStartEvent {
                event_type: "LOAD_PICKUP".into(),
                bus_or_gen_id: 0,
                description: "Load group 0 picked up 50.0 MW (CLPU 50.0 MW), f_nadir 49.900 Hz"
                    .into(),
                success: true,
            },
        )];
        let frac = sim.compute_restoration_fraction(&timeline);
        assert!((frac - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_saidi_contribution_positive() {
        let gens = vec![make_hydro_gen(0, 0, 100.0)];
        let loads = vec![make_load(0, 1, RestorePriority::Residential, 50.0, false)];
        let planner_for_plan = make_simple_planner(gens.clone(), loads.clone());
        let plan = planner_for_plan
            .generate_plan(4.0, 49.0, 51.0)
            .expect("plan");
        let mut sim = BlackStartSimulator::new(make_simple_planner(gens, loads), 0.5, 50.0);
        let result = sim.simulate(&plan, 4.0);
        assert!(result.saidi_contribution_min > 0.0);
    }

    #[test]
    fn test_plan_cranking_paths_recorded() {
        let gens = vec![
            make_hydro_gen(0, 0, 150.0),
            make_nuclear_gen(1, 2, 600.0), // must be cranked
        ];
        let planner = make_simple_planner(gens, vec![]);
        let plan = planner.generate_plan(2.0, 49.0, 51.0).expect("plan");
        assert!(!plan.cranking_paths.is_empty());
    }
}
