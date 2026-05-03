//! Black start planning for power system restoration.
//!
//! Implements greedy heuristic restoration sequencing from black-start capable
//! generators, with cold load pickup modelling, frequency response simulation,
//! and feasibility checking at every step.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use std::collections::{HashMap, HashSet, VecDeque};

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

/// A generator with black-start capability (can self-start without external power).
#[derive(Debug, Clone)]
pub struct BlackStartUnit {
    /// Generator index (0-based, matching `PowerNetwork::generators`).
    pub gen_id: usize,
    /// Bus external ID where this unit is connected.
    pub bus: usize,
    /// Rated active power output \[MW\].
    pub p_rated_mw: f64,
    /// Minimum stable output \[MW\].
    pub p_min_mw: f64,
    /// Ramp rate \[MW/min\] once running.
    pub ramp_rate_mw_per_min: f64,
    /// Time to reach minimum stable output from cold \[min\].
    pub crank_time_min: f64,
    /// Maximum transmission distance \[km\] over which this unit can crank others.
    pub max_crank_distance_km: f64,
    /// Station-service / auxiliary load \[MW\] consumed by this unit itself.
    pub auxiliary_load_mw: f64,
    /// Scheduling priority (lower number = higher priority).
    pub priority: usize,
}

/// A block of load to be restored together.
#[derive(Debug, Clone)]
pub struct LoadBlock {
    /// Block identifier.
    pub block_id: usize,
    /// External bus IDs included in this block.
    pub buses: Vec<usize>,
    /// Steady-state demand \[MW\].
    pub base_demand_mw: f64,
    /// Cold-load pickup multiplier (initial transient demand / steady-state demand).
    /// Typical value: 2.5.
    pub cold_load_pickup_factor: f64,
    /// Time constant for the cold-load decay back to normal \[min\].
    /// Typical value: 30 min.
    pub cold_load_decay_min: f64,
    /// Scheduling priority (lower number = restore first).
    pub priority: usize,
    /// `false` = must restore as soon as generation allows.
    pub can_defer: bool,
}

/// A transmission path that can energise a bus section or generator terminal.
#[derive(Debug, Clone)]
pub struct EnergizationPath {
    /// Source (energised) bus external ID.
    pub from_bus: usize,
    /// Target bus external ID to be energised.
    pub to_bus: usize,
    /// Ordered list of branch indices traversed.
    pub branch_sequence: Vec<usize>,
    /// Accumulated line length \[km\].
    pub total_length_km: f64,
    /// Reactive power absorbed by line-charging capacitance \[MVAr\].
    /// Positive = leading (over-excited, net generation to system).
    pub charging_current_mvar: f64,
    /// Earliest time this path can be energised \[min from restoration start\].
    pub can_energize_at_t: f64,
}

/// Configuration parameters for the black-start planner.
#[derive(Debug, Clone)]
pub struct BlackStartConfig {
    /// Available black-start capable generating units.
    pub black_start_units: Vec<BlackStartUnit>,
    /// Load blocks to restore, in priority order.
    pub load_blocks: Vec<LoadBlock>,
    /// Hard time limit on the entire restoration \[min\].  Default: 240 (4 h).
    pub max_restoration_time_min: f64,
    /// Permissible frequency deviation from 50 Hz \[Hz\].  Default: 0.5 Hz.
    pub frequency_tolerance_hz: f64,
    /// Permissible voltage deviation from 1.0 p.u.  Default: 0.05 p.u.
    pub voltage_tolerance_pu: f64,
    /// Maximum generator loading during restoration \[%\].  Default: 80.
    pub max_generator_loading_pct: f64,
    /// Reserve margin to maintain at every step \[%\].  Default: 20.
    pub reserve_margin_pct: f64,
}

impl Default for BlackStartConfig {
    fn default() -> Self {
        Self {
            black_start_units: Vec::new(),
            load_blocks: Vec::new(),
            max_restoration_time_min: 240.0,
            frequency_tolerance_hz: 0.5,
            voltage_tolerance_pu: 0.05,
            max_generator_loading_pct: 80.0,
            reserve_margin_pct: 20.0,
        }
    }
}

/// A single action taken during restoration.
#[derive(Debug, Clone)]
pub enum RestorationAction {
    /// Initiate self-start of a black-start unit.
    StartBlackStartUnit { gen_id: usize },
    /// Close breakers to energise a transmission path.
    EnergizePath { path: EnergizationPath },
    /// Provide cranking power from one generator to another.
    CrankGenerator {
        target_gen_id: usize,
        cranked_by: usize,
    },
    /// Connect a load block to the energised system.
    PickupLoadBlock { block_id: usize, actual_mw: f64 },
    /// Synchronise and interconnect two separately restored islands.
    SynchronizeIsland { island_a: usize, island_b: usize },
    /// Increase generator output to the given target.
    RampGenerator { gen_id: usize, target_mw: f64 },
}

/// One discrete step in the restoration sequence.
#[derive(Debug, Clone)]
pub struct RestorationStep {
    /// Sequential step index (1-based for operator readability).
    pub step_id: usize,
    /// Simulation clock at which this step occurs \[min\].
    pub time_min: f64,
    /// The operator action to execute.
    pub action: RestorationAction,
    /// Total dispatchable generation available after this step \[MW\].
    pub available_generation_mw: f64,
    /// Total connected load (cold-load adjusted) after this step \[MW\].
    pub connected_load_mw: f64,
    /// Estimated system frequency after this step \[Hz\].
    pub frequency_hz: f64,
    /// Free-text note for logging / reporting.
    pub notes: String,
}

/// The complete restoration plan produced by the planner.
#[derive(Debug, Clone)]
pub struct RestorationPlan {
    /// Ordered sequence of restoration actions.
    pub steps: Vec<RestorationStep>,
    /// Clock time at which the last action completes \[min\].
    pub total_time_min: f64,
    /// Fraction of total base demand that is eventually restored \[0, 1\].
    pub restored_load_pct: f64,
    /// Number of black-start units actually activated.
    pub n_black_start_units_used: usize,
    /// Time at which all priority-1 (critical) loads are restored \[min\].
    /// `f64::INFINITY` if not achieved within the time limit.
    pub critical_loads_restored_min: f64,
    /// `true` if all load blocks were restored within the time limit.
    pub feasible: bool,
    /// Human-readable descriptions of bottlenecks encountered.
    pub bottlenecks: Vec<String>,
}

// ─────────────────────────────────────────────────────────────────────────────
// BlackStartPlanner
// ─────────────────────────────────────────────────────────────────────────────

/// Plans the optimal (greedy heuristic) black-start restoration sequence.
pub struct BlackStartPlanner {
    /// Configuration: units, load blocks, and operating limits.
    pub config: BlackStartConfig,
}

impl BlackStartPlanner {
    /// Construct a new planner from the given configuration.
    pub fn new(config: BlackStartConfig) -> Self {
        Self { config }
    }

    // ── Public API ──────────────────────────────────────────────────────────

    /// Generate a feasible restoration sequence for `network`.
    ///
    /// Returns `Err` if no black-start units are configured.
    pub fn plan(&self, network: &PowerNetwork) -> Result<RestorationPlan> {
        if self.config.black_start_units.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "No black-start units configured".into(),
            ));
        }

        // Pre-compute all shortest energisation paths (BFS ignoring length constraint
        // at this stage; length filtering happens in cranking step).
        let paths = self.compute_all_energization_paths(network);

        self.greedy_restoration(network, &paths)
    }

    /// Simulate the frequency trajectory after a sudden load step.
    ///
    /// Uses a simplified first-order swing equation plus a proportional governor:
    ///
    /// ```text
    ///   M · df/dt = ΔP_gen - ΔP_load
    ///   P_gov(t)  = saturating_clamp(P_gov(t-dt) + (R·Δf)/T_gov · dt)
    /// ```
    ///
    /// Returns a `Vec<f64>` of frequency samples \[Hz\] at intervals of `dt_s`.
    pub fn simulate_frequency_response(
        &self,
        delta_load_mw: f64,
        total_inertia_mws: f64,
        governor_response_mw_per_hz: f64,
        dt_s: f64,
        duration_s: f64,
    ) -> Vec<f64> {
        let n = if dt_s > 0.0 {
            ((duration_s / dt_s).ceil() as usize).max(1)
        } else {
            1
        };

        let mut freq = vec![0.0f64; n];
        let f0 = 50.0_f64; // nominal frequency [Hz]
        let mut f_now = f0;
        let mut p_gov = 0.0_f64; // governor output [MW]
        let inertia = total_inertia_mws.max(1e-6);
        let t_gov = 10.0_f64; // governor time constant [s]

        for sample in freq.iter_mut() {
            // Net power imbalance: load step minus governor response
            let delta_p = p_gov - delta_load_mw;
            // Swing equation: df/dt = ΔP / (2H * f0) — simplified as ΔP / M
            let df_dt = delta_p / inertia;
            f_now += df_dt * dt_s;

            // Governor: proportional droop
            let delta_f = f_now - f0;
            let dp_gov = (-governor_response_mw_per_hz * delta_f - p_gov) / t_gov * dt_s;
            p_gov += dp_gov;

            *sample = f_now;
        }

        freq
    }

    // ── Private helpers ─────────────────────────────────────────────────────

    /// Greedy restoration heuristic.
    fn greedy_restoration(
        &self,
        network: &PowerNetwork,
        _paths: &[EnergizationPath],
    ) -> Result<RestorationPlan> {
        let mut steps: Vec<RestorationStep> = Vec::new();
        let mut step_id = 1usize;
        let mut clock = 0.0_f64; // minutes elapsed

        // Track state
        let mut energized_buses: HashSet<usize> = HashSet::new();
        let mut online_gen_mw: HashMap<usize, f64> = HashMap::new(); // gen_id → current output
        let mut restored_blocks: HashSet<usize> = HashSet::new();
        let mut connected_load_mw = 0.0_f64;
        let mut bottlenecks: Vec<String> = Vec::new();

        // Sort BS units by priority (lower = first)
        let mut bs_units: Vec<&BlackStartUnit> = self.config.black_start_units.iter().collect();
        bs_units.sort_by_key(|u| u.priority);

        // ── Step 1: Start the highest-priority black-start unit ──────────────
        let first_bs = bs_units[0];
        clock += first_bs.crank_time_min;
        energized_buses.insert(first_bs.bus);
        // Initial output: just above p_min to cover own auxiliaries
        let initial_mw = (first_bs.p_min_mw + first_bs.auxiliary_load_mw)
            .min(first_bs.p_rated_mw * self.config.max_generator_loading_pct / 100.0);
        online_gen_mw.insert(first_bs.gen_id, initial_mw);

        let avail = self.total_available_generation(&online_gen_mw);
        steps.push(RestorationStep {
            step_id,
            time_min: clock,
            action: RestorationAction::StartBlackStartUnit {
                gen_id: first_bs.gen_id,
            },
            available_generation_mw: avail,
            connected_load_mw,
            frequency_hz: 50.0,
            notes: format!(
                "BS unit {} started at bus {}, output {:.1} MW",
                first_bs.gen_id, first_bs.bus, initial_mw
            ),
        });
        step_id += 1;

        // ── Step 2: Iteratively crank generators and pick up loads ───────────
        let max_iter = (network.generators.len() + self.config.load_blocks.len() + 10) * 4;
        let mut iteration = 0usize;

        // Keep track of which BS units have been started
        let mut started_bs: HashSet<usize> = HashSet::from([first_bs.gen_id]);

        // Keep track of non-BS generators that have been cranked
        let mut cranked_gens: HashSet<usize> = HashSet::new();

        loop {
            iteration += 1;
            if iteration > max_iter {
                bottlenecks.push("Iteration limit reached".into());
                break;
            }
            if clock > self.config.max_restoration_time_min {
                bottlenecks.push(format!(
                    "Time limit {:.0} min exceeded at step {}",
                    self.config.max_restoration_time_min, step_id
                ));
                break;
            }

            let avail = self.total_available_generation(&online_gen_mw);
            let reserve_needed = avail * self.config.reserve_margin_pct / 100.0;
            let _headroom = avail - connected_load_mw - reserve_needed;

            // ── 2a: Ramp up running generators that have headroom ────────────
            for bs in &bs_units {
                if !started_bs.contains(&bs.gen_id) {
                    continue;
                }
                let current = *online_gen_mw.get(&bs.gen_id).unwrap_or(&0.0);
                let target_cap = bs.p_rated_mw * self.config.max_generator_loading_pct / 100.0;
                if current < target_cap - 1e-3 {
                    let ramp_mw = (bs.ramp_rate_mw_per_min * 5.0_f64).min(target_cap - current);
                    if ramp_mw > 0.5 {
                        let new_out = current + ramp_mw;
                        online_gen_mw.insert(bs.gen_id, new_out);
                        clock += 5.0;
                        steps.push(RestorationStep {
                            step_id,
                            time_min: clock,
                            action: RestorationAction::RampGenerator {
                                gen_id: bs.gen_id,
                                target_mw: new_out,
                            },
                            available_generation_mw: self
                                .total_available_generation(&online_gen_mw),
                            connected_load_mw,
                            frequency_hz: 50.0,
                            notes: format!("Ramp gen {} to {:.1} MW", bs.gen_id, new_out),
                        });
                        step_id += 1;
                    }
                }
            }

            // ── 2b: Start additional BS units if not yet started ─────────────
            let mut started_new_bs = false;
            for bs in &bs_units {
                if started_bs.contains(&bs.gen_id) {
                    continue;
                }
                // Can we reach this unit's bus via energized buses?
                if let Some(path) = self.find_cranking_path(
                    network,
                    *energized_buses.iter().next().unwrap_or(&0),
                    bs.bus,
                    &energized_buses.iter().copied().collect::<Vec<_>>(),
                ) {
                    // Energize path
                    clock += 10.0; // breaker closing time
                    for b in &path.branch_sequence {
                        let _ = b; // mark used
                    }
                    let to_bus = path.to_bus;
                    steps.push(RestorationStep {
                        step_id,
                        time_min: clock,
                        action: RestorationAction::EnergizePath { path: path.clone() },
                        available_generation_mw: self.total_available_generation(&online_gen_mw),
                        connected_load_mw,
                        frequency_hz: 50.0,
                        notes: format!("Energize path to bus {} for BS unit {}", to_bus, bs.gen_id),
                    });
                    step_id += 1;
                    energized_buses.insert(to_bus);

                    // Start the new BS unit
                    clock += bs.crank_time_min;
                    let out_mw = (bs.p_min_mw + bs.auxiliary_load_mw)
                        .min(bs.p_rated_mw * self.config.max_generator_loading_pct / 100.0);
                    online_gen_mw.insert(bs.gen_id, out_mw);
                    started_bs.insert(bs.gen_id);
                    steps.push(RestorationStep {
                        step_id,
                        time_min: clock,
                        action: RestorationAction::StartBlackStartUnit { gen_id: bs.gen_id },
                        available_generation_mw: self.total_available_generation(&online_gen_mw),
                        connected_load_mw,
                        frequency_hz: 50.0,
                        notes: format!(
                            "BS unit {} started at bus {}, output {:.1} MW",
                            bs.gen_id, bs.bus, out_mw
                        ),
                    });
                    step_id += 1;
                    started_new_bs = true;
                    break;
                }
            }

            // ── 2c: Crank non-black-start generators within reach ─────────────
            let mut cranked_new = false;
            for (gi, gen) in network.generators.iter().enumerate() {
                if cranked_gens.contains(&gi) {
                    continue;
                }
                if !gen.status {
                    continue;
                }
                // Skip if already a BS unit
                if self.config.black_start_units.iter().any(|b| b.gen_id == gi) {
                    continue;
                }

                // Find an energized BS unit that can crank this generator
                let cranking_bs = bs_units.iter().find(|bs| {
                    started_bs.contains(&bs.gen_id)
                        && self
                            .find_cranking_path(
                                network,
                                bs.bus,
                                gen.bus_id,
                                &energized_buses.iter().copied().collect::<Vec<_>>(),
                            )
                            .map(|p| p.total_length_km <= bs.max_crank_distance_km)
                            .unwrap_or(false)
                });

                if let Some(bs) = cranking_bs {
                    if let Some(path) = self.find_cranking_path(
                        network,
                        bs.bus,
                        gen.bus_id,
                        &energized_buses.iter().copied().collect::<Vec<_>>(),
                    ) {
                        let to_bus = path.to_bus;
                        clock += 15.0; // energize + crank time for non-BS gen
                        energized_buses.insert(to_bus);

                        steps.push(RestorationStep {
                            step_id,
                            time_min: clock,
                            action: RestorationAction::EnergizePath { path: path.clone() },
                            available_generation_mw: self
                                .total_available_generation(&online_gen_mw),
                            connected_load_mw,
                            frequency_hz: 50.0,
                            notes: format!(
                                "Energize cranking path to gen {} at bus {}",
                                gi, gen.bus_id
                            ),
                        });
                        step_id += 1;

                        steps.push(RestorationStep {
                            step_id,
                            time_min: clock,
                            action: RestorationAction::CrankGenerator {
                                target_gen_id: gi,
                                cranked_by: bs.gen_id,
                            },
                            available_generation_mw: self
                                .total_available_generation(&online_gen_mw),
                            connected_load_mw,
                            frequency_hz: 50.0,
                            notes: format!("Crank gen {} from BS unit {}", gi, bs.gen_id),
                        });
                        step_id += 1;

                        let gen_out = (gen.pmin + gen.pmax * 0.1)
                            .min(gen.pmax * self.config.max_generator_loading_pct / 100.0);
                        online_gen_mw.insert(gi, gen_out.max(0.0));
                        cranked_gens.insert(gi);
                        cranked_new = true;
                        break;
                    }
                }
            }

            // ── 2d: Pick up load blocks ──────────────────────────────────────
            let avail_now = self.total_available_generation(&online_gen_mw);
            let reserve_mw = avail_now * self.config.reserve_margin_pct / 100.0;

            let mut sorted_blocks: Vec<&LoadBlock> = self
                .config
                .load_blocks
                .iter()
                .filter(|b| !restored_blocks.contains(&b.block_id))
                .collect();
            sorted_blocks.sort_by_key(|a| a.priority);

            let mut picked_up = false;
            for block in sorted_blocks {
                let clp_demand = self.cold_load_pickup(block, clock, clock);
                if self.check_feasibility(avail_now, connected_load_mw, clp_demand)
                    && avail_now - connected_load_mw - clp_demand >= reserve_mw
                {
                    let actual_mw = clp_demand;
                    connected_load_mw += actual_mw;
                    restored_blocks.insert(block.block_id);

                    // Determine which buses in this block we just energized
                    for &bus_id in &block.buses {
                        energized_buses.insert(bus_id);
                    }

                    let f_traj =
                        self.simulate_frequency_response(actual_mw, 500.0, 100.0, 0.1, 10.0);
                    let f_nadir = f_traj.iter().cloned().fold(f64::INFINITY, f64::min);

                    steps.push(RestorationStep {
                        step_id,
                        time_min: clock,
                        action: RestorationAction::PickupLoadBlock {
                            block_id: block.block_id,
                            actual_mw,
                        },
                        available_generation_mw: avail_now,
                        connected_load_mw,
                        frequency_hz: f_nadir.max(48.0), // floor at 48 Hz
                        notes: format!(
                            "Load block {} picked up: {:.1} MW (CLP factor {:.2}), f_nadir={:.2} Hz",
                            block.block_id,
                            actual_mw,
                            block.cold_load_pickup_factor,
                            f_nadir
                        ),
                    });
                    step_id += 1;
                    picked_up = true;
                    clock += 5.0; // switching time between block pickups
                } else if !block.can_defer {
                    bottlenecks.push(format!(
                        "Insufficient generation for mandatory block {} at {:.0} min (need {:.1} MW, headroom {:.1} MW)",
                        block.block_id,
                        clock,
                        clp_demand,
                        avail_now - connected_load_mw - reserve_mw
                    ));
                }
            }

            // ── 2e: Check termination ────────────────────────────────────────
            let all_blocks_restored = restored_blocks.len() == self.config.load_blocks.len();
            if all_blocks_restored {
                break;
            }

            // If nothing progressed this iteration, advance clock
            if !started_new_bs && !cranked_new && !picked_up {
                // Try to avoid spinning: either we're done or stuck
                let unrestored: Vec<&LoadBlock> = self
                    .config
                    .load_blocks
                    .iter()
                    .filter(|b| !restored_blocks.contains(&b.block_id))
                    .collect();
                if unrestored.is_empty() {
                    break;
                }
                // Advance time to allow ramp-up or defer
                clock += 10.0;
                if clock > self.config.max_restoration_time_min {
                    bottlenecks.push("Stuck: no progress possible".into());
                    break;
                }
            }
        }

        // ── Build summary metrics ────────────────────────────────────────────
        let total_base_mw: f64 = self
            .config
            .load_blocks
            .iter()
            .map(|b| b.base_demand_mw)
            .sum();
        let restored_base_mw: f64 = self
            .config
            .load_blocks
            .iter()
            .filter(|b| restored_blocks.contains(&b.block_id))
            .map(|b| b.base_demand_mw)
            .sum();
        let restored_pct = if total_base_mw > 1e-9 {
            restored_base_mw / total_base_mw
        } else {
            1.0
        };

        let critical_blocks: Vec<&LoadBlock> = self
            .config
            .load_blocks
            .iter()
            .filter(|b| b.priority == 1)
            .collect();
        let critical_restored_min = if critical_blocks.is_empty()
            || critical_blocks
                .iter()
                .all(|b| restored_blocks.contains(&b.block_id))
        {
            // Find the step time when the last critical block was restored
            steps
                .iter()
                .filter_map(|s| {
                    if let RestorationAction::PickupLoadBlock { block_id, .. } = &s.action {
                        if critical_blocks.iter().any(|b| b.block_id == *block_id) {
                            Some(s.time_min)
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                })
                .fold(0.0_f64, f64::max)
        } else {
            f64::INFINITY
        };

        let feasible = restored_blocks.len() == self.config.load_blocks.len()
            && clock <= self.config.max_restoration_time_min;

        Ok(RestorationPlan {
            steps,
            total_time_min: clock,
            restored_load_pct: restored_pct,
            n_black_start_units_used: started_bs.len(),
            critical_loads_restored_min: critical_restored_min,
            feasible,
            bottlenecks,
        })
    }

    /// BFS-based shortest cranking path from `from_bus` to `to_bus`.
    ///
    /// Considers only branches whose `from_bus` or `to_bus` is currently
    /// energised (i.e., in `energized_buses`).  The first hop is always
    /// allowed from `from_bus`.
    pub fn find_cranking_path(
        &self,
        network: &PowerNetwork,
        from_bus: usize,
        to_bus: usize,
        energized_buses: &[usize],
    ) -> Option<EnergizationPath> {
        if from_bus == to_bus {
            return Some(EnergizationPath {
                from_bus,
                to_bus,
                branch_sequence: Vec::new(),
                total_length_km: 0.0,
                charging_current_mvar: 0.0,
                can_energize_at_t: 0.0,
            });
        }

        let energized_set: HashSet<usize> = energized_buses.iter().copied().collect();

        // BFS: (bus_id, accumulated_length_km, branch_path)
        let mut visited: HashSet<usize> = HashSet::new();
        let mut queue: VecDeque<(usize, f64, Vec<usize>)> = VecDeque::new();
        visited.insert(from_bus);
        queue.push_back((from_bus, 0.0, Vec::new()));

        while let Some((current, dist, path)) = queue.pop_front() {
            for (branch_idx, branch) in network.branches.iter().enumerate() {
                if !branch.status {
                    continue;
                }
                let neighbor = if branch.from_bus == current {
                    branch.to_bus
                } else if branch.to_bus == current {
                    branch.from_bus
                } else {
                    continue;
                };

                if visited.contains(&neighbor) {
                    continue;
                }

                // Approximate line length from impedance magnitude (0.3 Ω/km typical)
                let z_pu = (branch.r * branch.r + branch.x * branch.x).sqrt();
                let base_ohm = 1.0; // per-unit approximation
                let length_km = (z_pu * base_ohm / 0.3).max(0.5);

                let new_dist = dist + length_km;
                let mut new_path = path.clone();
                new_path.push(branch_idx);
                visited.insert(neighbor);

                if neighbor == to_bus {
                    // Estimate charging MVAr: b/2 per end summed
                    let charging: f64 = new_path
                        .iter()
                        .map(|&bi| network.branches[bi].b * 0.5)
                        .sum::<f64>()
                        * 100.0; // rough MVAr

                    return Some(EnergizationPath {
                        from_bus,
                        to_bus,
                        branch_sequence: new_path,
                        total_length_km: new_dist,
                        charging_current_mvar: charging,
                        can_energize_at_t: 0.0,
                    });
                }

                // Only expand through energized buses (or from_bus itself)
                if energized_set.contains(&neighbor) || neighbor == from_bus {
                    queue.push_back((neighbor, new_dist, new_path));
                } else {
                    // Allow traversal through un-energised buses to reach target
                    queue.push_back((neighbor, new_dist, new_path));
                }
            }
        }
        None
    }

    /// Compute the cold-load pickup demand for `block` at current time `t_now_min`,
    /// given that the block was restored at `t_restore_min`.
    ///
    /// ```text
    /// P(t) = base × (1 + (factor − 1) × exp(−(t_now − t_restore) / decay))
    /// ```
    pub fn cold_load_pickup(&self, block: &LoadBlock, t_restore_min: f64, t_now_min: f64) -> f64 {
        let elapsed = (t_now_min - t_restore_min).max(0.0);
        let transient_factor = 1.0
            + (block.cold_load_pickup_factor - 1.0)
                * (-elapsed / block.cold_load_decay_min.max(1e-6)).exp();
        block.base_demand_mw * transient_factor
    }

    /// Return `true` if adding `step_load_mw` to `current_load_mw` still
    /// leaves `available_gen_mw` covering the combined demand.
    pub fn check_feasibility(
        &self,
        available_gen_mw: f64,
        current_load_mw: f64,
        step_load_mw: f64,
    ) -> bool {
        available_gen_mw >= current_load_mw + step_load_mw
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    /// Sum of outputs for all generators currently online.
    fn total_available_generation(&self, online: &HashMap<usize, f64>) -> f64 {
        online.values().sum()
    }

    /// Pre-compute BFS-based shortest paths between every pair of buses.
    ///
    /// Returns one `EnergizationPath` per unique source→target pair that is
    /// reachable in the network graph.  Used to seed the greedy planner.
    fn compute_all_energization_paths(&self, network: &PowerNetwork) -> Vec<EnergizationPath> {
        let mut paths = Vec::new();
        // Collect bus IDs for BS units and generators
        let key_buses: HashSet<usize> = self
            .config
            .black_start_units
            .iter()
            .map(|b| b.bus)
            .chain(network.generators.iter().map(|g| g.bus_id))
            .collect();

        let all_buses: Vec<usize> = network.buses.iter().map(|b| b.id).collect();

        for &src in &key_buses {
            for &dst in &all_buses {
                if src == dst {
                    continue;
                }
                if let Some(p) = self.find_cranking_path(network, src, dst, &[src]) {
                    paths.push(p);
                }
            }
        }
        paths
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};

    fn simple_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.buses.push(Bus::new(3, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.10,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.01,
            x: 0.10,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 80.0,
            qg: 0.0,
            qmax: 50.0,
            qmin: -50.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 100.0,
            pmin: 10.0,
        });
        net
    }

    fn make_bs_unit(gen_id: usize, bus: usize) -> BlackStartUnit {
        BlackStartUnit {
            gen_id,
            bus,
            p_rated_mw: 100.0,
            p_min_mw: 10.0,
            ramp_rate_mw_per_min: 5.0,
            crank_time_min: 15.0,
            max_crank_distance_km: 100.0,
            auxiliary_load_mw: 2.0,
            priority: 1,
        }
    }

    fn make_load_block(block_id: usize, bus: usize, demand_mw: f64) -> LoadBlock {
        LoadBlock {
            block_id,
            buses: vec![bus],
            base_demand_mw: demand_mw,
            cold_load_pickup_factor: 2.5,
            cold_load_decay_min: 30.0,
            priority: 1,
            can_defer: false,
        }
    }

    #[test]
    fn test_plan_no_bs_units_returns_err() {
        let net = simple_network();
        let planner = BlackStartPlanner::new(BlackStartConfig::default());
        assert!(planner.plan(&net).is_err());
    }

    #[test]
    fn test_plan_with_bs_unit_returns_ok() {
        let net = simple_network();
        let cfg = BlackStartConfig {
            black_start_units: vec![make_bs_unit(0, 1)],
            load_blocks: vec![make_load_block(0, 2, 20.0)],
            ..BlackStartConfig::default()
        };
        let planner = BlackStartPlanner::new(cfg);
        let plan = planner.plan(&net).expect("plan should succeed");
        assert!(!plan.steps.is_empty());
        assert!(plan.n_black_start_units_used >= 1);
    }

    #[test]
    fn test_simulate_frequency_response_drops_after_load_step() {
        let planner = BlackStartPlanner::new(BlackStartConfig::default());
        // 10 MW load step on a 5 MWs inertia system, no governor
        let freq = planner.simulate_frequency_response(10.0, 5.0, 0.0, 0.1, 2.0);
        assert!(!freq.is_empty());
        // Frequency should drop below nominal (50 Hz) immediately
        assert!(freq[0] < 50.0, "freq[0] = {:.4}", freq[0]);
    }

    #[test]
    fn test_find_cranking_path_same_bus_returns_empty_sequence() {
        let net = simple_network();
        let planner = BlackStartPlanner::new(BlackStartConfig::default());
        let path = planner.find_cranking_path(&net, 1, 1, &[1]);
        let ep = path.expect("same-bus must return Some");
        assert!(ep.branch_sequence.is_empty());
        assert!((ep.total_length_km).abs() < 1e-9);
    }

    #[test]
    fn test_cold_load_pickup_at_restore_time_is_peak() {
        let planner = BlackStartPlanner::new(BlackStartConfig::default());
        let block = make_load_block(0, 1, 100.0);
        // At t = t_restore the exponential term = 1 → value = base * factor = 250
        let demand = planner.cold_load_pickup(&block, 10.0, 10.0);
        let expected = 100.0 * 2.5;
        assert!((demand - expected).abs() < 1e-6, "demand = {:.4}", demand);
    }

    #[test]
    fn test_check_feasibility_boundary() {
        let planner = BlackStartPlanner::new(BlackStartConfig::default());
        // Exactly at capacity boundary → feasible
        assert!(planner.check_feasibility(100.0, 60.0, 40.0));
        // One MW over → infeasible
        assert!(!planner.check_feasibility(100.0, 60.0, 41.0));
        // Zero step load is always feasible when gen >= current
        assert!(planner.check_feasibility(50.0, 50.0, 0.0));
    }
}
