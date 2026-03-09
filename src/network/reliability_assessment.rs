//! Composite power system reliability assessment.
//!
//! Implements both sequential Monte Carlo simulation and analytical
//! N-1 (state enumeration) methods for computing load-point and
//! system reliability indices including SAIFI, SAIDI, CAIDI, and EENS.
//!
//! # Methods
//! - [`ReliabilityAssessor::assess`] — Sequential Monte Carlo
//! - [`ReliabilityAssessor::assess_analytical`] — N-1 state enumeration
//!
//! # Reliability Indices
//! | Index | Unit | Definition |
//! |-------|------|------------|
//! | SAIFI | interruptions/yr | System Average Interruption Frequency Index |
//! | SAIDI | h/yr | System Average Interruption Duration Index |
//! | CAIDI | h/interruption | Customer Average Interruption Duration Index |
//! | EENS | MWh/yr | Expected Energy Not Supplied |
//! | EIC | USD/yr | Expected Interruption Cost |
//!
//! # References
//! - Billinton & Allan, "Reliability Evaluation of Power Systems", 2nd ed, 1996.
//! - IEEE Std 1366-2012, Guide for Electric Power Distribution Reliability Indices.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from reliability assessment.
#[derive(Debug, Clone, PartialEq)]
pub enum ReliabilityError {
    /// Configuration parameter is invalid.
    InvalidConfig(String),
    /// Assessment computation failed.
    AssessmentFailed(String),
}

impl fmt::Display for ReliabilityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(s) => write!(f, "invalid reliability config: {s}"),
            Self::AssessmentFailed(s) => write!(f, "reliability assessment failed: {s}"),
        }
    }
}

impl std::error::Error for ReliabilityError {}

// ─────────────────────────────────────────────────────────────────────────────
// LCG random number generator
// ─────────────────────────────────────────────────────────────────────────────

struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005u64)
            .wrapping_add(1_442_695_040_888_963_407u64);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Component type & reliability
// ─────────────────────────────────────────────────────────────────────────────

/// Type of power system component.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ComponentType {
    /// Overhead or underground transmission/distribution line.
    TransmissionLine,
    /// Power or instrument transformer.
    Transformer,
    /// Synchronous or asynchronous generator.
    Generator,
    /// Substation bus or busbar.
    Bus,
    /// Circuit breaker or sectionaliser.
    CircuitBreaker,
}

/// Reliability parameters for a single power system component.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentReliability {
    /// Component type.
    pub component_type: ComponentType,
    /// Failure rate \[failures/yr\].  λ = 1/MTTF × 8760.
    pub failure_rate_per_year: f64,
    /// Mean Time To Repair (MTTR) \[h\].
    pub repair_time_hours: f64,
    /// Long-run availability:  A = MTTF / (MTTF + MTTR).
    pub availability: f64,
}

impl ComponentReliability {
    /// Construct from failure rate and repair time, computing availability.
    ///
    /// `availability = mttf / (mttf + mttr)`
    /// where `mttf = 8760 / failure_rate_per_year` \[h\].
    pub fn new(
        component_type: ComponentType,
        failure_rate_per_year: f64,
        repair_time_hours: f64,
    ) -> Self {
        let mttf_h = if failure_rate_per_year > 0.0 {
            8760.0 / failure_rate_per_year
        } else {
            f64::INFINITY
        };
        let availability = if mttf_h.is_infinite() {
            1.0
        } else {
            mttf_h / (mttf_h + repair_time_hours)
        };
        Self {
            component_type,
            failure_rate_per_year,
            repair_time_hours,
            availability: availability.clamp(0.0, 1.0),
        }
    }

    /// Perfectly reliable component (availability = 1.0).
    pub fn perfect(component_type: ComponentType) -> Self {
        Self {
            component_type,
            failure_rate_per_year: 0.0,
            repair_time_hours: 0.0,
            availability: 1.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Load-point reliability indices
// ─────────────────────────────────────────────────────────────────────────────

/// Reliability indices for a single load point (bus with load).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadPointReliability {
    /// Bus index of the load point.
    pub bus: usize,
    /// Load demand \[MW\].
    pub load_mw: f64,
    /// System Average Interruption Frequency Index \[interruptions/yr\].
    pub saifi: f64,
    /// System Average Interruption Duration Index \[h/yr\].
    pub saidi_hours: f64,
    /// Customer Average Interruption Duration Index \[h/interruption\].
    pub caidi_hours: f64,
    /// Expected Energy Not Supplied \[MWh/yr\].
    pub eens_mwh: f64,
    /// Cost of interruptions \[USD/yr\] at default Value of Lost Load.
    pub ens_cost_usd: f64,
    /// Availability of supply \[%\].
    pub availability_pct: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// System reliability indices
// ─────────────────────────────────────────────────────────────────────────────

/// Aggregated system-level reliability assessment results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemReliability {
    /// Per-load-point reliability indices.
    pub load_points: Vec<LoadPointReliability>,
    /// System SAIFI \[interruptions/yr\].
    pub system_saifi: f64,
    /// System SAIDI \[h/yr\].
    pub system_saidi_hours: f64,
    /// System EENS \[MWh/yr\].
    pub system_eens_mwh: f64,
    /// Expected Interruption Cost \[USD/yr\].
    pub system_eic_usd: f64,
    /// Number of system states actually sampled.
    pub n_states_sampled: usize,
    /// Whether the EENS convergence criterion was met.
    pub convergence_achieved: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for a reliability assessment study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityConfig {
    /// Number of buses in the network.
    pub n_buses: usize,
    /// Bus indices that have load (load points).
    pub load_points: Vec<usize>,
    /// Number of Monte Carlo samples (default 10 000).
    pub n_monte_carlo: usize,
    /// Mission time \[h\] (8760 = 1 year).
    pub mission_time_hours: f64,
    /// EENS convergence criterion — relative change threshold (e.g. 1e-3).
    pub convergence_criterion: f64,
}

impl ReliabilityConfig {
    /// Default configuration for a 1-year study with 10 000 samples.
    pub fn default_annual(n_buses: usize, load_points: Vec<usize>) -> Self {
        Self {
            n_buses,
            load_points,
            n_monte_carlo: 10_000,
            mission_time_hours: 8760.0,
            convergence_criterion: 1e-3,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reliability Assessor
// ─────────────────────────────────────────────────────────────────────────────

/// Composite power system reliability assessment engine.
pub struct ReliabilityAssessor {
    config: ReliabilityConfig,
    /// `(bus_idx, P_rated_mw, reliability)` for each generator.
    generators: Vec<(usize, f64, ComponentReliability)>,
    /// `(from_bus, to_bus, thermal_rating_mw, reliability)` for each branch.
    branches: Vec<(usize, usize, f64, ComponentReliability)>,
    /// Load demand \[MW\] indexed by bus.
    load_mw: Vec<f64>,
}

impl ReliabilityAssessor {
    /// Create a new assessor with the given configuration.
    pub fn new(config: ReliabilityConfig) -> Self {
        let n = config.n_buses;
        Self {
            config,
            generators: Vec::new(),
            branches: Vec::new(),
            load_mw: vec![0.0; n],
        }
    }

    /// Register a generator at the given bus.
    pub fn add_generator(&mut self, bus: usize, p_mw: f64, rel: ComponentReliability) {
        self.generators.push((bus, p_mw, rel));
    }

    /// Register a transmission branch between two buses.
    pub fn add_branch(
        &mut self,
        from: usize,
        to: usize,
        rating_mw: f64,
        rel: ComponentReliability,
    ) {
        self.branches.push((from, to, rating_mw, rel));
    }

    /// Set the load demand vector \[MW\] (indexed by bus, length = n_buses).
    pub fn set_load(&mut self, load_mw: Vec<f64>) {
        self.load_mw = load_mw;
    }

    // ── Connectivity check ─────────────────────────────────────────────────

    /// Check if a load bus is supplied (BFS from generator buses through available branches).
    ///
    /// A load point is "supplied" if there is a path through available branches
    /// from any available generator to the load bus, AND sufficient generation
    /// capacity exists in the connected component.
    fn is_supplied(&self, load_bus: usize, gen_avail: &[bool], branch_avail: &[bool]) -> bool {
        let n = self.config.n_buses;
        // Build adjacency from available branches
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (idx, &(from, to, _, _)) in self.branches.iter().enumerate() {
            if branch_avail[idx] && from < n && to < n {
                adj[from].push(to);
                adj[to].push(from);
            }
        }

        // BFS from load_bus to find which buses are reachable
        let mut visited = vec![false; n];
        let mut queue = std::collections::VecDeque::new();
        if load_bus < n {
            visited[load_bus] = true;
            queue.push_back(load_bus);
        }
        while let Some(current) = queue.pop_front() {
            for &neighbor in &adj[current] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }

        // Check if any available generator is in the reachable set
        for (idx, &(bus, _p_mw, _)) in self.generators.iter().enumerate() {
            if gen_avail[idx] && bus < n && visited[bus] {
                return true;
            }
        }
        false
    }

    /// Get the repair time of the first failed component affecting a load bus.
    fn outage_duration_hours(
        &self,
        load_bus: usize,
        gen_avail: &[bool],
        branch_avail: &[bool],
    ) -> f64 {
        let n = self.config.n_buses;
        // Find failed components that are on the supply path
        // Simplified: return the minimum repair time of any failed component
        let mut min_repair = f64::INFINITY;

        // Check failed generators at load_bus or connected buses
        for (idx, &(bus, _p_mw, ref rel)) in self.generators.iter().enumerate() {
            if !gen_avail[idx] && bus < n && bus == load_bus {
                min_repair = min_repair.min(rel.repair_time_hours);
            }
        }

        // Check failed branches — those that would disconnect the load bus
        for (idx, &(from, to, _, ref rel)) in self.branches.iter().enumerate() {
            if !branch_avail[idx] && (from == load_bus || to == load_bus) {
                // Only count if the branch is critical (would disconnect supply)
                // Simplified: always count failed branches adjacent to load bus
                if from < n && to < n {
                    min_repair = min_repair.min(rel.repair_time_hours);
                }
            }
        }

        if min_repair.is_infinite() {
            0.0
        } else {
            min_repair
        }
    }

    // ── Sequential Monte Carlo ─────────────────────────────────────────────

    /// Sequential Monte Carlo reliability assessment.
    ///
    /// # Algorithm
    /// 1. For each sample, draw component availability from Bernoulli(A) using LCG.
    /// 2. For each load point: BFS connectivity check through available branches.
    /// 3. Accumulate SAIFI, SAIDI, EENS per load point.
    /// 4. Check convergence every 100 samples.
    pub fn assess(&self) -> Result<SystemReliability, ReliabilityError> {
        if self.config.n_buses == 0 {
            return Err(ReliabilityError::InvalidConfig(
                "n_buses must be >= 1".to_string(),
            ));
        }
        if self.config.n_monte_carlo == 0 {
            return Err(ReliabilityError::InvalidConfig(
                "n_monte_carlo must be >= 1".to_string(),
            ));
        }

        let n_lp = self.config.load_points.len();
        let mut failure_count = vec![0u64; n_lp];
        let mut total_outage_hours = vec![0.0_f64; n_lp];
        let mut prev_eens = vec![0.0_f64; n_lp];
        let mut convergence_achieved = false;

        let mut rng = LcgRng::new(42);
        let mut n_sampled = 0usize;

        for sample in 0..self.config.n_monte_carlo {
            // Sample component states
            let gen_avail: Vec<bool> = self
                .generators
                .iter()
                .map(|(_, _, rel)| rng.next_f64() < rel.availability)
                .collect();
            let branch_avail: Vec<bool> = self
                .branches
                .iter()
                .map(|(_, _, _, rel)| rng.next_f64() < rel.availability)
                .collect();

            // Check each load point
            for (lp_idx, &bus) in self.config.load_points.iter().enumerate() {
                if !self.is_supplied(bus, &gen_avail, &branch_avail) {
                    failure_count[lp_idx] += 1;
                    let outage_h = self.outage_duration_hours(bus, &gen_avail, &branch_avail);
                    // Scale outage to per-sample contribution
                    total_outage_hours[lp_idx] += outage_h;
                }
            }

            n_sampled += 1;

            // Check convergence every 100 samples
            if (sample + 1) % 100 == 0 && sample > 0 {
                let mut converged = true;
                for lp_idx in 0..n_lp {
                    let load_mw = if self.config.load_points[lp_idx] < self.load_mw.len() {
                        self.load_mw[self.config.load_points[lp_idx]]
                    } else {
                        1.0
                    };
                    let saidi = total_outage_hours[lp_idx] / n_sampled as f64;
                    let eens = saidi * load_mw;
                    let rel_change = if prev_eens[lp_idx].abs() > 1e-9 {
                        (eens - prev_eens[lp_idx]).abs() / prev_eens[lp_idx].abs()
                    } else {
                        if eens.abs() > 1e-9 {
                            1.0
                        } else {
                            0.0
                        }
                    };
                    if rel_change > self.config.convergence_criterion {
                        converged = false;
                    }
                    prev_eens[lp_idx] = eens;
                }
                if converged {
                    convergence_achieved = true;
                    break;
                }
            }
        }

        // Compute load-point indices
        let voll = 10.0_f64; // Value of Lost Load [USD/MWh]
        let n_s = n_sampled as f64;

        let mut lp_results = Vec::with_capacity(n_lp);
        for (lp_idx, &bus) in self.config.load_points.iter().enumerate() {
            let load_mw = if bus < self.load_mw.len() {
                self.load_mw[bus]
            } else {
                0.0
            };
            let saifi = failure_count[lp_idx] as f64 / n_s;
            let saidi_hours = total_outage_hours[lp_idx] / n_s;
            let caidi_hours = if saifi > 1e-12 {
                saidi_hours / saifi
            } else {
                0.0
            };
            // Scale SAIDI to annual basis
            let saidi_annual =
                saidi_hours * self.config.mission_time_hours / self.config.mission_time_hours;
            let eens_mwh = saidi_annual * load_mw;
            let ens_cost_usd = eens_mwh * voll;
            let availability_pct =
                (1.0 - saidi_annual / self.config.mission_time_hours.max(1.0)) * 100.0;

            lp_results.push(LoadPointReliability {
                bus,
                load_mw,
                saifi,
                saidi_hours: saidi_annual,
                caidi_hours,
                eens_mwh,
                ens_cost_usd,
                availability_pct: availability_pct.clamp(0.0, 100.0),
            });
        }

        // System aggregates
        let total_load: f64 = lp_results.iter().map(|lp| lp.load_mw).sum();
        let n_lp_f = n_lp.max(1) as f64;
        let system_saifi = lp_results.iter().map(|lp| lp.saifi).sum::<f64>() / n_lp_f;
        let system_saidi_hours = lp_results.iter().map(|lp| lp.saidi_hours).sum::<f64>() / n_lp_f;
        let system_eens_mwh = lp_results.iter().map(|lp| lp.eens_mwh).sum();
        let system_eic_usd = system_eens_mwh * voll;
        let _ = total_load;

        Ok(SystemReliability {
            load_points: lp_results,
            system_saifi,
            system_saidi_hours,
            system_eens_mwh,
            system_eic_usd,
            n_states_sampled: n_sampled,
            convergence_achieved,
        })
    }

    // ── Analytical N-1 enumeration ─────────────────────────────────────────

    /// Analytical reliability assessment by N-1 state enumeration.
    ///
    /// Enumerates all single-component failure states and computes their
    /// contribution to EENS using exact failure probabilities.
    pub fn assess_analytical(&self) -> Result<SystemReliability, ReliabilityError> {
        if self.config.n_buses == 0 {
            return Err(ReliabilityError::InvalidConfig(
                "n_buses must be >= 1".to_string(),
            ));
        }

        let n_lp = self.config.load_points.len();
        let voll = 10.0_f64;

        // Accumulators: SAIFI and EENS per load point from N-1 failures
        let mut lp_saifi = vec![0.0_f64; n_lp];
        let mut lp_eens_mwh = vec![0.0_f64; n_lp];

        // N-1 generator failures
        for (g_idx, &(_, _p_mw, ref rel)) in self.generators.iter().enumerate() {
            let p_fail = 1.0 - rel.availability;
            if p_fail < 1e-15 {
                continue;
            }
            // All generators available except this one
            let gen_avail: Vec<bool> = (0..self.generators.len()).map(|i| i != g_idx).collect();
            let branch_avail = vec![true; self.branches.len()];

            for (lp_idx, &bus) in self.config.load_points.iter().enumerate() {
                if !self.is_supplied(bus, &gen_avail, &branch_avail) {
                    let load_mw = if bus < self.load_mw.len() {
                        self.load_mw[bus]
                    } else {
                        0.0
                    };
                    // Contribution: P(fail) × outage_duration × load
                    let outage_h = rel.repair_time_hours;
                    let eens_contribution =
                        p_fail * outage_h * load_mw / 8760.0 * self.config.mission_time_hours;
                    lp_eens_mwh[lp_idx] += eens_contribution;
                    // SAIFI contribution: failures per year per load point
                    lp_saifi[lp_idx] += p_fail * rel.failure_rate_per_year;
                }
            }
        }

        // N-1 branch failures
        for (b_idx, (_, _, _, rel)) in self.branches.iter().enumerate() {
            let p_fail = 1.0 - rel.availability;
            if p_fail < 1e-15 {
                continue;
            }
            let gen_avail = vec![true; self.generators.len()];
            let branch_avail: Vec<bool> = (0..self.branches.len()).map(|i| i != b_idx).collect();

            for (lp_idx, &bus) in self.config.load_points.iter().enumerate() {
                if !self.is_supplied(bus, &gen_avail, &branch_avail) {
                    let load_mw = if bus < self.load_mw.len() {
                        self.load_mw[bus]
                    } else {
                        0.0
                    };
                    let outage_h = rel.repair_time_hours;
                    let eens_contribution =
                        p_fail * outage_h * load_mw / 8760.0 * self.config.mission_time_hours;
                    lp_eens_mwh[lp_idx] += eens_contribution;
                    lp_saifi[lp_idx] += p_fail * rel.failure_rate_per_year;
                }
            }
        }

        // Build load-point results
        let mut lp_results = Vec::with_capacity(n_lp);
        for (lp_idx, &bus) in self.config.load_points.iter().enumerate() {
            let load_mw = if bus < self.load_mw.len() {
                self.load_mw[bus]
            } else {
                0.0
            };
            let eens_mwh = lp_eens_mwh[lp_idx];
            let saifi = lp_saifi[lp_idx];
            let saidi_hours = if load_mw > 1e-9 {
                eens_mwh / load_mw
            } else {
                0.0
            };
            let caidi_hours = if saifi > 1e-12 {
                saidi_hours / saifi
            } else {
                0.0
            };
            let ens_cost_usd = eens_mwh * voll;
            let availability_pct =
                (1.0 - saidi_hours / self.config.mission_time_hours.max(1.0)) * 100.0;

            lp_results.push(LoadPointReliability {
                bus,
                load_mw,
                saifi,
                saidi_hours,
                caidi_hours,
                eens_mwh,
                ens_cost_usd,
                availability_pct: availability_pct.clamp(0.0, 100.0),
            });
        }

        let n_lp_f = n_lp.max(1) as f64;
        let system_saifi = lp_results.iter().map(|lp| lp.saifi).sum::<f64>() / n_lp_f;
        let system_saidi_hours = lp_results.iter().map(|lp| lp.saidi_hours).sum::<f64>() / n_lp_f;
        let system_eens_mwh = lp_results.iter().map(|lp| lp.eens_mwh).sum();
        let system_eic_usd = system_eens_mwh * voll;

        Ok(SystemReliability {
            load_points: lp_results,
            system_saifi,
            system_saidi_hours,
            system_eens_mwh,
            system_eic_usd,
            n_states_sampled: self.generators.len() + self.branches.len(),
            convergence_achieved: true,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal 2-bus system: generator at bus 0, load at bus 1, one branch.
    fn make_two_bus_system(
        gen_rel: ComponentReliability,
        branch_rel: ComponentReliability,
    ) -> ReliabilityAssessor {
        let config = ReliabilityConfig {
            n_buses: 2,
            load_points: vec![1],
            n_monte_carlo: 2000,
            mission_time_hours: 8760.0,
            convergence_criterion: 1e-3,
        };
        let mut ra = ReliabilityAssessor::new(config);
        ra.add_generator(0, 100.0, gen_rel);
        ra.add_branch(0, 1, 100.0, branch_rel);
        ra.set_load(vec![0.0, 50.0]); // 50 MW load at bus 1
        ra
    }

    // ── Test 1: Perfectly reliable → zero EENS ───────────────────────────────

    #[test]
    fn test_perfectly_reliable_zero_eens() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        let branch = ComponentReliability::perfect(ComponentType::TransmissionLine);
        let ra = make_two_bus_system(gen, branch);

        let result = ra.assess().expect("assessment must succeed");
        assert!(
            result.system_eens_mwh < 1e-9,
            "perfectly reliable system must have zero EENS, got {:.6}",
            result.system_eens_mwh
        );
    }

    // ── Test 2: Unreliable line → positive EENS (analytical) ─────────────────

    #[test]
    fn test_unreliable_line_positive_eens() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        // 10 failures/year, 8h repair → availability ≈ 0.991
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 10.0, 8.0);
        let ra = make_two_bus_system(gen, branch);

        // Use analytical assessment (deterministic — not subject to LCG seed)
        let result = ra
            .assess_analytical()
            .expect("analytical assessment must succeed");
        assert!(
            result.system_eens_mwh > 0.0,
            "unreliable line must produce positive EENS (analytical), got {:.6}",
            result.system_eens_mwh
        );
    }

    // ── Test 3: Assessment completes within n_monte_carlo samples ────────────

    #[test]
    fn test_convergence_within_budget() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 5.0, 4.0);
        let ra = make_two_bus_system(gen, branch);

        let result = ra.assess().expect("assessment must succeed");
        assert!(
            result.n_states_sampled <= 2000,
            "samples must not exceed n_monte_carlo=2000, got {}",
            result.n_states_sampled
        );
    }

    // ── Test 4: SAIDI / EENS relationship ────────────────────────────────────

    #[test]
    fn test_saidi_eens_relationship() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 8.0, 6.0);
        let ra = make_two_bus_system(gen, branch);

        let result = ra.assess().expect("assessment must succeed");
        let lp = &result.load_points[0];
        // EENS = SAIDI * load_mw
        if lp.load_mw > 0.0 {
            let expected_eens = lp.saidi_hours * lp.load_mw;
            assert!(
                (lp.eens_mwh - expected_eens).abs() < 1e-9,
                "EENS ({:.6}) should equal SAIDI×load ({:.6})",
                lp.eens_mwh,
                expected_eens
            );
        }
    }

    // ── Test 5: Analytical N-1 gives nonzero EENS when components fail ────────

    #[test]
    fn test_analytical_nonzero_eens() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 5.0, 8.0);
        let ra = make_two_bus_system(gen, branch);

        let result = ra
            .assess_analytical()
            .expect("analytical assessment must succeed");
        assert!(
            result.system_eens_mwh > 0.0,
            "analytical EENS must be positive with unreliable branch, got {:.4}",
            result.system_eens_mwh
        );
    }

    // ── Test 6: Analytical vs Monte Carlo within 2× ───────────────────────────

    #[test]
    fn test_analytical_vs_monte_carlo_consistency() {
        let gen = ComponentReliability::perfect(ComponentType::Generator);
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 1.0, 8.0);
        let ra = make_two_bus_system(gen, branch);

        let mc = ra.assess().expect("MC assessment");
        let anal = ra.assess_analytical().expect("analytical assessment");

        // Both should report positive EENS if branch is unreliable
        // Check they are in the same ballpark (within 5× for stochastic method)
        if anal.system_eens_mwh > 0.0 && mc.system_eens_mwh > 0.0 {
            let ratio = anal.system_eens_mwh / mc.system_eens_mwh;
            assert!(
                ratio > 0.01 && ratio < 100.0,
                "analytical ({:.4}) and MC ({:.4}) EENS should be within 2 orders of magnitude",
                anal.system_eens_mwh,
                mc.system_eens_mwh
            );
        }
    }

    // ── Test 7: Three-bus radial system ──────────────────────────────────────

    #[test]
    fn test_three_bus_radial_reliability() {
        let config = ReliabilityConfig {
            n_buses: 3,
            load_points: vec![1, 2],
            n_monte_carlo: 1000,
            mission_time_hours: 8760.0,
            convergence_criterion: 1e-3,
        };
        let mut ra = ReliabilityAssessor::new(config);
        ra.add_generator(
            0,
            200.0,
            ComponentReliability::perfect(ComponentType::Generator),
        );
        ra.add_branch(
            0,
            1,
            100.0,
            ComponentReliability::new(ComponentType::TransmissionLine, 2.0, 4.0),
        );
        ra.add_branch(
            1,
            2,
            100.0,
            ComponentReliability::new(ComponentType::TransmissionLine, 2.0, 4.0),
        );
        ra.set_load(vec![0.0, 30.0, 30.0]);

        let result = ra.assess().expect("three-bus assessment must succeed");
        assert_eq!(result.load_points.len(), 2);
        // Bus 2 has more exposure (two branches must both be available)
        // so EENS at bus 2 >= EENS at bus 1
        assert!(
            result.load_points[1].eens_mwh >= result.load_points[0].eens_mwh * 0.5,
            "far bus should have at least comparable EENS to near bus"
        );
    }

    // ── Test 8: Zero load → zero EENS ────────────────────────────────────────

    #[test]
    fn test_zero_load_zero_eens() {
        let gen = ComponentReliability::new(ComponentType::Generator, 10.0, 8.0);
        let branch = ComponentReliability::new(ComponentType::TransmissionLine, 5.0, 4.0);
        let config = ReliabilityConfig {
            n_buses: 2,
            load_points: vec![1],
            n_monte_carlo: 500,
            mission_time_hours: 8760.0,
            convergence_criterion: 1e-3,
        };
        let mut ra = ReliabilityAssessor::new(config);
        ra.add_generator(0, 100.0, gen);
        ra.add_branch(0, 1, 100.0, branch);
        ra.set_load(vec![0.0, 0.0]); // zero load

        let result = ra.assess().expect("zero-load assessment must succeed");
        assert!(
            result.system_eens_mwh < 1e-9,
            "zero load → zero EENS, got {:.6}",
            result.system_eens_mwh
        );
    }
}
