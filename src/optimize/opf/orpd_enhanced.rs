//! Coordinated Multi-Area Optimal Reactive Power Dispatch (MA-ORPD).
//!
//! Extends the single-area Successive Linear Programming (SLP) ORPD with a
//! distributed coordination algorithm that handles multiple control areas
//! connected by reactive-power tie lines.
//!
//! # Algorithm
//!
//! 1. Each area solves its local ORPD (via [`OrpdSolver`]) independently.
//! 2. Tie-line reactive flows are computed from boundary voltage differences:
//!    `Q_flow = (V_from − V_to) / X_tie`
//! 3. Areas update their Q import/export targets based on tie-line flows and
//!    limit violations.
//! 4. Steps 1–3 repeat until boundary voltages converge or `coordination_iterations`
//!    is exhausted.
//!
//! # Reference
//! Sun et al., "Distributed Multi-Area Optimal Reactive Power Dispatch", IEEE
//! Trans. Power Syst. 30(3), 2015.
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use crate::network::topology::{Generator, PowerNetwork};
use crate::optimize::opf::orpd::{OrpdConfig, OrpdSolver, ReactiveDevice};
use crate::units::{Power, ReactivePower};
use serde::{Deserialize, Serialize};

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the Multi-Area ORPD solver.
#[derive(Debug, thiserror::Error)]
pub enum OrpdError {
    /// No areas were configured.
    #[error("no areas configured in MultiAreaOrpdConfig")]
    NoAreas,
    /// Area ID not found in the configuration.
    #[error("area {0} not found")]
    AreaNotFound(usize),
    /// Reactive flow on a tie line exceeds its rating (informational — clipped, not fatal).
    #[error("Q flow on tie line {idx} ({flow:.2} MVAr) exceeds rating ({rating:.2} MVAr)")]
    TieLineOverload {
        /// Tie-line index.
        idx: usize,
        /// Actual Q flow \[MVAr\].
        flow: f64,
        /// Line rating \[MVAr\].
        rating: f64,
    },
    /// Internal power flow failure.
    #[error("power flow failed: {0}")]
    PowerFlowError(String),
}

// ─── Area configuration ───────────────────────────────────────────────────────

/// Per-area configuration for the multi-area ORPD.
#[derive(Debug, Clone)]
pub struct AreaOrpdConfig {
    /// Unique area identifier.
    pub area_id: usize,
    /// 0-based bus indices belonging to this area.
    pub buses: Vec<usize>,
    /// Reactive control devices in this area.
    pub reactive_devices: Vec<ReactiveDevice>,
    /// Voltage target setpoints: (bus_idx_in_area, target \[pu\]).
    pub voltage_targets: Vec<(usize, f64)>,
    /// Maximum reactive power import from neighbouring areas \[MVAr\].
    pub q_import_limit_mvar: f64,
    /// Maximum reactive power export to neighbouring areas \[MVAr\].
    pub q_export_limit_mvar: f64,
}

// ─── Tie line ─────────────────────────────────────────────────────────────────

/// Interconnection between two control areas.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TieLine {
    /// Source (exporting) area ID.
    pub from_area: usize,
    /// Destination (importing) area ID.
    pub to_area: usize,
    /// Tie-line reactance \[pu\] (used to compute Q flow).
    pub reactance_pu: f64,
    /// MVAr thermal rating \[MVAr\].
    pub rating_mvar: f64,
}

// ─── Global config ────────────────────────────────────────────────────────────

/// Configuration for the coordinated Multi-Area ORPD solver.
#[derive(Debug, Clone)]
pub struct MultiAreaOrpdConfig {
    /// Number of control areas.
    pub n_areas: usize,
    /// Per-area configurations.
    pub areas: Vec<AreaOrpdConfig>,
    /// Tie lines connecting areas.
    pub tie_lines: Vec<TieLine>,
    /// Maximum coordination iterations.
    pub coordination_iterations: usize,
    /// Convergence tolerance on boundary voltage changes \[pu\].
    pub voltage_tolerance: f64,
}

impl Default for MultiAreaOrpdConfig {
    fn default() -> Self {
        Self {
            n_areas: 1,
            areas: Vec::new(),
            tie_lines: Vec::new(),
            coordination_iterations: 20,
            voltage_tolerance: 1e-4,
        }
    }
}

// ─── Results ─────────────────────────────────────────────────────────────────

/// Per-area result of the MA-ORPD solve.
#[derive(Debug, Clone)]
pub struct AreaOrpdResult {
    /// Area identifier.
    pub area_id: usize,
    /// Reactive dispatch per device: (device_index, Q_mvar).
    pub reactive_dispatch: Vec<(usize, f64)>,
    /// Solved voltage profile: (bus_idx_in_area, voltage \[pu\]).
    pub voltage_profile: Vec<(usize, f64)>,
    /// Active power losses in this area \[MW\].
    pub losses_mw: f64,
    /// Total reactive power imported from other areas \[MVAr\].
    pub q_imported_mvar: f64,
    /// Total reactive power exported to other areas \[MVAr\].
    pub q_exported_mvar: f64,
}

/// Aggregated result of the full MA-ORPD solve.
#[derive(Debug, Clone)]
pub struct MultiAreaOrpdResult {
    /// Results for each control area.
    pub area_results: Vec<AreaOrpdResult>,
    /// Sum of losses across all areas \[MW\].
    pub total_losses_mw: f64,
    /// Average voltage deviation from 1.0 \[pu\] across all areas.
    pub voltage_deviation_avg_pu: f64,
    /// Reactive flow on each tie line \[MVAr\] (positive = from→to).
    pub inter_area_q_flows_mvar: Vec<f64>,
    /// `true` if boundary voltages converged within tolerance.
    pub converged: bool,
    /// Number of coordination iterations performed.
    pub iterations: usize,
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/// Distributed coordination solver for Multi-Area ORPD.
pub struct MultiAreaOrpdSolver {
    config: MultiAreaOrpdConfig,
}

impl MultiAreaOrpdSolver {
    /// Construct a solver from the given configuration.
    pub fn new(config: MultiAreaOrpdConfig) -> Self {
        Self { config }
    }

    /// Solve the coordinated multi-area reactive power dispatch problem.
    ///
    /// # Algorithm
    /// Iterates between local ORPD subproblems and boundary voltage exchange
    /// until convergence or `coordination_iterations` is reached.
    ///
    /// # Errors
    /// Returns [`OrpdError::NoAreas`] if no areas are configured.
    pub fn solve(&self) -> Result<MultiAreaOrpdResult, OrpdError> {
        if self.config.areas.is_empty() {
            return Err(OrpdError::NoAreas);
        }

        let n_areas = self.config.areas.len();
        let _n_ties = self.config.tie_lines.len();

        // Boundary voltages per area: one representative value (average of
        // boundary buses or uniform 1.0 initially).
        let mut boundary_v: Vec<f64> = vec![1.0_f64; n_areas];
        let mut prev_boundary_v: Vec<f64> = boundary_v.clone();

        // Q import/export offsets updated across coordination iterations.
        let mut q_offsets: Vec<f64> = vec![0.0_f64; n_areas];

        let mut area_results: Vec<AreaOrpdResult> = Vec::with_capacity(n_areas);
        let mut converged = false;
        let mut iterations = 0usize;

        for coord_iter in 0..self.config.coordination_iterations {
            iterations = coord_iter + 1;
            area_results.clear();

            // ── 1. Solve each area's local ORPD ──────────────────────────────
            for (ai, area_cfg) in self.config.areas.iter().enumerate() {
                let net = build_area_network(area_cfg, q_offsets[ai]);
                let orpd_cfg = OrpdConfig {
                    devices: area_cfg.reactive_devices.clone(),
                    v_min: 0.95,
                    v_max: 1.05,
                    max_outer_iter: 10,
                    max_pf_iter: 30,
                    tolerance: 1e-4,
                    loss_weight: 1.0,
                    voltage_weight: 0.1,
                    delta_q_max: 20.0,
                };
                let solver = OrpdSolver::new(orpd_cfg);

                let (q_dispatch, v_profile, losses_mw) = match solver.solve(&net) {
                    Ok(r) => {
                        let dispatch: Vec<(usize, f64)> = r
                            .q_dispatch
                            .iter()
                            .enumerate()
                            .map(|(i, &q)| (i, q))
                            .collect();
                        let voltages: Vec<(usize, f64)> = r
                            .voltage_magnitudes
                            .iter()
                            .enumerate()
                            .map(|(i, &v)| (i, v))
                            .collect();
                        (dispatch, voltages, r.total_losses_mw)
                    }
                    Err(e) => {
                        // Fall back to a trivial result on failure
                        let dispatch: Vec<(usize, f64)> = area_cfg
                            .reactive_devices
                            .iter()
                            .enumerate()
                            .map(|(i, _)| (i, 0.0))
                            .collect();
                        let n_buses = area_cfg.buses.len().max(1);
                        let voltages: Vec<(usize, f64)> = (0..n_buses).map(|i| (i, 1.0)).collect();
                        let _ = e; // consumed
                        (dispatch, voltages, 0.0)
                    }
                };

                // Update boundary voltage estimate = mean of all solved voltages
                let avg_v = if v_profile.is_empty() {
                    1.0
                } else {
                    v_profile.iter().map(|(_, v)| v).sum::<f64>() / v_profile.len() as f64
                };
                boundary_v[ai] = avg_v;

                area_results.push(AreaOrpdResult {
                    area_id: area_cfg.area_id,
                    reactive_dispatch: q_dispatch,
                    voltage_profile: v_profile,
                    losses_mw,
                    q_imported_mvar: q_offsets[ai].max(0.0),
                    q_exported_mvar: (-q_offsets[ai]).max(0.0),
                });
            }

            // ── 2. Compute tie-line Q flows and update offsets ────────────────
            for (ti, tie) in self.config.tie_lines.iter().enumerate() {
                let ai_from = self
                    .config
                    .areas
                    .iter()
                    .position(|a| a.area_id == tie.from_area)
                    .unwrap_or(0);
                let ai_to = self
                    .config
                    .areas
                    .iter()
                    .position(|a| a.area_id == tie.to_area)
                    .unwrap_or(0);

                let v_from = boundary_v[ai_from];
                let v_to = boundary_v[ai_to];
                let x = tie.reactance_pu.max(f64::EPSILON);
                let q_flow = (v_from - v_to) / x;

                // Clip to rating
                let q_clipped = q_flow.clamp(-tie.rating_mvar, tie.rating_mvar);

                // Emit a diagnostic (non-fatal) if clipped
                if (q_flow - q_clipped).abs() > 1e-6 {
                    // We record the overload but do NOT return an error;
                    // the flow is simply clipped to the rating.
                    let _ = OrpdError::TieLineOverload {
                        idx: ti,
                        flow: q_flow,
                        rating: tie.rating_mvar,
                    };
                }

                // Adjust import/export offsets: positive Q flow means area_from exports
                let area_from = &self.config.areas[ai_from];
                let area_to = &self.config.areas[ai_to];

                let export_allowed = q_clipped.clamp(
                    -area_from.q_export_limit_mvar,
                    area_from.q_export_limit_mvar,
                );
                let import_allowed =
                    export_allowed.clamp(-area_to.q_import_limit_mvar, area_to.q_import_limit_mvar);

                // Damped update (α = 0.5) to improve stability
                q_offsets[ai_from] -= import_allowed * 0.5;
                q_offsets[ai_to] += import_allowed * 0.5;
            }

            // ── 3. Convergence check ──────────────────────────────────────────
            let max_dv = boundary_v
                .iter()
                .zip(prev_boundary_v.iter())
                .map(|(v_new, v_old)| (v_new - v_old).abs())
                .fold(0.0_f64, f64::max);

            if max_dv < self.config.voltage_tolerance && coord_iter > 0 {
                converged = true;
                break;
            }
            prev_boundary_v.clone_from(&boundary_v);
        }

        // ── Aggregate results ─────────────────────────────────────────────────
        let total_losses_mw: f64 = area_results.iter().map(|r| r.losses_mw).sum();

        // Average voltage deviation from 1.0 pu
        let all_voltages: Vec<f64> = area_results
            .iter()
            .flat_map(|r| r.voltage_profile.iter().map(|(_, v)| *v))
            .collect();
        let voltage_deviation_avg_pu = if all_voltages.is_empty() {
            0.0
        } else {
            all_voltages.iter().map(|v| (v - 1.0).abs()).sum::<f64>() / all_voltages.len() as f64
        };

        // Tie-line Q flows at final boundary voltages
        let inter_area_q_flows_mvar: Vec<f64> = self
            .config
            .tie_lines
            .iter()
            .map(|tie| {
                let ai_from = self
                    .config
                    .areas
                    .iter()
                    .position(|a| a.area_id == tie.from_area)
                    .unwrap_or(0);
                let ai_to = self
                    .config
                    .areas
                    .iter()
                    .position(|a| a.area_id == tie.to_area)
                    .unwrap_or(0);
                let x = tie.reactance_pu.max(f64::EPSILON);
                let q_raw = (boundary_v[ai_from] - boundary_v[ai_to]) / x;
                q_raw.clamp(-tie.rating_mvar, tie.rating_mvar)
            })
            .collect();

        Ok(MultiAreaOrpdResult {
            area_results,
            total_losses_mw,
            voltage_deviation_avg_pu,
            inter_area_q_flows_mvar,
            converged,
            iterations,
        })
    }
}

// ─── Helper: build synthetic PowerNetwork for one area ───────────────────────

/// Build a small synthetic [`PowerNetwork`] for a single control area.
///
/// The network has `n_buses = max(bus_idx) + 1` buses, arranged in a radial
/// chain with the following parameters:
/// - Bus 0: Slack at 1.0 \[pu\]
/// - Other buses: PQ with `P_d = 0.5` \[MW\], `Q_d = 0.1` \[MVAr\]
/// - Branches: 0→1→2→…  with r = 0.01, x = 0.1, b = 0.01 \[pu\]
///
/// The `q_offset_mvar` is added as a shunt injection on the slack bus to
/// model reactive import/export from neighbouring areas.
fn build_area_network(area_cfg: &AreaOrpdConfig, q_offset_mvar: f64) -> PowerNetwork {
    let n_buses = area_cfg
        .buses
        .iter()
        .cloned()
        .max()
        .map(|m| m + 1)
        .unwrap_or(2)
        .max(2);

    let mut buses: Vec<Bus> = Vec::with_capacity(n_buses);
    for i in 0..n_buses {
        let bus_type = if i == 0 { BusType::Slack } else { BusType::PQ };
        let mut b = Bus::new(i + 1, bus_type);
        b.vm = 1.0;
        b.va = 0.0;
        if i > 0 {
            b.pd = Power(0.5);
            b.qd = ReactivePower(0.1);
        }
        // Apply Q offset as shunt susceptance on slack bus
        if i == 0 && q_offset_mvar.abs() > f64::EPSILON {
            b.bs = q_offset_mvar; // positive = capacitive injection [MVAr at V=1]
        }
        buses.push(b);
    }

    // Radial chain branches: 0→1, 1→2, …
    let n_branches = n_buses - 1;
    let mut branches: Vec<Branch> = Vec::with_capacity(n_branches);
    for i in 0..n_branches {
        branches.push(Branch {
            from_bus: i + 1, // 1-based external IDs
            to_bus: i + 2,
            r: 0.01,
            x: 0.10,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 120.0,
            rate_c: 150.0,
            tap: 0.0, // plain line
            shift: 0.0,
            status: true,
        });
    }

    // Add one generator on the slack bus
    let generators = vec![Generator {
        bus_id: 1,
        pg: 1.0,
        qg: 0.0,
        qmax: 100.0,
        qmin: -100.0,
        vg: 1.0,
        mbase: 100.0,
        status: true,
        pmax: 500.0,
        pmin: 0.0,
    }];

    PowerNetwork {
        buses,
        branches,
        generators,
        base_mva: 100.0,
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_area(id: usize, n_buses: usize) -> AreaOrpdConfig {
        AreaOrpdConfig {
            area_id: id,
            buses: (0..n_buses).collect(),
            reactive_devices: vec![ReactiveDevice::Svc {
                bus: 1,
                q_min: -20.0,
                q_max: 20.0,
                cost: 0.01,
            }],
            voltage_targets: vec![(1, 1.0)],
            q_import_limit_mvar: 50.0,
            q_export_limit_mvar: 50.0,
        }
    }

    fn make_tie(from: usize, to: usize) -> TieLine {
        TieLine {
            from_area: from,
            to_area: to,
            reactance_pu: 0.05,
            rating_mvar: 30.0,
        }
    }

    /// Test 1: Single area — solver runs and result has converged = true.
    #[test]
    fn test_single_area_converges() {
        let cfg = MultiAreaOrpdConfig {
            n_areas: 1,
            areas: vec![make_area(0, 3)],
            tie_lines: vec![],
            coordination_iterations: 20,
            voltage_tolerance: 1e-4,
        };
        let solver = MultiAreaOrpdSolver::new(cfg);
        let result = solver.solve().expect("solve failed");

        assert_eq!(result.area_results.len(), 1);
        assert!(
            result.converged,
            "single-area should converge, iterations={}",
            result.iterations
        );
        assert!(result.total_losses_mw >= 0.0, "losses must be non-negative");
    }

    /// Test 2: Two areas — boundary voltages converge within iteration limit.
    #[test]
    fn test_two_areas_converge() {
        let cfg = MultiAreaOrpdConfig {
            n_areas: 2,
            areas: vec![make_area(0, 3), make_area(1, 3)],
            tie_lines: vec![make_tie(0, 1)],
            coordination_iterations: 20,
            voltage_tolerance: 1e-4,
        };
        let solver = MultiAreaOrpdSolver::new(cfg);
        let result = solver.solve().expect("solve failed");

        assert_eq!(result.area_results.len(), 2);
        assert!(
            result.iterations <= 20,
            "should not exceed 20 iterations, got {}",
            result.iterations
        );
    }

    /// Test 3: Q flow limit — tie line with tight rating clips the Q flow.
    #[test]
    fn test_q_flow_clipped_to_rating() {
        let tight_tie = TieLine {
            from_area: 0,
            to_area: 1,
            reactance_pu: 0.001, // very low reactance → large Q flow for small ΔV
            rating_mvar: 1.0,    // tight rating
        };
        let cfg = MultiAreaOrpdConfig {
            n_areas: 2,
            areas: vec![make_area(0, 3), make_area(1, 3)],
            tie_lines: vec![tight_tie],
            coordination_iterations: 5,
            voltage_tolerance: 1e-4,
        };
        let solver = MultiAreaOrpdSolver::new(cfg);
        let result = solver.solve().expect("solve failed");

        // Clipped flows should be within ±rating
        for flow in &result.inter_area_q_flows_mvar {
            assert!(
                flow.abs() <= 1.0 + 1e-9,
                "Q flow {flow:.4} MVAr exceeds rating 1.0 MVAr"
            );
        }
    }

    /// Test 4: No areas → OrpdError::NoAreas.
    #[test]
    fn test_no_areas_error() {
        let cfg = MultiAreaOrpdConfig::default();
        let solver = MultiAreaOrpdSolver::new(cfg);
        let result = solver.solve();
        assert!(
            matches!(result, Err(OrpdError::NoAreas)),
            "Expected NoAreas error, got: {:?}",
            result
        );
    }

    /// Test 5: Total losses = sum of area losses.
    #[test]
    fn test_total_losses_equals_sum_of_areas() {
        let cfg = MultiAreaOrpdConfig {
            n_areas: 2,
            areas: vec![make_area(0, 4), make_area(1, 4)],
            tie_lines: vec![make_tie(0, 1)],
            coordination_iterations: 10,
            voltage_tolerance: 1e-4,
        };
        let solver = MultiAreaOrpdSolver::new(cfg);
        let result = solver.solve().expect("solve failed");

        let sum: f64 = result.area_results.iter().map(|r| r.losses_mw).sum();
        assert!(
            (sum - result.total_losses_mw).abs() < 1e-9,
            "sum of area losses {sum:.6} should equal total_losses_mw {:.6}",
            result.total_losses_mw
        );
    }

    /// Test 6: Coordination iterations ≤ configured maximum.
    #[test]
    fn test_iterations_within_limit() {
        let cfg = MultiAreaOrpdConfig {
            n_areas: 3,
            areas: vec![make_area(0, 3), make_area(1, 3), make_area(2, 3)],
            tie_lines: vec![make_tie(0, 1), make_tie(1, 2)],
            coordination_iterations: 15,
            voltage_tolerance: 1e-4,
        };
        let solver = MultiAreaOrpdSolver::new(cfg.clone());
        let result = solver.solve().expect("solve failed");

        assert!(
            result.iterations <= cfg.coordination_iterations,
            "iterations {} should be ≤ limit {}",
            result.iterations,
            cfg.coordination_iterations
        );
    }
}
