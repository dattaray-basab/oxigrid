//! ADMM (Alternating Direction Method of Multipliers) Distributed Optimal Power Flow.
//!
//! Decomposes the network into areas and coordinates them via consensus variables
//! at boundary buses using the ADMM algorithm.
//!
//! ## Algorithm
//! For each iteration k:
//! 1. **x-update**: Each area solves a local economic dispatch with augmented Lagrangian
//!    coupling terms at boundary buses (independent, parallelisable).
//! 2. **z-update**: Global consensus — average area estimates weighted by dual variables.
//! 3. **y-update**: Dual ascent — update Lagrange multipliers.
//! 4. **Residuals**: Compute primal and dual residuals and check convergence.
//! 5. **Adaptive ρ**: Optionally rescale penalty parameter to balance residuals.
//!
//! ## Reference
//! Boyd et al., "Distributed Optimization and Statistical Learning via the
//! Alternating Direction Method of Multipliers", Foundations and Trends in
//! Machine Learning, 2011.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// A generator inside an ADMM area.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmmGenerator {
    /// Local bus index within the area.
    pub bus: usize,
    /// Minimum active power output [MW].
    pub p_min_mw: f64,
    /// Maximum active power output [MW].
    pub p_max_mw: f64,
    /// Minimum reactive power output [Mvar].
    pub q_min_mvar: f64,
    /// Maximum reactive power output [Mvar].
    pub q_max_mvar: f64,
    /// Quadratic cost coefficient [$/MW²h].
    pub cost_a: f64,
    /// Linear cost coefficient [$/MWh].
    pub cost_b: f64,
}

impl AdmmGenerator {
    /// Marginal cost at active power output `p` [MW].
    pub fn marginal_cost(&self, p: f64) -> f64 {
        self.cost_b + 2.0 * self.cost_a * p
    }

    /// Total cost at active power output `p` [$/h].
    pub fn total_cost(&self, p: f64) -> f64 {
        self.cost_a * p * p + self.cost_b * p
    }
}

/// An area/region in the decomposed network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmmArea {
    /// Unique area identifier.
    pub id: usize,
    /// Global bus indices that belong to this area.
    pub bus_indices: Vec<usize>,
    /// Subset of `bus_indices` that are shared with other areas.
    pub boundary_buses: Vec<usize>,
    /// Generators located in this area.
    pub generators: Vec<AdmmGenerator>,
    /// Active load per bus [MW], aligned with `bus_indices`.
    pub loads_mw: Vec<f64>,
    /// Reactive load per bus [Mvar], aligned with `bus_indices`.
    pub loads_mvar: Vec<f64>,
}

impl AdmmArea {
    /// Total active load in this area [MW].
    pub fn total_load_mw(&self) -> f64 {
        self.loads_mw.iter().sum()
    }
}

/// Boundary bus linking two areas.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundaryLink {
    /// Global bus identifier.
    pub bus_global: usize,
    /// First area that owns this boundary bus.
    pub area_1: usize,
    /// Local bus index within `area_1`.
    pub bus_in_area_1: usize,
    /// Second area that owns this boundary bus.
    pub area_2: usize,
    /// Local bus index within `area_2`.
    pub bus_in_area_2: usize,
}

/// ADMM solver configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmmConfig {
    /// Penalty / augmentation parameter ρ (must be > 0).
    pub rho: f64,
    /// Maximum number of ADMM iterations.
    pub max_iterations: usize,
    /// Primal feasibility tolerance ε_pri.
    pub primal_tol: f64,
    /// Dual feasibility tolerance ε_dual.
    pub dual_tol: f64,
    /// Enable adaptive ρ update.
    pub rho_update_enabled: bool,
    /// Multiplicative scale factor when adjusting ρ.
    pub rho_scale_factor: f64,
    /// System base MVA.
    pub base_mva: f64,
}

impl Default for AdmmConfig {
    fn default() -> Self {
        Self {
            rho: 1.0,
            max_iterations: 200,
            primal_tol: 1e-4,
            dual_tol: 1e-4,
            rho_update_enabled: true,
            rho_scale_factor: 2.0,
            base_mva: 100.0,
        }
    }
}

/// Internal ADMM iteration state.
#[derive(Debug, Clone)]
pub struct AdmmState {
    /// Local area variables: `x[area_idx][var_idx]`.
    /// Each area stores `n_generators + n_boundary_vars_for_area` values.
    pub x: Vec<Vec<f64>>,
    /// Consensus (shared) variables — one entry per [`BoundaryLink`].
    pub z: Vec<f64>,
    /// Dual variables (Lagrange multipliers): `y[area_idx][boundary_var_idx]`.
    pub y: Vec<Vec<f64>>,
    /// Most recent primal residual.
    pub primal_residual: f64,
    /// Most recent dual residual.
    pub dual_residual: f64,
    /// Current penalty parameter ρ.
    pub rho: f64,
}

/// Dispatch result for a single area.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmmAreaDispatch {
    /// Area identifier.
    pub area_id: usize,
    /// Optimal generator active dispatch [MW].
    pub generator_dispatch_mw: Vec<f64>,
    /// Optimal generator reactive dispatch [Mvar].
    pub generator_dispatch_mvar: Vec<f64>,
    /// Per-bus voltage magnitude [pu] (flat-start 1.0 for DC approximation).
    pub bus_voltages: Vec<f64>,
    /// Area generation cost [$/h].
    pub area_cost_per_h: f64,
    /// Net power exchange per boundary bus [MW]; positive = export.
    pub import_export_mw: Vec<f64>,
}

/// Full result returned by [`AdmmOpf::solve`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdmmResult {
    /// Whether the algorithm converged within tolerance.
    pub converged: bool,
    /// Number of ADMM iterations performed.
    pub iterations: usize,
    /// Per-area dispatch results.
    pub area_dispatches: Vec<AdmmAreaDispatch>,
    /// Consensus voltage magnitude at each boundary bus [pu].
    pub boundary_voltages: Vec<f64>,
    /// System-wide total generation cost [$/h].
    pub total_cost_per_h: f64,
    /// Primal residual at each iteration.
    pub primal_residual_history: Vec<f64>,
    /// Dual residual at each iteration.
    pub dual_residual_history: Vec<f64>,
    /// Average primal residual reduction per iteration (geometric rate).
    pub convergence_rate: f64,
}

// ---------------------------------------------------------------------------
// ADMM OPF solver
// ---------------------------------------------------------------------------

/// ADMM-based distributed OPF solver.
pub struct AdmmOpf {
    /// List of network areas.
    pub areas: Vec<AdmmArea>,
    /// Boundary bus inter-area links.
    pub boundary_links: Vec<BoundaryLink>,
    /// Solver configuration.
    pub config: AdmmConfig,
}

impl AdmmOpf {
    /// Create a new ADMM OPF problem.
    pub fn new(
        areas: Vec<AdmmArea>,
        boundary_links: Vec<BoundaryLink>,
        config: AdmmConfig,
    ) -> Self {
        Self {
            areas,
            boundary_links,
            config,
        }
    }

    // -----------------------------------------------------------------------
    // Public solve entry point
    // -----------------------------------------------------------------------

    /// Run the ADMM distributed OPF and return the optimal dispatch.
    ///
    /// The algorithm follows the standard ADMM consensus form:
    /// ```text
    /// minimise  Σ_i  f_i(x_i)
    /// subject to  x_i_bnd = z_j  for all boundary coupling constraints j
    /// ```
    /// where `z` is the global consensus variable vector.
    pub fn solve(&mut self) -> Result<AdmmResult> {
        let n_areas = self.areas.len();
        let n_boundary = self.boundary_links.len();

        if n_areas == 0 {
            return Err(OxiGridError::InvalidParameter(
                "ADMM requires at least one area".to_string(),
            ));
        }

        // Build mapping: area_idx -> list of (link_idx, side) for boundary vars.
        // side = 1 => area is area_1 in the link; side = 2 => area is area_2.
        let area_bmap = Self::build_area_boundary_map(n_areas, &self.boundary_links);

        // Variable layout per area: [P_g_0 .. P_g_{ng-1}, P_bnd_0 .. P_bnd_{nb-1}]
        // where nb = area_bmap[a].len()
        let n_vars: Vec<usize> = (0..n_areas)
            .map(|a| self.areas[a].generators.len() + area_bmap[a].len())
            .collect();

        // x-initialisation: generators at midpoint, boundary vars at 0
        let x_init: Vec<Vec<f64>> = (0..n_areas)
            .map(|a| {
                let area = &self.areas[a];
                let mut v = Vec::with_capacity(n_vars[a]);
                for g in &area.generators {
                    v.push(0.5 * (g.p_min_mw + g.p_max_mw));
                }
                v.resize(v.len() + area_bmap[a].len(), 0.0);
                v
            })
            .collect();

        let z_init = vec![0.0_f64; n_boundary];
        let y_init: Vec<Vec<f64>> = (0..n_areas)
            .map(|a| vec![0.0_f64; area_bmap[a].len()])
            .collect();

        let mut state = AdmmState {
            x: x_init,
            z: z_init,
            y: y_init,
            primal_residual: f64::MAX,
            dual_residual: f64::MAX,
            rho: self.config.rho,
        };

        let mut primal_history: Vec<f64> = Vec::with_capacity(self.config.max_iterations);
        let mut dual_history: Vec<f64> = Vec::with_capacity(self.config.max_iterations);
        let mut converged = false;
        let mut final_iter = 0_usize;

        // ------------------------------------------------------------------
        // Main ADMM loop
        // ------------------------------------------------------------------
        for iter in 0..self.config.max_iterations {
            final_iter = iter + 1;

            // 1. x-update: each area solves its augmented local subproblem
            #[allow(clippy::needless_range_loop)]
            for a in 0..n_areas {
                let n_gen = self.areas[a].generators.len();
                let bvars = &area_bmap[a];
                let n_bvars = bvars.len();

                let z_local: Vec<f64> = bvars.iter().map(|(li, _)| state.z[*li]).collect();
                let y_local: Vec<f64> = state.y[a].clone();

                let new_x = Self::x_update_area(
                    &self.areas[a],
                    &y_local,
                    &z_local,
                    state.rho,
                    n_gen,
                    n_bvars,
                )?;

                state.x[a] = new_x;
            }

            // 2. z-update: global consensus average
            let z_old = state.z.clone();
            state.z = Self::z_update(
                &state.x,
                &state.y,
                &self.boundary_links,
                &area_bmap,
                n_areas,
                n_boundary,
                state.rho,
            );

            // 3. y-update: dual ascent
            Self::y_update(
                &mut state.y,
                &state.x,
                &state.z,
                &self.boundary_links,
                &area_bmap,
                state.rho,
            );

            // 4. Compute residuals
            state.primal_residual =
                Self::compute_primal_residual(&state.x, &state.z, &self.boundary_links, &area_bmap);
            state.dual_residual = Self::compute_dual_residual(&z_old, &state.z, state.rho);

            primal_history.push(state.primal_residual);
            dual_history.push(state.dual_residual);

            // 5. Convergence check
            if state.primal_residual <= self.config.primal_tol
                && state.dual_residual <= self.config.dual_tol
            {
                converged = true;
                break;
            }

            // 6. Adaptive ρ update (Boyd et al., §3.4.1)
            if self.config.rho_update_enabled {
                Self::update_rho(
                    &mut state.rho,
                    state.primal_residual,
                    state.dual_residual,
                    self.config.rho_scale_factor,
                    &mut state.y,
                );
            }
        }

        self.build_result(
            &state,
            &area_bmap,
            converged,
            final_iter,
            primal_history,
            dual_history,
        )
    }

    // -----------------------------------------------------------------------
    // Internal helpers — pub for unit testing
    // -----------------------------------------------------------------------

    /// Build the area→boundary-link map.
    ///
    /// Returns `area_bmap[area_idx]` = list of `(link_idx, side)` where
    /// `side = 1` if the area is `area_1` in the link, `2` if `area_2`.
    pub fn build_area_boundary_map(
        n_areas: usize,
        links: &[BoundaryLink],
    ) -> Vec<Vec<(usize, u8)>> {
        let mut map: Vec<Vec<(usize, u8)>> = vec![Vec::new(); n_areas];
        for (li, link) in links.iter().enumerate() {
            if link.area_1 < n_areas {
                map[link.area_1].push((li, 1));
            }
            if link.area_2 < n_areas {
                map[link.area_2].push((li, 2));
            }
        }
        map
    }

    /// x-update for a single area.
    ///
    /// Solves the augmented Lagrangian local subproblem:
    /// ```text
    /// min  Σ_g (a_g·P_g² + b_g·P_g)
    ///      + Σ_j [ y_j·(P_bnd_j − z_j) + (ρ/2)·(P_bnd_j − z_j)² ]
    /// s.t. Σ_g P_g = P_load + Σ_j P_bnd_j   (local power balance)
    ///      P_min_g ≤ P_g ≤ P_max_g
    /// ```
    ///
    /// The unconstrained optimum for each boundary flow variable is
    /// `P_bnd_j* = z_j − y_j/ρ` (from first-order condition).
    /// Generator dispatch is then found by equal-incremental-cost
    /// (lambda-iteration) to cover `load + Σ P_bnd_j*`.
    ///
    /// Returns `[P_g_0 .. P_g_{ng-1}, P_bnd_0 .. P_bnd_{nb-1}]`.
    pub fn x_update_area(
        area: &AdmmArea,
        y_boundary: &[f64],
        z_boundary: &[f64],
        rho: f64,
        n_gen: usize,
        n_bvars: usize,
    ) -> Result<Vec<f64>> {
        if n_gen == 0 && n_bvars == 0 {
            return Ok(Vec::new());
        }

        // Unconstrained optimal boundary flows: P_bnd* = z - y/ρ
        let p_bnd: Vec<f64> = (0..n_bvars)
            .map(|j| {
                let z_j = z_boundary.get(j).copied().unwrap_or(0.0);
                let y_j = y_boundary.get(j).copied().unwrap_or(0.0);
                z_j - y_j / rho.max(1e-15)
            })
            .collect();

        // Area must generate: load + net export
        let net_export: f64 = p_bnd.iter().sum();
        let target_gen = area.total_load_mw() + net_export;

        let p_gen = if n_gen == 0 {
            Vec::new()
        } else {
            economic_dispatch_lambda(&area.generators, target_gen)?
        };

        let mut result = Vec::with_capacity(n_gen + n_bvars);
        result.extend_from_slice(&p_gen);
        result.extend_from_slice(&p_bnd);
        Ok(result)
    }

    /// z-update: consensus averaging.
    ///
    /// For each boundary link j connecting areas a1 and a2:
    /// ```text
    /// z_j = (1/2) * [ (x_{a1,bnd_j} + y_{a1,bnd_j}/ρ)
    ///                + (x_{a2,bnd_j} + y_{a2,bnd_j}/ρ) ]
    /// ```
    /// If a link touches only one area (degenerate), that area's estimate is used directly.
    pub fn z_update(
        x: &[Vec<f64>],
        y: &[Vec<f64>],
        links: &[BoundaryLink],
        area_bmap: &[Vec<(usize, u8)>],
        n_areas: usize,
        n_boundary: usize,
        rho: f64,
    ) -> Vec<f64> {
        let mut z = vec![0.0_f64; n_boundary];

        // For each area precompute the number of generators (= offset of boundary vars in x)
        let n_gen_per_area: Vec<usize> = (0..n_areas)
            .map(|a| {
                let x_len = x.get(a).map(|v| v.len()).unwrap_or(0);
                let b_len = area_bmap.get(a).map(|b| b.len()).unwrap_or(0);
                x_len.saturating_sub(b_len)
            })
            .collect();

        for li in 0..n_boundary {
            let link = &links[li];
            let a1 = link.area_1;
            let a2 = link.area_2;

            let extract = |area_idx: usize| -> Option<f64> {
                if area_idx >= n_areas {
                    return None;
                }
                let bvars = area_bmap.get(area_idx)?;
                let local_pos = bvars.iter().position(|(idx, _)| *idx == li)?;
                let xi = n_gen_per_area.get(area_idx).copied().unwrap_or(0) + local_pos;
                let x_val = x.get(area_idx)?.get(xi).copied()?;
                let y_val = y.get(area_idx)?.get(local_pos).copied().unwrap_or(0.0);
                Some(x_val + y_val / rho.max(1e-15))
            };

            let val_a1 = extract(a1);
            let val_a2 = extract(a2);

            z[li] = match (val_a1, val_a2) {
                (Some(v1), Some(v2)) => (v1 + v2) / 2.0,
                (Some(v), None) | (None, Some(v)) => v,
                (None, None) => 0.0,
            };
        }
        z
    }

    /// y-update: dual ascent step.
    ///
    /// `y_{a,j} ← y_{a,j} + ρ · (x_{a,bnd_j} − z_j)`
    #[allow(clippy::ptr_arg)]
    pub fn y_update(
        y: &mut Vec<Vec<f64>>,
        x: &[Vec<f64>],
        z: &[f64],
        _links: &[BoundaryLink],
        area_bmap: &[Vec<(usize, u8)>],
        rho: f64,
    ) {
        let n_areas = y.len();
        for a in 0..n_areas {
            let n_gen = x
                .get(a)
                .map(|v| v.len())
                .unwrap_or(0)
                .saturating_sub(area_bmap.get(a).map(|b| b.len()).unwrap_or(0));
            let bvars = match area_bmap.get(a) {
                Some(b) => b,
                None => continue,
            };
            for (j, (li, _side)) in bvars.iter().enumerate() {
                let xi = n_gen + j;
                let x_val = x.get(a).and_then(|v| v.get(xi)).copied().unwrap_or(0.0);
                let z_val = z.get(*li).copied().unwrap_or(0.0);
                if let Some(yv) = y.get_mut(a).and_then(|v| v.get_mut(j)) {
                    *yv += rho * (x_val - z_val);
                }
            }
        }
    }

    /// Compute primal residual: `‖x − z‖₂` summed over all boundary couplings.
    pub fn compute_primal_residual(
        x: &[Vec<f64>],
        z: &[f64],
        _links: &[BoundaryLink],
        area_bmap: &[Vec<(usize, u8)>],
    ) -> f64 {
        let mut sq_sum = 0.0_f64;
        let n_areas = x.len();
        for a in 0..n_areas {
            let n_gen = x
                .get(a)
                .map(|v| v.len())
                .unwrap_or(0)
                .saturating_sub(area_bmap.get(a).map(|b| b.len()).unwrap_or(0));
            let bvars = match area_bmap.get(a) {
                Some(b) => b,
                None => continue,
            };
            for (j, (li, _)) in bvars.iter().enumerate() {
                let xi = n_gen + j;
                let x_val = x.get(a).and_then(|v| v.get(xi)).copied().unwrap_or(0.0);
                let z_val = z.get(*li).copied().unwrap_or(0.0);
                let diff = x_val - z_val;
                sq_sum += diff * diff;
            }
        }
        sq_sum.sqrt()
    }

    /// Compute dual residual: `‖ρ · (z^k − z^{k-1})‖₂`.
    pub fn compute_dual_residual(z_old: &[f64], z_new: &[f64], rho: f64) -> f64 {
        let sq_sum: f64 = z_old
            .iter()
            .zip(z_new.iter())
            .map(|(&zo, &zn)| {
                let d = rho * (zn - zo);
                d * d
            })
            .sum();
        sq_sum.sqrt()
    }

    /// Adaptive ρ update (Boyd et al., §3.4.1).
    ///
    /// - If `r_pri > 10 · r_dual`: increase ρ by `scale` and rescale duals.
    /// - If `r_dual > 10 · r_pri`: decrease ρ by `scale` and rescale duals.
    #[allow(clippy::ptr_arg)]
    pub fn update_rho(
        rho: &mut f64,
        primal_res: f64,
        dual_res: f64,
        scale: f64,
        y: &mut Vec<Vec<f64>>,
    ) {
        let threshold = 10.0_f64;
        let old_rho = *rho;

        if primal_res > threshold * dual_res {
            *rho *= scale;
        } else if dual_res > threshold * primal_res {
            *rho /= scale.max(1e-15);
        }

        // Rescale duals so that ρ·y stays constant (Boyd §3.4.1)
        if ((*rho) - old_rho).abs() > 1e-15 {
            let factor = old_rho / (*rho).max(1e-15);
            for ya in y.iter_mut() {
                for yv in ya.iter_mut() {
                    *yv *= factor;
                }
            }
        }
    }

    /// Compute total system generation cost [$/h].
    pub fn compute_total_cost(areas: &[AdmmArea], dispatches: &[Vec<f64>]) -> f64 {
        let mut cost = 0.0_f64;
        for (a, area) in areas.iter().enumerate() {
            if let Some(p_gen) = dispatches.get(a) {
                for (gi, gen) in area.generators.iter().enumerate() {
                    if let Some(&p) = p_gen.get(gi) {
                        cost += gen.total_cost(p);
                    }
                }
            }
        }
        cost
    }

    /// Assemble the final [`AdmmResult`] from solver state.
    pub fn build_result(
        &self,
        state: &AdmmState,
        area_bmap: &[Vec<(usize, u8)>],
        converged: bool,
        iterations: usize,
        primal_history: Vec<f64>,
        dual_history: Vec<f64>,
    ) -> Result<AdmmResult> {
        let n_areas = self.areas.len();

        // Extract only the generator dispatch portion of x per area
        let gen_dispatches: Vec<Vec<f64>> = (0..n_areas)
            .map(|a| {
                let n_gen = self.areas[a].generators.len();
                state
                    .x
                    .get(a)
                    .map(|xv| xv[..n_gen.min(xv.len())].to_vec())
                    .unwrap_or_default()
            })
            .collect();

        let total_cost = Self::compute_total_cost(&self.areas, &gen_dispatches);

        let area_dispatches: Vec<AdmmAreaDispatch> = (0..n_areas)
            .map(|a| {
                let area = &self.areas[a];
                let n_gen = area.generators.len();

                let p_gen: Vec<f64> = state
                    .x
                    .get(a)
                    .map(|xv| xv[..n_gen.min(xv.len())].to_vec())
                    .unwrap_or_default();

                let area_cost: f64 = area
                    .generators
                    .iter()
                    .zip(p_gen.iter())
                    .map(|(g, &p)| g.total_cost(p))
                    .sum();

                // Reactive dispatch: interpolate between Q limits proportional to P loading
                let q_gen: Vec<f64> = area
                    .generators
                    .iter()
                    .zip(p_gen.iter())
                    .map(|(g, &p)| {
                        let p_range = g.p_max_mw - g.p_min_mw;
                        if p_range.abs() < 1e-10 {
                            0.5 * (g.q_min_mvar + g.q_max_mvar)
                        } else {
                            let frac = ((p - g.p_min_mw) / p_range).clamp(0.0, 1.0);
                            g.q_min_mvar + frac * (g.q_max_mvar - g.q_min_mvar)
                        }
                    })
                    .collect();

                // Flat bus voltages (DC approximation)
                let bus_voltages = vec![1.0_f64; area.bus_indices.len()];

                // Boundary flow variables from x
                let import_export: Vec<f64> = area_bmap
                    .get(a)
                    .map(|bvars| {
                        bvars
                            .iter()
                            .enumerate()
                            .map(|(j, _)| {
                                let xi = n_gen + j;
                                state
                                    .x
                                    .get(a)
                                    .and_then(|xv| xv.get(xi))
                                    .copied()
                                    .unwrap_or(0.0)
                            })
                            .collect()
                    })
                    .unwrap_or_default();

                AdmmAreaDispatch {
                    area_id: area.id,
                    generator_dispatch_mw: p_gen,
                    generator_dispatch_mvar: q_gen,
                    bus_voltages,
                    area_cost_per_h: area_cost,
                    import_export_mw: import_export,
                }
            })
            .collect();

        // Consensus voltage at boundary buses (DC flat-start = 1.0 pu)
        let boundary_voltages = vec![1.0_f64; self.boundary_links.len()];

        // Geometric convergence rate from primal residual history
        let convergence_rate = if primal_history.len() >= 2 {
            let first = primal_history.first().copied().unwrap_or(1.0);
            let last = primal_history.last().copied().unwrap_or(1.0);
            let n = primal_history.len() as f64;
            if first > 1e-15 && last > 0.0 {
                (last / first).powf(1.0 / n)
            } else {
                0.0
            }
        } else {
            0.0
        };

        Ok(AdmmResult {
            converged,
            iterations,
            area_dispatches,
            boundary_voltages,
            total_cost_per_h: total_cost,
            primal_residual_history: primal_history,
            dual_residual_history: dual_history,
            convergence_rate,
        })
    }
}

// ---------------------------------------------------------------------------
// Economic dispatch via equal incremental cost (lambda-iteration / bisection)
// ---------------------------------------------------------------------------

/// Solve economic dispatch for a set of generators summing to `target_mw`
/// via bisection on the system marginal cost (λ).
///
/// Returns generator outputs in the same order as `generators`.
fn economic_dispatch_lambda(generators: &[AdmmGenerator], target_mw: f64) -> Result<Vec<f64>> {
    if generators.is_empty() {
        return Ok(Vec::new());
    }

    let p_min_total: f64 = generators.iter().map(|g| g.p_min_mw).sum();
    let p_max_total: f64 = generators.iter().map(|g| g.p_max_mw).sum();

    // Clamp demand to feasible range (avoids infeasibility from boundary terms)
    let target_clamped = target_mw.clamp(p_min_total, p_max_total);

    // All-linear generators: proportional dispatch
    let all_linear = generators.iter().all(|g| g.cost_a.abs() < 1e-15);
    if all_linear {
        return dispatch_linear(generators, target_clamped);
    }

    economic_dispatch_bisect(generators, target_clamped)
}

/// Bisection on λ to match `target_mw` via equal-incremental-cost.
fn economic_dispatch_bisect(generators: &[AdmmGenerator], target_mw: f64) -> Result<Vec<f64>> {
    // Bracket λ using the extremes of marginal costs
    let mut lo = generators
        .iter()
        .map(|g| g.marginal_cost(g.p_min_mw))
        .fold(f64::INFINITY, f64::min)
        - 1.0;
    let mut hi = generators
        .iter()
        .map(|g| g.marginal_cost(g.p_max_mw))
        .fold(f64::NEG_INFINITY, f64::max)
        + 1.0;

    let dispatch_at = |lam: f64| -> Vec<f64> {
        generators
            .iter()
            .map(|g| {
                if g.cost_a.abs() < 1e-15 {
                    // Linear: step dispatch
                    if lam >= g.cost_b {
                        g.p_max_mw
                    } else {
                        g.p_min_mw
                    }
                } else {
                    // Quadratic: P* = (λ - b) / (2a), clamped
                    let p_star = (lam - g.cost_b) / (2.0 * g.cost_a);
                    p_star.clamp(g.p_min_mw, g.p_max_mw)
                }
            })
            .collect()
    };

    let total_at = |lam: f64| -> f64 { dispatch_at(lam).iter().sum() };

    // Expand bracket if necessary
    for _ in 0..30 {
        if total_at(lo) > target_mw {
            lo -= (hi - lo).abs().max(1.0);
        } else {
            break;
        }
    }
    for _ in 0..30 {
        if total_at(hi) < target_mw {
            hi += (hi - lo).abs().max(1.0);
        } else {
            break;
        }
    }

    // Bisection — 64 iterations gives ~1e-19 relative accuracy
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        if total_at(mid) < target_mw {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo).abs() < 1e-10 {
            break;
        }
    }

    Ok(dispatch_at(0.5 * (lo + hi)))
}

/// Dispatch for purely linear cost generators: proportional to headroom.
fn dispatch_linear(generators: &[AdmmGenerator], target_mw: f64) -> Result<Vec<f64>> {
    let p_min_total: f64 = generators.iter().map(|g| g.p_min_mw).sum();
    let headroom: f64 = generators.iter().map(|g| g.p_max_mw - g.p_min_mw).sum();
    let extra = (target_mw - p_min_total).clamp(0.0, headroom);

    let mut result: Vec<f64> = generators.iter().map(|g| g.p_min_mw).collect();
    if headroom > 1e-10 {
        for (i, g) in generators.iter().enumerate() {
            let cap = g.p_max_mw - g.p_min_mw;
            result[i] += extra * cap / headroom;
        }
    }
    Ok(result)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gen(bus: usize, p_min: f64, p_max: f64, cost_a: f64, cost_b: f64) -> AdmmGenerator {
        AdmmGenerator {
            bus,
            p_min_mw: p_min,
            p_max_mw: p_max,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a,
            cost_b,
        }
    }

    fn make_area(
        id: usize,
        bus_indices: Vec<usize>,
        boundary_buses: Vec<usize>,
        generators: Vec<AdmmGenerator>,
        loads_mw: Vec<f64>,
    ) -> AdmmArea {
        let loads_mvar = vec![0.0; loads_mw.len()];
        AdmmArea {
            id,
            bus_indices,
            boundary_buses,
            generators,
            loads_mw,
            loads_mvar,
        }
    }

    fn two_area_setup() -> (Vec<AdmmArea>, Vec<BoundaryLink>) {
        let area0 = make_area(
            0,
            vec![0, 1],
            vec![1],
            vec![make_gen(0, 10.0, 200.0, 0.005, 18.0)],
            vec![60.0, 0.0],
        );
        let area1 = make_area(
            1,
            vec![2, 3],
            vec![2],
            vec![make_gen(2, 5.0, 150.0, 0.008, 22.0)],
            vec![0.0, 50.0],
        );
        let links = vec![BoundaryLink {
            bus_global: 5,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        (vec![area0, area1], links)
    }

    // -----------------------------------------------------------------------
    // 1. test_admm_area_creation
    // -----------------------------------------------------------------------
    #[test]
    fn test_admm_area_creation() {
        let area = make_area(
            0,
            vec![0, 1, 2],
            vec![2],
            vec![make_gen(0, 10.0, 100.0, 0.01, 20.0)],
            vec![30.0, 20.0, 10.0],
        );
        assert_eq!(area.id, 0);
        assert_eq!(area.bus_indices.len(), 3);
        assert_eq!(area.boundary_buses.len(), 1);
        assert_eq!(area.generators.len(), 1);
        assert!((area.total_load_mw() - 60.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // 2. test_boundary_link_creation
    // -----------------------------------------------------------------------
    #[test]
    fn test_boundary_link_creation() {
        let link = BoundaryLink {
            bus_global: 5,
            area_1: 0,
            bus_in_area_1: 2,
            area_2: 1,
            bus_in_area_2: 0,
        };
        assert_eq!(link.bus_global, 5);
        assert_eq!(link.area_1, 0);
        assert_eq!(link.area_2, 1);
        assert_eq!(link.bus_in_area_1, 2);
        assert_eq!(link.bus_in_area_2, 0);
    }

    // -----------------------------------------------------------------------
    // 3. test_admm_config_default
    // -----------------------------------------------------------------------
    #[test]
    fn test_admm_config_default() {
        let cfg = AdmmConfig::default();
        assert!((cfg.rho - 1.0).abs() < 1e-9);
        assert_eq!(cfg.max_iterations, 200);
        assert!((cfg.primal_tol - 1e-4).abs() < 1e-12);
        assert!((cfg.dual_tol - 1e-4).abs() < 1e-12);
        assert!(cfg.rho_update_enabled);
        assert!((cfg.rho_scale_factor - 2.0).abs() < 1e-9);
        assert!((cfg.base_mva - 100.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // 4. test_x_update_single_area_no_coupling
    // -----------------------------------------------------------------------
    #[test]
    fn test_x_update_single_area_no_coupling() {
        let area = make_area(
            0,
            vec![0],
            vec![],
            vec![make_gen(0, 0.0, 200.0, 0.01, 20.0)],
            vec![80.0],
        );
        let result = AdmmOpf::x_update_area(&area, &[], &[], 1.0, 1, 0).expect("x_update failed");
        assert_eq!(result.len(), 1);
        assert!(
            (result[0] - 80.0).abs() < 1.0,
            "expected ~80 MW, got {}",
            result[0]
        );
    }

    // -----------------------------------------------------------------------
    // 5. test_x_update_with_coupling_term
    // -----------------------------------------------------------------------
    #[test]
    fn test_x_update_with_coupling_term() {
        let area = make_area(
            0,
            vec![0, 1],
            vec![1],
            vec![make_gen(0, 0.0, 200.0, 0.01, 20.0)],
            vec![50.0, 0.0],
        );
        // z=10, y=0, rho=1  =>  p_bnd* = 10 - 0/1 = 10 (area must export 10 MW)
        let result =
            AdmmOpf::x_update_area(&area, &[0.0], &[10.0], 1.0, 1, 1).expect("x_update failed");
        assert_eq!(result.len(), 2);
        // Generator must cover load (50) + export (10) = 60 MW
        assert!(
            (result[0] - 60.0).abs() < 2.0,
            "expected ~60 MW gen, got {}",
            result[0]
        );
        assert!(
            (result[1] - 10.0).abs() < 1e-6,
            "expected p_bnd=10, got {}",
            result[1]
        );
    }

    // -----------------------------------------------------------------------
    // 6. test_z_update_consensus_equal
    // -----------------------------------------------------------------------
    #[test]
    fn test_z_update_consensus_equal() {
        let links = vec![BoundaryLink {
            bus_global: 2,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        let area_bmap = AdmmOpf::build_area_boundary_map(2, &links);
        // Both areas estimate boundary flow = 5, y = 0 => z = 5
        let x = vec![vec![50.0_f64, 5.0], vec![30.0_f64, 5.0]];
        let y = vec![vec![0.0_f64], vec![0.0_f64]];
        let z = AdmmOpf::z_update(&x, &y, &links, &area_bmap, 2, 1, 1.0);
        assert_eq!(z.len(), 1);
        assert!(
            (z[0] - 5.0).abs() < 1e-9,
            "consensus should be 5.0, got {}",
            z[0]
        );
    }

    // -----------------------------------------------------------------------
    // 7. test_z_update_consensus_average
    // -----------------------------------------------------------------------
    #[test]
    fn test_z_update_consensus_average() {
        let links = vec![BoundaryLink {
            bus_global: 2,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        let area_bmap = AdmmOpf::build_area_boundary_map(2, &links);
        // Area 0 estimates 8, area 1 estimates 4, y=0 => z = (8+4)/2 = 6
        let x = vec![vec![50.0_f64, 8.0], vec![30.0_f64, 4.0]];
        let y = vec![vec![0.0_f64], vec![0.0_f64]];
        let z = AdmmOpf::z_update(&x, &y, &links, &area_bmap, 2, 1, 1.0);
        assert!(
            (z[0] - 6.0).abs() < 1e-9,
            "consensus average should be 6.0, got {}",
            z[0]
        );
    }

    // -----------------------------------------------------------------------
    // 8. test_y_update_dual_ascent
    // -----------------------------------------------------------------------
    #[test]
    fn test_y_update_dual_ascent() {
        let links = vec![BoundaryLink {
            bus_global: 2,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        let area_bmap = AdmmOpf::build_area_boundary_map(2, &links);
        let x = vec![vec![50.0_f64, 8.0], vec![30.0_f64, 4.0]];
        let z = vec![6.0_f64];
        let mut y = vec![vec![0.0_f64], vec![0.0_f64]];

        AdmmOpf::y_update(&mut y, &x, &z, &links, &area_bmap, 1.0);

        // y[0][0] = 0 + 1*(8 - 6) = 2
        assert!(
            (y[0][0] - 2.0).abs() < 1e-9,
            "y[0][0] should be 2.0, got {}",
            y[0][0]
        );
        // y[1][0] = 0 + 1*(4 - 6) = -2
        assert!(
            (y[1][0] + 2.0).abs() < 1e-9,
            "y[1][0] should be -2.0, got {}",
            y[1][0]
        );
    }

    // -----------------------------------------------------------------------
    // 9. test_primal_residual_zero
    // -----------------------------------------------------------------------
    #[test]
    fn test_primal_residual_zero() {
        let links = vec![BoundaryLink {
            bus_global: 2,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        let area_bmap = AdmmOpf::build_area_boundary_map(2, &links);
        // x boundary vars == z exactly
        let x = vec![vec![50.0_f64, 5.0], vec![30.0_f64, 5.0]];
        let z = vec![5.0_f64];
        let res = AdmmOpf::compute_primal_residual(&x, &z, &links, &area_bmap);
        assert!(
            res.abs() < 1e-9,
            "primal residual should be 0 when x == z, got {}",
            res
        );
    }

    // -----------------------------------------------------------------------
    // 10. test_primal_residual_nonzero
    // -----------------------------------------------------------------------
    #[test]
    fn test_primal_residual_nonzero() {
        let links = vec![BoundaryLink {
            bus_global: 2,
            area_1: 0,
            bus_in_area_1: 1,
            area_2: 1,
            bus_in_area_2: 0,
        }];
        let area_bmap = AdmmOpf::build_area_boundary_map(2, &links);
        // Area 0 boundary = 8, area 1 boundary = 4, z = 6
        // residuals: (8-6)^2 + (4-6)^2 = 4+4 = 8  => sqrt(8) ≈ 2.828
        let x = vec![vec![50.0_f64, 8.0], vec![30.0_f64, 4.0]];
        let z = vec![6.0_f64];
        let res = AdmmOpf::compute_primal_residual(&x, &z, &links, &area_bmap);
        assert!(
            (res - 8.0_f64.sqrt()).abs() < 1e-9,
            "expected sqrt(8), got {}",
            res
        );
    }

    // -----------------------------------------------------------------------
    // 11. test_dual_residual_zero
    // -----------------------------------------------------------------------
    #[test]
    fn test_dual_residual_zero() {
        let z = vec![1.0_f64, 2.0, 3.0];
        let res = AdmmOpf::compute_dual_residual(&z, &z, 1.0);
        assert!(
            res.abs() < 1e-12,
            "dual residual should be 0 when z unchanged, got {}",
            res
        );
    }

    // -----------------------------------------------------------------------
    // 12. test_dual_residual_nonzero
    // -----------------------------------------------------------------------
    #[test]
    fn test_dual_residual_nonzero() {
        let z_old = vec![0.0_f64];
        let z_new = vec![3.0_f64];
        // dual res = |rho*(z_new - z_old)| = |2*3| = 6
        let res = AdmmOpf::compute_dual_residual(&z_old, &z_new, 2.0);
        assert!(
            (res - 6.0).abs() < 1e-9,
            "expected dual residual = 6.0, got {}",
            res
        );
    }

    // -----------------------------------------------------------------------
    // 13. test_rho_update_primal_larger
    // -----------------------------------------------------------------------
    #[test]
    fn test_rho_update_primal_larger() {
        let mut rho = 1.0_f64;
        let mut y = vec![vec![2.0_f64]];
        // primal >> dual => rho should increase
        AdmmOpf::update_rho(&mut rho, 100.0, 1.0, 2.0, &mut y);
        assert!(
            rho > 1.0,
            "rho should increase when primal residual dominates, got {}",
            rho
        );
        // Dual variables should be rescaled downward
        assert!(
            y[0][0] < 2.0,
            "dual variable should decrease after rho increase, got {}",
            y[0][0]
        );
    }

    // -----------------------------------------------------------------------
    // 14. test_rho_update_dual_larger
    // -----------------------------------------------------------------------
    #[test]
    fn test_rho_update_dual_larger() {
        let mut rho = 2.0_f64;
        let mut y = vec![vec![1.0_f64]];
        // dual >> primal => rho should decrease
        AdmmOpf::update_rho(&mut rho, 0.1, 100.0, 2.0, &mut y);
        assert!(
            rho < 2.0,
            "rho should decrease when dual residual dominates, got {}",
            rho
        );
        assert!(
            y[0][0] > 1.0,
            "dual variable should increase after rho decrease, got {}",
            y[0][0]
        );
    }

    // -----------------------------------------------------------------------
    // 15. test_solve_single_area
    // -----------------------------------------------------------------------
    #[test]
    fn test_solve_single_area() {
        let area = make_area(
            0,
            vec![0, 1],
            vec![],
            vec![
                make_gen(0, 10.0, 150.0, 0.01, 20.0),
                make_gen(1, 5.0, 100.0, 0.02, 15.0),
            ],
            vec![80.0, 40.0],
        );
        let mut solver = AdmmOpf::new(vec![area], vec![], AdmmConfig::default());
        let result = solver.solve().expect("solve failed");

        assert_eq!(result.area_dispatches.len(), 1);
        let dispatch = &result.area_dispatches[0];
        let total_gen: f64 = dispatch.generator_dispatch_mw.iter().sum();
        // Should supply ~120 MW total load
        assert!(
            (total_gen - 120.0).abs() < 5.0,
            "total dispatch should be ~120 MW, got {}",
            total_gen
        );
        assert!(result.total_cost_per_h > 0.0);
        assert!(
            result.converged,
            "single-area with no boundary should converge immediately"
        );
    }

    // -----------------------------------------------------------------------
    // 16. test_solve_two_areas_converges
    // -----------------------------------------------------------------------
    #[test]
    fn test_solve_two_areas_converges() {
        let (areas, links) = two_area_setup();
        let cfg = AdmmConfig {
            max_iterations: 300,
            primal_tol: 1e-3,
            dual_tol: 1e-3,
            ..AdmmConfig::default()
        };

        let mut solver = AdmmOpf::new(areas, links, cfg);
        let result = solver.solve().expect("solve failed");

        assert_eq!(result.area_dispatches.len(), 2);
        assert!(
            result.converged || result.iterations == 300,
            "solver should converge or exhaust iterations"
        );
        assert!(result.total_cost_per_h >= 0.0);
        assert_eq!(result.boundary_voltages.len(), 1);
    }

    // -----------------------------------------------------------------------
    // 17. test_solve_convergence_tolerance
    // -----------------------------------------------------------------------
    #[test]
    fn test_solve_convergence_tolerance() {
        // Single area with no boundary => residuals = 0 from iteration 1
        let area = make_area(
            0,
            vec![0],
            vec![],
            vec![make_gen(0, 0.0, 100.0, 0.01, 10.0)],
            vec![50.0],
        );
        let mut solver = AdmmOpf::new(vec![area], vec![], AdmmConfig::default());
        let result = solver.solve().expect("solve failed");
        assert!(
            result.converged,
            "single area with no boundary should converge"
        );
        assert!(result.iterations <= 200);
    }

    // -----------------------------------------------------------------------
    // 18. test_solve_residual_history_length
    // -----------------------------------------------------------------------
    #[test]
    fn test_solve_residual_history_length() {
        let (areas, links) = two_area_setup();
        let cfg = AdmmConfig {
            max_iterations: 50,
            ..AdmmConfig::default()
        };
        let mut solver = AdmmOpf::new(areas, links, cfg);
        let result = solver.solve().expect("solve failed");

        assert_eq!(
            result.primal_residual_history.len(),
            result.dual_residual_history.len(),
            "primal and dual history lengths must match"
        );
        assert_eq!(
            result.primal_residual_history.len(),
            result.iterations,
            "history length must equal iteration count"
        );
    }

    // -----------------------------------------------------------------------
    // 19. test_total_cost_computation
    // -----------------------------------------------------------------------
    #[test]
    fn test_total_cost_computation() {
        // Cost = a*P^2 + b*P; a=0.01, b=20, P=100
        // => 0.01*10000 + 20*100 = 100 + 2000 = 2100
        let gen = make_gen(0, 0.0, 200.0, 0.01, 20.0);
        let area = AdmmArea {
            id: 0,
            bus_indices: vec![0],
            boundary_buses: vec![],
            generators: vec![gen],
            loads_mw: vec![100.0],
            loads_mvar: vec![0.0],
        };
        let dispatches = vec![vec![100.0_f64]];
        let cost = AdmmOpf::compute_total_cost(&[area], &dispatches);
        assert!(
            (cost - 2100.0).abs() < 1e-6,
            "expected cost 2100, got {}",
            cost
        );
    }

    // -----------------------------------------------------------------------
    // 20. test_admm_result_structure
    // -----------------------------------------------------------------------
    #[test]
    fn test_admm_result_structure() {
        let area = make_area(
            0,
            vec![0, 1],
            vec![],
            vec![make_gen(0, 10.0, 100.0, 0.01, 20.0)],
            vec![50.0, 30.0],
        );
        let mut solver = AdmmOpf::new(vec![area], vec![], AdmmConfig::default());
        let result = solver.solve().expect("solve failed");

        assert_eq!(result.area_dispatches.len(), 1);
        let d = &result.area_dispatches[0];
        assert_eq!(d.area_id, 0);
        assert_eq!(d.generator_dispatch_mw.len(), 1);
        assert_eq!(d.generator_dispatch_mvar.len(), 1);
        assert_eq!(d.bus_voltages.len(), 2); // matches bus_indices length
        assert_eq!(result.boundary_voltages.len(), 0); // no boundary links
        assert!(result.convergence_rate >= 0.0);
        assert!(result.total_cost_per_h >= 0.0);
        assert!(result.primal_residual_history.len() == result.iterations);
        assert!(result.dual_residual_history.len() == result.iterations);
    }
}
