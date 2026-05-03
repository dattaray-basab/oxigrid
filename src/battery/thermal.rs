/// Battery thermal models.
///
/// # Lumped Thermal Model
///
/// A single thermal node (battery cell / pack) described by:
///
///   m·Cp·dT/dt = Q_gen − Q_cool
///
/// Heat generation:
///   Q_gen = I²·R_eff + I·T·ΔS/nF   (Joule + entropic)
///
/// Convective cooling:
///   Q_cool = h·A·(T − T_amb)
///
/// Discrete update (forward Euler):
///   T(k+1) = T(k) + Δt/(m·Cp) · (Q_gen − Q_cool)
use crate::units::Temperature;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LumpedThermalModel {
    /// Cell / pack mass `kg`
    pub mass_kg: f64,
    /// Specific heat capacity [J/(kg·K)]
    pub heat_capacity: f64,
    /// Convective heat transfer coefficient [W/(m²·K)]
    pub h_conv: f64,
    /// Heat transfer area `m²`
    pub area_m2: f64,
    /// Ambient temperature `K`
    pub t_ambient: f64,
    /// Entropic heat coefficient [J/(K·A·s)] (dU/dT * I — usually small)
    pub entropic_coeff: f64,

    // State
    pub temperature: f64,
}

impl LumpedThermalModel {
    pub fn new(mass_kg: f64, heat_capacity: f64, h_conv: f64, area_m2: f64) -> Self {
        Self {
            mass_kg,
            heat_capacity,
            h_conv,
            area_m2,
            t_ambient: 298.15,
            entropic_coeff: 0.0,
            temperature: 298.15,
        }
    }

    /// Simple cylindrical 18650 cell defaults.
    pub fn cell_18650() -> Self {
        Self::new(
            0.045,   // 45 g
            1000.0,  // J/(kg·K)  (approx LFP/NMC)
            10.0,    // natural convection
            0.003_5, // ≈ surface area of 18650
        )
    }

    /// Prismatic pouch cell (75 Ah) defaults.
    pub fn pouch_75ah() -> Self {
        Self::new(
            1.5,    // 1.5 kg
            1000.0, // J/(kg·K)
            15.0,   // forced convection
            0.08,   // m² (approximate)
        )
    }

    /// Advance thermal model by one time step.
    ///
    /// `current_a` — battery current `A` (positive = discharge)
    /// `resistance` — effective internal resistance `Ω`
    /// `dt` — time step `s`
    pub fn step(&mut self, current_a: f64, resistance: f64, dt: f64) -> Temperature {
        // Joule heating
        let q_joule = current_a * current_a * resistance;
        // Entropic heating (positive for exothermic discharge in many Li-ion chemistries)
        let q_entropic = self.entropic_coeff * current_a * self.temperature;
        let q_gen = q_joule + q_entropic;
        // Convective cooling
        let q_cool = self.h_conv * self.area_m2 * (self.temperature - self.t_ambient);

        let d_t = dt / (self.mass_kg * self.heat_capacity) * (q_gen - q_cool);
        self.temperature += d_t;

        Temperature(self.temperature)
    }

    pub fn temperature(&self) -> Temperature {
        Temperature(self.temperature)
    }

    /// Steady-state temperature at constant current.
    pub fn steady_state_temp(&self, current_a: f64, resistance: f64) -> Temperature {
        let q_gen = current_a * current_a * resistance;
        let h_eff = self.h_conv * self.area_m2;
        Temperature(self.t_ambient + q_gen / h_eff)
    }
}

// ── 1D Thermal Model (cylindrical cell) ─────────────────────────────────────

/// Simple 1D radial thermal model for a cylindrical cell.
///
/// Divides the cell into N radial shells.  Heat is generated uniformly
/// in the active material (inner node) and transferred outward.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadialThermalModel {
    pub n_nodes: usize,
    pub radius_m: f64,
    pub length_m: f64,
    pub conductivity: f64,  // W/(m·K)
    pub density: f64,       // kg/m³
    pub specific_heat: f64, // J/(kg·K)
    pub h_conv: f64,
    pub t_ambient: f64,
    pub temperatures: Vec<f64>, // K, inner to outer
}

impl RadialThermalModel {
    pub fn new(n_nodes: usize, radius_m: f64, length_m: f64) -> Self {
        Self {
            n_nodes,
            radius_m,
            length_m,
            conductivity: 1.0, // W/(m·K) typical Li-ion
            density: 2500.0,   // kg/m³
            specific_heat: 1000.0,
            h_conv: 10.0,
            t_ambient: 298.15,
            temperatures: vec![298.15; n_nodes],
        }
    }

    /// Advance one time step using explicit Euler finite differences.
    pub fn step(&mut self, heat_gen_w_per_m3: f64, dt: f64) {
        let dr = self.radius_m / self.n_nodes as f64;
        let rho_cp = self.density * self.specific_heat;
        let mut new_temps = self.temperatures.clone();

        #[allow(clippy::needless_range_loop)]
        for k in 0..self.n_nodes {
            let r = (k as f64 + 0.5) * dr;
            let flux_in = if k > 0 {
                self.conductivity / dr
                    * (self.temperatures[k - 1] - self.temperatures[k])
                    * (r - dr / 2.0)
                    / r
            } else {
                0.0 // symmetry at centre
            };
            let flux_out = if k < self.n_nodes - 1 {
                self.conductivity / dr
                    * (self.temperatures[k] - self.temperatures[k + 1])
                    * (r + dr / 2.0)
                    / r
            } else {
                // Outer surface: convection
                self.h_conv * (self.temperatures[k] - self.t_ambient) * (r + dr / 2.0) / r
            };
            new_temps[k] +=
                dt / (rho_cp * dr) * (flux_in - flux_out) + dt / rho_cp * heat_gen_w_per_m3;
        }
        self.temperatures = new_temps;
    }

    pub fn max_temperature(&self) -> Temperature {
        Temperature(
            self.temperatures
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max),
        )
    }

    pub fn surface_temperature(&self) -> Temperature {
        Temperature(*self.temperatures.last().unwrap_or(&298.15))
    }
}

// ── 1D Axial Thermal Model ────────────────────────────────────────────────────

/// Boundary condition for each end of the 1D axial cell.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AxialBoundary {
    /// Convective Robin BC: k·dT/dz·n̂ = h·(T_amb − T_surf).
    Convective { h_conv: f64 },
    /// Fixed-temperature Dirichlet BC: T_surf = T_fixed.
    Dirichlet { t_fixed_k: f64 },
}

/// Configuration parameters for [`Thermal1DAxial`].
///
/// Bundling the nine physical inputs into a single struct avoids the
/// `clippy::too_many_arguments` lint and makes call sites more readable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Axial1DConfig {
    /// Number of axial discretisation nodes (≥ 2).
    pub n_nodes: usize,
    /// Total axial length `m`.
    pub length_m: f64,
    /// Perpendicular cross-sectional area `m²`.
    pub cross_area_m2: f64,
    /// Material density [kg/m³].
    pub density: f64,
    /// Specific heat capacity [J/(kg·K)].
    pub cp: f64,
    /// Axial thermal conductivity [W/(m·K)].
    pub k: f64,
    /// Ambient temperature `K`.
    pub t_ambient_k: f64,
    /// Boundary condition at z = 0 (left end).
    pub bc_left: AxialBoundary,
    /// Boundary condition at z = L (right end).
    pub bc_right: AxialBoundary,
}

/// 1D axial finite-difference thermal model for a prismatic/cylindrical cell.
///
/// ## Mathematical background
///
/// The heat equation along the axial direction z ∈ [0, L]:
///
/// ```text
///   ρ·Cp·∂T/∂t = k·∂²T/∂z² + q_gen(z, t)
/// ```
///
/// Discretised using N uniform nodes spaced dx = L/(N-1):
///
/// ```text
///   T_i(t + Δt) = T_i(t) + α·Δt/dx² · (T_{i-1} − 2·T_i + T_{i+1}) + Δt·q_i/(ρ·Cp)
/// ```
///
/// where α = k/(ρ·Cp) is the thermal diffusivity. Explicit Euler is
/// conditionally stable when Δt ≤ dx²/(2·α). The `step` method adaptively
/// sub-steps to satisfy this CFL condition.
///
/// Boundary conditions at both ends are applied via the ghost-node method.
/// Construct via [`Thermal1DAxial::new`] (accepts an [`Axial1DConfig`]).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Thermal1DAxial {
    /// Number of axial discretisation nodes (≥ 2).
    pub n_nodes: usize,
    /// Total axial length `m`.
    pub length_m: f64,
    /// Perpendicular cross-sectional area `m²`.
    pub cross_area_m2: f64,
    /// Material density [kg/m³].
    pub density: f64,
    /// Specific heat capacity [J/(kg·K)].
    pub cp: f64,
    /// Axial thermal conductivity [W/(m·K)].
    pub k: f64,
    /// Ambient temperature `K`.
    pub t_ambient_k: f64,
    /// Boundary condition at z = 0.
    pub bc_left: AxialBoundary,
    /// Boundary condition at z = L.
    pub bc_right: AxialBoundary,
    /// Current nodal temperatures `K`, length = n_nodes.
    pub temperatures: Vec<f64>,
}

impl Thermal1DAxial {
    /// Create a new 1D axial model from an [`Axial1DConfig`], initialised at ambient temperature.
    ///
    /// # Errors
    ///
    /// Returns [`crate::error::OxiGridError::InvalidParameter`] if `n_nodes < 2` or any physical
    /// parameter (`length_m`, `cross_area_m2`, `density`, `cp`, `k`) is ≤ 0.
    pub fn new(cfg: Axial1DConfig) -> Result<Self, crate::error::OxiGridError> {
        if cfg.n_nodes < 2 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "Thermal1DAxial requires at least 2 nodes".into(),
            ));
        }
        if cfg.length_m <= 0.0
            || cfg.cross_area_m2 <= 0.0
            || cfg.density <= 0.0
            || cfg.cp <= 0.0
            || cfg.k <= 0.0
        {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "Thermal1DAxial: all physical parameters must be positive".into(),
            ));
        }
        let t_init = cfg.t_ambient_k;
        let n = cfg.n_nodes;
        Ok(Self {
            n_nodes: cfg.n_nodes,
            length_m: cfg.length_m,
            cross_area_m2: cfg.cross_area_m2,
            density: cfg.density,
            cp: cfg.cp,
            k: cfg.k,
            t_ambient_k: cfg.t_ambient_k,
            bc_left: cfg.bc_left,
            bc_right: cfg.bc_right,
            temperatures: vec![t_init; n],
        })
    }

    /// Thermal diffusivity α = k / (ρ·Cp) [m²/s].
    pub fn diffusivity(&self) -> f64 {
        self.k / (self.density * self.cp)
    }

    /// Spatial step Δx = L / (N-1) `m`.
    pub fn dx(&self) -> f64 {
        self.length_m / (self.n_nodes - 1) as f64
    }

    /// CFL-stable time step: Δt_cfl = dx² / (2·α) `s`.
    pub fn dt_cfl(&self) -> f64 {
        let dx = self.dx();
        let alpha = self.diffusivity();
        (dx * dx) / (2.0 * alpha)
    }

    /// Advance temperatures by `dt` seconds with heat source `q_gen_per_node_w`
    /// (one value per node, in Watts applied uniformly to each node volume).
    ///
    /// Adaptively sub-steps to satisfy the CFL condition.
    pub fn step(
        &mut self,
        dt: f64,
        q_gen_per_node_w: &[f64],
    ) -> Result<(), crate::error::OxiGridError> {
        if q_gen_per_node_w.len() != self.n_nodes {
            return Err(crate::error::OxiGridError::InvalidParameter(format!(
                "q_gen length {} != n_nodes {}",
                q_gen_per_node_w.len(),
                self.n_nodes
            )));
        }
        let dt_cfl = self.dt_cfl();
        let sub_steps = ((dt / dt_cfl).ceil() as usize).max(1);
        let dt_sub = dt / sub_steps as f64;
        for _ in 0..sub_steps {
            self.euler_step(dt_sub, q_gen_per_node_w);
        }
        Ok(())
    }

    /// Implicit-Euler tridiagonal solve for stiff regimes (Thomas algorithm).
    ///
    /// Use when `dt >> dt_cfl` and the sub-stepping in `step` would be too slow.
    pub fn step_implicit(
        &mut self,
        dt: f64,
        q_gen_per_node_w: &[f64],
    ) -> Result<(), crate::error::OxiGridError> {
        if q_gen_per_node_w.len() != self.n_nodes {
            return Err(crate::error::OxiGridError::InvalidParameter(format!(
                "q_gen length {} != n_nodes {}",
                q_gen_per_node_w.len(),
                self.n_nodes
            )));
        }
        let n = self.n_nodes;
        let dx = self.dx();
        let alpha = self.diffusivity();
        let r = alpha * dt / (dx * dx); // Fourier number

        // Build tridiagonal system (I + r·L)·T^{n+1} = T^n + dt·q/(ρ·Cp)
        // L is the 1D Laplacian.
        let mut a = vec![0.0f64; n]; // sub-diagonal
        let mut b = vec![1.0 + 2.0 * r; n]; // main diagonal
        let mut c = vec![-r; n]; // super-diagonal
        let mut d = vec![0.0f64; n]; // RHS

        let vol = (self.length_m * self.cross_area_m2) / n as f64; // node volume approx
        let rho_cp = self.density * self.cp;

        for i in 0..n {
            a[i] = -r;
            let q_src = dt * q_gen_per_node_w[i] / (rho_cp * vol);
            d[i] = self.temperatures[i] + q_src;
        }

        // Apply boundary conditions (ghost-node at boundaries)
        self.apply_bc_implicit(dx, r, &mut a, &mut b, &mut c, &mut d);

        // Thomas algorithm
        self.thomas_solve(&a, &b, &c, &mut d)?;
        self.temperatures = d;
        Ok(())
    }

    fn euler_step(&mut self, dt: f64, q_gen: &[f64]) {
        let n = self.n_nodes;
        let dx = self.dx();
        let alpha = self.diffusivity();
        let rho_cp = self.density * self.cp;
        let vol = (self.length_m * self.cross_area_m2) / n as f64;
        let r = alpha * dt / (dx * dx);

        let t_old = self.temperatures.clone();

        // Compute ghost nodes before taking mutable borrow.
        let t_ghost_left = self.ghost_node_left(&t_old, dx);
        let t_ghost_right = self.ghost_node_right(&t_old, dx);

        let temps = &mut self.temperatures;

        // Interior nodes
        for i in 1..n - 1 {
            let laplacian = t_old[i - 1] - 2.0 * t_old[i] + t_old[i + 1];
            temps[i] = t_old[i] + r * laplacian + dt * q_gen[i] / (rho_cp * vol);
        }

        // Left boundary (i=0) with ghost node T_{-1}
        let laplacian_left = t_ghost_left - 2.0 * t_old[0] + t_old[1];
        temps[0] = t_old[0] + r * laplacian_left + dt * q_gen[0] / (rho_cp * vol);

        // Right boundary (i=n-1) with ghost node T_{n}
        let laplacian_right = t_old[n - 2] - 2.0 * t_old[n - 1] + t_ghost_right;
        temps[n - 1] = t_old[n - 1] + r * laplacian_right + dt * q_gen[n - 1] / (rho_cp * vol);
    }

    fn ghost_node_left(&self, t: &[f64], dx: f64) -> f64 {
        match &self.bc_left {
            AxialBoundary::Dirichlet { t_fixed_k } => {
                // T_{-1} = 2·T_fixed − T_0 (ghost node for Dirichlet)
                2.0 * t_fixed_k - t[0]
            }
            AxialBoundary::Convective { h_conv } => {
                // k·(T_0 − T_{-1})/(2·dx) = h·(T_amb − T_0)
                // T_{-1} = T_0 + 2·dx·h/k·(T_amb − T_0)
                t[0] + 2.0 * dx * h_conv / self.k * (self.t_ambient_k - t[0])
            }
        }
    }

    fn ghost_node_right(&self, t: &[f64], dx: f64) -> f64 {
        let n = t.len();
        match &self.bc_right {
            AxialBoundary::Dirichlet { t_fixed_k } => 2.0 * t_fixed_k - t[n - 1],
            AxialBoundary::Convective { h_conv } => {
                t[n - 1] + 2.0 * dx * h_conv / self.k * (self.t_ambient_k - t[n - 1])
            }
        }
    }

    fn apply_bc_implicit(
        &self,
        dx: f64,
        r: f64,
        a: &mut [f64],
        b: &mut [f64],
        c: &mut [f64],
        d: &mut [f64],
    ) {
        let n = self.n_nodes;

        // Left BC modification to row 0
        match &self.bc_left {
            AxialBoundary::Dirichlet { t_fixed_k } => {
                // T_0 = T_fixed: set row to identity
                a[0] = 0.0;
                b[0] = 1.0;
                c[0] = 0.0;
                d[0] = *t_fixed_k;
            }
            AxialBoundary::Convective { h_conv } => {
                let biot = h_conv * dx / self.k;
                b[0] += 2.0 * r * biot;
                d[0] += 2.0 * r * biot * self.t_ambient_k;
            }
        }

        // Right BC modification to row n-1
        match &self.bc_right {
            AxialBoundary::Dirichlet { t_fixed_k } => {
                a[n - 1] = 0.0;
                b[n - 1] = 1.0;
                c[n - 1] = 0.0;
                d[n - 1] = *t_fixed_k;
            }
            AxialBoundary::Convective { h_conv } => {
                let biot = h_conv * dx / self.k;
                b[n - 1] += 2.0 * r * biot;
                d[n - 1] += 2.0 * r * biot * self.t_ambient_k;
            }
        }
    }

    /// Thomas algorithm (tridiagonal solver). Solves in-place; result stored in `d`.
    fn thomas_solve(
        &self,
        a: &[f64],
        b: &[f64],
        c: &[f64],
        d: &mut [f64],
    ) -> Result<(), crate::error::OxiGridError> {
        let n = self.n_nodes;
        // Forward sweep
        let mut w = vec![0.0f64; n];
        let mut g = vec![0.0f64; n];
        if b[0].abs() < 1e-14 {
            return Err(crate::error::OxiGridError::LinearAlgebra(
                "Zero diagonal in Thomas algorithm".into(),
            ));
        }
        w[0] = c[0] / b[0];
        g[0] = d[0] / b[0];
        for i in 1..n {
            let denom = b[i] - a[i] * w[i - 1];
            if denom.abs() < 1e-14 {
                return Err(crate::error::OxiGridError::LinearAlgebra(format!(
                    "Near-zero pivot at row {} in Thomas algorithm",
                    i
                )));
            }
            w[i] = if i < n - 1 { c[i] / denom } else { 0.0 };
            g[i] = (d[i] - a[i] * g[i - 1]) / denom;
        }
        // Back substitution
        d[n - 1] = g[n - 1];
        for i in (0..n - 1).rev() {
            d[i] = g[i] - w[i] * d[i + 1];
        }
        Ok(())
    }

    /// Current temperature at node `i`.
    pub fn temperature_at(&self, i: usize) -> Option<f64> {
        self.temperatures.get(i).copied()
    }

    /// Maximum temperature across all nodes.
    pub fn max_temperature(&self) -> f64 {
        self.temperatures
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Minimum temperature across all nodes.
    pub fn min_temperature(&self) -> f64 {
        self.temperatures
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lumped_heats_up_under_load() {
        let mut model = LumpedThermalModel::cell_18650();
        let init_temp = model.temperature;
        // 5A discharge (moderate load)
        for _ in 0..600 {
            model.step(5.0, 0.05, 1.0);
        }
        assert!(model.temperature > init_temp);
    }

    #[test]
    fn test_steady_state() {
        let model = LumpedThermalModel::cell_18650();
        let ss = model.steady_state_temp(5.0, 0.05);
        // With 5A and 0.05Ω: Q = 1.25W, h*A = 10*0.0035 = 0.035
        // ΔT = 1.25/0.035 ≈ 35.7K
        assert!(ss.0 > model.t_ambient);
        assert!(ss.0 < model.t_ambient + 100.0);
    }

    #[test]
    fn test_radial_heats_up() {
        let mut model = RadialThermalModel::new(5, 0.009, 0.065);
        let init_temp = model.temperatures[0];
        // 10 kW/m³ internal heat generation
        for _ in 0..100 {
            model.step(10_000.0, 1.0);
        }
        assert!(model.temperatures[0] > init_temp);
    }

    #[test]
    fn test_lumped_steady_state_18650() {
        let model = LumpedThermalModel::cell_18650();
        // I=5A, R=0.05Ω → Q=1.25W; h*A=10*0.0035=0.035 W/K → ΔT=35.71K
        let ss = model.steady_state_temp(5.0, 0.05);
        let expected = model.t_ambient + 1.25 / 0.035;
        assert!(
            (ss.0 - expected).abs() < 1e-3,
            "ss={:.4}, exp={:.4}",
            ss.0,
            expected
        );
    }

    #[test]
    fn test_lumped_pouch_cools_to_ambient_at_zero_current() {
        let mut model = LumpedThermalModel::pouch_75ah();
        // Force a high initial temperature
        model.temperature = 340.0;
        for _ in 0..10_000 {
            model.step(0.0, 0.0, 1.0);
        }
        // With no heat generation the cell should cool toward ambient (298.15 K)
        assert!(
            (model.temperature - model.t_ambient).abs() < 1.0,
            "final T={:.2}, ambient={:.2}",
            model.temperature,
            model.t_ambient
        );
    }

    #[test]
    fn test_axial_1d_new_rejects_one_node() {
        let cfg = Axial1DConfig {
            n_nodes: 1,
            length_m: 0.1,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Convective { h_conv: 10.0 },
            bc_right: AxialBoundary::Convective { h_conv: 10.0 },
        };
        assert!(Thermal1DAxial::new(cfg).is_err());
    }

    #[test]
    fn test_axial_1d_heats_up_with_source() {
        let cfg = Axial1DConfig {
            n_nodes: 5,
            length_m: 0.1,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Convective { h_conv: 10.0 },
            bc_right: AxialBoundary::Convective { h_conv: 10.0 },
        };
        let mut model = Thermal1DAxial::new(cfg).expect("valid config");
        let init = model.temperatures[2];
        let q = vec![50.0; 5]; // 50 W per node
        model.step(1.0, &q).expect("step ok");
        assert!(model.temperatures[2] > init, "temperature must increase");
    }

    #[test]
    fn test_axial_1d_step_wrong_q_len_returns_err() {
        let cfg = Axial1DConfig {
            n_nodes: 4,
            length_m: 0.08,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Dirichlet { t_fixed_k: 298.15 },
            bc_right: AxialBoundary::Dirichlet { t_fixed_k: 298.15 },
        };
        let mut model = Thermal1DAxial::new(cfg).expect("valid config");
        // Wrong q length → Err
        assert!(model.step(1.0, &[10.0; 3]).is_err());
    }

    #[test]
    fn test_axial_1d_dt_cfl_is_positive() {
        let cfg = Axial1DConfig {
            n_nodes: 10,
            length_m: 0.1,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Convective { h_conv: 10.0 },
            bc_right: AxialBoundary::Convective { h_conv: 10.0 },
        };
        let model = Thermal1DAxial::new(cfg).expect("valid");
        assert!(model.dt_cfl() > 0.0);
    }

    #[test]
    fn test_axial_1d_implicit_step_heats_up() {
        let cfg = Axial1DConfig {
            n_nodes: 5,
            length_m: 0.1,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Convective { h_conv: 5.0 },
            bc_right: AxialBoundary::Convective { h_conv: 5.0 },
        };
        let mut model = Thermal1DAxial::new(cfg).expect("valid");
        let init = model.temperatures[2];
        let q = vec![100.0; 5];
        model.step_implicit(0.5, &q).expect("implicit step ok");
        assert!(model.temperatures[2] > init);
    }

    #[test]
    fn test_temperature_at_out_of_bounds_returns_none() {
        let cfg = Axial1DConfig {
            n_nodes: 3,
            length_m: 0.06,
            cross_area_m2: 1e-4,
            density: 2500.0,
            cp: 800.0,
            k: 1.5,
            t_ambient_k: 298.15,
            bc_left: AxialBoundary::Convective { h_conv: 10.0 },
            bc_right: AxialBoundary::Convective { h_conv: 10.0 },
        };
        let model = Thermal1DAxial::new(cfg).expect("valid");
        assert!(model.temperature_at(2).is_some());
        assert!(model.temperature_at(3).is_none());
    }
}
