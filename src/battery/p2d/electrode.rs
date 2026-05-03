/// Electrode model for the Single Particle Model (SPM).
///
/// Each electrode (anode or cathode) is represented by a single spherical
/// particle with radial solid-phase lithium diffusion.  This is the
/// simplest physics-based battery model that captures diffusion limitations.
///
/// # Governing equation
/// Solid-phase diffusion (Fick's law in spherical coordinates):
///   ∂c_s/∂t = (D_s/r²) ∂/∂r(r² ∂c_s/∂r)
///
/// Boundary conditions:
///   ∂c_s/∂r|_{r=0} = 0              (symmetry)
///   −D_s ∂c_s/∂r|_{r=Rp} = j_n/F   (surface flux from pore-wall reaction)
///
/// # Reference
/// Doyle, Fuller, Newman, "Modeling of Galvanostatic Charge and Discharge of
/// the Lithium/Polymer/Insertion Cell," J. Electrochem. Soc., 140(6), 1993.
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Type of electrode chemistry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ElectrodeType {
    /// Graphite negative electrode (anode during discharge)
    Graphite,
    /// LiFePO4 positive electrode
    LFP,
    /// NMC (LiNiMnCoO2) positive electrode
    NMC,
}

/// Electrode material parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrodeParams {
    pub electrode_type: ElectrodeType,
    /// Particle radius `m`
    pub r_particle: f64,
    /// Solid-phase diffusivity at reference temperature [m²/s]
    pub d_s_ref: f64,
    /// Maximum lithium concentration in solid [mol/m³]
    pub c_s_max: f64,
    /// Initial stoichiometry (θ = c_s_avg / c_s_max)
    pub theta_init: f64,
    /// Electrode thickness `m`
    pub thickness: f64,
    /// Active material volume fraction
    pub epsilon_s: f64,
    /// Electrode area `m²`
    pub area: f64,
    /// Activation energy for diffusivity [J/mol]
    pub e_a_ds: f64,
    /// Butler-Volmer exchange current density pre-factor [A/m²]
    pub k_bv: f64,
}

impl ElectrodeParams {
    /// Graphite anode for a ~3 Ah cell.
    pub fn graphite_anode() -> Self {
        Self {
            electrode_type: ElectrodeType::Graphite,
            r_particle: 12.5e-6, // 12.5 µm
            d_s_ref: 3.9e-14,    // m²/s
            c_s_max: 31_833.0,   // mol/m³
            theta_init: 0.80,    // ~80% lithiated at full charge
            thickness: 100e-6,   // 100 µm
            epsilon_s: 0.60,
            area: 0.0626,     // m²
            e_a_ds: 35_000.0, // J/mol
            k_bv: 2.0e-11,
        }
    }

    /// LiFePO4 cathode for a ~3 Ah cell.
    pub fn lfp_cathode() -> Self {
        Self {
            electrode_type: ElectrodeType::LFP,
            r_particle: 2.5e-6, // 2.5 µm
            d_s_ref: 1.0e-14,   // m²/s
            c_s_max: 22_806.0,  // mol/m³
            theta_init: 0.20,   // ~20% lithiated at full charge
            thickness: 80e-6,
            epsilon_s: 0.50,
            area: 0.0626,
            e_a_ds: 20_000.0,
            k_bv: 2.0e-11,
        }
    }

    /// NMC811 cathode.
    pub fn nmc_cathode() -> Self {
        Self {
            electrode_type: ElectrodeType::NMC,
            r_particle: 5.0e-6,
            d_s_ref: 1.5e-14,
            c_s_max: 49_000.0,
            theta_init: 0.15,
            thickness: 90e-6,
            epsilon_s: 0.55,
            area: 0.0626,
            e_a_ds: 25_000.0,
            k_bv: 2.0e-11,
        }
    }

    /// Temperature-corrected diffusivity [m²/s] using Arrhenius.
    pub fn d_s(&self, temp_k: f64) -> f64 {
        const R_GAS: f64 = 8.314;
        const T_REF: f64 = 298.15;
        self.d_s_ref * (self.e_a_ds / R_GAS * (1.0 / T_REF - 1.0 / temp_k)).exp()
    }

    /// Specific interfacial area a_s = 3*ε_s/R_p [m²/m³].
    pub fn specific_area(&self) -> f64 {
        3.0 * self.epsilon_s / self.r_particle
    }

    /// Open-circuit potential (OCP) `V` as a function of stoichiometry θ ∈ `0,1`.
    pub fn ocp(&self, theta: f64) -> f64 {
        let x = theta.clamp(0.01, 0.99);
        match self.electrode_type {
            ElectrodeType::Graphite => ocp_graphite(x),
            ElectrodeType::LFP => ocp_lfp(x),
            ElectrodeType::NMC => ocp_nmc(x),
        }
    }

    /// dOCP/dθ (numerical) — needed for EIS and impedance models.
    pub fn docp_dtheta(&self, theta: f64) -> f64 {
        let eps = 1e-4;
        let t = theta.clamp(eps, 1.0 - eps);
        (self.ocp(t + eps) - self.ocp(t - eps)) / (2.0 * eps)
    }
}

/// Open-circuit potential for graphite (Ramadass et al. 2004).
fn ocp_graphite(x: f64) -> f64 {
    0.7222 + 0.1387 * x + 0.029 * x.powf(0.5) - 0.0172 / x
        + 0.0019 / x.powf(1.5)
        + 0.2808 * (-0.9 / x).exp()
        - 0.7984 * (0.4465 * x - 0.4108).exp()
}

/// Open-circuit potential for LiFePO4 (simplified empirical polynomial fit).
///
/// LFP has a very flat plateau at ≈ 3.4 V for θ ∈ [0.05, 0.95].
/// Logistic sigmoid terms model the steep rise/fall at extreme stoichiometries.
fn ocp_lfp(x: f64) -> f64 {
    let x = x.clamp(0.01, 0.99);
    // Flat-plateau polynomial backbone + sigmoid end corrections
    let backbone = 3.414 + 0.12 * x.powi(2) - 0.17 * x.powi(3) + 0.05 * x.powi(4);
    // Low-SOC drop (x → 0): subtract a logistic rising term at small x
    let low_soc = 0.20 / (1.0 + (80.0 * (x - 0.08)).exp());
    // High-SOC rise (x → 1): add a logistic rising term near x=0.92
    let high_soc = 0.15 / (1.0 + (80.0 * (0.92 - x)).exp());
    backbone - low_soc + high_soc
}

/// Open-circuit potential for NMC811 (simplified empirical fit).
///
/// Polynomial fit valid for x ∈ [0.1, 0.9], giving ≈ 4.2 V at x=0.1 and
/// ≈ 3.5 V at x=0.9 (fully discharged).
fn ocp_nmc(x: f64) -> f64 {
    let x = x.clamp(0.1, 0.9);
    4.20 - 1.50 * x + 1.20 * x.powi(2) - 0.60 * x.powi(3) + 0.08 / (1.0 + (30.0 * (x - 0.85)).exp())
    // end-of-charge rise (bounded)
}

/// Single-particle solid-phase diffusion model.
///
/// Uses finite differences in the radial direction (N nodes).
/// The state is the Li concentration profile c_s[0..N].
pub struct ParticleDiffusion {
    /// Electrode parameters
    pub params: ElectrodeParams,
    /// Li concentration profile [mol/m³], index 0 = center, N-1 = surface
    pub c_s: Vec<f64>,
    /// Number of radial nodes
    pub n_nodes: usize,
    /// Radial step size `m`
    dr: f64,
}

impl ParticleDiffusion {
    /// Initialise with uniform concentration at `theta_init`.
    pub fn new(params: ElectrodeParams, n_nodes: usize) -> Self {
        let c_init = params.theta_init * params.c_s_max;
        let dr = params.r_particle / (n_nodes - 1) as f64;
        Self {
            c_s: vec![c_init; n_nodes],
            n_nodes,
            dr,
            params,
        }
    }

    /// Surface stoichiometry θ_s = c_s[N-1] / c_s_max.
    pub fn theta_surface(&self) -> f64 {
        self.c_s[self.n_nodes - 1] / self.params.c_s_max
    }

    /// Volume-averaged stoichiometry θ_avg.
    pub fn theta_avg(&self) -> f64 {
        self.c_avg() / self.params.c_s_max
    }

    /// Volume-averaged concentration [mol/m³] via Simpson's rule.
    pub fn c_avg(&self) -> f64 {
        let n = self.n_nodes;
        let r_max = self.params.r_particle;
        let mut sum = 0.0_f64;
        let mut sum_r2 = 0.0_f64;
        for i in 0..n {
            let r = i as f64 * self.dr;
            let w = if i == 0 || i == n - 1 {
                1.0
            } else if i % 2 == 0 {
                2.0
            } else {
                4.0
            };
            sum += w * r * r * self.c_s[i];
            sum_r2 += w * r * r;
        }
        if sum_r2 < 1e-30 {
            return self.c_s[0];
        }
        3.0 * sum / (r_max * r_max * r_max) * self.dr / 3.0
    }

    /// Advance concentration profile by one time step.
    ///
    /// `j_n` — pore-wall flux [mol/(m²·s)], positive for intercalation (charging into this electrode)
    /// `dt`  — time step `s`
    /// `temp_k` — temperature `K`
    pub fn step(&mut self, j_n: f64, dt: f64, temp_k: f64) {
        let n = self.n_nodes;
        let dr = self.dr;
        let d_s = self.params.d_s(temp_k);
        let mut dc = vec![0.0_f64; n];

        // Center node (i=0): L'Hôpital → dc/dt = 6·D_s·(c[1]-c[0])/dr²
        dc[0] = 6.0 * d_s * (self.c_s[1] - self.c_s[0]) / (dr * dr);

        // Interior nodes (spherical Laplacian)
        #[allow(clippy::needless_range_loop)]
        for i in 1..n - 1 {
            let r = i as f64 * dr;
            let d2c = (self.c_s[i + 1] - 2.0 * self.c_s[i] + self.c_s[i - 1]) / (dr * dr);
            let dc_dr = (self.c_s[i + 1] - self.c_s[i - 1]) / (2.0 * dr);
            dc[i] = d_s * (d2c + 2.0 / r * dc_dr);
        }

        // Surface node (i=N-1): Neumann BC −D_s ∂c/∂r = j_n
        // Ghost node: c[N] = c[N-2] + 2·dr·j_n / D_s (backwards difference)
        let ghost = self.c_s[n - 2] + 2.0 * dr * j_n / d_s;
        let r_surf = (n - 1) as f64 * dr;
        let d2c = (ghost - 2.0 * self.c_s[n - 1] + self.c_s[n - 2]) / (dr * dr);
        let dc_dr_surf = (ghost - self.c_s[n - 2]) / (2.0 * dr);
        dc[n - 1] = d_s * (d2c + 2.0 / r_surf * dc_dr_surf);

        // Explicit Euler update with clamping
        let c_min = 0.0;
        let c_max = self.params.c_s_max;
        for (cs, dc_val) in self.c_s.iter_mut().zip(dc.iter()) {
            *cs = (*cs + dt * dc_val).clamp(c_min, c_max);
        }
    }

    /// Maximum stable time step (explicit Euler stability criterion).
    pub fn dt_stable(&self, temp_k: f64) -> f64 {
        let d_s = self.params.d_s(temp_k);
        0.5 * self.dr * self.dr / d_s
    }

    /// Electrode volume `m³`.
    pub fn volume(&self) -> f64 {
        self.params.area * self.params.thickness
    }

    /// Electrode capacity `C` = F * c_s_max * ε_s * V_electrode.
    pub fn capacity_coulombs(&self) -> f64 {
        const F: f64 = 96_485.0;
        F * self.params.c_s_max * self.params.epsilon_s * self.volume()
    }

    /// Open-circuit potential at the particle surface `V`.
    pub fn ocp_surface(&self) -> f64 {
        self.params.ocp(self.theta_surface())
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self) {
        let c_init = self.params.theta_init * self.params.c_s_max;
        self.c_s.fill(c_init);
    }
}

/// Spherical particle volume for a given radius `m³`.
pub fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_electrode_params_graphite() {
        let params = ElectrodeParams::graphite_anode();
        assert!(params.r_particle > 0.0);
        assert!(params.d_s_ref > 0.0);
        assert!(params.c_s_max > 0.0);
        let d_s_300 = params.d_s(300.0);
        let d_s_298 = params.d_s(298.15);
        assert!(d_s_300 > d_s_298, "Diffusivity increases with temperature");
    }

    #[test]
    fn test_ocp_graphite_in_range() {
        let params = ElectrodeParams::graphite_anode();
        for theta in [0.1, 0.3, 0.5, 0.7, 0.9] {
            let v = params.ocp(theta);
            assert!(
                v > 0.0 && v < 1.5,
                "Graphite OCP out of range: {:.3} V at θ={}",
                v,
                theta
            );
        }
    }

    #[test]
    fn test_ocp_lfp_plateau() {
        let params = ElectrodeParams::lfp_cathode();
        // LFP has a flat plateau around 3.4 V
        let v_mid = params.ocp(0.5);
        assert!(
            v_mid > 3.0 && v_mid < 3.8,
            "LFP OCP plateau: {:.3} V",
            v_mid
        );
    }

    #[test]
    fn test_particle_diffusion_init() {
        let params = ElectrodeParams::graphite_anode();
        let theta0 = params.theta_init;
        let particle = ParticleDiffusion::new(params, 5);
        assert!(
            (particle.theta_avg() - theta0).abs() < 0.01,
            "Initial avg θ should be theta_init"
        );
    }

    #[test]
    fn test_particle_step_reduces_conc_on_discharge() {
        let params = ElectrodeParams::graphite_anode();
        let mut particle = ParticleDiffusion::new(params, 10);
        let c_before = particle.theta_surface();
        // Discharge: negative j_n (Li leaves graphite anode = oxidation)
        let j_n = -1e-5; // mol/(m²·s)
        particle.step(j_n, particle.dt_stable(298.15) * 0.4, 298.15);
        let c_after = particle.theta_surface();
        assert!(
            c_after < c_before,
            "Surface θ should decrease on anode discharge"
        );
    }

    #[test]
    fn test_capacity_positive() {
        let params = ElectrodeParams::lfp_cathode();
        let particle = ParticleDiffusion::new(params, 5);
        assert!(
            particle.capacity_coulombs() > 1000.0,
            "Capacity should be > 1000 C"
        );
    }

    #[test]
    fn test_specific_area() {
        let params = ElectrodeParams::graphite_anode();
        let a_s = params.specific_area();
        assert!(
            a_s > 1e4,
            "Specific area should be > 10,000 m⁻¹: {:.0}",
            a_s
        );
    }
}
