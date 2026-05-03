/// Electrolyte transport model for the Pseudo-2D (P2D/DFN) battery model.
///
/// Models lithium-ion concentration and electric potential in the liquid
/// electrolyte phase across anode, separator, and cathode regions.
///
/// # Governing equations
/// Species conservation (dilute solution theory):
///   ε ∂c_e/∂t = ∂/∂x(D_e_eff ∂c_e/∂x) + (1-t⁺) j_n·a_s
///
/// Ohm's law in electrolyte:
///   i_e = −κ_eff ∂φ_e/∂x + 2κ_eff RT/F (1+∂lnf/∂lnc)(1-t⁺) ∂ln(c_e)/∂x
///
/// # Reference
/// Doyle, Fuller, Newman (1993) J. Electrochem. Soc. 140(6).
use serde::{Deserialize, Serialize};

/// Electrolyte material parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectrolyteParams {
    /// Ionic diffusivity at reference concentration [m²/s]
    pub d_e_ref: f64,
    /// Reference ionic conductivity [S/m]
    pub kappa_ref: f64,
    /// Transference number of Li⁺ (dimensionless, 0–1)
    pub t_plus: f64,
    /// Bruggeman exponent for effective transport
    pub bruggeman: f64,
    /// Initial (uniform) electrolyte concentration [mol/m³]
    pub c_e_init: f64,
    /// Activation energy for diffusivity [J/mol]
    pub e_a_de: f64,
    /// Activation energy for conductivity [J/mol]
    pub e_a_kappa: f64,
}

impl ElectrolyteParams {
    /// Default 1M LiPF6 in EC/DMC electrolyte.
    pub fn lipf6_ec_dmc() -> Self {
        Self {
            d_e_ref: 7.5e-11, // m²/s at 298 K
            kappa_ref: 1.1,   // S/m at 298 K, c_e = 1000 mol/m³
            t_plus: 0.364,
            bruggeman: 1.5,
            c_e_init: 1000.0, // mol/m³ = 1M
            e_a_de: 17_000.0,
            e_a_kappa: 12_000.0,
        }
    }

    /// Temperature-corrected diffusivity [m²/s].
    pub fn d_e(&self, temp_k: f64) -> f64 {
        const R_GAS: f64 = 8.314;
        const T_REF: f64 = 298.15;
        self.d_e_ref * (self.e_a_de / R_GAS * (1.0 / T_REF - 1.0 / temp_k)).exp()
    }

    /// Temperature-corrected conductivity [S/m].
    pub fn kappa(&self, temp_k: f64) -> f64 {
        const R_GAS: f64 = 8.314;
        const T_REF: f64 = 298.15;
        self.kappa_ref * (self.e_a_kappa / R_GAS * (1.0 / T_REF - 1.0 / temp_k)).exp()
    }

    /// Effective diffusivity in a porous medium: D_eff = D_e · ε^bruggeman.
    pub fn d_e_eff(&self, porosity: f64, temp_k: f64) -> f64 {
        self.d_e(temp_k) * porosity.powf(self.bruggeman)
    }

    /// Effective conductivity: κ_eff = κ · ε^bruggeman.
    pub fn kappa_eff(&self, porosity: f64, temp_k: f64) -> f64 {
        self.kappa(temp_k) * porosity.powf(self.bruggeman)
    }
}

/// Electrolyte concentration profile across the cell [mol/m³].
///
/// The cell is discretised into N_neg + N_sep + N_pos nodes.
pub struct ElectrolyteState {
    /// Electrolyte parameters
    pub params: ElectrolyteParams,
    /// Li concentration at each node [mol/m³]
    pub c_e: Vec<f64>,
    /// Electrolyte potential at each node `V`
    pub phi_e: Vec<f64>,
    /// Node count in anode region
    pub n_neg: usize,
    /// Node count in separator region
    pub n_sep: usize,
    /// Node count in cathode region
    pub n_pos: usize,
    /// Node width (uniform) `m`
    dx_neg: f64,
    dx_sep: f64,
    dx_pos: f64,
    /// Porosity in each region
    eps_neg: f64,
    eps_sep: f64,
    eps_pos: f64,
}

impl ElectrolyteState {
    /// Create uniform initial state.
    ///
    /// Lengths in `m`; porosities dimensionless.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        params: ElectrolyteParams,
        l_neg: f64,
        l_sep: f64,
        l_pos: f64,
        eps_neg: f64,
        eps_sep: f64,
        eps_pos: f64,
        n_neg: usize,
        n_sep: usize,
        n_pos: usize,
    ) -> Self {
        let n_total = n_neg + n_sep + n_pos;
        let c_init = params.c_e_init;
        Self {
            c_e: vec![c_init; n_total],
            phi_e: vec![0.0; n_total],
            params,
            n_neg,
            n_sep,
            n_pos,
            dx_neg: l_neg / n_neg as f64,
            dx_sep: l_sep / n_sep as f64,
            dx_pos: l_pos / n_pos as f64,
            eps_neg,
            eps_sep,
            eps_pos,
        }
    }

    /// Total number of nodes.
    pub fn n_total(&self) -> usize {
        self.n_neg + self.n_sep + self.n_pos
    }

    /// Average electrolyte concentration [mol/m³].
    pub fn c_avg(&self) -> f64 {
        self.c_e.iter().sum::<f64>() / self.c_e.len() as f64
    }

    /// Advance concentration profile by one time step (explicit Euler).
    ///
    /// `j_n_neg` — volumetric reaction current in anode [mol/(m³·s)]
    /// `j_n_pos` — volumetric reaction current in cathode [mol/(m³·s)]
    pub fn step_concentration(&mut self, j_n_neg: f64, j_n_pos: f64, dt: f64, temp_k: f64) {
        let n = self.n_total();
        let mut dc = vec![0.0f64; n];

        let (d_neg, d_sep, d_pos) = (
            self.params.d_e_eff(self.eps_neg, temp_k),
            self.params.d_e_eff(self.eps_sep, temp_k),
            self.params.d_e_eff(self.eps_pos, temp_k),
        );

        // Source term coefficient: (1 - t⁺)
        let src_coeff = 1.0 - self.params.t_plus;

        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            let (eps, dx, d_eff, src) = if i < self.n_neg {
                (self.eps_neg, self.dx_neg, d_neg, src_coeff * j_n_neg)
            } else if i < self.n_neg + self.n_sep {
                (self.eps_sep, self.dx_sep, d_sep, 0.0)
            } else {
                (self.eps_pos, self.dx_pos, d_pos, src_coeff * j_n_pos)
            };

            // Finite difference Laplacian (zero-flux BCs at ends)
            let c_left = if i == 0 { self.c_e[0] } else { self.c_e[i - 1] };
            let c_right = if i == n - 1 {
                self.c_e[n - 1]
            } else {
                self.c_e[i + 1]
            };
            let d2c = (c_left - 2.0 * self.c_e[i] + c_right) / (dx * dx);
            dc[i] = (d_eff * d2c + src) / eps;
        }

        for (ce, dc_val) in self.c_e.iter_mut().zip(dc.iter()) {
            *ce = (*ce + dt * dc_val).max(1e-10);
        }
    }

    /// Maximum stable time step for concentration update (explicit stability).
    pub fn dt_stable(&self, temp_k: f64) -> f64 {
        let d_max = self
            .params
            .d_e_eff(self.eps_neg.min(self.eps_sep).min(self.eps_pos), temp_k);
        let dx_min = self.dx_neg.min(self.dx_sep).min(self.dx_pos);
        0.5 * dx_min * dx_min / d_max
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self) {
        let c_init = self.params.c_e_init;
        self.c_e.fill(c_init);
        self.phi_e.fill(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_state() -> ElectrolyteState {
        let params = ElectrolyteParams::lipf6_ec_dmc();
        ElectrolyteState::new(
            params, 100e-6, 25e-6, 80e-6, // l_neg, l_sep, l_pos
            0.30, 0.40, 0.30, // porosities
            10, 5, 8, // nodes
        )
    }

    #[test]
    fn test_initial_concentration_uniform() {
        let state = default_state();
        let c0 = state.params.c_e_init;
        for &c in &state.c_e {
            assert!((c - c0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_n_total() {
        let state = default_state();
        assert_eq!(state.n_total(), 23);
    }

    #[test]
    fn test_arrhenius_diffusivity() {
        let p = ElectrolyteParams::lipf6_ec_dmc();
        let d_300 = p.d_e(300.0);
        let d_298 = p.d_e(298.15);
        assert!(d_300 > d_298, "D_e should increase with temperature");
    }

    #[test]
    fn test_effective_diffusivity_less_than_bulk() {
        let p = ElectrolyteParams::lipf6_ec_dmc();
        let eps = 0.30;
        assert!(p.d_e_eff(eps, 298.15) < p.d_e(298.15));
    }

    #[test]
    fn test_dt_stable_positive() {
        let state = default_state();
        assert!(state.dt_stable(298.15) > 0.0);
    }

    #[test]
    fn test_step_concentration_conserves_mass() {
        let mut state = default_state();
        let c_before: f64 = state.c_e.iter().sum();
        // Zero source terms → mass conserved (to numerical precision)
        let dt = state.dt_stable(298.15) * 0.4;
        state.step_concentration(0.0, 0.0, dt, 298.15);
        let c_after: f64 = state.c_e.iter().sum();
        assert!((c_after - c_before).abs() < 1e-6 * c_before.abs() + 1e-10);
    }

    #[test]
    fn test_reset_restores_initial() {
        let mut state = default_state();
        let dt = state.dt_stable(298.15) * 0.4;
        state.step_concentration(1e-4, -1e-4, dt, 298.15);
        state.reset();
        let c0 = state.params.c_e_init;
        for &c in &state.c_e {
            assert!((c - c0).abs() < 1e-10);
        }
    }
}
