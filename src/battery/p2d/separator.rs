/// Separator model for the Pseudo-2D (P2D/DFN) battery model.
///
/// The separator is an ion-permeable, electronically insulating membrane
/// between anode and cathode. It allows Li⁺ transport through the liquid
/// electrolyte but blocks electron flow.
///
/// In the P2D model the separator is a passive region — no electrochemical
/// reactions occur — and is modelled as a porous medium with given thickness,
/// porosity, and tortuosity.
use serde::{Deserialize, Serialize};

/// Physical parameters of the separator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeparatorParams {
    /// Thickness [m]
    pub thickness: f64,
    /// Electrolyte volume fraction (porosity)
    pub porosity: f64,
    /// Bruggeman exponent for effective transport (typ. 1.5)
    pub bruggeman: f64,
    /// Maximum temperature before thermal runaway risk [K]
    pub t_max_k: f64,
}

impl SeparatorParams {
    /// Celgard 2500 polypropylene membrane (25 µm, ~55 % porosity).
    pub fn celgard_2500() -> Self {
        Self {
            thickness: 25e-6,
            porosity: 0.55,
            bruggeman: 1.5,
            t_max_k: 403.15, // 130 °C
        }
    }

    /// Thinner ceramic-coated separator (16 µm, ~40 % porosity).
    pub fn ceramic_16um() -> Self {
        Self {
            thickness: 16e-6,
            porosity: 0.40,
            bruggeman: 1.5,
            t_max_k: 473.15, // 200 °C
        }
    }

    /// Effective diffusivity factor ε^bruggeman (dimensionless).
    ///
    /// Multiply the bulk electrolyte diffusivity/conductivity by this factor
    /// to get the effective value inside the separator.
    pub fn effective_transport_factor(&self) -> f64 {
        self.porosity.powf(self.bruggeman)
    }

    /// Tortuosity τ = ε^(1−bruggeman) (dimensionless).
    pub fn tortuosity(&self) -> f64 {
        self.porosity.powf(1.0 - self.bruggeman)
    }

    /// Ohmic resistance contribution of separator [Ω·m²].
    ///
    /// R_sep = L_sep / (κ_eff) = L_sep / (κ · ε^b)
    /// where κ is the electrolyte conductivity [S/m].
    pub fn resistance_area(&self, kappa_electrolyte: f64) -> f64 {
        let kappa_eff = kappa_electrolyte * self.effective_transport_factor();
        if kappa_eff < 1e-20 {
            return f64::INFINITY;
        }
        self.thickness / kappa_eff
    }

    /// Returns true if the temperature exceeds the safe operating limit.
    pub fn is_overtemp(&self, temp_k: f64) -> bool {
        temp_k > self.t_max_k
    }
}

/// Separator state (concentration and potential profile across separator).
pub struct SeparatorState {
    pub params: SeparatorParams,
    /// Li⁺ concentration at each separator node [mol/m³]
    pub c_e: Vec<f64>,
    /// Electrolyte potential at each separator node [V]
    pub phi_e: Vec<f64>,
    /// Number of nodes
    pub n_nodes: usize,
    #[allow(dead_code)]
    dx: f64,
}

impl SeparatorState {
    /// Initialise separator with uniform concentration.
    pub fn new(params: SeparatorParams, c_e_init: f64, n_nodes: usize) -> Self {
        let dx = params.thickness / n_nodes as f64;
        Self {
            c_e: vec![c_e_init; n_nodes],
            phi_e: vec![0.0; n_nodes],
            n_nodes,
            dx,
            params,
        }
    }

    /// Interpolate concentrations from boundary values using linear profile.
    ///
    /// Assumes a quasi-steady concentration profile (simplification for
    /// systems where separator diffusion is fast relative to electrode diffusion).
    pub fn set_boundary_concentrations(&mut self, c_left: f64, c_right: f64) {
        for i in 0..self.n_nodes {
            let alpha = (i as f64 + 0.5) / self.n_nodes as f64;
            self.c_e[i] = c_left + alpha * (c_right - c_left);
        }
    }

    /// Average concentration [mol/m³].
    pub fn c_avg(&self) -> f64 {
        self.c_e.iter().sum::<f64>() / self.n_nodes as f64
    }

    /// Ohmic voltage drop across separator [V] for a given current density [A/m²].
    pub fn ohmic_drop(&self, current_density_a_m2: f64, kappa_electrolyte: f64) -> f64 {
        current_density_a_m2 * self.params.resistance_area(kappa_electrolyte)
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self, c_e_init: f64) {
        self.c_e.fill(c_e_init);
        self.phi_e.fill(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_celgard_thickness() {
        let sep = SeparatorParams::celgard_2500();
        assert!((sep.thickness - 25e-6).abs() < 1e-10);
    }

    #[test]
    fn test_effective_transport_factor_lt_one() {
        let sep = SeparatorParams::celgard_2500();
        let eta = sep.effective_transport_factor();
        assert!(eta > 0.0 && eta < 1.0, "Factor should be in (0,1): {}", eta);
    }

    #[test]
    fn test_resistance_positive() {
        let sep = SeparatorParams::celgard_2500();
        let r = sep.resistance_area(1.1); // κ ≈ 1.1 S/m
        assert!(r > 0.0, "Resistance should be positive: {}", r);
    }

    #[test]
    fn test_overtemp_detection() {
        let sep = SeparatorParams::celgard_2500();
        assert!(!sep.is_overtemp(298.15));
        assert!(sep.is_overtemp(420.0)); // above 130°C
    }

    #[test]
    fn test_separator_state_init() {
        let params = SeparatorParams::celgard_2500();
        let state = SeparatorState::new(params, 1000.0, 5);
        assert_eq!(state.n_nodes, 5);
        for &c in &state.c_e {
            assert!((c - 1000.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_linear_concentration_profile() {
        let params = SeparatorParams::celgard_2500();
        let mut state = SeparatorState::new(params, 1000.0, 10);
        state.set_boundary_concentrations(900.0, 1100.0);
        // Check monotone
        for i in 1..state.n_nodes {
            assert!(state.c_e[i] > state.c_e[i - 1]);
        }
        // Average should be ≈ midpoint
        let mid = (900.0 + 1100.0) / 2.0;
        assert!((state.c_avg() - mid).abs() < 10.0);
    }

    #[test]
    fn test_ceramic_vs_celgard_resistance() {
        let kappa = 1.1;
        let sep1 = SeparatorParams::celgard_2500();
        let sep2 = SeparatorParams::ceramic_16um();
        let r1 = sep1.resistance_area(kappa);
        let r2 = sep2.resistance_area(kappa);
        // ceramic is thinner but lower porosity; both should be positive
        assert!(r1 > 0.0 && r2 > 0.0);
    }
}
