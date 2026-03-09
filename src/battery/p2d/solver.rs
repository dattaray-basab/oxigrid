/// Single Particle Model (SPM) solver — simplified P2D cell simulation.
///
/// The SPM couples one `ParticleDiffusion` per electrode with a simplified
/// electrolyte (uniform concentration) and Butler-Volmer kinetics to compute
/// cell voltage during charge/discharge.
///
/// # Physics included
/// - Solid-phase Li diffusion in anode and cathode (finite differences)
/// - Open-circuit potentials from stoichiometry
/// - Butler-Volmer overpotential at each electrode surface
/// - Ohmic drop through electrolyte (separator resistance)
///
/// # Physics omitted (relative to full DFN)
/// - Spatially varying electrolyte concentration
/// - Electrolyte potential distribution
/// - Side reactions (SEI growth handled separately in aging module)
use serde::{Deserialize, Serialize};

use super::electrode::{ElectrodeParams, ParticleDiffusion};

/// Operating mode for the SPM solver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SpmMode {
    /// Constant current [A] (positive = discharge)
    GalvanostaticDischarge,
    /// Constant current [A] (positive = charge)
    GalvanostaticCharge,
}

/// Configuration for the SPM solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpmConfig {
    /// Number of radial nodes in each particle
    pub n_nodes: usize,
    /// Minimum allowed cell voltage [V]
    pub v_min: f64,
    /// Maximum allowed cell voltage [V]
    pub v_max: f64,
    /// Faraday constant [C/mol]
    pub faraday: f64,
    /// Universal gas constant [J/(mol·K)]
    pub r_gas: f64,
    /// Electrolyte resistance (area-specific, Ω·m²) — includes separator + electrolyte
    pub r_electrolyte_ohm_m2: f64,
}

impl Default for SpmConfig {
    fn default() -> Self {
        Self {
            n_nodes: 11, // odd → even intervals → composite Simpson's rule exact
            v_min: 2.5,
            v_max: 4.2,
            faraday: 96_485.0,
            r_gas: 8.314,
            r_electrolyte_ohm_m2: 2e-4, // 0.2 mΩ·m² typical
        }
    }
}

/// Instantaneous cell state from the SPM.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SpmState {
    /// Terminal voltage [V]
    pub voltage: f64,
    /// Cell current [A] (positive = discharge)
    pub current: f64,
    /// Anode surface stoichiometry θ_neg
    pub theta_neg: f64,
    /// Cathode surface stoichiometry θ_pos
    pub theta_pos: f64,
    /// Anode average stoichiometry θ_neg_avg
    pub theta_neg_avg: f64,
    /// Cathode average stoichiometry θ_pos_avg
    pub theta_pos_avg: f64,
    /// Elapsed simulation time [s]
    pub time_s: f64,
    /// True if voltage limit was hit
    pub cutoff: bool,
}

/// Single Particle Model solver.
pub struct SpmSolver {
    pub config: SpmConfig,
    pub anode: ParticleDiffusion,
    pub cathode: ParticleDiffusion,
    /// Area of electrode [m²] (used to convert current → current density)
    pub electrode_area: f64,
    /// Simulation time [s]
    pub time_s: f64,
}

impl SpmSolver {
    /// Create an SPM for a graphite-LFP cell.
    pub fn graphite_lfp(config: SpmConfig) -> Self {
        let n = config.n_nodes;
        let anode = ParticleDiffusion::new(ElectrodeParams::graphite_anode(), n);
        let cathode = ParticleDiffusion::new(ElectrodeParams::lfp_cathode(), n);
        let area = anode.params.area;
        Self {
            config,
            anode,
            cathode,
            electrode_area: area,
            time_s: 0.0,
        }
    }

    /// Create an SPM for a graphite-NMC cell.
    pub fn graphite_nmc(config: SpmConfig) -> Self {
        let n = config.n_nodes;
        let anode = ParticleDiffusion::new(ElectrodeParams::graphite_anode(), n);
        let cathode = ParticleDiffusion::new(ElectrodeParams::nmc_cathode(), n);
        let area = anode.params.area;
        Self {
            config,
            anode,
            cathode,
            electrode_area: area,
            time_s: 0.0,
        }
    }

    /// Compute Butler-Volmer overpotential [V] given pore-wall flux j_n.
    ///
    /// j_n = i_0/(F) · [exp(α_a·F·η/RT) − exp(−α_c·F·η/RT)]
    ///
    /// Symmetric BV (α_a = α_c = 0.5):
    ///   η = (2RT/F) · arcsinh(j_n·F / (2·i_0_mol))
    ///
    /// where i_0_mol = k_bv · sqrt(c_ss · (c_smax − c_ss) · c_e) [mol/(m²·s)]
    fn bv_overpotential(&self, j_n: f64, particle: &ParticleDiffusion, temp_k: f64) -> f64 {
        let f = self.config.faraday;
        let r = self.config.r_gas;

        let theta_s = particle.theta_surface().clamp(0.01, 0.99);
        let c_ss = theta_s * particle.params.c_s_max;
        let c_smax = particle.params.c_s_max;
        // Exchange current density [mol/(m²·s)]: simplified (ignore c_e variation)
        let i_0_mol = particle.params.k_bv * (c_ss * (c_smax - c_ss) * 1000.0).sqrt();
        if i_0_mol < 1e-30 {
            return 0.0;
        }
        let arg = j_n / (2.0 * i_0_mol);
        (2.0 * r * temp_k / f) * arg.asinh()
    }

    /// Advance the cell by one time step and return state.
    ///
    /// `current_a` — applied current [A] (positive = discharge)
    /// `dt`        — time step [s]
    /// `temp_k`    — temperature [K]
    pub fn step(&mut self, current_a: f64, dt: f64, temp_k: f64) -> SpmState {
        let f = self.config.faraday;
        let area = self.electrode_area;

        // Current density [A/m²]
        let i_app = current_a / area;

        // Pore-wall molar flux [mol/(m²·s)] at each electrode
        // Discharge: Li leaves anode (j_neg < 0), enters cathode (j_pos > 0)
        let a_s_neg = self.anode.params.specific_area();
        let a_s_pos = self.cathode.params.specific_area();
        let l_neg = self.anode.params.thickness;
        let l_pos = self.cathode.params.thickness;

        // j_n = i_app / (a_s · L · F)
        let j_n_neg = -i_app / (a_s_neg * l_neg * f); // negative for discharge
        let j_n_pos = i_app / (a_s_pos * l_pos * f); // positive for discharge

        // Update solid-phase concentration profiles
        let dt_neg = self.anode.dt_stable(temp_k) * 0.45;
        let dt_pos = self.cathode.dt_stable(temp_k) * 0.45;
        let dt_use = dt.min(dt_neg).min(dt_pos);

        // Sub-step if needed
        let n_sub = (dt / dt_use).ceil() as usize;
        let dt_sub = dt / n_sub as f64;
        for _ in 0..n_sub {
            self.anode.step(j_n_neg, dt_sub, temp_k);
            self.cathode.step(j_n_pos, dt_sub, temp_k);
        }

        // OCP at particle surface
        let u_neg = self.anode.ocp_surface();
        let u_pos = self.cathode.ocp_surface();

        // Overpotentials.
        // Convention (Moura et al.): η_i = (2RT/F) arcsinh(j_n / (2·i_0))
        // where j_n > 0 = extraction (anodic), j_n < 0 = insertion (cathodic).
        // Our electrode.rs convention is j_n > 0 = insertion, so we flip signs.
        // Discharge: cathode η_pos < 0, anode η_neg > 0  →  V < OCV as expected.
        let eta_neg = -self.bv_overpotential(j_n_neg, &self.anode, temp_k);
        let eta_pos = -self.bv_overpotential(j_n_pos, &self.cathode, temp_k);

        // Ohmic drop (electrolyte + contact resistance)
        let v_ohm = i_app * self.config.r_electrolyte_ohm_m2; // [A/m²] × [Ω·m²] = V

        // Terminal voltage: V = (U_pos + η_pos) - (U_neg + η_neg) - V_ohm
        // For discharge: η_pos < 0, η_neg > 0, V_ohm > 0  → V < OCV
        let voltage = (u_pos + eta_pos) - (u_neg + eta_neg) - v_ohm * current_a.signum();

        self.time_s += dt;

        let cutoff = voltage < self.config.v_min || voltage > self.config.v_max;

        SpmState {
            voltage,
            current: current_a,
            theta_neg: self.anode.theta_surface(),
            theta_pos: self.cathode.theta_surface(),
            theta_neg_avg: self.anode.theta_avg(),
            theta_pos_avg: self.cathode.theta_avg(),
            time_s: self.time_s,
            cutoff,
        }
    }

    /// Simulate a constant-current discharge until cutoff or max_time_s.
    ///
    /// Returns a vector of states at each time step.
    pub fn simulate_discharge(
        &mut self,
        current_a: f64,
        dt: f64,
        temp_k: f64,
        max_time_s: f64,
    ) -> Vec<SpmState> {
        let mut states = Vec::new();
        let mut t = 0.0;
        while t < max_time_s {
            let state = self.step(current_a, dt, temp_k);
            states.push(state);
            t += dt;
            if state.cutoff {
                break;
            }
        }
        states
    }

    /// Open-circuit voltage (at rest): U_pos - U_neg.
    pub fn ocv(&self) -> f64 {
        self.cathode.ocp_surface() - self.anode.ocp_surface()
    }

    /// State of charge estimate from anode average stoichiometry.
    ///
    /// Normalised to [0,1] using full-charge and full-discharge stoichiometry.
    pub fn soc_estimate(&self) -> f64 {
        let theta = self.anode.theta_avg();
        let theta_100 = 0.80; // full charge
        let theta_0 = 0.20; // full discharge
        ((theta - theta_0) / (theta_100 - theta_0)).clamp(0.0, 1.0)
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self) {
        self.anode.reset();
        self.cathode.reset();
        self.time_s = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ocv_in_range() {
        let solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let ocv = solver.ocv();
        assert!(ocv > 2.5 && ocv < 4.5, "OCV out of range: {} V", ocv);
    }

    #[test]
    fn test_discharge_reduces_voltage() {
        let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let v0 = solver.ocv();
        // 1C discharge (≈ 3 A for a ~3 Ah cell)
        let state = solver.step(3.0, 1.0, 298.15);
        assert!(
            state.voltage < v0 + 0.01,
            "Voltage should drop on discharge"
        );
    }

    #[test]
    fn test_soc_initial() {
        let solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let soc = solver.soc_estimate();
        // theta_init = 0.80 → SoC ≈ 1.0
        assert!(soc > 0.9, "Initial SoC should be high: {:.3}", soc);
    }

    #[test]
    fn test_simulate_discharge_returns_states() {
        let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let states = solver.simulate_discharge(3.0, 10.0, 298.15, 60.0);
        assert!(!states.is_empty(), "Should return at least one state");
        // Voltage should generally trend downward
        if states.len() > 1 {
            assert!(states.last().unwrap().voltage <= states.first().unwrap().voltage + 0.1);
        }
    }

    #[test]
    fn test_reset_restores_ocv() {
        let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let ocv_before = solver.ocv();
        solver.simulate_discharge(3.0, 10.0, 298.15, 300.0);
        solver.reset();
        let ocv_after = solver.ocv();
        assert!(
            (ocv_after - ocv_before).abs() < 0.01,
            "OCV should be restored after reset"
        );
    }

    #[test]
    fn test_graphite_nmc_solver() {
        let mut solver = SpmSolver::graphite_nmc(SpmConfig::default());
        let ocv = solver.ocv();
        assert!(ocv > 3.0 && ocv < 5.0, "NMC OCV out of range: {} V", ocv);
        let state = solver.step(3.0, 1.0, 298.15);
        assert!(!state.cutoff || state.voltage <= solver.config.v_min + 0.01);
    }

    #[test]
    fn test_bv_zero_current_zero_overpotential() {
        let solver = SpmSolver::graphite_lfp(SpmConfig::default());
        let eta = solver.bv_overpotential(0.0, &solver.anode, 298.15);
        assert!(eta.abs() < 1e-12, "Zero flux → zero overpotential");
    }
}
