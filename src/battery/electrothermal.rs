//! Coupled electrothermal battery model.
//!
//! Simultaneously solves electrical (2nd-order RC ECM) and thermal (lumped)
//! domains with bidirectional coupling:
//! - Heat generation from electrical losses (ohmic + polarization + entropic)
//! - Temperature-dependent electrical parameters via Arrhenius relations
//!
//! Supports multiple thermal management systems (natural convection, forced air,
//! liquid cooling, PCM) and pack-level simulation with inter-cell heat transfer.

/// Universal gas constant (J/(mol*K)).
const R_GAS: f64 = 8.314;

/// OCV polynomial coefficients for a realistic flat-plateau shape.
const OCV_COEFFS: [f64; 5] = [0.05, 0.8, -0.4, 0.5, 0.05];

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Electrothermal cell model with 2nd-order RC ECM and lumped thermal.
///
/// Parameters are specified at a reference temperature and scaled via Arrhenius
/// relations during simulation.
#[derive(Debug, Clone)]
pub struct ElectrothermalCell {
    /// Nominal capacity (Ah).
    pub capacity_ah: f64,
    /// Nominal voltage (V).
    pub v_nominal: f64,
    /// Maximum voltage (V).
    pub v_max: f64,
    /// Minimum voltage (V).
    pub v_min: f64,
    /// Reference temperature for parameters (deg C), typically 25.
    pub t_ref_c: f64,
    /// R0 at reference temperature (Ohm).
    pub r0_ref: f64,
    /// R1 at reference temperature (Ohm) -- first RC pair.
    pub r1_ref: f64,
    /// C1 at reference temperature (F).
    pub c1_ref: f64,
    /// R2 at reference temperature (Ohm) -- second RC pair.
    pub r2_ref: f64,
    /// C2 at reference temperature (F).
    pub c2_ref: f64,
    /// Activation energy for R0 (J/mol) -- Arrhenius.
    pub ea_r0: f64,
    /// Activation energy for diffusion (J/mol).
    pub ea_diff: f64,
    /// Thermal mass (J/K).
    pub thermal_mass: f64,
    /// Convective thermal resistance to coolant (K/W).
    pub r_conv: f64,
    /// Cell mass (kg).
    pub mass_kg: f64,
    /// Specific heat (J/(kg*K)).
    pub cp: f64,
    /// Entropic heat coefficient dOCV/dT (V/K) -- reversible heat.
    pub entropy_coeff: f64,
}

impl ElectrothermalCell {
    /// Create a default NMC-type 18650 cell.
    pub fn default_nmc() -> Self {
        Self {
            capacity_ah: 3.0,
            v_nominal: 3.7,
            v_max: 4.2,
            v_min: 2.5,
            t_ref_c: 25.0,
            r0_ref: 0.02,
            r1_ref: 0.01,
            c1_ref: 1000.0,
            r2_ref: 0.005,
            c2_ref: 5000.0,
            ea_r0: 20000.0,
            ea_diff: 30000.0,
            thermal_mass: 50.0,
            r_conv: 10.0,
            mass_kg: 0.048,
            cp: 1000.0,
            entropy_coeff: -0.0001,
        }
    }
}

/// Thermal management system type.
#[derive(Debug, Clone)]
pub enum ThermalManagement {
    /// Natural convection (h in W/(m^2*K), area in m^2).
    NaturalConvection {
        /// Convective heat transfer coefficient (W/(m^2*K)).
        h_conv: f64,
        /// Heat transfer surface area (m^2).
        area_m2: f64,
    },
    /// Forced air cooling.
    ForcedAir {
        /// Volumetric air flow rate (m^3/s).
        flow_rate_m3_s: f64,
        /// Convective heat transfer coefficient (W/(m^2*K)).
        h_conv: f64,
        /// Heat transfer surface area (m^2).
        area_m2: f64,
    },
    /// Liquid cooling system.
    LiquidCooling {
        /// Coolant flow rate (L/min).
        flow_rate_l_min: f64,
        /// Coolant inlet temperature (deg C).
        t_coolant_in_c: f64,
        /// Coolant specific heat (J/(kg*K)).
        cp_coolant: f64,
        /// Convective heat transfer coefficient (W/(m^2*K)).
        h_conv: f64,
        /// Heat transfer surface area (m^2).
        area_m2: f64,
    },
    /// Phase change material.
    Pcm {
        /// Melting temperature (deg C).
        t_melt_c: f64,
        /// Latent heat (J/kg).
        latent_heat_j_kg: f64,
        /// PCM mass (kg).
        pcm_mass_kg: f64,
    },
}

impl ThermalManagement {
    /// Default natural convection TMS.
    pub fn default_natural() -> Self {
        Self::NaturalConvection {
            h_conv: 10.0,
            area_m2: 0.004,
        }
    }

    /// Default liquid cooling TMS.
    pub fn default_liquid() -> Self {
        Self::LiquidCooling {
            flow_rate_l_min: 2.0,
            t_coolant_in_c: 25.0,
            cp_coolant: 4186.0,
            h_conv: 500.0,
            area_m2: 0.004,
        }
    }
}

/// Instantaneous electrothermal state of a cell.
#[derive(Debug, Clone)]
pub struct ElectrothermalState {
    /// State of charge [0, 1].
    pub soc: f64,
    /// Terminal voltage (V).
    pub v_terminal: f64,
    /// Open circuit voltage (V).
    pub v_ocv: f64,
    /// Ohmic voltage drop (V).
    pub v_r0: f64,
    /// First RC pair voltage (V).
    pub v_rc1: f64,
    /// Second RC pair voltage (V).
    pub v_rc2: f64,
    /// Applied current (A). Positive = discharge.
    pub current_a: f64,
    /// Cell temperature (deg C).
    pub temperature_c: f64,
    /// Total heat generation (W).
    pub q_gen_w: f64,
    /// Irreversible heat (I^2 R) (W).
    pub q_irrev_w: f64,
    /// Reversible (entropic) heat (W).
    pub q_rev_w: f64,
    /// Heat removed by TMS (W).
    pub q_removed_w: f64,
    /// Simulation time (s).
    pub time_s: f64,
}

impl ElectrothermalState {
    /// Create an initial state from configuration parameters.
    fn initial(soc: f64, temp_c: f64, v_min: f64, v_max: f64) -> Self {
        let v_ocv = ElectrothermalSimulator::ocv(soc, v_min, v_max);
        Self {
            soc,
            v_terminal: v_ocv,
            v_ocv,
            v_r0: 0.0,
            v_rc1: 0.0,
            v_rc2: 0.0,
            current_a: 0.0,
            temperature_c: temp_c,
            q_gen_w: 0.0,
            q_irrev_w: 0.0,
            q_rev_w: 0.0,
            q_removed_w: 0.0,
            time_s: 0.0,
        }
    }
}

/// Simulation configuration.
#[derive(Debug, Clone)]
pub struct EtSimConfig {
    /// Time step (s). Default 1.0.
    pub dt_s: f64,
    /// Total simulation duration (s).
    pub duration_s: f64,
    /// Ambient temperature (deg C).
    pub ambient_temp_c: f64,
    /// Initial state of charge [0, 1].
    pub initial_soc: f64,
    /// Initial cell temperature (deg C).
    pub initial_temp_c: f64,
}

impl Default for EtSimConfig {
    fn default() -> Self {
        Self {
            dt_s: 1.0,
            duration_s: 3600.0,
            ambient_temp_c: 25.0,
            initial_soc: 0.8,
            initial_temp_c: 25.0,
        }
    }
}

/// A single segment of a current profile.
#[derive(Debug, Clone)]
pub struct CurrentStep {
    /// Start time (s).
    pub start_s: f64,
    /// End time (s).
    pub end_s: f64,
    /// Applied current (A). Positive = discharge.
    pub current_a: f64,
}

/// Drive cycle for EV applications.
#[derive(Debug, Clone)]
pub struct DriveCycle {
    /// Cycle name (e.g. "WLTP", "US06").
    pub name: String,
    /// Power / current steps composing the cycle.
    pub power_steps: Vec<CurrentStep>,
}

/// Aggregated result of an electrothermal simulation.
#[derive(Debug, Clone)]
pub struct EtSimResult {
    /// Time-series of cell states.
    pub states: Vec<ElectrothermalState>,
    /// Peak cell temperature (deg C).
    pub max_temp_c: f64,
    /// Minimum cell temperature (deg C).
    pub min_temp_c: f64,
    /// Total electrical energy (Wh). Positive = discharged.
    pub energy_wh: f64,
    /// Total heat generated (J).
    pub total_heat_j: f64,
    /// Thermal efficiency (%), defined as 1 - Q_total / E_total.
    pub thermal_efficiency_pct: f64,
    /// Total heat removed by TMS (J).
    pub tms_energy_removed_j: f64,
}

/// Pack-level electrothermal model.
///
/// Models `n_series * n_parallel` cells with individual thermal states and
/// optional inter-cell heat conduction.
#[derive(Debug, Clone)]
pub struct ElectrothermalPack {
    /// Individual cell models (one per physical cell).
    pub cells: Vec<ElectrothermalCell>,
    /// Number of cells in series.
    pub n_series: usize,
    /// Number of parallel strings.
    pub n_parallel: usize,
    /// Thermal management system shared by pack.
    pub tms: ThermalManagement,
    /// Inter-cell thermal resistance (K/W).
    pub r_cell_to_cell: f64,
}

/// Pack-level simulation result.
#[derive(Debug, Clone)]
pub struct PackEtResult {
    /// Per-cell simulation results.
    pub cell_results: Vec<EtSimResult>,
    /// Pack terminal voltage at each time step.
    pub pack_voltage: Vec<f64>,
    /// Pack current at each time step.
    pub pack_current: Vec<f64>,
    /// Maximum temperature across all cells (deg C).
    pub max_cell_temp_c: f64,
    /// Minimum temperature across all cells (deg C).
    pub min_cell_temp_c: f64,
    /// Temperature spread across cells (deg C).
    pub temp_spread_c: f64,
    /// Total pack energy (Wh).
    pub pack_energy_wh: f64,
}

/// Electrothermal simulator for a single cell.
///
/// Couples a 2nd-order RC equivalent circuit model with a lumped thermal model.
/// Uses RK4 integration for both electrical and thermal states.
#[derive(Debug, Clone)]
pub struct ElectrothermalSimulator {
    /// Cell model parameters.
    pub cell: ElectrothermalCell,
    /// Thermal management system.
    pub tms: ThermalManagement,
    /// Simulation configuration.
    pub config: EtSimConfig,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl ElectrothermalSimulator {
    /// Create a new electrothermal simulator.
    pub fn new(cell: ElectrothermalCell, tms: ThermalManagement, config: EtSimConfig) -> Self {
        Self { cell, tms, config }
    }

    /// Compute open-circuit voltage from SoC using polynomial model.
    ///
    /// `OCV(SoC) = v_min + (v_max - v_min) * P(SoC)` where P is a 4th-order
    /// polynomial producing a realistic flat-plateau shape.
    pub fn ocv(soc: f64, v_min: f64, v_max: f64) -> f64 {
        let s = soc.clamp(0.0, 1.0);
        let a = &OCV_COEFFS;
        let poly = a[0] + a[1] * s + a[2] * s * s + a[3] * s * s * s + a[4] * s * s * s * s;
        v_min + (v_max - v_min) * poly
    }

    /// Arrhenius scaling factor for temperature-dependent parameters.
    ///
    /// Returns `exp(Ea/R * (1/T - 1/T_ref))` where T is in Kelvin.
    pub fn arrhenius_factor(ea: f64, t_c: f64, t_ref_c: f64) -> f64 {
        let t_k = t_c + 273.15;
        let t_ref_k = t_ref_c + 273.15;
        if t_k <= 0.0 || t_ref_k <= 0.0 {
            return 1.0;
        }
        let exponent = ea / R_GAS * (1.0 / t_k - 1.0 / t_ref_k);
        // Clamp exponent to avoid overflow
        exponent.clamp(-50.0, 50.0).exp()
    }

    /// Compute temperature-dependent ECM parameters.
    fn temp_dependent_params(&self, t_c: f64) -> (f64, f64, f64, f64, f64) {
        let c = &self.cell;
        let arr_r0 = Self::arrhenius_factor(c.ea_r0, t_c, c.t_ref_c);
        let arr_diff = Self::arrhenius_factor(c.ea_diff, t_c, c.t_ref_c);
        let r0 = c.r0_ref * arr_r0;
        let r1 = c.r1_ref * arr_diff;
        let c1 = c.c1_ref / arr_diff; // inverse relation
        let r2 = c.r2_ref * arr_diff;
        let c2 = c.c2_ref / arr_diff;
        (r0, r1, c1, r2, c2)
    }

    /// Compute heat generation (irreversible, reversible) in watts.
    fn heat_generation(&self, state: &ElectrothermalState, temp_c: f64) -> (f64, f64) {
        let (r0, r1, _, r2, _) = self.temp_dependent_params(temp_c);
        let i = state.current_a;

        // Irreversible: I^2*R0 + V_rc1^2/R1 + V_rc2^2/R2
        let q_r0 = i * i * r0;
        let q_rc1 = if r1.abs() > 1e-15 {
            state.v_rc1 * state.v_rc1 / r1
        } else {
            0.0
        };
        let q_rc2 = if r2.abs() > 1e-15 {
            state.v_rc2 * state.v_rc2 / r2
        } else {
            0.0
        };
        let q_irrev = q_r0 + q_rc1 + q_rc2;

        // Reversible: I * T * dOCV/dT
        let t_k = temp_c + 273.15;
        let q_rev = i * t_k * self.cell.entropy_coeff;

        (q_irrev, q_rev)
    }

    /// Compute heat removed by TMS (W) given cell temperature.
    fn tms_heat_removal(&self, temp_c: f64) -> f64 {
        match &self.tms {
            ThermalManagement::NaturalConvection { h_conv, area_m2 } => {
                let dt = temp_c - self.config.ambient_temp_c;
                h_conv * area_m2 * dt
            }
            ThermalManagement::ForcedAir {
                flow_rate_m3_s: _,
                h_conv,
                area_m2,
            } => {
                let dt = temp_c - self.config.ambient_temp_c;
                h_conv * area_m2 * dt
            }
            ThermalManagement::LiquidCooling {
                flow_rate_l_min: _,
                t_coolant_in_c,
                cp_coolant: _,
                h_conv,
                area_m2,
            } => {
                let dt = temp_c - t_coolant_in_c;
                h_conv * area_m2 * dt
            }
            ThermalManagement::Pcm {
                t_melt_c,
                latent_heat_j_kg,
                pcm_mass_kg,
            } => {
                // PCM absorbs heat when temperature is at or above melt point
                if temp_c >= *t_melt_c {
                    // Rate limited by latent heat capacity; approximate as
                    // proportional removal near melt point
                    let capacity = latent_heat_j_kg * pcm_mass_kg;
                    let dt = temp_c - t_melt_c;
                    // Simple model: removal proportional to superheat,
                    // limited by total PCM capacity spread over duration
                    let q_max = capacity / self.config.duration_s.max(1.0);
                    let q = dt * self.cell.mass_kg * self.cell.cp * 0.1;
                    q.min(q_max).max(0.0)
                } else {
                    0.0
                }
            }
        }
    }

    /// Advance one time step using RK4 integration.
    ///
    /// Returns the new [`ElectrothermalState`] after `dt` seconds.
    fn step(&self, state: &ElectrothermalState, current_a: f64, dt: f64) -> ElectrothermalState {
        // State vector: [soc, v_rc1, v_rc2, temperature_c]
        let y0 = [state.soc, state.v_rc1, state.v_rc2, state.temperature_c];

        let derivatives = |y: [f64; 4], i: f64| -> [f64; 4] {
            let _soc = y[0].clamp(0.0, 1.0);
            let v_rc1 = y[1];
            let v_rc2 = y[2];
            let temp = y[3];

            let (r0, r1, c1, r2, c2) = self.temp_dependent_params(temp);

            // Electrical dynamics
            let tau1 = r1 * c1;
            let tau2 = r2 * c2;

            let dsoc_dt = -i / (3600.0 * self.cell.capacity_ah);
            let dv_rc1_dt = if tau1.abs() > 1e-15 {
                -v_rc1 / tau1 + i / c1.max(1e-12)
            } else {
                0.0
            };
            let dv_rc2_dt = if tau2.abs() > 1e-15 {
                -v_rc2 / tau2 + i / c2.max(1e-12)
            } else {
                0.0
            };

            // Heat generation
            let q_r0 = i * i * r0;
            let q_rc1 = if r1.abs() > 1e-15 {
                v_rc1 * v_rc1 / r1
            } else {
                0.0
            };
            let q_rc2 = if r2.abs() > 1e-15 {
                v_rc2 * v_rc2 / r2
            } else {
                0.0
            };
            let q_irrev = q_r0 + q_rc1 + q_rc2;
            let t_k = temp + 273.15;
            let q_rev = i * t_k * self.cell.entropy_coeff;
            let q_total = q_irrev + q_rev;

            // TMS removal (inline to avoid &self borrow issues)
            let q_removed = match &self.tms {
                ThermalManagement::NaturalConvection { h_conv, area_m2 } => {
                    h_conv * area_m2 * (temp - self.config.ambient_temp_c)
                }
                ThermalManagement::ForcedAir {
                    h_conv, area_m2, ..
                } => h_conv * area_m2 * (temp - self.config.ambient_temp_c),
                ThermalManagement::LiquidCooling {
                    t_coolant_in_c,
                    h_conv,
                    area_m2,
                    ..
                } => h_conv * area_m2 * (temp - t_coolant_in_c),
                ThermalManagement::Pcm {
                    t_melt_c,
                    latent_heat_j_kg,
                    pcm_mass_kg,
                } => {
                    if temp >= *t_melt_c {
                        let capacity = latent_heat_j_kg * pcm_mass_kg;
                        let dt_pcm = temp - t_melt_c;
                        let q_max = capacity / self.config.duration_s.max(1.0);
                        let q = dt_pcm * self.cell.mass_kg * self.cell.cp * 0.1;
                        q.min(q_max).max(0.0)
                    } else {
                        0.0
                    }
                }
            };

            // Thermal dynamics: m*cp * dT/dt = Q_total - Q_removed
            let thermal_cap = self.cell.mass_kg * self.cell.cp;
            let dt_dt = if thermal_cap > 1e-12 {
                (q_total - q_removed) / thermal_cap
            } else {
                0.0
            };

            [dsoc_dt, dv_rc1_dt, dv_rc2_dt, dt_dt]
        };

        // RK4 integration
        let k1 = derivatives(y0, current_a);

        let y1: [f64; 4] = [
            y0[0] + 0.5 * dt * k1[0],
            y0[1] + 0.5 * dt * k1[1],
            y0[2] + 0.5 * dt * k1[2],
            y0[3] + 0.5 * dt * k1[3],
        ];
        let k2 = derivatives(y1, current_a);

        let y2: [f64; 4] = [
            y0[0] + 0.5 * dt * k2[0],
            y0[1] + 0.5 * dt * k2[1],
            y0[2] + 0.5 * dt * k2[2],
            y0[3] + 0.5 * dt * k2[3],
        ];
        let k3 = derivatives(y2, current_a);

        let y3: [f64; 4] = [
            y0[0] + dt * k3[0],
            y0[1] + dt * k3[1],
            y0[2] + dt * k3[2],
            y0[3] + dt * k3[3],
        ];
        let k4 = derivatives(y3, current_a);

        let new_soc =
            (y0[0] + dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])).clamp(0.0, 1.0);
        let new_vrc1 = y0[1] + dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
        let new_vrc2 = y0[2] + dt / 6.0 * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);
        let new_temp = y0[3] + dt / 6.0 * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);

        let (r0, _, _, _, _) = self.temp_dependent_params(new_temp);
        let v_ocv = Self::ocv(new_soc, self.cell.v_min, self.cell.v_max);
        let v_r0_drop = current_a * r0;
        let v_terminal = v_ocv - v_r0_drop - new_vrc1 - new_vrc2;

        let (q_irrev, q_rev) = self.heat_generation(
            &ElectrothermalState {
                soc: new_soc,
                v_terminal,
                v_ocv,
                v_r0: v_r0_drop,
                v_rc1: new_vrc1,
                v_rc2: new_vrc2,
                current_a,
                temperature_c: new_temp,
                q_gen_w: 0.0,
                q_irrev_w: 0.0,
                q_rev_w: 0.0,
                q_removed_w: 0.0,
                time_s: 0.0,
            },
            new_temp,
        );
        let q_gen = q_irrev + q_rev;
        let q_removed = self.tms_heat_removal(new_temp);

        ElectrothermalState {
            soc: new_soc,
            v_terminal,
            v_ocv,
            v_r0: v_r0_drop,
            v_rc1: new_vrc1,
            v_rc2: new_vrc2,
            current_a,
            temperature_c: new_temp,
            q_gen_w: q_gen,
            q_irrev_w: q_irrev,
            q_rev_w: q_rev,
            q_removed_w: q_removed,
            time_s: state.time_s + dt,
        }
    }

    /// Simulate constant-current discharge (or charge if negative).
    pub fn simulate_constant_current(&self, current_a: f64) -> Result<EtSimResult, String> {
        if self.config.dt_s <= 0.0 {
            return Err("Time step must be positive".into());
        }
        if self.config.duration_s <= 0.0 {
            return Err("Duration must be positive".into());
        }

        let mut state = ElectrothermalState::initial(
            self.config.initial_soc,
            self.config.initial_temp_c,
            self.cell.v_min,
            self.cell.v_max,
        );
        let n_steps = (self.config.duration_s / self.config.dt_s).ceil() as usize;
        let mut states = Vec::with_capacity(n_steps + 1);
        states.push(state.clone());

        for _ in 0..n_steps {
            state = self.step(&state, current_a, self.config.dt_s);
            // Stop if SoC limits hit
            if state.soc <= 0.0 || state.soc >= 1.0 {
                states.push(state.clone());
                break;
            }
            states.push(state.clone());
        }

        Self::build_result(states, self.config.dt_s)
    }

    /// Simulate a piecewise-constant current profile.
    pub fn simulate_profile(&self, profile: &[CurrentStep]) -> Result<EtSimResult, String> {
        if self.config.dt_s <= 0.0 {
            return Err("Time step must be positive".into());
        }
        if profile.is_empty() {
            return Err("Profile must not be empty".into());
        }

        let mut state = ElectrothermalState::initial(
            self.config.initial_soc,
            self.config.initial_temp_c,
            self.cell.v_min,
            self.cell.v_max,
        );
        let mut states = Vec::new();
        states.push(state.clone());

        for step in profile {
            if step.end_s <= step.start_s {
                continue;
            }
            let n_sub = ((step.end_s - step.start_s) / self.config.dt_s).ceil() as usize;
            for _ in 0..n_sub {
                state = self.step(&state, step.current_a, self.config.dt_s);
                states.push(state.clone());
                if state.soc <= 0.0 || state.soc >= 1.0 {
                    break;
                }
            }
        }

        Self::build_result(states, self.config.dt_s)
    }

    /// Simulate a drive cycle.
    pub fn simulate_drive_cycle(&self, cycle: &DriveCycle) -> Result<EtSimResult, String> {
        self.simulate_profile(&cycle.power_steps)
    }

    /// Build aggregated result from state time series.
    fn build_result(states: Vec<ElectrothermalState>, dt_s: f64) -> Result<EtSimResult, String> {
        if states.is_empty() {
            return Err("No states produced".into());
        }

        let mut max_temp = f64::NEG_INFINITY;
        let mut min_temp = f64::INFINITY;
        let mut total_energy_ws: f64 = 0.0;
        let mut total_heat_j: f64 = 0.0;
        let mut tms_removed_j: f64 = 0.0;

        for s in &states {
            if s.temperature_c > max_temp {
                max_temp = s.temperature_c;
            }
            if s.temperature_c < min_temp {
                min_temp = s.temperature_c;
            }
            // Energy: P = V * I, integrate
            total_energy_ws += s.v_terminal * s.current_a * dt_s;
            total_heat_j += s.q_gen_w * dt_s;
            tms_removed_j += s.q_removed_w * dt_s;
        }

        let energy_wh = total_energy_ws.abs() / 3600.0;
        let thermal_eff = if total_energy_ws.abs() > 1e-12 {
            (1.0 - total_heat_j.abs() / total_energy_ws.abs()) * 100.0
        } else {
            100.0
        };

        Ok(EtSimResult {
            states,
            max_temp_c: max_temp,
            min_temp_c: min_temp,
            energy_wh,
            total_heat_j,
            thermal_efficiency_pct: thermal_eff.clamp(0.0, 100.0),
            tms_energy_removed_j: tms_removed_j,
        })
    }
}

// ---------------------------------------------------------------------------
// Pack-level simulation
// ---------------------------------------------------------------------------

impl ElectrothermalPack {
    /// Create a new electrothermal pack model.
    pub fn new(
        cells: Vec<ElectrothermalCell>,
        n_series: usize,
        n_parallel: usize,
        tms: ThermalManagement,
    ) -> Self {
        Self {
            cells,
            n_series,
            n_parallel,
            tms,
            r_cell_to_cell: 5.0,
        }
    }

    /// Simulate the full pack with a current profile.
    ///
    /// Pack current is split equally among parallel strings. All series cells
    /// in a string carry the same current. Inter-cell heat transfer is modeled
    /// at each time step.
    pub fn simulate(
        &self,
        profile: &[CurrentStep],
        config: &EtSimConfig,
    ) -> Result<PackEtResult, String> {
        if self.cells.is_empty() {
            return Err("Pack has no cells".into());
        }
        if config.dt_s <= 0.0 {
            return Err("Time step must be positive".into());
        }
        if profile.is_empty() {
            return Err("Profile must not be empty".into());
        }

        let n_cells = self.cells.len();
        let cell_current_factor = if self.n_parallel > 0 {
            1.0 / self.n_parallel as f64
        } else {
            1.0
        };

        // Initialize per-cell simulators and states
        let simulators: Vec<ElectrothermalSimulator> = self
            .cells
            .iter()
            .map(|c| ElectrothermalSimulator::new(c.clone(), self.tms.clone(), config.clone()))
            .collect();

        let mut cell_states: Vec<ElectrothermalState> = simulators
            .iter()
            .map(|sim| {
                ElectrothermalState::initial(
                    sim.config.initial_soc,
                    sim.config.initial_temp_c,
                    sim.cell.v_min,
                    sim.cell.v_max,
                )
            })
            .collect();

        // Per-cell state histories
        let mut cell_histories: Vec<Vec<ElectrothermalState>> =
            (0..n_cells).map(|i| vec![cell_states[i].clone()]).collect();
        let mut pack_voltages: Vec<f64> = Vec::new();
        let mut pack_currents: Vec<f64> = Vec::new();

        // Compute initial pack voltage
        let init_pack_v: f64 = cell_states
            .iter()
            .take(self.n_series.min(n_cells))
            .map(|s| s.v_terminal)
            .sum();
        pack_voltages.push(init_pack_v);
        pack_currents.push(0.0);

        for step_def in profile {
            if step_def.end_s <= step_def.start_s {
                continue;
            }
            let cell_current = step_def.current_a * cell_current_factor;
            let n_sub = ((step_def.end_s - step_def.start_s) / config.dt_s).ceil() as usize;

            for _ in 0..n_sub {
                // Step each cell
                for (idx, sim) in simulators.iter().enumerate() {
                    cell_states[idx] = sim.step(&cell_states[idx], cell_current, config.dt_s);
                }

                // Inter-cell heat transfer (simple pairwise between adjacent cells)
                if n_cells > 1 && self.r_cell_to_cell > 1e-12 {
                    let mut delta_temps = vec![0.0f64; n_cells];
                    for i in 0..n_cells - 1 {
                        let dt_ij = cell_states[i].temperature_c - cell_states[i + 1].temperature_c;
                        let q_transfer = dt_ij / self.r_cell_to_cell;
                        let cap_i = self.cells[i].mass_kg * self.cells[i].cp;
                        let cap_j = self.cells[i + 1].mass_kg * self.cells[i + 1].cp;
                        if cap_i > 1e-12 {
                            delta_temps[i] -= q_transfer * config.dt_s / cap_i;
                        }
                        if cap_j > 1e-12 {
                            delta_temps[i + 1] += q_transfer * config.dt_s / cap_j;
                        }
                    }
                    for (i, dt) in delta_temps.iter().enumerate() {
                        cell_states[i].temperature_c += dt;
                    }
                }

                // Record
                for (idx, s) in cell_states.iter().enumerate() {
                    cell_histories[idx].push(s.clone());
                }

                let pack_v: f64 = cell_states
                    .iter()
                    .take(self.n_series.min(n_cells))
                    .map(|s| s.v_terminal)
                    .sum();
                pack_voltages.push(pack_v);
                pack_currents.push(step_def.current_a);
            }
        }

        // Build per-cell results
        let mut cell_results = Vec::with_capacity(n_cells);
        for history in cell_histories {
            cell_results.push(ElectrothermalSimulator::build_result(history, config.dt_s)?);
        }

        let max_cell_temp = cell_results
            .iter()
            .map(|r| r.max_temp_c)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_cell_temp = cell_results
            .iter()
            .map(|r| r.min_temp_c)
            .fold(f64::INFINITY, f64::min);
        let temp_spread = max_cell_temp - min_cell_temp;
        let pack_energy_wh: f64 = cell_results.iter().map(|r| r.energy_wh).sum();

        Ok(PackEtResult {
            cell_results,
            pack_voltage: pack_voltages,
            pack_current: pack_currents,
            max_cell_temp_c: max_cell_temp,
            min_cell_temp_c: min_cell_temp,
            temp_spread_c: temp_spread,
            pack_energy_wh,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_cell() -> ElectrothermalCell {
        ElectrothermalCell::default_nmc()
    }

    fn default_sim(current_dur_s: f64) -> ElectrothermalSimulator {
        ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig {
                dt_s: 1.0,
                duration_s: current_dur_s,
                ambient_temp_c: 25.0,
                initial_soc: 0.8,
                initial_temp_c: 25.0,
            },
        )
    }

    // 1. Constant discharge: SoC decreases over time
    #[test]
    fn test_constant_discharge_soc_decreases() {
        let sim = default_sim(60.0);
        let res = sim.simulate_constant_current(1.0).expect("sim failed");
        let first_soc = res.states.first().map(|s| s.soc).unwrap_or(1.0);
        let last_soc = res.states.last().map(|s| s.soc).unwrap_or(1.0);
        assert!(last_soc < first_soc, "SoC should decrease during discharge");
    }

    // 2. Constant discharge: temperature increases
    #[test]
    fn test_constant_discharge_temp_increases() {
        let sim = default_sim(120.0);
        let res = sim.simulate_constant_current(3.0).expect("sim failed");
        let first_t = res.states.first().map(|s| s.temperature_c).unwrap_or(25.0);
        let last_t = res.states.last().map(|s| s.temperature_c).unwrap_or(25.0);
        assert!(
            last_t > first_t,
            "Temperature should increase during discharge: {} vs {}",
            last_t,
            first_t
        );
    }

    // 3. V_terminal < OCV during discharge
    #[test]
    fn test_vterminal_lt_ocv_discharge() {
        let sim = default_sim(10.0);
        let res = sim.simulate_constant_current(2.0).expect("sim failed");
        for s in &res.states[1..] {
            assert!(
                s.v_terminal < s.v_ocv + 1e-9,
                "V_terminal should be < OCV during discharge"
            );
        }
    }

    // 4. V_terminal > OCV during charge (negative current)
    #[test]
    fn test_vterminal_gt_ocv_charge() {
        let sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig {
                dt_s: 1.0,
                duration_s: 10.0,
                ambient_temp_c: 25.0,
                initial_soc: 0.5,
                initial_temp_c: 25.0,
            },
        );
        let res = sim.simulate_constant_current(-2.0).expect("sim failed");
        for s in &res.states[1..] {
            assert!(
                s.v_terminal > s.v_ocv - 1e-9,
                "V_terminal should be > OCV during charge"
            );
        }
    }

    // 5. R0 increases at lower temperature (Arrhenius)
    #[test]
    fn test_r0_increases_at_low_temp() {
        let cell = default_cell();
        let factor_cold = ElectrothermalSimulator::arrhenius_factor(cell.ea_r0, 0.0, cell.t_ref_c);
        let factor_ref = ElectrothermalSimulator::arrhenius_factor(cell.ea_r0, 25.0, cell.t_ref_c);
        assert!(
            factor_cold > factor_ref,
            "R0 factor should be larger at low temperature"
        );
    }

    // 6. Heat generation: Q > 0 during discharge
    #[test]
    fn test_heat_gen_positive_discharge() {
        let sim = default_sim(30.0);
        let res = sim.simulate_constant_current(3.0).expect("sim failed");
        assert!(res.total_heat_j > 0.0, "Total heat should be positive");
    }

    // 7. Energy: energy_wh > 0 for discharge
    #[test]
    fn test_energy_positive_discharge() {
        let sim = default_sim(60.0);
        let res = sim.simulate_constant_current(1.0).expect("sim failed");
        assert!(
            res.energy_wh > 0.0,
            "Energy should be positive for discharge"
        );
    }

    // 8. Thermal efficiency: > 0 and < 100
    #[test]
    fn test_thermal_efficiency_range() {
        let sim = default_sim(60.0);
        let res = sim.simulate_constant_current(2.0).expect("sim failed");
        assert!(
            res.thermal_efficiency_pct > 0.0 && res.thermal_efficiency_pct < 100.0,
            "Thermal efficiency should be between 0 and 100: {}",
            res.thermal_efficiency_pct
        );
    }

    // 9. OCV(0) ~ V_min, OCV(1) ~ V_max
    #[test]
    fn test_ocv_endpoints() {
        let cell = default_cell();
        let ocv_0 = ElectrothermalSimulator::ocv(0.0, cell.v_min, cell.v_max);
        let ocv_1 = ElectrothermalSimulator::ocv(1.0, cell.v_min, cell.v_max);
        // OCV(0) = v_min + (v_max - v_min) * a0 = v_min + range*0.05
        // OCV(1) = v_min + (v_max - v_min) * sum(a) = v_min + range*1.0
        let range = cell.v_max - cell.v_min;
        assert!(
            (ocv_0 - (cell.v_min + range * 0.05)).abs() < 1e-6,
            "OCV(0) should be near v_min"
        );
        let sum_a: f64 = OCV_COEFFS.iter().sum();
        assert!(
            (ocv_1 - (cell.v_min + range * sum_a)).abs() < 1e-6,
            "OCV(1) should be near v_max"
        );
    }

    // 10. RC voltage builds up during constant current
    #[test]
    fn test_rc_voltage_builds_up() {
        let sim = default_sim(30.0);
        let res = sim.simulate_constant_current(3.0).expect("sim failed");
        let last = res.states.last().expect("empty states");
        assert!(
            last.v_rc1.abs() > 1e-6,
            "RC1 voltage should build up: {}",
            last.v_rc1
        );
    }

    // 11. RC voltage decays during rest
    #[test]
    fn test_rc_voltage_decays_at_rest() {
        // First discharge to build up RC voltages
        let sim = default_sim(30.0);
        let res = sim.simulate_constant_current(3.0).expect("sim failed");
        let after_discharge = res.states.last().expect("empty states");
        let vrc1_after = after_discharge.v_rc1;

        // Now rest
        let rest_sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig {
                dt_s: 1.0,
                duration_s: 60.0,
                ambient_temp_c: 25.0,
                initial_soc: after_discharge.soc,
                initial_temp_c: after_discharge.temperature_c,
            },
        );
        // Start with RC voltages already built up
        let mut state = after_discharge.clone();
        state.time_s = 0.0;
        let mut final_state = state.clone();
        for _ in 0..60 {
            final_state = rest_sim.step(&final_state, 0.0, 1.0);
        }
        assert!(
            final_state.v_rc1.abs() < vrc1_after.abs(),
            "RC1 voltage should decay during rest: {} vs {}",
            final_state.v_rc1,
            vrc1_after
        );
    }

    // 12. TMS natural convection: removes heat proportional to dT
    #[test]
    fn test_tms_natural_convection() {
        let sim = default_sim(60.0);
        let q1 = sim.tms_heat_removal(30.0); // dT = 5
        let q2 = sim.tms_heat_removal(35.0); // dT = 10
        assert!(q2 > q1, "More heat removal at higher dT");
        // Should be proportional
        assert!((q2 / q1 - 2.0).abs() < 0.01, "Should be proportional to dT");
    }

    // 13. TMS liquid cooling: Q_removed > natural convection
    #[test]
    fn test_liquid_better_than_natural() {
        let natural = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig::default(),
        );
        let liquid = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_liquid(),
            EtSimConfig::default(),
        );
        let q_nat = natural.tms_heat_removal(35.0);
        let q_liq = liquid.tms_heat_removal(35.0);
        assert!(
            q_liq > q_nat,
            "Liquid cooling should remove more heat: {} vs {}",
            q_liq,
            q_nat
        );
    }

    // 14. TMS PCM: absorbs heat near melt point
    #[test]
    fn test_pcm_absorbs_at_melt() {
        let pcm_sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::Pcm {
                t_melt_c: 30.0,
                latent_heat_j_kg: 200000.0,
                pcm_mass_kg: 0.05,
            },
            EtSimConfig {
                duration_s: 100.0,
                ..EtSimConfig::default()
            },
        );
        let q_below = pcm_sim.tms_heat_removal(28.0);
        let q_above = pcm_sim.tms_heat_removal(32.0);
        assert!(q_below < 1e-12, "No PCM absorption below melt point");
        assert!(q_above > 0.0, "PCM should absorb heat above melt point");
    }

    // 15. Forced air: higher h_conv -> more cooling
    #[test]
    fn test_forced_air_more_cooling() {
        let low_h = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::ForcedAir {
                flow_rate_m3_s: 0.01,
                h_conv: 25.0,
                area_m2: 0.004,
            },
            EtSimConfig::default(),
        );
        let high_h = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::ForcedAir {
                flow_rate_m3_s: 0.05,
                h_conv: 100.0,
                area_m2: 0.004,
            },
            EtSimConfig::default(),
        );
        assert!(
            high_h.tms_heat_removal(35.0) > low_h.tms_heat_removal(35.0),
            "Higher h_conv should give more cooling"
        );
    }

    // 16. Pack: n_series * cell_voltage ~ pack_voltage
    #[test]
    fn test_pack_voltage_series() {
        let n_s = 4;
        let cells: Vec<ElectrothermalCell> = (0..n_s).map(|_| default_cell()).collect();
        let pack = ElectrothermalPack::new(cells, n_s, 1, ThermalManagement::default_natural());
        let profile = vec![CurrentStep {
            start_s: 0.0,
            end_s: 10.0,
            current_a: 1.0,
        }];
        let config = EtSimConfig {
            dt_s: 1.0,
            duration_s: 10.0,
            ..EtSimConfig::default()
        };
        let res = pack.simulate(&profile, &config).expect("pack sim failed");

        // At t=0, pack voltage should be n_s * cell OCV
        let cell_v = ElectrothermalSimulator::ocv(0.8, default_cell().v_min, default_cell().v_max);
        let expected = n_s as f64 * cell_v;
        let pack_v0 = res.pack_voltage.first().copied().unwrap_or(0.0);
        assert!(
            (pack_v0 - expected).abs() < 0.1,
            "Pack voltage {} should be close to {} ({}*{})",
            pack_v0,
            expected,
            n_s,
            cell_v
        );
    }

    // 17. Pack: temp_spread ~ 0 when all cells identical
    #[test]
    fn test_pack_temp_spread_identical() {
        let n_s = 3;
        let cells: Vec<ElectrothermalCell> = (0..n_s).map(|_| default_cell()).collect();
        let pack = ElectrothermalPack::new(cells, n_s, 1, ThermalManagement::default_natural());
        let profile = vec![CurrentStep {
            start_s: 0.0,
            end_s: 30.0,
            current_a: 2.0,
        }];
        let config = EtSimConfig {
            dt_s: 1.0,
            duration_s: 30.0,
            ..EtSimConfig::default()
        };
        let res = pack.simulate(&profile, &config).expect("pack sim failed");
        assert!(
            res.temp_spread_c < 1.0,
            "Temp spread should be small for identical cells: {}",
            res.temp_spread_c
        );
    }

    // 18. Zero current: no heat generation, temp constant
    #[test]
    fn test_zero_current_no_heat() {
        let sim = default_sim(30.0);
        let res = sim.simulate_constant_current(0.0).expect("sim failed");
        let last = res.states.last().expect("empty states");
        assert!(
            (last.temperature_c - 25.0).abs() < 0.01,
            "Temperature should be constant with zero current: {}",
            last.temperature_c
        );
        assert!(
            res.total_heat_j.abs() < 1e-6,
            "No heat should be generated with zero current"
        );
    }

    // 19. High current: faster SoC change
    #[test]
    fn test_high_current_faster_soc() {
        let sim_low = default_sim(60.0);
        let sim_high = default_sim(60.0);
        let res_low = sim_low.simulate_constant_current(1.0).expect("sim failed");
        let res_high = sim_high.simulate_constant_current(5.0).expect("sim failed");
        let dsoc_low = res_low.states.first().map(|s| s.soc).unwrap_or(0.8)
            - res_low.states.last().map(|s| s.soc).unwrap_or(0.8);
        let dsoc_high = res_high.states.first().map(|s| s.soc).unwrap_or(0.8)
            - res_high.states.last().map(|s| s.soc).unwrap_or(0.8);
        assert!(
            dsoc_high > dsoc_low,
            "Higher current should drain SoC faster"
        );
    }

    // 20. Low temperature: higher ohmic loss
    #[test]
    fn test_low_temp_higher_loss() {
        let cold_sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig {
                dt_s: 1.0,
                duration_s: 30.0,
                ambient_temp_c: 0.0,
                initial_soc: 0.8,
                initial_temp_c: 0.0,
            },
        );
        let warm_sim = default_sim(30.0);
        let cold_res = cold_sim.simulate_constant_current(3.0).expect("sim failed");
        let warm_res = warm_sim.simulate_constant_current(3.0).expect("sim failed");
        assert!(
            cold_res.total_heat_j > warm_res.total_heat_j,
            "Cold cell should have higher losses: {} vs {}",
            cold_res.total_heat_j,
            warm_res.total_heat_j
        );
    }

    // 21. Drive cycle: non-empty result
    #[test]
    fn test_drive_cycle_nonempty() {
        let sim = default_sim(100.0);
        let cycle = DriveCycle {
            name: "test_cycle".to_string(),
            power_steps: vec![
                CurrentStep {
                    start_s: 0.0,
                    end_s: 20.0,
                    current_a: 5.0,
                },
                CurrentStep {
                    start_s: 20.0,
                    end_s: 40.0,
                    current_a: 0.0,
                },
                CurrentStep {
                    start_s: 40.0,
                    end_s: 60.0,
                    current_a: -3.0,
                },
            ],
        };
        let res = sim
            .simulate_drive_cycle(&cycle)
            .expect("drive cycle failed");
        assert!(!res.states.is_empty(), "Drive cycle should produce states");
        assert!(res.states.len() > 3, "Should have multiple time steps");
    }

    // 22. Profile simulation: multiple current steps work
    #[test]
    fn test_profile_multi_step() {
        let sim = default_sim(100.0);
        let profile = vec![
            CurrentStep {
                start_s: 0.0,
                end_s: 30.0,
                current_a: 2.0,
            },
            CurrentStep {
                start_s: 30.0,
                end_s: 60.0,
                current_a: -1.0,
            },
        ];
        let res = sim.simulate_profile(&profile).expect("profile failed");
        assert!(res.states.len() > 50, "Should have enough time steps");
    }

    // 23. Arrhenius factor = 1.0 at T = T_ref
    #[test]
    fn test_arrhenius_at_ref() {
        let factor = ElectrothermalSimulator::arrhenius_factor(20000.0, 25.0, 25.0);
        assert!(
            (factor - 1.0).abs() < 1e-12,
            "Arrhenius factor at T_ref should be 1.0: {}",
            factor
        );
    }

    // 24. Max temp bounded by energy input
    #[test]
    fn test_max_temp_bounded() {
        let cell = default_cell();
        let sim = ElectrothermalSimulator::new(
            cell.clone(),
            ThermalManagement::NaturalConvection {
                h_conv: 0.0, // no cooling
                area_m2: 0.004,
            },
            EtSimConfig {
                dt_s: 1.0,
                duration_s: 60.0,
                ambient_temp_c: 25.0,
                initial_soc: 0.8,
                initial_temp_c: 25.0,
            },
        );
        let res = sim.simulate_constant_current(3.0).expect("sim failed");
        // Without cooling, all heat goes to temperature rise
        // Max temp rise = Q_total / (m * cp)
        let max_rise = res.total_heat_j / (cell.mass_kg * cell.cp);
        let actual_rise = res.max_temp_c - 25.0;
        // Actual rise should be close to but not exceed the theoretical max
        // (some numerical tolerance due to RK4 and heat computation timing)
        assert!(
            actual_rise <= max_rise + 1.0,
            "Temp rise {} should not greatly exceed theoretical max {}",
            actual_rise,
            max_rise
        );
    }

    // 25. SoC stays in [0, 1]
    #[test]
    fn test_soc_clamped() {
        let sim = default_sim(7200.0); // 2 hours at high discharge
        let res = sim.simulate_constant_current(5.0).expect("sim failed");
        for s in &res.states {
            assert!(
                s.soc >= 0.0 && s.soc <= 1.0,
                "SoC should be clamped: {}",
                s.soc
            );
        }
    }

    // 26. Charge increases SoC
    #[test]
    fn test_charge_increases_soc() {
        let sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig {
                dt_s: 1.0,
                duration_s: 60.0,
                ambient_temp_c: 25.0,
                initial_soc: 0.3,
                initial_temp_c: 25.0,
            },
        );
        let res = sim.simulate_constant_current(-2.0).expect("sim failed");
        let first_soc = res.states.first().map(|s| s.soc).unwrap_or(0.3);
        let last_soc = res.states.last().map(|s| s.soc).unwrap_or(0.3);
        assert!(last_soc > first_soc, "SoC should increase during charge");
    }

    // 27. OCV is monotonically increasing
    #[test]
    fn test_ocv_monotonic() {
        let cell = default_cell();
        let mut prev = ElectrothermalSimulator::ocv(0.0, cell.v_min, cell.v_max);
        for i in 1..=100 {
            let soc = i as f64 / 100.0;
            let v = ElectrothermalSimulator::ocv(soc, cell.v_min, cell.v_max);
            assert!(v >= prev - 1e-10, "OCV should be monotonic at SoC={}", soc);
            prev = v;
        }
    }

    // 28. EtSimConfig default produces valid simulation
    #[test]
    fn test_default_config() {
        let sim = ElectrothermalSimulator::new(
            default_cell(),
            ThermalManagement::default_natural(),
            EtSimConfig::default(),
        );
        let res = sim.simulate_constant_current(1.0).expect("sim failed");
        assert!(!res.states.is_empty());
        assert!(res.max_temp_c >= 25.0);
    }
}
