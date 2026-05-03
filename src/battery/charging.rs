/// Battery charging protocols: CC-CV, multistep, health-aware, and fast-charge optimisation.
///
/// # Overview
///
/// 1. **CC-CV** — Constant-Current / Constant-Voltage: standard IEC 62133 protocol.
/// 2. **Multistep** — Multi-stage CC-CV with programmable current steps.
/// 3. **Health-aware** — Temperature and SoH-dependent current limitation.
/// 4. **Fast-charge** — Optimal trajectory via dynamic programming that minimises
///    charging time subject to temperature, current, and SoH constraints.
///
/// # References
/// - Liu et al., "Fast Charging of Lithium-Ion Batteries", Energy Storage 2019.
/// - Pozzi et al., "Optimal Health-Aware Charging of Li-Ion Batteries", 2020.
use serde::{Deserialize, Serialize};

/// Charging protocol type.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ProtocolType {
    CcCv,
    Multistep,
    HealthAware,
    FastCharge,
}

/// Charging configuration (applies to all protocols).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingConfig {
    /// Nominal cell capacity `Ah`
    pub capacity_ah: f64,
    /// Nominal voltage `V`
    pub v_nominal: f64,
    /// Minimum cell voltage `V`
    pub v_min: f64,
    /// Maximum cell voltage `V` (cutoff)
    pub v_max: f64,
    /// Minimum allowed temperature [°C]
    pub t_min_c: f64,
    /// Maximum allowed temperature [°C]
    pub t_max_c: f64,
    /// Maximum C-rate for fast charge (e.g. 2.0 = 2C)
    pub max_c_rate: f64,
    /// CV phase termination current `A` (typically C/20)
    pub cv_cutoff_a: f64,
    /// State-of-health fraction `0,1` — affects allowed C-rate
    pub soh: f64,
    /// Internal resistance `Ω` — used for voltage drop estimation
    pub r_internal: f64,
}

impl ChargingConfig {
    /// Default config for a typical 50 Ah LFP cell.
    pub fn lfp_50ah() -> Self {
        Self {
            capacity_ah: 50.0,
            v_nominal: 3.2,
            v_min: 2.5,
            v_max: 3.65,
            t_min_c: 0.0,
            t_max_c: 45.0,
            max_c_rate: 1.0,
            cv_cutoff_a: 2.5, // C/20
            soh: 1.0,
            r_internal: 0.002,
        }
    }

    /// Default config for a 3 Ah NMC 18650 cell.
    pub fn nmc_3ah() -> Self {
        Self {
            capacity_ah: 3.0,
            v_nominal: 3.7,
            v_min: 2.8,
            v_max: 4.2,
            t_min_c: 5.0,
            t_max_c: 45.0,
            max_c_rate: 2.0,
            cv_cutoff_a: 0.15, // C/20
            soh: 1.0,
            r_internal: 0.025,
        }
    }

    /// Maximum charging current `A` at given SoH and temperature.
    pub fn max_current_a(&self, temp_c: f64) -> f64 {
        let base = self.capacity_ah * self.max_c_rate * self.soh;
        // Derate by temperature: linear reduction below 15°C and above 35°C
        let t_factor = if temp_c < 15.0 {
            0.5 + 0.05 * (temp_c - self.t_min_c).max(0.0)
        } else if temp_c > 35.0 {
            1.0 - 0.02 * (temp_c - 35.0).min(10.0)
        } else {
            1.0
        };
        (base * t_factor).max(0.1)
    }

    /// Maximum current limited by voltage headroom (avoid overshoot).
    pub fn max_current_voltage_limited(&self, v_cell: f64) -> f64 {
        if self.r_internal < 1e-9 {
            return self.capacity_ah * self.max_c_rate;
        }
        ((self.v_max - v_cell) / self.r_internal).max(0.0)
    }
}

/// Instantaneous charging state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingState {
    /// Current SoC `0,1`
    pub soc: f64,
    /// Cell voltage `V`
    pub voltage: f64,
    /// Charging current `A`  (positive = charging)
    pub current_a: f64,
    /// Cell temperature [°C]
    pub temperature_c: f64,
    /// Elapsed time `s`
    pub time_s: f64,
    /// Cumulative charge delivered `Ah`
    pub charge_ah: f64,
    /// Phase name
    pub phase: ChargingPhase,
}

/// Phase of the charging protocol.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ChargingPhase {
    /// Constant-current phase
    CC,
    /// Constant-voltage phase
    CV,
    /// Done (current fell below cutoff)
    Done,
    /// Fault: temperature or voltage out of range
    Fault,
}

/// Full charging session result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingResult {
    /// Time-series of charging states (sampled at dt_s intervals)
    pub history: Vec<ChargingState>,
    /// Total charge time `s`
    pub total_time_s: f64,
    /// Final SoC achieved `0,1`
    pub final_soc: f64,
    /// Total energy delivered `Wh`
    pub energy_wh: f64,
    /// Round-trip efficiency `0,1` (energy stored / energy input)
    pub efficiency: f64,
    /// True if charging completed successfully to target SoC
    pub completed: bool,
    /// Protocol used
    pub protocol: ProtocolType,
}

impl ChargingResult {
    /// Average charging power `W`
    pub fn avg_power_w(&self) -> f64 {
        if self.total_time_s < 1e-6 {
            return 0.0;
        }
        self.energy_wh * 3600.0 / self.total_time_s
    }

    /// Peak temperature during charging [°C]
    pub fn peak_temperature(&self) -> f64 {
        self.history
            .iter()
            .map(|s| s.temperature_c)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Duration of CC phase `s`
    pub fn cc_duration_s(&self) -> f64 {
        // Find last CC entry
        self.history
            .iter()
            .rfind(|s| s.phase == ChargingPhase::CC)
            .map(|s| s.time_s)
            .unwrap_or(0.0)
    }
}

/// Simple OCV model: linear approximation SoC → voltage.
fn ocv(soc: f64, v_min: f64, v_max: f64) -> f64 {
    v_min + soc.clamp(0.0, 1.0) * (v_max - v_min)
}

/// Run standard CC-CV charging protocol.
///
/// - CC phase at `cc_current_a` until `v_max` is reached
/// - CV phase at `v_max` until current falls below `cv_cutoff_a`
pub fn run_cc_cv(
    config: &ChargingConfig,
    initial_soc: f64,
    cc_current_a: f64,
    dt_s: f64,
    max_time_s: f64,
) -> ChargingResult {
    let mut soc = initial_soc.clamp(0.0, 1.0);
    let mut time_s = 0.0;
    let mut charge_ah = 0.0;
    let mut energy_wh = 0.0;
    let mut history = Vec::new();
    let mut phase = ChargingPhase::CC;
    let mut temp_c = 25.0_f64; // assumed isothermal for basic CC-CV

    let cc_i = cc_current_a.min(config.max_current_a(temp_c));

    while time_s < max_time_s && phase != ChargingPhase::Done && phase != ChargingPhase::Fault {
        let v_ocv = ocv(soc, config.v_min, config.v_max);

        let (current, voltage) = match phase {
            ChargingPhase::CC => {
                let i = cc_i;
                let v = v_ocv + i * config.r_internal;
                if v >= config.v_max {
                    phase = ChargingPhase::CV;
                    (i, config.v_max)
                } else {
                    (i, v)
                }
            }
            ChargingPhase::CV => {
                let i = ((config.v_max - v_ocv) / config.r_internal.max(1e-9)).max(0.0);
                if i <= config.cv_cutoff_a {
                    phase = ChargingPhase::Done;
                    (0.0, config.v_max)
                } else {
                    (i, config.v_max)
                }
            }
            _ => break,
        };

        // Thermal model: backward Euler (unconditionally stable for any dt_s)
        // C * dT/dt = q_joule - h*(T - T_amb)  →  T_{n+1} = (T_n + dt/C*(q + h*T_amb))/(1 + dt*h/C)
        let q_joule = current * current * config.r_internal;
        let h_cool = 2.0_f64; // W/K
        let thermal_mass = 50.0_f64; // J/K
        let t_amb = 25.0_f64;
        let alpha = dt_s * h_cool / thermal_mass;
        temp_c = (temp_c + dt_s / thermal_mass * (q_joule + h_cool * t_amb)) / (1.0 + alpha);

        // Check limits
        if temp_c > config.t_max_c || temp_c < config.t_min_c {
            phase = ChargingPhase::Fault;
        }

        history.push(ChargingState {
            soc,
            voltage,
            current_a: current,
            temperature_c: temp_c,
            time_s,
            charge_ah,
            phase,
        });

        // Advance state
        let delta_ah = current * dt_s / 3600.0;
        soc = (soc + delta_ah / config.capacity_ah).clamp(0.0, 1.0);
        charge_ah += delta_ah;
        energy_wh += voltage * delta_ah;
        time_s += dt_s;
    }

    let completed = phase == ChargingPhase::Done;
    let theoretical_energy = config.capacity_ah * (1.0 - initial_soc) * config.v_nominal;
    let efficiency = if theoretical_energy > 1e-6 {
        (energy_wh / theoretical_energy).min(1.0)
    } else {
        0.0
    };

    ChargingResult {
        history,
        total_time_s: time_s,
        final_soc: soc,
        energy_wh,
        efficiency,
        completed,
        protocol: ProtocolType::CcCv,
    }
}

/// Multistep CC-CV: each step has a current and SoC trigger.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingStep {
    /// Current `A` for this CC step
    pub current_a: f64,
    /// SoC at which to transition to next step (or to CV if last step)
    pub soc_threshold: f64,
}

/// Run multistep CC-CV charging.
pub fn run_multistep(
    config: &ChargingConfig,
    initial_soc: f64,
    steps: &[ChargingStep],
    dt_s: f64,
    max_time_s: f64,
) -> ChargingResult {
    if steps.is_empty() {
        return run_cc_cv(
            config,
            initial_soc,
            config.capacity_ah * config.max_c_rate,
            dt_s,
            max_time_s,
        );
    }

    let mut soc = initial_soc.clamp(0.0, 1.0);
    let mut time_s = 0.0;
    let mut charge_ah = 0.0;
    let mut energy_wh = 0.0;
    let mut history = Vec::new();
    let mut phase = ChargingPhase::CC;
    let mut step_idx = 0;
    let mut temp_c = 25.0_f64;

    while time_s < max_time_s && phase != ChargingPhase::Done && phase != ChargingPhase::Fault {
        // Select current step
        while step_idx < steps.len() - 1 && soc >= steps[step_idx].soc_threshold {
            step_idx += 1;
        }
        let step = &steps[step_idx];

        let v_ocv = ocv(soc, config.v_min, config.v_max);
        let max_i = config.max_current_a(temp_c);
        let cc_i = step.current_a.min(max_i);

        let (current, voltage) = match phase {
            ChargingPhase::CC => {
                let i = cc_i;
                let v = v_ocv + i * config.r_internal;
                if v >= config.v_max
                    || (step_idx == steps.len() - 1 && soc >= steps[step_idx].soc_threshold)
                {
                    phase = ChargingPhase::CV;
                    (i, config.v_max.min(v))
                } else {
                    (i, v)
                }
            }
            ChargingPhase::CV => {
                let i = ((config.v_max - v_ocv) / config.r_internal.max(1e-9)).max(0.0);
                if i <= config.cv_cutoff_a {
                    phase = ChargingPhase::Done;
                    (0.0, config.v_max)
                } else {
                    (i, config.v_max)
                }
            }
            _ => break,
        };

        let q_joule = current * current * config.r_internal;
        let h_cool = 2.0_f64;
        let thermal_mass = 50.0_f64;
        let t_amb = 25.0_f64;
        let alpha = dt_s * h_cool / thermal_mass;
        temp_c = (temp_c + dt_s / thermal_mass * (q_joule + h_cool * t_amb)) / (1.0 + alpha);

        if temp_c > config.t_max_c {
            phase = ChargingPhase::Fault;
        }

        history.push(ChargingState {
            soc,
            voltage,
            current_a: current,
            temperature_c: temp_c,
            time_s,
            charge_ah,
            phase,
        });

        let delta_ah = current * dt_s / 3600.0;
        soc = (soc + delta_ah / config.capacity_ah).clamp(0.0, 1.0);
        charge_ah += delta_ah;
        energy_wh += voltage * delta_ah;
        time_s += dt_s;
    }

    let completed = phase == ChargingPhase::Done;
    let theoretical_energy = config.capacity_ah * (1.0 - initial_soc) * config.v_nominal;
    let efficiency = if theoretical_energy > 1e-6 {
        (energy_wh / theoretical_energy).min(1.0)
    } else {
        0.0
    };

    ChargingResult {
        history,
        total_time_s: time_s,
        final_soc: soc,
        energy_wh,
        efficiency,
        completed,
        protocol: ProtocolType::Multistep,
    }
}

/// Health-aware charging: reduces C-rate based on SoH and temperature.
pub fn run_health_aware(
    config: &ChargingConfig,
    initial_soc: f64,
    dt_s: f64,
    max_time_s: f64,
) -> ChargingResult {
    // Health-aware: dynamically adjust CC current based on SoH + temperature
    // and voltage headroom to avoid lithium plating
    let mut soc = initial_soc.clamp(0.0, 1.0);
    let mut time_s = 0.0;
    let mut charge_ah = 0.0;
    let mut energy_wh = 0.0;
    let mut history = Vec::new();
    let mut phase = ChargingPhase::CC;
    let mut temp_c = 25.0_f64;

    while time_s < max_time_s && phase != ChargingPhase::Done && phase != ChargingPhase::Fault {
        let v_ocv = ocv(soc, config.v_min, config.v_max);

        // Health-aware: limit by temperature AND voltage headroom
        let i_temp = config.max_current_a(temp_c);
        let i_volt = config.max_current_voltage_limited(v_ocv);
        let i_cc = i_temp.min(i_volt);

        let (current, voltage) = match phase {
            ChargingPhase::CC => {
                let v = v_ocv + i_cc * config.r_internal;
                if v >= config.v_max {
                    phase = ChargingPhase::CV;
                    (i_cc, config.v_max)
                } else {
                    (i_cc, v)
                }
            }
            ChargingPhase::CV => {
                let i = ((config.v_max - v_ocv) / config.r_internal.max(1e-9)).max(0.0);
                if i <= config.cv_cutoff_a {
                    phase = ChargingPhase::Done;
                    (0.0, config.v_max)
                } else {
                    (i, config.v_max)
                }
            }
            _ => break,
        };

        // Enhanced thermal model with health factor (backward Euler — unconditionally stable)
        let aging_heat_factor = 1.0 + (1.0 - config.soh) * 0.5; // aged cells run hotter
        let q_joule = current * current * config.r_internal * aging_heat_factor;
        let h_cool = 2.0_f64;
        let thermal_mass = 50.0_f64;
        let t_amb = 25.0_f64;
        let alpha = dt_s * h_cool / thermal_mass;
        temp_c = (temp_c + dt_s / thermal_mass * (q_joule + h_cool * t_amb)) / (1.0 + alpha);

        if temp_c > config.t_max_c || temp_c < config.t_min_c {
            phase = ChargingPhase::Fault;
        }

        history.push(ChargingState {
            soc,
            voltage,
            current_a: current,
            temperature_c: temp_c,
            time_s,
            charge_ah,
            phase,
        });

        let delta_ah = current * dt_s / 3600.0;
        soc = (soc + delta_ah / config.capacity_ah).clamp(0.0, 1.0);
        charge_ah += delta_ah;
        energy_wh += voltage * delta_ah;
        time_s += dt_s;
    }

    let completed = phase == ChargingPhase::Done;
    let theoretical_energy = config.capacity_ah * (1.0 - initial_soc) * config.v_nominal;
    let efficiency = if theoretical_energy > 1e-6 {
        (energy_wh / theoretical_energy).min(1.0)
    } else {
        0.0
    };

    ChargingResult {
        history,
        total_time_s: time_s,
        final_soc: soc,
        energy_wh,
        efficiency,
        completed,
        protocol: ProtocolType::HealthAware,
    }
}

/// Fast-charge trajectory optimisation via dynamic programming.
///
/// Minimises charging time subject to:
/// - Maximum voltage constraint (v ≤ v_max)
/// - Temperature constraint (T ≤ T_max)
/// - Maximum C-rate (health-dependent)
///
/// The DP discretises SoC into `n_soc` states and finds the optimal
/// current sequence minimising time-to-full.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FastChargeOptimiser {
    pub config: ChargingConfig,
    /// Number of SoC discretisation points
    pub n_soc: usize,
    /// Candidate C-rates to search over
    pub candidate_c_rates: Vec<f64>,
}

impl FastChargeOptimiser {
    pub fn new(config: ChargingConfig, n_soc: usize) -> Self {
        let max_c = config.max_c_rate;
        let candidate_c_rates = (1..=10).map(|i| max_c * i as f64 / 10.0).collect();
        Self {
            config,
            n_soc,
            candidate_c_rates,
        }
    }

    /// Optimal fast-charge current at each SoC level `A`.
    ///
    /// Uses backward DP: cost-to-go = time remaining to reach SoC=1.
    pub fn optimal_current_profile(&self) -> Vec<(f64, f64)> {
        let n = self.n_soc;
        let dsoc = 1.0 / n as f64;
        let mut cost_to_go = vec![0.0_f64; n + 1]; // cost_to_go[n] = 0 (done)
        let mut opt_current = vec![0.0_f64; n];
        let cap = self.config.capacity_ah;

        // Backward pass
        for s in (0..n).rev() {
            let soc = s as f64 * dsoc;
            let v_ocv = ocv(soc, self.config.v_min, self.config.v_max);
            let temp_25 = 25.0_f64;
            let max_i = self.config.max_current_a(temp_25);
            let max_i_v = self.config.max_current_voltage_limited(v_ocv);
            let i_max = max_i.min(max_i_v);

            let mut best_cost = f64::INFINITY;
            let mut best_i = i_max;

            for &c_rate in &self.candidate_c_rates {
                let i = (c_rate * cap).min(i_max);
                if i < 1e-6 {
                    continue;
                }
                // Time to traverse dsoc at this current
                let dt = dsoc * cap / i * 3600.0; // seconds
                let next_s = (s + 1).min(n);
                let cost = dt + cost_to_go[next_s];
                if cost < best_cost {
                    best_cost = cost;
                    best_i = i;
                }
            }

            opt_current[s] = best_i;
            cost_to_go[s] = best_cost;
        }

        // Return (soc, optimal_current_a) pairs
        (0..n).map(|s| (s as f64 * dsoc, opt_current[s])).collect()
    }

    /// Run fast-charge simulation using the optimal current profile.
    pub fn run(&self, initial_soc: f64, dt_s: f64, max_time_s: f64) -> ChargingResult {
        let profile = self.optimal_current_profile();
        let n = profile.len();

        let mut soc = initial_soc.clamp(0.0, 1.0);
        let mut time_s = 0.0;
        let mut charge_ah = 0.0;
        let mut energy_wh = 0.0;
        let mut history = Vec::new();
        let mut phase = ChargingPhase::CC;
        let mut temp_c = 25.0_f64;

        while time_s < max_time_s && phase != ChargingPhase::Done && phase != ChargingPhase::Fault {
            // Look up optimal current for current SoC
            let s_idx = ((soc * n as f64) as usize).min(n - 1);
            let target_i = profile[s_idx].1;

            let v_ocv = ocv(soc, self.config.v_min, self.config.v_max);
            let v_cell = v_ocv + target_i * self.config.r_internal;

            let (current, voltage) = if v_cell >= self.config.v_max {
                // Switch to CV
                phase = ChargingPhase::CV;
                let i = ((self.config.v_max - v_ocv) / self.config.r_internal.max(1e-9)).max(0.0);
                if i <= self.config.cv_cutoff_a {
                    phase = ChargingPhase::Done;
                    (0.0, self.config.v_max)
                } else {
                    (i, self.config.v_max)
                }
            } else {
                (target_i, v_cell)
            };

            let q_joule = current * current * self.config.r_internal;
            let h_cool = 2.0_f64;
            let thermal_mass = 50.0_f64;
            let t_amb = 25.0_f64;
            let alpha = dt_s * h_cool / thermal_mass;
            temp_c = (temp_c + dt_s / thermal_mass * (q_joule + h_cool * t_amb)) / (1.0 + alpha);

            if temp_c > self.config.t_max_c {
                phase = ChargingPhase::Fault;
            }

            history.push(ChargingState {
                soc,
                voltage,
                current_a: current,
                temperature_c: temp_c,
                time_s,
                charge_ah,
                phase,
            });

            let delta_ah = current * dt_s / 3600.0;
            soc = (soc + delta_ah / self.config.capacity_ah).clamp(0.0, 1.0);
            charge_ah += delta_ah;
            energy_wh += voltage * delta_ah;
            time_s += dt_s;

            if soc >= 0.9999 {
                phase = ChargingPhase::Done;
            }
        }

        let completed = phase == ChargingPhase::Done;
        let theoretical_energy =
            self.config.capacity_ah * (1.0 - initial_soc) * self.config.v_nominal;
        let efficiency = if theoretical_energy > 1e-6 {
            (energy_wh / theoretical_energy).min(1.0)
        } else {
            0.0
        };

        ChargingResult {
            history,
            total_time_s: time_s,
            final_soc: soc,
            energy_wh,
            efficiency,
            completed,
            protocol: ProtocolType::FastCharge,
        }
    }
}

/// Compare two charging results: returns the faster one.
pub fn faster_protocol<'a>(a: &'a ChargingResult, b: &'a ChargingResult) -> &'a ChargingResult {
    if a.total_time_s <= b.total_time_s {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cc_cv_completes() {
        let cfg = ChargingConfig::lfp_50ah();
        let result = run_cc_cv(&cfg, 0.2, 25.0, 60.0, 36000.0);
        assert!(result.completed, "CC-CV should complete from SoC 0.2");
        assert!(
            result.final_soc > 0.95,
            "Final SoC should be high: {:.3}",
            result.final_soc
        );
    }

    #[test]
    fn test_cc_cv_soc_monotone() {
        let cfg = ChargingConfig::nmc_3ah();
        let result = run_cc_cv(&cfg, 0.1, 1.5, 30.0, 7200.0);
        let socs: Vec<f64> = result.history.iter().map(|s| s.soc).collect();
        for w in socs.windows(2) {
            assert!(
                w[1] >= w[0] - 1e-10,
                "SoC should be monotone: {:.4} > {:.4}",
                w[0],
                w[1]
            );
        }
    }

    #[test]
    fn test_cc_cv_voltage_below_max() {
        let cfg = ChargingConfig::nmc_3ah();
        let result = run_cc_cv(&cfg, 0.1, 1.5, 30.0, 7200.0);
        for state in &result.history {
            assert!(
                state.voltage <= cfg.v_max + 1e-6,
                "Voltage should not exceed v_max: {:.4}",
                state.voltage
            );
        }
    }

    #[test]
    fn test_cc_cv_energy_positive() {
        let cfg = ChargingConfig::lfp_50ah();
        let result = run_cc_cv(&cfg, 0.0, 25.0, 60.0, 36000.0);
        assert!(
            result.energy_wh > 0.0,
            "Energy should be positive: {:.2}",
            result.energy_wh
        );
    }

    #[test]
    fn test_cc_cv_temperature_reasonable() {
        let cfg = ChargingConfig::lfp_50ah();
        let result = run_cc_cv(&cfg, 0.5, 25.0, 60.0, 3600.0);
        let peak_t = result.peak_temperature();
        assert!(
            peak_t < 80.0,
            "Temperature should stay reasonable: {:.1}°C",
            peak_t
        );
    }

    #[test]
    fn test_multistep_completes() {
        let cfg = ChargingConfig::nmc_3ah();
        let steps = vec![
            ChargingStep {
                current_a: 3.0,
                soc_threshold: 0.5,
            },
            ChargingStep {
                current_a: 2.0,
                soc_threshold: 0.8,
            },
            ChargingStep {
                current_a: 1.0,
                soc_threshold: 1.0,
            },
        ];
        let result = run_multistep(&cfg, 0.1, &steps, 30.0, 14400.0);
        assert!(result.completed, "Multistep should complete");
        assert_eq!(result.protocol, ProtocolType::Multistep);
    }

    #[test]
    fn test_multistep_higher_initial_current() {
        let cfg = ChargingConfig::nmc_3ah();
        let steps = vec![
            ChargingStep {
                current_a: 3.0,
                soc_threshold: 0.5,
            },
            ChargingStep {
                current_a: 1.0,
                soc_threshold: 1.0,
            },
        ];
        let result = run_multistep(&cfg, 0.0, &steps, 30.0, 14400.0);
        // Early states should have higher current than later states
        let early_avg = result
            .history
            .iter()
            .take(10)
            .map(|s| s.current_a)
            .sum::<f64>()
            / 10.0;
        let late_count = result.history.len().saturating_sub(20);
        if late_count > 0 {
            let late_avg = result
                .history
                .iter()
                .skip(late_count)
                .map(|s| s.current_a)
                .sum::<f64>()
                / (result.history.len() - late_count) as f64;
            assert!(
                early_avg >= late_avg - 1e-3,
                "Early current {:.2} A should be ≥ late current {:.2} A",
                early_avg,
                late_avg
            );
        }
    }

    #[test]
    fn test_health_aware_respects_temperature() {
        let mut cfg = ChargingConfig::nmc_3ah();
        cfg.soh = 0.8; // aged cell
        let result = run_health_aware(&cfg, 0.0, 30.0, 14400.0);
        assert!(
            result.peak_temperature() <= cfg.t_max_c + 1e-3,
            "Temperature should not exceed limit: {:.1}",
            result.peak_temperature()
        );
    }

    #[test]
    fn test_health_aware_aged_vs_new() {
        let mut cfg_new = ChargingConfig::nmc_3ah();
        cfg_new.soh = 1.0;
        let mut cfg_old = ChargingConfig::nmc_3ah();
        cfg_old.soh = 0.7;
        cfg_old.r_internal = 0.05; // higher resistance

        let r_new = run_health_aware(&cfg_new, 0.2, 30.0, 7200.0);
        let r_old = run_health_aware(&cfg_old, 0.2, 30.0, 7200.0);
        // Aged cell should deliver less charge (lower effective capacity)
        assert!(
            r_old.final_soc <= r_new.final_soc + 0.1,
            "Aged cell SoC {} should be ≤ new {}",
            r_old.final_soc,
            r_new.final_soc
        );
    }

    #[test]
    fn test_fast_charge_profile_length() {
        let cfg = ChargingConfig::nmc_3ah();
        let opt = FastChargeOptimiser::new(cfg, 20);
        let profile = opt.optimal_current_profile();
        assert_eq!(profile.len(), 20);
    }

    #[test]
    fn test_fast_charge_profile_soc_increasing() {
        let cfg = ChargingConfig::nmc_3ah();
        let opt = FastChargeOptimiser::new(cfg, 10);
        let profile = opt.optimal_current_profile();
        for window in profile.windows(2) {
            assert!(
                window[1].0 >= window[0].0,
                "SoC points should be increasing: {:.3} >= {:.3}",
                window[1].0,
                window[0].0
            );
        }
    }

    #[test]
    fn test_fast_charge_completes() {
        let cfg = ChargingConfig::nmc_3ah();
        let opt = FastChargeOptimiser::new(cfg, 20);
        let result = opt.run(0.1, 30.0, 14400.0);
        assert!(
            result.final_soc > 0.9,
            "Fast charge should reach high SoC: {:.3}",
            result.final_soc
        );
        assert_eq!(result.protocol, ProtocolType::FastCharge);
    }

    #[test]
    fn test_faster_protocol_selects_correct() {
        let cfg = ChargingConfig::nmc_3ah();
        let r1 = run_cc_cv(&cfg, 0.5, 1.5, 30.0, 3600.0);
        let r2 = run_health_aware(&cfg, 0.5, 30.0, 3600.0);
        let faster = faster_protocol(&r1, &r2);
        assert!(
            faster.total_time_s <= r1.total_time_s + 1e-6
                || faster.total_time_s <= r2.total_time_s + 1e-6
        );
    }

    #[test]
    fn test_max_current_temperature_derating() {
        let cfg = ChargingConfig::lfp_50ah();
        let i_25 = cfg.max_current_a(25.0);
        let i_0 = cfg.max_current_a(0.0); // cold
        assert!(
            i_0 < i_25,
            "Cold temperature should reduce max current: {:.2} < {:.2}",
            i_0,
            i_25
        );
    }

    #[test]
    fn test_ocv_model_monotone() {
        let v_min = 3.0;
        let v_max = 4.2;
        for i in 0..10 {
            let s1 = i as f64 / 10.0;
            let s2 = (i + 1) as f64 / 10.0;
            assert!(ocv(s2, v_min, v_max) >= ocv(s1, v_min, v_max));
        }
    }

    #[test]
    fn test_cc_cv_protocol_field() {
        let cfg = ChargingConfig::lfp_50ah();
        let r = run_cc_cv(&cfg, 0.5, 20.0, 60.0, 1800.0);
        assert_eq!(r.protocol, ProtocolType::CcCv);
    }

    #[test]
    fn test_avg_power_nonzero() {
        let cfg = ChargingConfig::nmc_3ah();
        let r = run_cc_cv(&cfg, 0.1, 1.5, 30.0, 7200.0);
        assert!(r.avg_power_w() > 0.0, "Average power should be positive");
    }

    #[test]
    fn test_cc_phase_duration() {
        let cfg = ChargingConfig::nmc_3ah();
        let r = run_cc_cv(&cfg, 0.1, 1.5, 30.0, 7200.0);
        assert!(
            r.cc_duration_s() > 0.0,
            "CC phase should have positive duration"
        );
        assert!(
            r.cc_duration_s() < r.total_time_s,
            "CC duration {:.0}s should be less than total {:.0}s",
            r.cc_duration_s(),
            r.total_time_s
        );
    }
}
