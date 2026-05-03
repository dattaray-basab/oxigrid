/// Battery safety monitoring: thermal runaway, cell-to-cell propagation, early warning.
///
/// # Overview
///
/// Battery thermal runaway is an exothermic chain reaction triggered when
/// cell temperature exceeds critical thresholds, leading to venting, fire,
/// or explosion.  This module provides:
///
/// 1. **Thermal runaway trigger model** — tracks cell temperature and detects
///    onset based on threshold exceedance (self-heating rate dT/dt > limit).
///
/// 2. **Cell-to-cell propagation** — models how thermal runaway spreads from
///    one cell to adjacent cells via conduction, radiation, and hot gas venting.
///
/// 3. **Early warning index (EWI)** — a composite risk score combining multiple
///    indicators: temperature rise rate, voltage anomalies, gas evolution proxy,
///    internal resistance increase, and SoC stress.
///
/// # References
/// - Feng et al., "Thermal Runaway Mechanism of Lithium Ion Battery for Electric
///   Vehicles", J. Power Sources 2018.
/// - Lamb et al., "Evaluation of Mechanical Abuse Techniques in Lithium Ion
///   Batteries", J. Power Sources 2015.
/// - Kim et al., "Battery Cell Thermal Runaway Propagation", 2019.
use serde::{Deserialize, Serialize};

/// Thermal runaway state of one cell.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CellState {
    /// Normal operation
    Normal,
    /// Early warning: temperature rising faster than expected
    Warning,
    /// Critical: onset of thermal runaway detected
    Critical,
    /// Thermal runaway in progress
    Runaway,
    /// Cell has vented/burned out
    Dead,
}

impl CellState {
    /// Risk level: 0=normal, 1=warning, 2=critical, 3=runaway, 4=dead.
    pub fn risk_level(&self) -> u8 {
        match self {
            Self::Normal => 0,
            Self::Warning => 1,
            Self::Critical => 2,
            Self::Runaway => 3,
            Self::Dead => 4,
        }
    }
}

/// Thermal runaway configuration (per cell).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalRunawayConfig {
    /// Temperature threshold for warning state [°C]
    pub t_warning_c: f64,
    /// Temperature threshold for critical state (SEI decomposition onset) [°C]
    pub t_critical_c: f64,
    /// Temperature threshold for runaway onset [°C]
    pub t_runaway_c: f64,
    /// Maximum safe dT/dt [°C/s]
    pub dt_dt_max: f64,
    /// Thermal runaway heat release `J` (determines propagation energy)
    pub runaway_energy_j: f64,
    /// Cell thermal mass [J/K]
    pub thermal_mass_jk: f64,
    /// Adiabatic temperature rise during runaway [°C]
    pub adiabatic_temp_rise_c: f64,
}

impl ThermalRunawayConfig {
    /// Typical 18650 NMC cell.
    pub fn nmc_18650() -> Self {
        Self {
            t_warning_c: 60.0,
            t_critical_c: 90.0,
            t_runaway_c: 150.0,
            dt_dt_max: 1.0,             // 1°C/s normal rate limit
            runaway_energy_j: 80_000.0, // ~80 kJ for 3Ah NMC
            thermal_mass_jk: 50.0,
            adiabatic_temp_rise_c: 700.0,
        }
    }

    /// LFP cell — much more thermally stable.
    pub fn lfp_pouch() -> Self {
        Self {
            t_warning_c: 70.0,
            t_critical_c: 130.0,
            t_runaway_c: 220.0,
            dt_dt_max: 2.0,
            runaway_energy_j: 30_000.0,
            thermal_mass_jk: 80.0,
            adiabatic_temp_rise_c: 350.0,
        }
    }

    /// NCR/NCA — most susceptible chemistry.
    pub fn nca_18650() -> Self {
        Self {
            t_warning_c: 55.0,
            t_critical_c: 80.0,
            t_runaway_c: 130.0,
            dt_dt_max: 0.5,
            runaway_energy_j: 100_000.0,
            thermal_mass_jk: 45.0,
            adiabatic_temp_rise_c: 900.0,
        }
    }
}

/// State of one cell in the safety monitor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CellSafetyState {
    pub cell_index: usize,
    /// Current temperature [°C]
    pub temperature_c: f64,
    /// Temperature rate of change [°C/s]
    pub dt_dt: f64,
    /// Runaway state
    pub state: CellState,
    /// Energy released (during runaway) `J`
    pub energy_released_j: f64,
    /// Time since onset of runaway `s` (0 if not in runaway)
    pub runaway_time_s: f64,
}

impl CellSafetyState {
    pub fn new(cell_index: usize, temperature_c: f64) -> Self {
        Self {
            cell_index,
            temperature_c,
            dt_dt: 0.0,
            state: CellState::Normal,
            energy_released_j: 0.0,
            runaway_time_s: 0.0,
        }
    }
}

/// Thermal runaway monitor for a multi-cell pack.
pub struct ThermalRunawayMonitor {
    pub config: ThermalRunawayConfig,
    pub cells: Vec<CellSafetyState>,
    /// Cell adjacency (cell_i → list of adjacent cell indices)
    pub adjacency: Vec<Vec<usize>>,
    /// Propagation coefficient [W/K] (thermal coupling between adjacent cells)
    pub coupling_w_per_k: f64,
}

impl ThermalRunawayMonitor {
    /// Create a linear string of n_cells cells.
    pub fn linear_string(
        n_cells: usize,
        t_ambient_c: f64,
        config: ThermalRunawayConfig,
        coupling: f64,
    ) -> Self {
        let cells = (0..n_cells)
            .map(|i| CellSafetyState::new(i, t_ambient_c))
            .collect();
        let adjacency = (0..n_cells)
            .map(|i| {
                let mut adj = Vec::new();
                if i > 0 {
                    adj.push(i - 1);
                }
                if i < n_cells - 1 {
                    adj.push(i + 1);
                }
                adj
            })
            .collect();
        Self {
            config,
            cells,
            adjacency,
            coupling_w_per_k: coupling,
        }
    }

    /// Create a 2D grid of cells (rows × cols).
    pub fn grid(
        rows: usize,
        cols: usize,
        t_ambient_c: f64,
        config: ThermalRunawayConfig,
        coupling: f64,
    ) -> Self {
        let n = rows * cols;
        let cells = (0..n)
            .map(|i| CellSafetyState::new(i, t_ambient_c))
            .collect();
        let adjacency = (0..n)
            .map(|idx| {
                let r = idx / cols;
                let c = idx % cols;
                let mut adj = Vec::new();
                if r > 0 {
                    adj.push((r - 1) * cols + c);
                }
                if r < rows - 1 {
                    adj.push((r + 1) * cols + c);
                }
                if c > 0 {
                    adj.push(r * cols + c - 1);
                }
                if c < cols - 1 {
                    adj.push(r * cols + c + 1);
                }
                adj
            })
            .collect();
        Self {
            config,
            cells,
            adjacency,
            coupling_w_per_k: coupling,
        }
    }

    /// Update the monitor by one time step.
    ///
    /// - `heat_gen_w`: external heat generation per cell `W`
    /// - `t_ambient_c`: ambient temperature [°C]
    /// - `dt`: time step `s`
    pub fn step(&mut self, heat_gen_w: &[f64], t_ambient_c: f64, dt: f64) {
        let n = self.cells.len();
        let mut new_temps = vec![0.0f64; n];
        let mut new_dtdt = vec![0.0f64; n];

        for i in 0..n {
            let cell = &self.cells[i];
            let t = cell.temperature_c;

            // External heat generation
            let q_ext = if i < heat_gen_w.len() {
                heat_gen_w[i]
            } else {
                0.0
            };

            // Self-heating during runaway (exponential growth)
            let q_runaway = if cell.state == CellState::Runaway {
                let rate = (self.config.runaway_energy_j - cell.energy_released_j).max(0.0) * 0.1; // decay rate
                rate.min(1e6) // cap to physical limit
            } else {
                0.0
            };

            // Conductive coupling from adjacent cells in runaway
            let q_propagation: f64 = self.adjacency[i]
                .iter()
                .map(|&j| {
                    let tj = self.cells[j].temperature_c;
                    if tj > t {
                        self.coupling_w_per_k * (tj - t)
                    } else {
                        0.0
                    }
                })
                .sum();

            // Ambient cooling (convective)
            let h_cool = 5.0; // [W/K] approximate for 18650
            let q_cool = h_cool * (t - t_ambient_c);

            let q_net = q_ext + q_runaway + q_propagation - q_cool;
            let dt_dt = q_net / self.config.thermal_mass_jk;

            new_dtdt[i] = dt_dt;
            new_temps[i] = t + dt_dt * dt;
        }

        // Update cell states
        for i in 0..n {
            let prev_t = self.cells[i].temperature_c;
            let new_t = new_temps[i];
            let dt_dt = new_dtdt[i];

            // Update runaway tracking
            if self.cells[i].state == CellState::Runaway {
                self.cells[i].runaway_time_s += dt;
                // Energy released: proportional to temperature rise
                let energy_rate = dt_dt.max(0.0) * self.config.thermal_mass_jk;
                self.cells[i].energy_released_j += energy_rate * dt;

                if self.cells[i].energy_released_j >= self.config.runaway_energy_j * 0.9 {
                    self.cells[i].state = CellState::Dead;
                }
            }

            self.cells[i].temperature_c = new_t;
            self.cells[i].dt_dt = dt_dt;

            // State transitions
            let state = &self.cells[i].state;
            match state {
                CellState::Normal => {
                    if new_t >= self.config.t_warning_c || dt_dt > self.config.dt_dt_max {
                        self.cells[i].state = CellState::Warning;
                    }
                }
                CellState::Warning => {
                    if new_t >= self.config.t_critical_c {
                        self.cells[i].state = CellState::Critical;
                    } else if new_t < self.config.t_warning_c - 5.0 && dt_dt < 0.1 {
                        self.cells[i].state = CellState::Normal;
                    }
                }
                CellState::Critical => {
                    if new_t >= self.config.t_runaway_c {
                        self.cells[i].state = CellState::Runaway;
                    }
                }
                CellState::Runaway | CellState::Dead => {}
            }

            let _ = prev_t;
        }
    }

    /// Trigger thermal runaway in cell `cell_idx` (for simulation).
    pub fn trigger_runaway(&mut self, cell_idx: usize) {
        if cell_idx < self.cells.len() {
            self.cells[cell_idx].state = CellState::Runaway;
            self.cells[cell_idx].temperature_c = self.config.t_runaway_c + 50.0;
        }
    }

    /// Count cells in each state.
    pub fn state_counts(&self) -> [usize; 5] {
        let mut counts = [0usize; 5];
        for cell in &self.cells {
            counts[cell.state.risk_level() as usize] += 1;
        }
        counts
    }

    /// Maximum cell temperature [°C].
    pub fn max_temperature(&self) -> f64 {
        self.cells
            .iter()
            .map(|c| c.temperature_c)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// True if any cell is in runaway or dead state.
    pub fn is_thermal_runaway_active(&self) -> bool {
        self.cells
            .iter()
            .any(|c| matches!(c.state, CellState::Runaway | CellState::Dead))
    }
}

/// Early Warning Index (EWI) for a battery pack.
///
/// Composite risk score in [0, 1] combining:
///   - Temperature deviation above baseline
///   - Rate of temperature rise
///   - Voltage anomaly (drop from expected OCV)
///   - Internal resistance increase fraction
///   - SoC stress (high SoC + high temperature)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EarlyWarningIndex {
    /// Weights for each indicator [T_dev, dT/dt, V_anom, R_inc, SoC_stress]
    pub weights: [f64; 5],
    /// Normalisation limits for each indicator
    pub limits: [f64; 5],
}

impl EarlyWarningIndex {
    /// Default EWI configuration.
    pub fn default_config() -> Self {
        Self {
            weights: [0.25, 0.30, 0.20, 0.15, 0.10],
            limits: [40.0, 5.0, 0.5, 0.5, 1.0], // [°C, °C/s, V, fraction, combined]
        }
    }

    /// Compute EWI score [0, 1].
    ///
    /// - `t_current_c`  — current cell temperature [°C]
    /// - `t_baseline_c` — expected temperature [°C]
    /// - `dt_dt`        — temperature rate [°C/s]
    /// - `v_anom_v`     — voltage anomaly (|V_measured - V_expected|) `V`
    /// - `r_increase`   — relative resistance increase (R/R0 - 1) [0,∞)
    /// - `soc`          — state of charge `0,1`
    pub fn compute(
        &self,
        t_current_c: f64,
        t_baseline_c: f64,
        dt_dt: f64,
        v_anom_v: f64,
        r_increase: f64,
        soc: f64,
    ) -> f64 {
        let indicators = [
            (t_current_c - t_baseline_c).max(0.0),
            dt_dt.max(0.0),
            v_anom_v.abs(),
            r_increase.max(0.0),
            (soc * (t_current_c - 25.0).max(0.0) / 40.0).clamp(0.0, 1.0),
        ];

        let score: f64 = indicators
            .iter()
            .zip(self.weights.iter())
            .zip(self.limits.iter())
            .map(|((&ind, &w), &lim)| {
                let normalised = (ind / lim.max(1e-10)).clamp(0.0, 1.0);
                w * normalised
            })
            .sum();

        score.clamp(0.0, 1.0)
    }

    /// Alert level based on EWI score.
    pub fn alert_level(&self, ewi: f64) -> &'static str {
        if ewi < 0.2 {
            "NORMAL"
        } else if ewi < 0.5 {
            "CAUTION"
        } else if ewi < 0.75 {
            "WARNING"
        } else {
            "CRITICAL"
        }
    }
}

/// Isolation resistance monitor for detecting internal short circuits.
///
/// An isolation resistance (IR) fault causes leakage current to chassis ground.
/// The fault is detected by monitoring the isolation resistance R_iso ≥ R_min.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IsolationMonitor {
    /// Minimum acceptable isolation resistance `Ω`
    pub r_iso_min: f64,
    /// Current isolation resistance estimate `Ω`
    pub r_iso: f64,
    /// Pack voltage `V` (for leakage current calculation)
    pub v_pack: f64,
}

impl IsolationMonitor {
    pub fn new(v_pack: f64) -> Self {
        Self {
            r_iso_min: 100_000.0,
            r_iso: 10_000_000.0,
            v_pack,
        }
    }

    /// Leakage current to chassis `A`.
    pub fn leakage_current_a(&self) -> f64 {
        if self.r_iso < 1.0 {
            return self.v_pack;
        }
        self.v_pack / self.r_iso
    }

    /// True if isolation fault detected.
    pub fn fault_detected(&self) -> bool {
        self.r_iso < self.r_iso_min
    }

    /// Update isolation resistance estimate.
    pub fn update_resistance(&mut self, r_measured: f64) {
        self.r_iso = r_measured.max(1.0);
    }

    /// ISO 6469-3 fault severity level.
    pub fn fault_severity(&self) -> &'static str {
        if self.r_iso >= 1_000_000.0 {
            "NONE"
        } else if self.r_iso >= 500_000.0 {
            "LOW"
        } else if self.r_iso >= self.r_iso_min {
            "MEDIUM"
        } else {
            "HIGH"
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cell_state_risk_levels() {
        assert_eq!(CellState::Normal.risk_level(), 0);
        assert_eq!(CellState::Warning.risk_level(), 1);
        assert_eq!(CellState::Critical.risk_level(), 2);
        assert_eq!(CellState::Runaway.risk_level(), 3);
        assert_eq!(CellState::Dead.risk_level(), 4);
    }

    #[test]
    fn test_thermal_runaway_config_thresholds_ordered() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        assert!(cfg.t_warning_c < cfg.t_critical_c);
        assert!(cfg.t_critical_c < cfg.t_runaway_c);
    }

    #[test]
    fn test_monitor_linear_normal_operation() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        let mut mon = ThermalRunawayMonitor::linear_string(5, 25.0, cfg, 0.5);
        let heat = vec![0.1; 5]; // small heat
        for _ in 0..100 {
            mon.step(&heat, 25.0, 1.0);
        }
        // No cell should have runaway with small heat
        assert!(
            !mon.is_thermal_runaway_active(),
            "Should not runaway with small heat"
        );
    }

    #[test]
    fn test_monitor_trigger_runaway() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        let mut mon = ThermalRunawayMonitor::linear_string(5, 25.0, cfg, 0.5);
        mon.trigger_runaway(2);
        assert!(mon.is_thermal_runaway_active());
        assert!(mon.cells[2].state == CellState::Runaway);
    }

    #[test]
    fn test_monitor_state_counts() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        let mut mon = ThermalRunawayMonitor::linear_string(4, 25.0, cfg, 0.5);
        mon.trigger_runaway(0);
        let counts = mon.state_counts();
        assert_eq!(counts[3], 1, "One cell in runaway"); // Runaway
        assert_eq!(counts[0], 3, "Three cells normal"); // Normal
    }

    #[test]
    fn test_monitor_max_temperature() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        let mut mon = ThermalRunawayMonitor::linear_string(3, 25.0, cfg, 0.5);
        mon.trigger_runaway(1);
        assert!(
            mon.max_temperature() > 25.0,
            "Max temp should exceed ambient: {:.1}",
            mon.max_temperature()
        );
    }

    #[test]
    fn test_propagation_from_runaway_cell() {
        let cfg = ThermalRunawayConfig::nmc_18650();
        let mut mon = ThermalRunawayMonitor::linear_string(5, 25.0, cfg, 50.0); // high coupling
        mon.trigger_runaway(2);
        let t_before = mon.cells[3].temperature_c;
        mon.step(&[0.0; 5], 25.0, 10.0); // 10s step
        let t_after = mon.cells[3].temperature_c;
        // Adjacent cell should heat up due to propagation
        assert!(
            t_after >= t_before,
            "Adjacent cell should heat up: {:.2} >= {:.2}",
            t_after,
            t_before
        );
    }

    #[test]
    fn test_grid_adjacency() {
        let cfg = ThermalRunawayConfig::lfp_pouch();
        let mon = ThermalRunawayMonitor::grid(3, 3, 25.0, cfg, 1.0);
        // Centre cell (index 4) should have 4 neighbours
        assert_eq!(
            mon.adjacency[4].len(),
            4,
            "Centre cell: {:?}",
            mon.adjacency[4]
        );
        // Corner cell (index 0) should have 2 neighbours
        assert_eq!(mon.adjacency[0].len(), 2);
    }

    #[test]
    fn test_ewi_zero_at_normal() {
        let ewi = EarlyWarningIndex::default_config();
        let score = ewi.compute(25.0, 25.0, 0.0, 0.0, 0.0, 0.5);
        assert!(
            (score).abs() < 1e-10,
            "EWI should be 0 at normal: {:.6}",
            score
        );
    }

    #[test]
    fn test_ewi_increases_with_temperature() {
        let ewi = EarlyWarningIndex::default_config();
        let s1 = ewi.compute(30.0, 25.0, 0.0, 0.0, 0.0, 0.5);
        let s2 = ewi.compute(50.0, 25.0, 0.0, 0.0, 0.0, 0.5);
        assert!(
            s2 > s1,
            "EWI increases with temperature: {:.4} > {:.4}",
            s2,
            s1
        );
    }

    #[test]
    fn test_ewi_alert_levels() {
        let ewi = EarlyWarningIndex::default_config();
        assert_eq!(ewi.alert_level(0.1), "NORMAL");
        assert_eq!(ewi.alert_level(0.35), "CAUTION");
        assert_eq!(ewi.alert_level(0.6), "WARNING");
        assert_eq!(ewi.alert_level(0.9), "CRITICAL");
    }

    #[test]
    fn test_isolation_monitor_normal() {
        let mon = IsolationMonitor::new(400.0);
        assert!(!mon.fault_detected());
        assert_eq!(mon.fault_severity(), "NONE");
        let leakage = mon.leakage_current_a();
        assert!(leakage < 0.001, "Leakage should be tiny: {:.6e}", leakage);
    }

    #[test]
    fn test_isolation_monitor_fault() {
        let mut mon = IsolationMonitor::new(400.0);
        mon.update_resistance(50_000.0); // below min
        assert!(mon.fault_detected());
        assert_eq!(mon.fault_severity(), "HIGH");
    }

    #[test]
    fn test_isolation_leakage_increases_with_lower_r() {
        let mut mon = IsolationMonitor::new(400.0);
        mon.update_resistance(1_000_000.0);
        let l1 = mon.leakage_current_a();
        mon.update_resistance(100_000.0);
        let l2 = mon.leakage_current_a();
        assert!(
            l2 > l1,
            "Lower R_iso → higher leakage: {:.6e} > {:.6e}",
            l2,
            l1
        );
    }
}
