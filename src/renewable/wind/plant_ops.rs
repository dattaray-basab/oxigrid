//! Wind power plant operations module.
//!
//! Covers curtailment control, active power regulation, wake steering,
//! and grid code compliance for wind farm plant-level operations.
//!
//! Key components:
//! - [`PlantController`] — farm-level active power dispatch
//! - [`WakeSteering`]    — yaw-offset-based wake loss mitigation
//! - [`PlantOperationsManager`] — orchestrates all plant operations
//! - [`FrequencyResponseController`] — FCR/droop response
//! - [`CurtailmentSchedule`]  — time-based curtailment rules

use serde::{Deserialize, Serialize};

// ── Constants ──────────────────────────────────────────────────────────────────
/// Cut-in wind speed [m/s] for the simplified turbine power curve.
const V_CUT_IN: f64 = 3.0;
/// Rated wind speed [m/s].
const V_RATED: f64 = 12.0;
/// Cut-out wind speed [m/s].
const V_CUT_OUT: f64 = 25.0;
/// Jensen wake decay constant (onshore default).
const K_WAKE: f64 = 0.07;
/// Thrust coefficient used in the wake model.
const CT: f64 = 0.8;
/// Hours per year.
const HOURS_PER_YEAR: f64 = 8_760.0;

// ── Enums ──────────────────────────────────────────────────────────────────────

/// Reason for curtailing a turbine's power output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CurtailmentReason {
    /// Transmission or distribution congestion.
    GridCongestion,
    /// System frequency deviation requires output reduction.
    SystemFrequency,
    /// Noise-based curtailment (residential proximity).
    NoiseCurtailment,
    /// Thermal rating limit of collector or transformer.
    ThermalLimit,
    /// Turbulence intensity too high for safe operation.
    TurbulenceAvoidance,
    /// Ice accretion on blades detected.
    IcingCondition,
    /// Maintenance access requires nearby turbines offline.
    MaintenanceAccess,
    /// Bat-friendly operation (low-wind-speed curtailment).
    BatFriendly,
    /// Day-ahead or real-time market price below threshold.
    MarketPrice,
}

/// Active power control mode for the wind farm plant controller.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ActivePowerMode {
    /// Operate at maximum available power (MPPT, no curtailment).
    MaximumPower,
    /// Delta control: maintain a spinning reserve headroom.
    DeltaControl,
    /// Fixed absolute power output [MW].
    AbsolutePower,
    /// Follow an external balancing regulation signal.
    BalancingMode,
    /// Ramp-rate-limited power change.
    RampRateLimited,
}

/// Wake steering operational mode.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum WakeSteeringMode {
    /// Wake steering disabled; all turbines face the wind directly.
    Disabled,
    /// Fixed yaw offset applied to all upstream turbines.
    Static,
    /// Dynamic optimisation based on real-time wind direction.
    Dynamic,
    /// Sector-based look-up table offsets.
    Sector,
}

// ── Structs ────────────────────────────────────────────────────────────────────

/// Per-turbine power and yaw setpoint issued by the plant controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurbineSetpoint {
    /// Zero-based turbine index within the farm.
    pub turbine_id: usize,
    /// Active power setpoint [kW].
    pub power_setpoint_kw: f64,
    /// Yaw offset applied for wake steering [°].
    pub yaw_offset_deg: f64,
    /// Curtailment reason, if any.
    pub curtailment_reason: Option<CurtailmentReason>,
    /// Blade pitch angle setpoint [°].
    pub pitch_angle_deg: f64,
}

/// Snapshot of farm operating conditions and setpoints at a single time step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FarmOperatingPoint {
    /// Simulation timestamp [h].
    pub timestamp_h: f64,
    /// Ambient wind speed [m/s].
    pub wind_speed_ms: f64,
    /// Wind direction (meteorological, from north) [°].
    pub wind_direction_deg: f64,
    /// Ambient temperature [°C].
    pub ambient_temperature_c: f64,
    /// Total farm active power output [MW].
    pub total_power_mw: f64,
    /// Total power available without curtailment [MW].
    pub available_power_mw: f64,
    /// Power withheld due to curtailment [MW].
    pub curtailed_power_mw: f64,
    /// Turbine-level setpoints.
    pub turbine_setpoints: Vec<TurbineSetpoint>,
    /// Farm efficiency = actual / available (0–1).
    pub farm_efficiency: f64,
}

/// Wind farm active power plant controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlantController {
    /// Unique farm identifier.
    pub farm_id: usize,
    /// Human-readable farm name.
    pub name: String,
    /// Number of turbines in the farm.
    pub n_turbines: usize,
    /// Rated power per turbine [MW].
    pub rated_power_mw: f64,
    /// Total farm rated power [MW].
    pub farm_rated_mw: f64,
    /// Active power control mode.
    pub control_mode: ActivePowerMode,
    /// Power setpoint received from grid operator [MW].
    pub power_setpoint_mw: f64,
    /// Frequency deadband; no response within ±deadband [Hz].
    pub frequency_deadband_hz: f64,
    /// Droop characteristic [%].
    pub frequency_droop_pct: f64,
    /// Maximum power ramp rate [MW/min].
    pub ramp_rate_mw_per_min: f64,
    /// Delta-control spinning reserve fraction [%].
    pub delta_reserve_pct: f64,
}

impl PlantController {
    /// Create a new plant controller with sensible defaults.
    ///
    /// # Arguments
    /// - `farm_id`         — Unique farm identifier.
    /// - `n_turbines`      — Number of turbines.
    /// - `rated_power_mw`  — Rated power per turbine [MW].
    pub fn new(farm_id: usize, n_turbines: usize, rated_power_mw: f64) -> Self {
        let farm_rated_mw = rated_power_mw * n_turbines as f64;
        Self {
            farm_id,
            name: format!("Farm-{}", farm_id),
            n_turbines,
            rated_power_mw,
            farm_rated_mw,
            control_mode: ActivePowerMode::MaximumPower,
            power_setpoint_mw: farm_rated_mw,
            frequency_deadband_hz: 0.2,
            frequency_droop_pct: 4.0,
            ramp_rate_mw_per_min: farm_rated_mw * 0.1, // 10 %/min default
            delta_reserve_pct: 10.0,
        }
    }

    /// Compute per-turbine setpoints given available farm power and grid frequency.
    ///
    /// Distributes the farm-level power target uniformly across turbines and
    /// applies frequency droop correction.
    pub fn compute_setpoints(
        &self,
        available_power_mw: f64,
        grid_frequency_hz: f64,
    ) -> Vec<TurbineSetpoint> {
        let target_mw = self.target_farm_power(available_power_mw, grid_frequency_hz);
        let per_turbine_mw = if self.n_turbines > 0 {
            (target_mw / self.n_turbines as f64).max(0.0)
        } else {
            0.0
        };
        let per_turbine_kw = per_turbine_mw * 1000.0;
        let rated_kw = self.rated_power_mw * 1000.0;

        // Pitch angle heuristic: at rated output pitch = 0°, below rated pitch increases
        let pitch_deg = if per_turbine_kw >= rated_kw {
            0.0
        } else {
            // linear from 20° at zero power to 0° at rated
            20.0 * (1.0 - per_turbine_kw / rated_kw.max(1.0))
        };

        let curtailment_reason = self.curtailment_reason_for_mode(available_power_mw, target_mw);

        (0..self.n_turbines)
            .map(|id| TurbineSetpoint {
                turbine_id: id,
                power_setpoint_kw: per_turbine_kw,
                yaw_offset_deg: 0.0, // wake steering applied later
                curtailment_reason,
                pitch_angle_deg: pitch_deg,
            })
            .collect()
    }

    /// Compute incremental frequency response power [MW] (positive = increase).
    ///
    /// Returns zero within the deadband; outside uses a linear droop.
    pub fn compute_frequency_response(&self, grid_frequency_hz: f64, current_power_mw: f64) -> f64 {
        let delta_f = 50.0 - grid_frequency_hz; // positive when freq is low
        if delta_f.abs() <= self.frequency_deadband_hz {
            return 0.0;
        }
        let p_ref = current_power_mw.max(0.0);
        // ΔP = P_ref × (Δf / f0) / (droop% / 100)
        let response = p_ref * (delta_f / 50.0) / (self.frequency_droop_pct / 100.0);
        response.clamp(-self.farm_rated_mw, self.farm_rated_mw)
    }

    /// Apply ramp-rate limit when moving from `current_mw` towards `target_mw`.
    ///
    /// Returns the achievable power given `dt_s` seconds have elapsed.
    pub fn apply_ramp_limit(&self, current_mw: f64, target_mw: f64, dt_s: f64) -> f64 {
        let max_change = self.ramp_rate_mw_per_min * dt_s / 60.0;
        let delta = (target_mw - current_mw).clamp(-max_change, max_change);
        (current_mw + delta).clamp(0.0, self.farm_rated_mw)
    }

    /// Compute headroom [MW] reserved for up-regulation under delta control.
    pub fn compute_delta_reserve(&self, available_mw: f64) -> f64 {
        available_mw * self.delta_reserve_pct / 100.0
    }

    // ── private helpers ────────────────────────────────────────────────────────

    fn target_farm_power(&self, available_mw: f64, freq_hz: f64) -> f64 {
        let base = match self.control_mode {
            ActivePowerMode::MaximumPower => available_mw,
            ActivePowerMode::DeltaControl => {
                available_mw - self.compute_delta_reserve(available_mw)
            }
            ActivePowerMode::AbsolutePower => self.power_setpoint_mw.min(available_mw),
            ActivePowerMode::BalancingMode => self.power_setpoint_mw.min(available_mw),
            ActivePowerMode::RampRateLimited => available_mw,
        };
        // Add frequency droop correction on top
        let freq_delta = self.compute_frequency_response(freq_hz, base);
        (base + freq_delta).clamp(0.0, available_mw)
    }

    fn curtailment_reason_for_mode(
        &self,
        available_mw: f64,
        target_mw: f64,
    ) -> Option<CurtailmentReason> {
        if target_mw >= available_mw - 1e-3 {
            return None;
        }
        match self.control_mode {
            ActivePowerMode::AbsolutePower | ActivePowerMode::BalancingMode => {
                Some(CurtailmentReason::GridCongestion)
            }
            ActivePowerMode::DeltaControl => Some(CurtailmentReason::SystemFrequency),
            _ => None,
        }
    }
}

// ── Wake Steering ──────────────────────────────────────────────────────────────

/// Wake steering controller using yaw-offset-based wake deflection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WakeSteering {
    /// Steering operational mode.
    pub mode: WakeSteeringMode,
    /// Turbine positions in farm coordinates [m], (x, y).
    pub layout: Vec<(f64, f64)>,
    /// Rotor diameter [m].
    pub rotor_diameter_m: f64,
    /// Maximum allowable yaw offset [°].
    pub max_yaw_offset_deg: f64,
    /// Yaw actuation rate [°/s].
    pub yaw_rate_deg_per_s: f64,
}

impl WakeSteering {
    /// Construct a wake steering controller for the given farm layout.
    pub fn new(layout: Vec<(f64, f64)>, rotor_diameter_m: f64) -> Self {
        Self {
            mode: WakeSteeringMode::Disabled,
            layout,
            rotor_diameter_m,
            max_yaw_offset_deg: 20.0,
            yaw_rate_deg_per_s: 0.3,
        }
    }

    /// Compute yaw offset [°] for each turbine given free-stream conditions.
    ///
    /// Returns a vector the same length as `layout`.
    /// Positive offset deflects the wake to the right of the wind direction.
    pub fn compute_yaw_offsets(&self, wind_direction_deg: f64, wind_speed_ms: f64) -> Vec<f64> {
        let n = self.layout.len();
        match self.mode {
            WakeSteeringMode::Disabled => vec![0.0; n],
            WakeSteeringMode::Static => {
                // Fixed offset only for turbines that have a downstream neighbour
                (0..n)
                    .map(|i| {
                        let has_downstream =
                            (0..n).any(|j| j != i && self.is_upstream(i, j, wind_direction_deg));
                        if has_downstream {
                            self.max_yaw_offset_deg * 0.5
                        } else {
                            0.0
                        }
                    })
                    .collect()
            }
            WakeSteeringMode::Dynamic => self.dynamic_offsets(wind_direction_deg, wind_speed_ms),
            WakeSteeringMode::Sector => {
                // 30°-sector look-up: use offset when wind aligns with
                // inter-row direction (within ±15° of cardinal/intercardinal axes)
                let sector_aligned = [0.0_f64, 90.0, 180.0, 270.0]
                    .iter()
                    .any(|&s| angle_diff(wind_direction_deg, s) < 15.0);
                let offset = if sector_aligned {
                    self.max_yaw_offset_deg * 0.6
                } else {
                    0.0
                };
                (0..n)
                    .map(|i| {
                        let has_downstream =
                            (0..n).any(|j| j != i && self.is_upstream(i, j, wind_direction_deg));
                        if has_downstream {
                            offset
                        } else {
                            0.0
                        }
                    })
                    .collect()
            }
        }
    }

    /// Estimate the wake loss fraction for the farm at a given wind direction.
    ///
    /// Returns a value in [0, 0.3].
    pub fn compute_wake_loss_factor(&self, wind_direction_deg: f64) -> f64 {
        let n = self.layout.len();
        if n <= 1 {
            return 0.0;
        }
        let rotated = self.rotate_to_wind_frame(wind_direction_deg);
        let d = self.rotor_diameter_m;
        let mut total_deficit = 0.0_f64;

        for i in 0..n {
            let mut deficit_sq = 0.0_f64;
            for j in 0..n {
                if i == j {
                    continue;
                }
                let dx = rotated[i].0 - rotated[j].0;
                let dy = (rotated[i].1 - rotated[j].1).abs();
                if dx <= 0.0 {
                    continue;
                }
                let r_wake = d / 2.0 + K_WAKE * dx;
                if dy < r_wake {
                    let deficit = (1.0 - (1.0 - CT).sqrt()) * (d / 2.0 / r_wake).powi(2);
                    deficit_sq += deficit * deficit;
                }
            }
            total_deficit += deficit_sq.sqrt();
        }
        (total_deficit / n as f64).clamp(0.0, 0.3)
    }

    /// Identify which turbines are upstream of `turbine_idx` for the given wind direction.
    pub fn upstream_turbines(&self, turbine_idx: usize, wind_dir_deg: f64) -> Vec<usize> {
        let n = self.layout.len();
        let rotated = self.rotate_to_wind_frame(wind_dir_deg);
        let d = self.rotor_diameter_m;
        let (xi, yi) = rotated[turbine_idx];

        (0..n)
            .filter(|&j| {
                if j == turbine_idx {
                    return false;
                }
                let (xj, yj) = rotated[j];
                let dx = xi - xj; // positive: j is further upwind than i
                if dx <= 0.0 {
                    return false;
                }
                let dy = (yi - yj).abs();
                let r_wake = d / 2.0 + K_WAKE * dx;
                dy < r_wake
            })
            .collect()
    }

    // ── private helpers ────────────────────────────────────────────────────────

    /// Rotate farm layout to wind-aligned frame.
    /// x_w = downwind component (positive = further downwind), y_w = crosswind.
    fn rotate_to_wind_frame(&self, wind_dir_deg: f64) -> Vec<(f64, f64)> {
        let dir_rad = wind_dir_deg.to_radians();
        let (sin_d, cos_d) = (dir_rad.sin(), dir_rad.cos());
        self.layout
            .iter()
            .map(|&(x, y)| {
                let x_w = -x * sin_d - y * cos_d;
                let y_w = -x * cos_d + y * sin_d;
                (x_w, y_w)
            })
            .collect()
    }

    /// Dynamic yaw offsets: apply offset proportional to downstream impact,
    /// scaled inversely with wind speed (more aggressive at lower speeds).
    fn dynamic_offsets(&self, wind_direction_deg: f64, wind_speed_ms: f64) -> Vec<f64> {
        let n = self.layout.len();
        let speed_factor = ((wind_speed_ms - V_CUT_IN) / (V_RATED - V_CUT_IN)).clamp(0.0, 1.0);
        (0..n)
            .map(|i| {
                let n_downstream = (0..n)
                    .filter(|&j| j != i && self.is_upstream(i, j, wind_direction_deg))
                    .count();
                if n_downstream == 0 {
                    0.0
                } else {
                    let base = self.max_yaw_offset_deg
                        * (n_downstream as f64 / n as f64).min(1.0)
                        * (1.0 - speed_factor * 0.4);
                    base.min(self.max_yaw_offset_deg)
                }
            })
            .collect()
    }

    /// Returns `true` when turbine `i` is upstream of turbine `j`.
    fn is_upstream(&self, i: usize, j: usize, wind_dir_deg: f64) -> bool {
        let rotated = self.rotate_to_wind_frame(wind_dir_deg);
        let (x_wi, y_wi) = rotated[i];
        let (x_wj, y_wj) = rotated[j];
        let dx = x_wj - x_wi; // positive: j is further downwind than i
        if dx <= 0.0 {
            return false;
        }
        let dy = (y_wj - y_wi).abs();
        let r_wake = self.rotor_diameter_m / 2.0 + K_WAKE * dx;
        dy < r_wake
    }
}

// ── Curtailment Schedule ───────────────────────────────────────────────────────

/// Time-based curtailment rule for a subset of turbines.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CurtailmentSchedule {
    /// Schedule start time [h] (0–24 or within simulation horizon).
    pub start_hour: f64,
    /// Schedule end time [h].
    pub end_hour: f64,
    /// Maximum allowed power output as percentage of rated (0–100 %).
    pub max_power_pct: f64,
    /// Reason for curtailment.
    pub reason: CurtailmentReason,
    /// Turbine IDs affected; an empty vector means all turbines.
    pub turbine_ids: Vec<usize>,
}

impl CurtailmentSchedule {
    /// Returns `true` if the schedule is active at `timestamp_h`.
    pub fn is_active(&self, timestamp_h: f64) -> bool {
        // Support schedules that wrap midnight (start_hour > end_hour)
        if self.start_hour <= self.end_hour {
            timestamp_h >= self.start_hour && timestamp_h < self.end_hour
        } else {
            timestamp_h >= self.start_hour || timestamp_h < self.end_hour
        }
    }

    /// Returns `true` if this schedule applies to `turbine_id`.
    pub fn applies_to(&self, turbine_id: usize) -> bool {
        self.turbine_ids.is_empty() || self.turbine_ids.contains(&turbine_id)
    }
}

// ── Frequency Response Controller ─────────────────────────────────────────────

/// Frequency response controller for FCR/droop-based power adjustment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyResponseController {
    /// Droop characteristic [%].
    pub droop_setting_pct: f64,
    /// Response time constant [s].
    pub response_time_s: f64,
    /// Frequency deadband [Hz]; no response within ±deadband.
    pub deadband_hz: f64,
    /// Minimum spinning reserve that must be kept available [%].
    pub reserved_capacity_pct: f64,
    /// Nominal system frequency [Hz].
    pub nominal_freq_hz: f64,
}

impl FrequencyResponseController {
    /// Create a controller with European 50 Hz defaults.
    pub fn new_50hz(droop_setting_pct: f64) -> Self {
        Self {
            droop_setting_pct,
            response_time_s: 2.0,
            deadband_hz: 0.015, // ENTSO-E ±15 mHz standard deadband
            reserved_capacity_pct: 5.0,
            nominal_freq_hz: 50.0,
        }
    }

    /// Compute the frequency response power delta [MW].
    ///
    /// Positive delta means the farm should increase output (under-frequency event).
    pub fn response_power_mw(
        &self,
        grid_freq_hz: f64,
        current_power_mw: f64,
        capacity_mw: f64,
    ) -> f64 {
        let delta_f = self.nominal_freq_hz - grid_freq_hz;
        if delta_f.abs() <= self.deadband_hz {
            return 0.0;
        }
        let reserved_mw = capacity_mw * self.reserved_capacity_pct / 100.0;
        let max_up = reserved_mw;
        let max_down = current_power_mw;
        let raw =
            current_power_mw * (delta_f / self.nominal_freq_hz) / (self.droop_setting_pct / 100.0);
        raw.clamp(-max_down, max_up)
    }
}

// ── Plant Operations Manager ───────────────────────────────────────────────────

/// Top-level wind power plant operations manager.
///
/// Orchestrates the plant controller, wake steering, curtailment schedules
/// and frequency response for a complete operational time step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlantOperationsManager {
    /// Farm-level active power controller.
    pub controller: PlantController,
    /// Wake steering subsystem.
    pub wake_steering: WakeSteering,
    /// Time-based curtailment rules.
    pub curtailment_schedule: Vec<CurtailmentSchedule>,
    /// Frequency response controller.
    pub frequency_controller: FrequencyResponseController,
    /// Turbine positions in farm coordinates [m], (x, y).
    pub turbine_positions: Vec<(f64, f64)>,
}

impl PlantOperationsManager {
    /// Create a manager with default frequency controller and no curtailment schedules.
    pub fn new(controller: PlantController, turbine_positions: Vec<(f64, f64)>) -> Self {
        let layout = turbine_positions.clone();
        let rotor_diameter_m = 120.0; // default 120 m rotor
        let droop = controller.frequency_droop_pct;
        Self {
            wake_steering: WakeSteering::new(layout, rotor_diameter_m),
            curtailment_schedule: Vec::new(),
            frequency_controller: FrequencyResponseController::new_50hz(droop),
            turbine_positions,
            controller,
        }
    }

    /// Execute one plant operations time step and return the operating point.
    ///
    /// # Arguments
    /// - `wind_speed_ms`    — Hub-height wind speed [m/s].
    /// - `wind_dir_deg`     — Wind direction (met. convention) [°].
    /// - `ambient_temp_c`   — Ambient temperature [°C].
    /// - `grid_freq_hz`     — Grid frequency [Hz].
    /// - `timestamp_h`      — Current simulation time [h].
    pub fn run_operating_step(
        &mut self,
        wind_speed_ms: f64,
        wind_dir_deg: f64,
        ambient_temp_c: f64,
        grid_freq_hz: f64,
        timestamp_h: f64,
    ) -> FarmOperatingPoint {
        // 1. Compute wake-corrected per-turbine wind speeds
        let wake_speeds = self.wake_corrected_speeds(wind_speed_ms, wind_dir_deg);

        // 2. Available power from each turbine's power curve
        let rated_kw = self.controller.rated_power_mw * 1000.0;
        let available_kw: Vec<f64> = wake_speeds
            .iter()
            .map(|&v| turbine_power_kw(v, rated_kw))
            .collect();
        let available_power_mw = available_kw.iter().sum::<f64>() / 1000.0;

        // 3. Plant controller computes base setpoints
        let mut setpoints = self
            .controller
            .compute_setpoints(available_power_mw, grid_freq_hz);

        // 4. Apply wake steering yaw offsets
        let yaw_offsets = self
            .wake_steering
            .compute_yaw_offsets(wind_dir_deg, wind_speed_ms);
        for sp in setpoints.iter_mut() {
            let offset = yaw_offsets.get(sp.turbine_id).copied().unwrap_or(0.0);
            sp.yaw_offset_deg = offset;
        }

        // 5. Apply curtailment schedule overrides
        setpoints = self.apply_curtailment_schedule(timestamp_h, setpoints);

        // 6. Compute farm totals
        let total_power_mw = setpoints.iter().map(|s| s.power_setpoint_kw).sum::<f64>() / 1000.0;
        let curtailed_power_mw = (available_power_mw - total_power_mw).max(0.0);
        let farm_efficiency = if available_power_mw > 1e-6 {
            (total_power_mw / available_power_mw).clamp(0.0, 1.0)
        } else {
            1.0
        };

        // ambient_temp_c recorded but not used in this simplified thermal model
        let _ = ambient_temp_c;

        FarmOperatingPoint {
            timestamp_h,
            wind_speed_ms,
            wind_direction_deg: wind_dir_deg,
            ambient_temperature_c: ambient_temp_c,
            total_power_mw,
            available_power_mw,
            curtailed_power_mw,
            turbine_setpoints: setpoints,
            farm_efficiency,
        }
    }

    /// Apply active curtailment schedule rules to a set of turbine setpoints.
    ///
    /// Turbine setpoints may be reduced and the curtailment reason set.
    pub fn apply_curtailment_schedule(
        &self,
        timestamp_h: f64,
        mut setpoints: Vec<TurbineSetpoint>,
    ) -> Vec<TurbineSetpoint> {
        for rule in &self.curtailment_schedule {
            if !rule.is_active(timestamp_h) {
                continue;
            }
            let rated_kw = self.controller.rated_power_mw * 1000.0;
            let max_kw = rated_kw * rule.max_power_pct / 100.0;
            for sp in setpoints.iter_mut() {
                if !rule.applies_to(sp.turbine_id) {
                    continue;
                }
                if sp.power_setpoint_kw > max_kw {
                    sp.power_setpoint_kw = max_kw;
                    sp.curtailment_reason = Some(rule.reason);
                }
            }
        }
        setpoints
    }

    /// Compute farm power [MW] at each wind speed in `wind_speeds`.
    ///
    /// Uses the simplified power curve without wake effects.
    pub fn compute_farm_power_curve(&self, wind_speeds: &[f64]) -> Vec<f64> {
        let rated_kw = self.controller.rated_power_mw * 1000.0;
        let n = self.controller.n_turbines as f64;
        wind_speeds
            .iter()
            .map(|&v| turbine_power_kw(v, rated_kw) * n / 1000.0)
            .collect()
    }

    /// Estimate Annual Energy Production [MWh/year] from a wind rose.
    ///
    /// # Arguments
    /// - `wind_rose` — Slice of `(direction_deg, speed_ms, frequency)` tuples.
    ///   Frequencies should sum to approximately 1.0.
    pub fn estimate_annual_energy_production(&self, wind_rose: &[(f64, f64, f64)]) -> f64 {
        let rated_kw = self.controller.rated_power_mw * 1000.0;
        wind_rose
            .iter()
            .map(|&(dir_deg, speed_ms, freq)| {
                let wake_speeds = self.wake_corrected_speeds(speed_ms, dir_deg);
                let total_kw: f64 = wake_speeds
                    .iter()
                    .map(|&v| turbine_power_kw(v, rated_kw))
                    .sum();
                let power_mw = total_kw / 1000.0;
                power_mw * freq * HOURS_PER_YEAR
            })
            .sum()
    }

    // ── private helpers ────────────────────────────────────────────────────────

    /// Compute Jensen-model wake-corrected wind speeds at each turbine.
    fn wake_corrected_speeds(&self, u_inf: f64, wind_dir_deg: f64) -> Vec<f64> {
        let n = self.turbine_positions.len();
        if n == 0 {
            return Vec::new();
        }
        let dir_rad = wind_dir_deg.to_radians();
        let (sin_d, cos_d) = (dir_rad.sin(), dir_rad.cos());

        // Rotate to wind-aligned frame
        let rotated: Vec<(f64, f64)> = self
            .turbine_positions
            .iter()
            .map(|&(x, y)| {
                let x_w = -x * sin_d - y * cos_d;
                let y_w = -x * cos_d + y * sin_d;
                (x_w, y_w)
            })
            .collect();

        let d = self.wake_steering.rotor_diameter_m;
        let mut speeds = vec![u_inf; n];

        for i in 0..n {
            let mut deficit_sq_sum = 0.0_f64;
            for j in 0..n {
                if i == j {
                    continue;
                }
                let dx = rotated[i].0 - rotated[j].0;
                let dy = (rotated[i].1 - rotated[j].1).abs();
                if dx <= 0.0 {
                    continue;
                }
                let r_wake = d / 2.0 + K_WAKE * dx;
                if dy < r_wake {
                    let deficit = (1.0 - (1.0 - CT).sqrt()) * ((d / 2.0) / r_wake).powi(2);
                    deficit_sq_sum += deficit * deficit;
                }
            }
            speeds[i] = (u_inf * (1.0 - deficit_sq_sum.sqrt())).max(0.0);
        }
        speeds
    }
}

// ── Stand-alone turbine power curve ───────────────────────────────────────────

/// Simplified turbine power curve [kW].
///
/// - Below cut-in (3 m/s) or at/above cut-out (25 m/s): 0 kW
/// - Cut-in to rated (12 m/s): cubic ramp — `P = P_rated × ((v−3)/(12−3))^3`
/// - Rated to cut-out: constant at `rated_kw`
pub fn turbine_power_kw(wind_speed_ms: f64, rated_kw: f64) -> f64 {
    if !(V_CUT_IN..V_CUT_OUT).contains(&wind_speed_ms) {
        0.0
    } else if wind_speed_ms >= V_RATED {
        rated_kw
    } else {
        let frac = (wind_speed_ms - V_CUT_IN) / (V_RATED - V_CUT_IN);
        rated_kw * frac.powi(3)
    }
}

// ── Utilities ─────────────────────────────────────────────────────────────────

/// Absolute angular difference between two bearings [°], result in [0, 180].
fn angle_diff(a: f64, b: f64) -> f64 {
    ((a - b).rem_euclid(360.0)).min((b - a).rem_euclid(360.0))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── helpers ────────────────────────────────────────────────────────────────

    fn make_controller(n: usize, rated_mw: f64) -> PlantController {
        PlantController::new(1, n, rated_mw)
    }

    fn two_turbine_layout() -> Vec<(f64, f64)> {
        vec![(0.0, 0.0), (600.0, 0.0)] // 600 m apart along x-axis
    }

    fn make_manager(n: usize, rated_mw: f64) -> PlantOperationsManager {
        let positions: Vec<(f64, f64)> = (0..n).map(|i| (i as f64 * 600.0, 0.0)).collect();
        let ctrl = make_controller(n, rated_mw);
        PlantOperationsManager::new(ctrl, positions)
    }

    // ── power curve ────────────────────────────────────────────────────────────

    #[test]
    fn test_turbine_power_curve_cutin() {
        assert_eq!(turbine_power_kw(2.0, 2000.0), 0.0);
        assert_eq!(turbine_power_kw(0.0, 2000.0), 0.0);
    }

    #[test]
    fn test_turbine_power_curve_rated() {
        let p = turbine_power_kw(V_RATED, 2000.0);
        assert!((p - 2000.0).abs() < 1e-6, "p={}", p);
    }

    #[test]
    fn test_turbine_power_curve_cutout() {
        assert_eq!(turbine_power_kw(26.0, 2000.0), 0.0);
        assert_eq!(turbine_power_kw(25.0, 2000.0), 0.0); // cut-out is exclusive
    }

    #[test]
    fn test_turbine_power_curve_cubic() {
        // At midpoint of ramp: frac = 0.5, power = rated * 0.5^3 = rated * 0.125
        let v_mid = V_CUT_IN + (V_RATED - V_CUT_IN) * 0.5;
        let p = turbine_power_kw(v_mid, 2000.0);
        let expected = 2000.0 * 0.125;
        assert!((p - expected).abs() < 1.0, "p={} expected≈{}", p, expected);
    }

    // ── frequency droop ────────────────────────────────────────────────────────

    #[test]
    fn test_frequency_droop_high_freq() {
        let ctrl = make_controller(4, 2.0);
        // Frequency 0.5 Hz above nominal → reduce output
        let delta = ctrl.compute_frequency_response(50.5, 4.0);
        assert!(delta < 0.0, "delta should be negative, got {}", delta);
    }

    #[test]
    fn test_frequency_droop_low_freq() {
        let ctrl = make_controller(4, 2.0);
        // Frequency 0.5 Hz below nominal → increase output
        let delta = ctrl.compute_frequency_response(49.5, 4.0);
        assert!(delta > 0.0, "delta should be positive, got {}", delta);
    }

    #[test]
    fn test_frequency_droop_deadband() {
        let ctrl = make_controller(4, 2.0);
        // Within deadband (default ±0.2 Hz)
        let delta = ctrl.compute_frequency_response(50.1, 4.0);
        assert_eq!(delta, 0.0, "within deadband → no response");
    }

    // ── delta control ──────────────────────────────────────────────────────────

    #[test]
    fn test_delta_control_reserve() {
        let mut ctrl = make_controller(4, 2.0);
        ctrl.control_mode = ActivePowerMode::DeltaControl;
        ctrl.delta_reserve_pct = 10.0;
        let available = 6.0_f64;
        let reserve = ctrl.compute_delta_reserve(available);
        assert!((reserve - 0.6).abs() < 1e-9, "reserve={}", reserve);
        // Setpoints should sum to ≤ available - reserve
        let setpoints = ctrl.compute_setpoints(available, 50.0);
        let total: f64 = setpoints.iter().map(|s| s.power_setpoint_kw).sum::<f64>() / 1000.0;
        assert!(
            total <= available - reserve + 1e-6,
            "total={} reserve={}",
            total,
            reserve
        );
    }

    // ── ramp rate ──────────────────────────────────────────────────────────────

    #[test]
    fn test_ramp_rate_limit_up() {
        let ctrl = make_controller(4, 2.0);
        // default ramp_rate = 10%/min of farm_rated = 0.8 MW/min for 4×2 MW
        let dt_s = 30.0; // 0.5 min → max ramp = 0.4 MW
        let max_ramp = ctrl.ramp_rate_mw_per_min * dt_s / 60.0;
        let achieved = ctrl.apply_ramp_limit(0.0, 10.0, dt_s);
        assert!(
            achieved <= max_ramp + 1e-9,
            "achieved={} max_ramp={}",
            achieved,
            max_ramp
        );
    }

    #[test]
    fn test_ramp_rate_limit_down() {
        let ctrl = make_controller(4, 2.0);
        let dt_s = 30.0;
        let max_ramp = ctrl.ramp_rate_mw_per_min * dt_s / 60.0;
        let achieved = ctrl.apply_ramp_limit(8.0, 0.0, dt_s);
        assert!(
            achieved >= 8.0 - max_ramp - 1e-9,
            "achieved={} max_ramp={}",
            achieved,
            max_ramp
        );
    }

    // ── wake model ─────────────────────────────────────────────────────────────

    #[test]
    fn test_wake_loss_downwind() {
        let layout = two_turbine_layout();
        let ws = WakeSteering::new(layout, 120.0);
        // Wind from west (270°): turbine 0 wakes turbine 1
        let factor = ws.compute_wake_loss_factor(270.0);
        assert!(
            factor > 0.0,
            "downwind turbine should show wake loss, factor={}",
            factor
        );
    }

    #[test]
    fn test_wake_loss_upwind() {
        let layout = two_turbine_layout();
        let ws = WakeSteering::new(layout, 120.0);
        // Wind from east (90°): turbine 1 is upwind of turbine 0
        let factor = ws.compute_wake_loss_factor(90.0);
        assert!(factor >= 0.0, "wake loss factor must be non-negative");
    }

    // ── yaw offsets ────────────────────────────────────────────────────────────

    #[test]
    fn test_yaw_offset_static() {
        let layout = two_turbine_layout();
        let mut ws = WakeSteering::new(layout, 120.0);
        ws.mode = WakeSteeringMode::Static;
        // Wind from west (270°): turbine 0 (x=0) is upstream of turbine 1 (x=600)
        let offsets = ws.compute_yaw_offsets(270.0, 8.0);
        assert_eq!(offsets.len(), 2);
        assert!(
            offsets[0] > 0.0,
            "upstream turbine should have yaw offset, got {}",
            offsets[0]
        );
        assert_eq!(
            offsets[1], 0.0,
            "downstream turbine should not have yaw offset"
        );
    }

    // ── curtailment schedule ───────────────────────────────────────────────────

    #[test]
    fn test_curtailment_schedule_active() {
        let mut mgr = make_manager(2, 2.0);
        mgr.curtailment_schedule.push(CurtailmentSchedule {
            start_hour: 22.0,
            end_hour: 6.0,
            max_power_pct: 50.0,
            reason: CurtailmentReason::NoiseCurtailment,
            turbine_ids: vec![],
        });
        let setpoints: Vec<TurbineSetpoint> = (0..2)
            .map(|id| TurbineSetpoint {
                turbine_id: id,
                power_setpoint_kw: 2000.0,
                yaw_offset_deg: 0.0,
                curtailment_reason: None,
                pitch_angle_deg: 0.0,
            })
            .collect();
        // At 23h: schedule active (wraps midnight)
        let result = mgr.apply_curtailment_schedule(23.0, setpoints);
        for sp in &result {
            assert!(
                sp.power_setpoint_kw <= 1000.0 + 1e-9,
                "turbine {} should be curtailed to 50%: {}",
                sp.turbine_id,
                sp.power_setpoint_kw
            );
        }
    }

    #[test]
    fn test_curtailment_schedule_inactive() {
        let mut mgr = make_manager(2, 2.0);
        mgr.curtailment_schedule.push(CurtailmentSchedule {
            start_hour: 22.0,
            end_hour: 6.0,
            max_power_pct: 50.0,
            reason: CurtailmentReason::NoiseCurtailment,
            turbine_ids: vec![],
        });
        let setpoints: Vec<TurbineSetpoint> = (0..2)
            .map(|id| TurbineSetpoint {
                turbine_id: id,
                power_setpoint_kw: 2000.0,
                yaw_offset_deg: 0.0,
                curtailment_reason: None,
                pitch_angle_deg: 0.0,
            })
            .collect();
        // At 12h: schedule inactive
        let result = mgr.apply_curtailment_schedule(12.0, setpoints);
        for sp in &result {
            assert!(
                sp.power_setpoint_kw > 1000.0,
                "turbine {} should not be curtailed at noon: {}",
                sp.turbine_id,
                sp.power_setpoint_kw
            );
        }
    }

    // ── setpoint sum ───────────────────────────────────────────────────────────

    #[test]
    fn test_setpoints_sum() {
        let ctrl = make_controller(5, 2.0);
        let available = 8.0_f64;
        let setpoints = ctrl.compute_setpoints(available, 50.0);
        let total_mw: f64 = setpoints.iter().map(|s| s.power_setpoint_kw).sum::<f64>() / 1000.0;
        // MaximumPower mode: total ≈ available
        assert!(
            (total_mw - available).abs() < 1e-4,
            "total={} available={}",
            total_mw,
            available
        );
    }

    // ── upstream identification ────────────────────────────────────────────────

    #[test]
    fn test_upstream_turbine_identification() {
        let layout = two_turbine_layout();
        let ws = WakeSteering::new(layout, 120.0);
        // Wind from west (270°): turbine 0 (x=0) is upstream of turbine 1 (x=600)
        let upstream_of_1 = ws.upstream_turbines(1, 270.0);
        assert!(
            upstream_of_1.contains(&0),
            "turbine 0 should be upstream of turbine 1: {:?}",
            upstream_of_1
        );
        // Turbine 0 should have no upstream for westerly wind
        let upstream_of_0 = ws.upstream_turbines(0, 270.0);
        assert!(
            upstream_of_0.is_empty(),
            "turbine 0 should have no upstream for westerly wind: {:?}",
            upstream_of_0
        );
    }

    // ── farm power curve ───────────────────────────────────────────────────────

    #[test]
    fn test_farm_power_curve_shape() {
        let mgr = make_manager(3, 2.0);
        let speeds: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let powers = mgr.compute_farm_power_curve(&speeds);
        // Below cut-in → 0
        assert_eq!(powers[0], 0.0, "0 m/s → 0 MW");
        assert_eq!(powers[2], 0.0, "2 m/s → 0 MW");
        // Monotone in ramp region (V_CUT_IN to V_RATED)
        let ramp: Vec<f64> = speeds
            .iter()
            .zip(powers.iter())
            .filter(|&(&v, _)| (V_CUT_IN..V_RATED).contains(&v))
            .map(|(_, &p)| p)
            .collect();
        for w in ramp.windows(2) {
            assert!(
                w[1] >= w[0] - 1e-9,
                "power curve not monotone: {} < {}",
                w[1],
                w[0]
            );
        }
    }

    // ── annual energy production ───────────────────────────────────────────────

    #[test]
    fn test_annual_energy_estimation() {
        let mgr = make_manager(5, 2.0);
        // 4-direction wind rose with equal frequency summing to 1.0
        let wind_rose = vec![
            (0.0, 10.0, 0.25),
            (90.0, 10.0, 0.25),
            (180.0, 10.0, 0.25),
            (270.0, 10.0, 0.25),
        ];
        let aep = mgr.estimate_annual_energy_production(&wind_rose);
        assert!(aep > 0.0, "AEP should be positive, got {}", aep);
        // 5 × 2 MW × 8760 h = 87_600 MWh maximum
        assert!(aep < 100_000.0, "AEP unreasonably large: {}", aep);
    }

    // ── operating step ─────────────────────────────────────────────────────────

    #[test]
    fn test_operating_step_output() {
        let mut mgr = make_manager(3, 2.0);
        let op = mgr.run_operating_step(10.0, 270.0, 15.0, 50.0, 0.0);
        assert_eq!(
            op.turbine_setpoints.len(),
            3,
            "should have one setpoint per turbine"
        );
        assert!(
            op.available_power_mw >= 0.0,
            "available_power_mw must be non-negative"
        );
        assert!(
            op.total_power_mw >= 0.0,
            "total_power_mw must be non-negative"
        );
        assert!(
            op.farm_efficiency >= 0.0 && op.farm_efficiency <= 1.0 + 1e-9,
            "farm_efficiency out of range: {}",
            op.farm_efficiency
        );
        assert!(
            op.curtailed_power_mw >= -1e-6,
            "curtailed_power_mw should not be strongly negative"
        );
    }
}
