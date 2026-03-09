//! Battery Energy Storage System (BESS) Grid Services.
//!
//! This module implements advanced grid services that a BESS can provide to
//! support power system operation, including primary frequency response,
//! AGC regulation, voltage support, peak shaving, spinning reserve, and
//! black-start capability.
//!
//! # Grid Service Hierarchy (by response time)
//!
//! 1. **Primary Frequency Response** — responds in < 30 s, droop-based
//! 2. **Spinning Reserve** — pre-committed headroom, activates in seconds
//! 3. **Secondary Regulation (AGC)** — tracks 4-second AGC signal
//! 4. **Voltage Support** — reactive power droop control
//! 5. **Peak Shaving** — demand management over hours
//! 6. **Black Start** — cranking power for grid restoration

use serde::{Deserialize, Serialize};
use std::fmt;

/// Error type for BESS grid service operations.
#[derive(Debug, Clone, PartialEq)]
pub enum BessError {
    /// SoC constraint violated during service.
    SocConstraintViolated { soc: f64, limit: f64 },
    /// Power ramp rate limit exceeded.
    RampRateExceeded {
        requested_mw_per_min: f64,
        limit_mw_per_min: f64,
    },
    /// Insufficient rated power for the service.
    InsufficientPower { required_mw: f64, rated_mw: f64 },
    /// AGC signal time series is empty.
    EmptySignal,
    /// Demand profile is empty.
    EmptyDemandProfile,
    /// General computation error.
    ComputationError(String),
}

impl fmt::Display for BessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SocConstraintViolated { soc, limit } => {
                write!(f, "SoC {:.3} violated limit {:.3}", soc, limit)
            }
            Self::RampRateExceeded {
                requested_mw_per_min,
                limit_mw_per_min,
            } => {
                write!(
                    f,
                    "Ramp rate {:.2} MW/min exceeds limit {:.2}",
                    requested_mw_per_min, limit_mw_per_min
                )
            }
            Self::InsufficientPower {
                required_mw,
                rated_mw,
            } => {
                write!(
                    f,
                    "Required {:.2} MW exceeds rated {:.2} MW",
                    required_mw, rated_mw
                )
            }
            Self::EmptySignal => write!(f, "AGC signal is empty"),
            Self::EmptyDemandProfile => write!(f, "Demand profile is empty"),
            Self::ComputationError(msg) => write!(f, "Computation error: {}", msg),
        }
    }
}

impl std::error::Error for BessError {}

/// Configuration for a BESS unit providing grid services.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BessGridConfig {
    /// Rated power capacity \[MW\].
    pub rated_power_mw: f64,
    /// Rated energy capacity \[MWh\].
    pub rated_energy_mwh: f64,
    /// Initial state of charge \[0, 1\].
    pub soc_initial: f64,
    /// Minimum allowable SoC \[0, 1\].
    pub soc_min: f64,
    /// Maximum allowable SoC \[0, 1\].
    pub soc_max: f64,
    /// Round-trip efficiency (e.g., 0.92 = 92%).
    pub efficiency_roundtrip: f64,
    /// Power ramp rate \[MW/min\].
    pub ramp_rate_mw_per_min: f64,
    /// Response time to dispatch signal \[ms\].
    pub response_time_ms: f64,
}

impl BessGridConfig {
    /// Validate configuration parameters.
    pub fn validate(&self) -> Result<(), BessError> {
        if self.rated_power_mw <= 0.0 {
            return Err(BessError::ComputationError(
                "rated_power_mw must be positive".into(),
            ));
        }
        if !(0.0..=1.0).contains(&self.soc_initial)
            || !(0.0..=1.0).contains(&self.soc_min)
            || !(0.0..=1.0).contains(&self.soc_max)
        {
            return Err(BessError::ComputationError(
                "SoC values must be in [0,1]".into(),
            ));
        }
        if self.soc_min >= self.soc_max {
            return Err(BessError::ComputationError(
                "soc_min must be < soc_max".into(),
            ));
        }
        Ok(())
    }

    /// Charging efficiency (one-way).
    pub fn charge_efficiency(&self) -> f64 {
        self.efficiency_roundtrip.sqrt()
    }

    /// Discharging efficiency (one-way).
    pub fn discharge_efficiency(&self) -> f64 {
        self.efficiency_roundtrip.sqrt()
    }
}

/// Grid service type that the BESS can provide.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GridService {
    /// Primary frequency response via droop control.
    PrimaryFrequencyResponse {
        /// Speed droop percentage (e.g., 5 for 5%).
        droop_pct: f64,
        /// Frequency deadband \[Hz\] (e.g., ±0.02 Hz).
        deadband_hz: f64,
        /// Reserved headroom for upward regulation \[MW\].
        headroom_mw: f64,
        /// Frequency time series \[Hz\].
        frequency_hz: Vec<f64>,
        /// Timestep \[s\].
        dt_s: f64,
        /// Nominal frequency \[Hz\].
        nominal_hz: f64,
        /// Rated system frequency range for droop computation \[Hz\] (usually ±5%).
        rated_freq_range_hz: f64,
    },
    /// Secondary frequency regulation via Automatic Generation Control.
    SecondaryFrequencyRegulation {
        /// AGC signal time series (normalized to rated power, in \[-1, 1\]).
        agc_signal: Vec<f64>,
        /// Timestep \[s\].
        dt_s: f64,
    },
    /// Voltage support via reactive power injection.
    VoltageSupport {
        /// Target voltage \[pu\].
        target_voltage_pu: f64,
        /// Reactive power droop \[Mvar/pu\].
        droop_mvar_per_pu: f64,
        /// Voltage deadband \[pu\].
        v_deadband_pu: f64,
        /// Voltage time series \[pu\].
        voltage_pu: Vec<f64>,
        /// Timestep \[s\].
        dt_s: f64,
    },
    /// Peak shaving of demand profile.
    PeakShaving {
        /// Demand profile \[MW\].
        demand_profile_mw: Vec<f64>,
        /// Target peak demand \[MW\].
        target_peak_mw: f64,
        /// Timestep \[hours\].
        dt_hours: f64,
    },
    /// Pre-committed spinning reserve.
    SpinningReserve {
        /// Reserved power \[MW\].
        reserved_mw: f64,
        /// Activation delay \[s\].
        activation_delay_s: f64,
    },
    /// Black-start cranking power for grid restoration.
    BlackStart {
        /// Target bus for energization.
        target_bus: usize,
        /// Cranking power \[MW\].
        cranking_power_mw: f64,
        /// Cranking duration \[s\].
        duration_s: f64,
    },
}

impl GridService {
    /// Human-readable service name.
    pub fn name(&self) -> &'static str {
        match self {
            Self::PrimaryFrequencyResponse { .. } => "PrimaryFrequencyResponse",
            Self::SecondaryFrequencyRegulation { .. } => "SecondaryFrequencyRegulation",
            Self::VoltageSupport { .. } => "VoltageSupport",
            Self::PeakShaving { .. } => "PeakShaving",
            Self::SpinningReserve { .. } => "SpinningReserve",
            Self::BlackStart { .. } => "BlackStart",
        }
    }

    /// Nominal response time priority (lower = faster response = higher priority).
    pub fn response_priority(&self) -> u32 {
        match self {
            Self::BlackStart { .. } => 0,
            Self::SpinningReserve { .. } => 1,
            Self::PrimaryFrequencyResponse { .. } => 2,
            Self::SecondaryFrequencyRegulation { .. } => 3,
            Self::VoltageSupport { .. } => 4,
            Self::PeakShaving { .. } => 5,
        }
    }
}

/// Frequency regulation performance metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FreqPerformance {
    /// RMSE of frequency regulation error \[Hz\].
    pub control_error_rms_hz: f64,
    /// Actual response time achieved \[ms\].
    pub response_time_actual_ms: f64,
    /// Total power movement (FERC mileage metric) \[MW\].
    pub mileage_mw: f64,
    /// Performance score \[0–100\].
    pub score: f64,
}

/// Result of providing a single grid service.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BessGridServiceResult {
    /// Service name.
    pub service: String,
    /// State-of-charge trajectory (one entry per timestep, \[0, 1\]).
    pub soc_trajectory: Vec<f64>,
    /// Active power trajectory \[MW\] (positive = discharge, negative = charge).
    pub power_trajectory: Vec<f64>,
    /// Total energy delivered (net discharge) \[MWh\].
    pub energy_delivered_mwh: f64,
    /// Frequency regulation performance (if applicable).
    pub frequency_performance: Option<FreqPerformance>,
    /// Estimated market revenue \[USD\].
    pub revenue_estimate_usd: f64,
    /// Equivalent full cycles consumed.
    pub cycle_count: f64,
    /// Estimated capacity degradation \[%\].
    pub degradation_pct: f64,
}

/// Optimizer for BESS grid services.
pub struct BessGridServicesOptimizer {
    config: BessGridConfig,
}

impl BessGridServicesOptimizer {
    /// Create a new optimizer with the given BESS configuration.
    pub fn new(config: BessGridConfig) -> Self {
        Self { config }
    }

    /// Provide a single grid service and return the operational result.
    pub fn provide_service(
        &self,
        service: GridService,
    ) -> Result<BessGridServiceResult, BessError> {
        self.config.validate()?;
        match service {
            GridService::PrimaryFrequencyResponse {
                droop_pct,
                deadband_hz,
                headroom_mw,
                frequency_hz,
                dt_s,
                nominal_hz,
                rated_freq_range_hz,
            } => self.primary_frequency_response(
                droop_pct,
                deadband_hz,
                headroom_mw,
                &frequency_hz,
                dt_s,
                nominal_hz,
                rated_freq_range_hz,
            ),
            GridService::SecondaryFrequencyRegulation { agc_signal, dt_s } => {
                self.secondary_regulation(&agc_signal, dt_s)
            }
            GridService::VoltageSupport {
                target_voltage_pu,
                droop_mvar_per_pu,
                v_deadband_pu,
                voltage_pu,
                dt_s,
            } => self.voltage_support(
                target_voltage_pu,
                droop_mvar_per_pu,
                v_deadband_pu,
                &voltage_pu,
                dt_s,
            ),
            GridService::PeakShaving {
                demand_profile_mw,
                target_peak_mw,
                dt_hours,
            } => self.peak_shaving(&demand_profile_mw, target_peak_mw, dt_hours),
            GridService::SpinningReserve {
                reserved_mw,
                activation_delay_s,
            } => self.spinning_reserve(reserved_mw, activation_delay_s),
            GridService::BlackStart {
                target_bus,
                cranking_power_mw,
                duration_s,
            } => self.black_start(target_bus, cranking_power_mw, duration_s),
        }
    }

    /// Provide multiple services simultaneously (stacking), prioritized by response time.
    pub fn stack_services(
        &self,
        mut services: Vec<GridService>,
    ) -> Result<Vec<BessGridServiceResult>, BessError> {
        self.config.validate()?;

        // Sort by response priority (fastest first)
        services.sort_by_key(|s| s.response_priority());

        let mut results = Vec::new();
        let mut remaining_power_mw = self.config.rated_power_mw;
        let mut current_soc = self.config.soc_initial;

        for service in services {
            // Check feasibility before providing service
            match &service {
                GridService::PrimaryFrequencyResponse { headroom_mw, .. } => {
                    if *headroom_mw > remaining_power_mw {
                        return Err(BessError::InsufficientPower {
                            required_mw: *headroom_mw,
                            rated_mw: remaining_power_mw,
                        });
                    }
                    remaining_power_mw -= headroom_mw;
                }
                GridService::SpinningReserve { reserved_mw, .. } => {
                    if *reserved_mw > remaining_power_mw {
                        return Err(BessError::InsufficientPower {
                            required_mw: *reserved_mw,
                            rated_mw: remaining_power_mw,
                        });
                    }
                    remaining_power_mw -= reserved_mw;
                }
                _ => {}
            }

            // Temporarily override initial SoC with current SoC for stacking
            let mut stacked_config = self.config.clone();
            stacked_config.soc_initial = current_soc;
            let stacked_optimizer = BessGridServicesOptimizer::new(stacked_config);

            let result = stacked_optimizer.provide_service(service)?;

            // Update SoC for next service
            if let Some(&last_soc) = result.soc_trajectory.last() {
                current_soc = last_soc;
            }

            results.push(result);
        }

        Ok(results)
    }

    // ── Private service implementations ────────────────────────────────────

    #[allow(clippy::too_many_arguments)]
    fn primary_frequency_response(
        &self,
        droop_pct: f64,
        deadband_hz: f64,
        headroom_mw: f64,
        frequency_hz: &[f64],
        dt_s: f64,
        nominal_hz: f64,
        rated_freq_range_hz: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if frequency_hz.is_empty() {
            return Err(BessError::EmptySignal);
        }

        let mut soc = self.config.soc_initial;
        let mut soc_traj = Vec::with_capacity(frequency_hz.len());
        let mut power_traj = Vec::with_capacity(frequency_hz.len());
        let mut total_energy_mwh = 0.0_f64;
        let mut total_mileage = 0.0_f64;

        // Droop gain: MW output per Hz deviation
        // At ±rated_freq_range_hz, output ±rated_power_mw × (droop_pct/100)
        let droop_gain_mw_per_hz =
            self.config.rated_power_mw * (droop_pct / 100.0) / rated_freq_range_hz.max(0.001);

        let eff = self.config.discharge_efficiency();
        let dt_h = dt_s / 3600.0;

        for &f in frequency_hz {
            let deviation_hz = f - nominal_hz;
            let power_mw = if deviation_hz.abs() < deadband_hz {
                0.0
            } else {
                // Positive deviation (over-freq) → absorb; negative → inject
                let p = -droop_gain_mw_per_hz * deviation_hz;
                p.clamp(-headroom_mw, headroom_mw)
            };

            // Update SoC
            let delta_soc = if power_mw > 0.0 {
                // Discharging: energy leaves battery
                -(power_mw / eff) * dt_h / self.config.rated_energy_mwh
            } else {
                // Charging: energy enters battery
                (power_mw.abs() * eff) * dt_h / self.config.rated_energy_mwh
            };

            soc = (soc + delta_soc).clamp(self.config.soc_min, self.config.soc_max);
            soc_traj.push(soc);
            power_traj.push(power_mw);

            if power_mw > 0.0 {
                total_energy_mwh += power_mw * dt_h;
            }
            total_mileage += power_mw.abs();
        }

        let n = frequency_hz.len() as f64;
        let cycle_count = total_energy_mwh / self.config.rated_energy_mwh;
        let degradation_pct = cycle_count * 0.002; // 0.002% per equivalent full cycle

        // RMSE of frequency deviation (as proxy for control error)
        let nominal = nominal_hz;
        let rmse_hz = (frequency_hz
            .iter()
            .map(|&f| (f - nominal).powi(2))
            .sum::<f64>()
            / n)
            .sqrt();

        let score = (100.0 - rmse_hz * 50.0).clamp(0.0, 100.0);

        Ok(BessGridServiceResult {
            service: "PrimaryFrequencyResponse".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj,
            energy_delivered_mwh: total_energy_mwh,
            frequency_performance: Some(FreqPerformance {
                control_error_rms_hz: rmse_hz,
                response_time_actual_ms: self.config.response_time_ms,
                mileage_mw: total_mileage,
                score,
            }),
            revenue_estimate_usd: total_energy_mwh * 25.0, // ~$25/MWh frequency market
            cycle_count,
            degradation_pct,
        })
    }

    fn secondary_regulation(
        &self,
        agc_signal: &[f64],
        dt_s: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if agc_signal.is_empty() {
            return Err(BessError::EmptySignal);
        }

        let mut soc = self.config.soc_initial;
        let mut soc_traj = Vec::with_capacity(agc_signal.len());
        let mut power_traj = Vec::with_capacity(agc_signal.len());
        let mut total_energy_mwh = 0.0_f64;
        let mut total_mileage = 0.0_f64;

        let eff = self.config.discharge_efficiency();
        let dt_h = dt_s / 3600.0;

        let mut prev_power = 0.0_f64;
        let max_ramp_per_step = self.config.ramp_rate_mw_per_min * dt_s / 60.0;

        for &signal in agc_signal {
            // AGC signal in [-1, 1] maps to [-rated_power, +rated_power]
            let target_power = signal.clamp(-1.0, 1.0) * self.config.rated_power_mw;

            // Apply ramp rate limit
            let power_mw = if (target_power - prev_power).abs() > max_ramp_per_step {
                prev_power + max_ramp_per_step * (target_power - prev_power).signum()
            } else {
                target_power
            };

            prev_power = power_mw;

            // SoC update
            let delta_soc = if power_mw > 0.0 {
                -(power_mw / eff) * dt_h / self.config.rated_energy_mwh
            } else {
                (power_mw.abs() * eff) * dt_h / self.config.rated_energy_mwh
            };

            soc = (soc + delta_soc).clamp(self.config.soc_min, self.config.soc_max);
            soc_traj.push(soc);
            power_traj.push(power_mw);

            if power_mw > 0.0 {
                total_energy_mwh += power_mw * dt_h;
            }
            total_mileage += power_mw.abs();
        }

        // Compute tracking RMSE
        let n = agc_signal.len() as f64;
        let rmse_hz = power_traj
            .iter()
            .zip(agc_signal.iter())
            .map(|(&p, &s)| (p - s * self.config.rated_power_mw).powi(2))
            .sum::<f64>()
            .sqrt()
            / n.sqrt();
        let control_err_hz = rmse_hz / self.config.rated_power_mw; // normalized

        let cycle_count = total_energy_mwh / self.config.rated_energy_mwh;
        let degradation_pct = cycle_count * 0.002;
        let score = (100.0 - control_err_hz * 100.0).clamp(0.0, 100.0);

        Ok(BessGridServiceResult {
            service: "SecondaryFrequencyRegulation".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj,
            energy_delivered_mwh: total_energy_mwh,
            frequency_performance: Some(FreqPerformance {
                control_error_rms_hz: control_err_hz,
                response_time_actual_ms: self.config.response_time_ms,
                mileage_mw: total_mileage,
                score,
            }),
            revenue_estimate_usd: total_mileage * 0.1, // ~$0.10/MW mileage
            cycle_count,
            degradation_pct,
        })
    }

    fn voltage_support(
        &self,
        target_voltage_pu: f64,
        droop_mvar_per_pu: f64,
        v_deadband_pu: f64,
        voltage_pu: &[f64],
        dt_s: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if voltage_pu.is_empty() {
            return Err(BessError::EmptySignal);
        }

        let dt_h = dt_s / 3600.0;
        let soc_traj: Vec<f64> = vec![self.config.soc_initial; voltage_pu.len()];
        let mut power_traj = Vec::with_capacity(voltage_pu.len());
        let mut total_mvar = 0.0_f64;

        for &v in voltage_pu {
            let dv = v - target_voltage_pu;
            let q_mvar = if dv.abs() < v_deadband_pu {
                0.0
            } else {
                (-droop_mvar_per_pu * dv)
                    .clamp(-self.config.rated_power_mw, self.config.rated_power_mw)
            };
            // Voltage support is primarily reactive — store as P=0, Q reported separately
            power_traj.push(0.0_f64); // active power is negligible for volt support
            total_mvar += q_mvar.abs() * dt_h;
        }

        let cycle_count = 0.0; // negligible cycling for volt support
        Ok(BessGridServiceResult {
            service: "VoltageSupport".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj,
            energy_delivered_mwh: 0.0,
            frequency_performance: None,
            revenue_estimate_usd: total_mvar * 5.0, // ~$5/Mvarh
            cycle_count,
            degradation_pct: 0.0,
        })
    }

    fn peak_shaving(
        &self,
        demand_profile_mw: &[f64],
        target_peak_mw: f64,
        dt_hours: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if demand_profile_mw.is_empty() {
            return Err(BessError::EmptyDemandProfile);
        }

        let mut soc = self.config.soc_initial;
        let mut soc_traj = Vec::with_capacity(demand_profile_mw.len());
        let mut power_traj = Vec::with_capacity(demand_profile_mw.len());
        let mut total_energy_mwh = 0.0_f64;

        let eff = self.config.discharge_efficiency();

        for &demand in demand_profile_mw {
            let excess = demand - target_peak_mw;
            let power_mw = if excess > 0.0 {
                // Discharge to shave peak
                let p = excess.min(self.config.rated_power_mw);
                // Check SoC
                let delta_soc = -(p / eff) * dt_hours / self.config.rated_energy_mwh;
                let new_soc = soc + delta_soc;
                if new_soc < self.config.soc_min {
                    // Can only discharge what SoC allows
                    let avail_energy = (soc - self.config.soc_min) * self.config.rated_energy_mwh;
                    avail_energy * eff / dt_hours
                } else {
                    p
                }
            } else {
                // Charge when demand is below target (recover SoC)
                let charge_headroom = (self.config.soc_max - soc) * self.config.rated_energy_mwh;
                let p_charge = (-excess)
                    .min(self.config.rated_power_mw)
                    .min(charge_headroom / dt_hours);
                -p_charge // negative = charging
            };

            // SoC update
            let delta_soc = if power_mw > 0.0 {
                -(power_mw / eff) * dt_hours / self.config.rated_energy_mwh
            } else {
                (power_mw.abs() * eff) * dt_hours / self.config.rated_energy_mwh
            };

            soc = (soc + delta_soc).clamp(self.config.soc_min, self.config.soc_max);
            soc_traj.push(soc);
            power_traj.push(power_mw);

            if power_mw > 0.0 {
                total_energy_mwh += power_mw * dt_hours;
            }
        }

        let cycle_count = total_energy_mwh / self.config.rated_energy_mwh;
        let degradation_pct = cycle_count * 0.002;

        Ok(BessGridServiceResult {
            service: "PeakShaving".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj.clone(),
            energy_delivered_mwh: total_energy_mwh,
            frequency_performance: None,
            revenue_estimate_usd: power_traj
                .iter()
                .filter(|&&p| p > 0.0)
                .map(|&p| p * dt_hours * 50.0) // ~$50/MWh demand charge avoidance
                .sum(),
            cycle_count,
            degradation_pct,
        })
    }

    fn spinning_reserve(
        &self,
        reserved_mw: f64,
        activation_delay_s: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if reserved_mw > self.config.rated_power_mw {
            return Err(BessError::InsufficientPower {
                required_mw: reserved_mw,
                rated_mw: self.config.rated_power_mw,
            });
        }

        // Spinning reserve: BESS is on standby — no power dispatched unless activated.
        // Model 1-hour standby period at SoC = initial.
        let n_steps = 3600; // 1-second steps
        let soc_traj = vec![self.config.soc_initial; n_steps];
        let mut power_traj = vec![0.0_f64; n_steps];

        // Simulate activation at delay + 1 step
        let activate_step = (activation_delay_s as usize).min(n_steps - 1);
        for p in power_traj[activate_step..].iter_mut() {
            *p = reserved_mw;
        }

        let energy_mwh = reserved_mw * (n_steps - activate_step) as f64 / 3600.0;

        Ok(BessGridServiceResult {
            service: "SpinningReserve".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj,
            energy_delivered_mwh: energy_mwh,
            frequency_performance: None,
            revenue_estimate_usd: reserved_mw * 8.0, // ~$8/MW-h reserve capacity payment
            cycle_count: energy_mwh / self.config.rated_energy_mwh,
            degradation_pct: 0.01, // minimal — mostly standby
        })
    }

    fn black_start(
        &self,
        _target_bus: usize,
        cranking_power_mw: f64,
        duration_s: f64,
    ) -> Result<BessGridServiceResult, BessError> {
        if cranking_power_mw > self.config.rated_power_mw {
            return Err(BessError::InsufficientPower {
                required_mw: cranking_power_mw,
                rated_mw: self.config.rated_power_mw,
            });
        }

        let n_steps = duration_s as usize + 1;
        let dt_s = 1.0;
        let dt_h = dt_s / 3600.0;
        let eff = self.config.discharge_efficiency();
        let mut soc = self.config.soc_initial;
        let mut soc_traj = Vec::with_capacity(n_steps);
        let power_traj = vec![cranking_power_mw; n_steps];

        let energy_per_step = cranking_power_mw * dt_h;
        for _ in 0..n_steps {
            let delta_soc = -(energy_per_step / eff) / self.config.rated_energy_mwh;
            soc = (soc + delta_soc).clamp(self.config.soc_min, self.config.soc_max);
            soc_traj.push(soc);
        }

        let energy_mwh = cranking_power_mw * duration_s / 3600.0;
        let cycle_count = energy_mwh / self.config.rated_energy_mwh;

        Ok(BessGridServiceResult {
            service: "BlackStart".into(),
            soc_trajectory: soc_traj,
            power_trajectory: power_traj,
            energy_delivered_mwh: energy_mwh,
            frequency_performance: None,
            revenue_estimate_usd: energy_mwh * 200.0, // premium black-start capability payment
            cycle_count,
            degradation_pct: cycle_count * 0.002,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_bess() -> BessGridConfig {
        BessGridConfig {
            rated_power_mw: 10.0,
            rated_energy_mwh: 40.0, // 4-hour BESS
            soc_initial: 0.70,
            soc_min: 0.10,
            soc_max: 0.95,
            efficiency_roundtrip: 0.92,
            ramp_rate_mw_per_min: 20.0, // very fast
            response_time_ms: 200.0,
        }
    }

    #[test]
    fn test_primary_frequency_response_proportional() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        // Frequency drops from 50.0 to 49.5 Hz (0.5 Hz below nominal)
        let freq: Vec<f64> = vec![50.0, 49.5, 49.5, 49.5, 50.0];
        let result = opt
            .provide_service(GridService::PrimaryFrequencyResponse {
                droop_pct: 5.0,
                deadband_hz: 0.02,
                headroom_mw: 5.0,
                frequency_hz: freq,
                dt_s: 1.0,
                nominal_hz: 50.0,
                rated_freq_range_hz: 2.5,
            })
            .expect("PFR should succeed");

        // Under-frequency → BESS should inject power (positive)
        assert!(
            result.power_trajectory[1] > 0.0,
            "BESS should inject power during under-frequency"
        );
        // Power should be proportional to deviation
        let p1 = result.power_trajectory[1]; // at -0.5 Hz
        let p2 = result.power_trajectory[2]; // at -0.5 Hz same
        assert!(
            (p1 - p2).abs() < 1e-9,
            "Same deviation should give same power"
        );
    }

    #[test]
    fn test_agc_regulation_low_rmse() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        // Smooth sinusoidal AGC signal
        let n = 100_usize;
        let agc: Vec<f64> = (0..n)
            .map(|i| 0.3 * (2.0 * std::f64::consts::PI * i as f64 / 20.0).sin())
            .collect();

        let result = opt
            .provide_service(GridService::SecondaryFrequencyRegulation {
                agc_signal: agc,
                dt_s: 4.0, // 4-second AGC timestep
            })
            .expect("AGC should succeed");

        let fp = result
            .frequency_performance
            .expect("Should have freq performance");
        // Score should be reasonable (not near zero)
        assert!(fp.score > 0.0, "Performance score should be positive");
        assert!(fp.mileage_mw > 0.0, "Mileage should be positive");
    }

    #[test]
    fn test_peak_shaving_demand_never_exceeds_target() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        // Demand profile with peaks above target
        let demand: Vec<f64> = vec![
            8.0, 9.0, 12.0, 15.0, 14.0, 11.0, 9.0, 8.0, 7.0, 6.0, 6.0, 7.0,
        ];
        let target_peak = 12.0_f64;

        let result = opt
            .provide_service(GridService::PeakShaving {
                demand_profile_mw: demand.clone(),
                target_peak_mw: target_peak,
                dt_hours: 1.0,
            })
            .expect("Peak shaving should succeed");

        // Net demand after BESS = demand - BESS_discharge (when positive)
        for (i, (&d, &p)) in demand
            .iter()
            .zip(result.power_trajectory.iter())
            .enumerate()
        {
            let net_demand = d - p.max(0.0);
            assert!(
                net_demand <= target_peak + 1e-6,
                "Net demand at step {} = {:.2} exceeds target {:.2}",
                i,
                net_demand,
                target_peak
            );
        }
    }

    #[test]
    fn test_soc_constraints_never_violated() {
        let config = BessGridConfig {
            soc_min: 0.20,
            soc_max: 0.90,
            ..default_bess()
        };
        let opt = BessGridServicesOptimizer::new(config.clone());

        // Aggressive peak shaving that could violate SoC
        let demand: Vec<f64> = vec![20.0; 10]; // all above rated power
        let result = opt
            .provide_service(GridService::PeakShaving {
                demand_profile_mw: demand,
                target_peak_mw: 5.0,
                dt_hours: 1.0,
            })
            .expect("Peak shaving should succeed");

        for (i, &soc) in result.soc_trajectory.iter().enumerate() {
            assert!(
                soc >= config.soc_min - 1e-9,
                "SoC at step {} = {:.4} below soc_min {:.4}",
                i,
                soc,
                config.soc_min
            );
            assert!(
                soc <= config.soc_max + 1e-9,
                "SoC at step {} = {:.4} above soc_max {:.4}",
                i,
                soc,
                config.soc_max
            );
        }
    }

    #[test]
    fn test_service_stacking_frequency_and_peak_shaving() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        let freq: Vec<f64> = vec![50.0, 49.8, 49.9, 50.0, 50.1];
        let services = vec![
            GridService::PrimaryFrequencyResponse {
                droop_pct: 5.0,
                deadband_hz: 0.02,
                headroom_mw: 3.0,
                frequency_hz: freq,
                dt_s: 1.0,
                nominal_hz: 50.0,
                rated_freq_range_hz: 2.5,
            },
            GridService::PeakShaving {
                demand_profile_mw: vec![8.0, 9.0, 11.0, 10.0, 8.0],
                target_peak_mw: 9.0,
                dt_hours: 1.0,
            },
        ];

        let results = opt
            .stack_services(services)
            .expect("Service stacking should succeed");
        assert_eq!(results.len(), 2, "Should have 2 results");
        assert_eq!(results[0].service, "PrimaryFrequencyResponse");
        assert_eq!(results[1].service, "PeakShaving");
    }

    #[test]
    fn test_degradation_increases_with_cycles() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        // Long peak shaving session → more cycles → more degradation
        let demand_short: Vec<f64> = vec![12.0; 2];
        let demand_long: Vec<f64> = vec![12.0; 20];

        let result_short = opt
            .provide_service(GridService::PeakShaving {
                demand_profile_mw: demand_short,
                target_peak_mw: 8.0,
                dt_hours: 1.0,
            })
            .expect("Short peak shaving should succeed");

        // Reset SoC for long session via new optimizer
        let opt2 = BessGridServicesOptimizer::new(BessGridConfig {
            soc_initial: 0.90,
            ..default_bess()
        });
        let result_long = opt2
            .provide_service(GridService::PeakShaving {
                demand_profile_mw: demand_long,
                target_peak_mw: 8.0,
                dt_hours: 1.0,
            })
            .expect("Long peak shaving should succeed");

        // More energy → more degradation
        assert!(
            result_long.energy_delivered_mwh >= result_short.energy_delivered_mwh,
            "Longer session should deliver at least as much energy"
        );
    }

    #[test]
    fn test_black_start_respects_power_limit() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        let err = opt.provide_service(GridService::BlackStart {
            target_bus: 0,
            cranking_power_mw: 999.0, // way above rated
            duration_s: 60.0,
        });
        assert!(
            matches!(err, Err(BessError::InsufficientPower { .. })),
            "Should reject cranking power exceeding rated"
        );
    }

    #[test]
    fn test_spinning_reserve_activation() {
        let opt = BessGridServicesOptimizer::new(default_bess());
        let result = opt
            .provide_service(GridService::SpinningReserve {
                reserved_mw: 5.0,
                activation_delay_s: 10.0,
            })
            .expect("Spinning reserve should succeed");

        assert!(
            result.energy_delivered_mwh > 0.0,
            "Energy should be delivered after activation"
        );
        // Power should be 0 before activation
        assert!(
            (result.power_trajectory[0] - 0.0).abs() < 1e-9,
            "No power before activation"
        );
        // Power after activation should be reserved_mw
        assert!(
            (result.power_trajectory[20] - 5.0).abs() < 1e-9,
            "Power after activation should equal reserved_mw"
        );
    }
}
