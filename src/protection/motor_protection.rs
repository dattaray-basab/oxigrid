//! Motor drive protection system implementing NEMA MG-1 / IEC 60034-1.
//!
//! Provides thermal overload (trip classes 5/10/20/30), stall / locked-rotor
//! detection, phase-voltage unbalance, ground-fault, and definite-time
//! undervoltage protection for induction and synchronous motors.
//!
//! # Quick start
//! ```rust,no_run
//! use oxigrid::protection::motor_protection::{
//!     MotorProtectionRelay, MotorProtConfig, ThermalClass,
//! };
//! let config = MotorProtConfig::default();
//! let relay = MotorProtectionRelay::new(
//!     "M1".into(), 100.0, 0.4, 1.15, ThermalClass::ClassF, config,
//! );
//! ```

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Thermal insulation class
// ---------------------------------------------------------------------------

/// Insulation thermal class per IEC 60034-1.
///
/// Determines the maximum winding temperature \[°C\] and influences thermal
/// time-constant scaling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThermalClass {
    /// Class A — maximum temperature 105 \[°C\]
    ClassA,
    /// Class B — maximum temperature 130 \[°C\]
    ClassB,
    /// Class F — maximum temperature 155 \[°C\]
    ClassF,
    /// Class H — maximum temperature 180 \[°C\]
    ClassH,
}

impl ThermalClass {
    /// Maximum allowable winding temperature \[°C\].
    pub fn max_temp_celsius(self) -> f64 {
        match self {
            ThermalClass::ClassA => 105.0,
            ThermalClass::ClassB => 130.0,
            ThermalClass::ClassF => 155.0,
            ThermalClass::ClassH => 180.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the motor protection relay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorProtConfig {
    /// NEMA trip class (5, 10, 20, or 30).
    pub overload_trip_class: u8,
    /// Locked-rotor thermal limit \[s\] — maximum allowable stall duration.
    pub locked_rotor_time_s: f64,
    /// Phase-voltage unbalance trip threshold \[%\] (NEMA MG-1 definition).
    pub phase_unbalance_pct_trip: f64,
    /// Under-voltage trip threshold \[pu\] — definite time, 500 ms delay.
    pub undervoltage_pu_trip: f64,
    /// Ground-fault (earth-leakage) instantaneous trip threshold \[A\].
    pub ground_fault_threshold_a: f64,
    /// Minimum time between motor restarts \[s\] (thermal restart lockout).
    pub restart_lockout_s: f64,
    /// Service-factor overload allowance applied to rated current.
    pub overload_sf_factor: f64,
}

impl Default for MotorProtConfig {
    fn default() -> Self {
        Self {
            overload_trip_class: 10,
            locked_rotor_time_s: 10.0,
            phase_unbalance_pct_trip: 5.0,
            undervoltage_pu_trip: 0.85,
            ground_fault_threshold_a: 1.0,
            restart_lockout_s: 300.0,
            overload_sf_factor: 1.15,
        }
    }
}

// ---------------------------------------------------------------------------
// Trip cause / record
// ---------------------------------------------------------------------------

/// Cause of a motor protection trip.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TripCause {
    /// Thermal overload — winding temperature model reached 100 %.
    ThermalOverload,
    /// Locked rotor / stall — motor failed to accelerate within time limit.
    LockedRotor,
    /// Phase-voltage unbalance exceeded threshold.
    PhaseUnbalance,
    /// Ground / earth-leakage current exceeded threshold.
    GroundFault,
    /// Supply voltage depressed below threshold for > 500 \[ms\].
    Undervoltage,
    /// Manual trip command.
    ManualTrip,
}

/// Single motor protection trip event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorTrip {
    /// Simulation / relay time at which trip occurred \[s\].
    pub time_s: f64,
    /// Root cause of this trip.
    pub cause: TripCause,
    /// Thermal state (0.0–1.0+) at trip instant.
    pub thermal_state_at_trip: f64,
    /// Phase current magnitude at trip instant \[A\].
    pub current_at_trip_a: f64,
}

// ---------------------------------------------------------------------------
// Protection errors
// ---------------------------------------------------------------------------

/// Error returned by protection check functions when a trip condition is met.
#[derive(Debug, Clone)]
pub enum MotorProtError {
    /// Thermal model reached 100 % — winding at limit.
    ThermalTrip,
    /// Locked-rotor / stall time exceeded.
    LockedRotorTrip,
    /// Phase-voltage unbalance exceeded threshold; carries unbalance \[%\].
    PhaseUnbalanceTrip(f64),
    /// Earth-leakage current exceeded threshold.
    GroundFaultTrip,
    /// Supply voltage below threshold for > 500 \[ms\].
    UndervoltageTrip,
}

impl std::fmt::Display for MotorProtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MotorProtError::ThermalTrip => write!(f, "Motor thermal overload trip"),
            MotorProtError::LockedRotorTrip => write!(f, "Motor locked-rotor trip"),
            MotorProtError::PhaseUnbalanceTrip(pct) => {
                write!(f, "Phase unbalance trip: {:.2} %", pct)
            }
            MotorProtError::GroundFaultTrip => write!(f, "Motor ground-fault trip"),
            MotorProtError::UndervoltageTrip => write!(f, "Motor under-voltage trip"),
        }
    }
}

impl std::error::Error for MotorProtError {}

// ---------------------------------------------------------------------------
// Status snapshot
// ---------------------------------------------------------------------------

/// Snapshot of all motor protection alarm and trip bits.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorProtStatus {
    /// Current thermal utilisation (0.0 = cold, 1.0 = at limit).
    pub thermal_state: f64,
    /// Thermal alarm active (θ > 0.9).
    pub thermal_alarm: bool,
    /// Thermal trip has occurred (θ reached 1.0).
    pub thermal_trip: bool,
    /// Last computed phase-voltage unbalance \[%\].
    pub phase_unbalance_pct: f64,
    /// Phase-unbalance trip flag.
    pub phase_unbalance_trip: bool,
    /// Locked-rotor trip flag.
    pub locked_rotor_trip: bool,
    /// Ground-fault trip flag.
    pub ground_fault_trip: bool,
    /// Under-voltage trip flag.
    pub undervoltage_trip: bool,
    /// Restart lockout active (cooling period not yet elapsed).
    pub restart_locked_out: bool,
    /// Total number of trips recorded in history.
    pub trip_count: usize,
}

// ---------------------------------------------------------------------------
// Relay
// ---------------------------------------------------------------------------

/// Motor drive protection relay implementing NEMA MG-1 / IEC 60034-1.
///
/// # Thermal model
/// The thermal utilisation θ (0 = cold, 1 = winding limit) evolves as:
/// - **Heating**: `dθ/dt = ((I/I_rated)² − θ) / τ_heat`
/// - **Cooling**: `dθ/dt = −θ / τ_cool`  (τ_cool = 2 × τ_heat)
///
/// where `τ_heat = trip_class × 60 / ln(M² / (M² − 1))` with `M = I/I_rated`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorProtectionRelay {
    /// Motor identifier tag.
    pub motor_id: String,
    /// Full-load (rated) current \[A\].
    pub rated_current_a: f64,
    /// Rated terminal voltage \[kV\].
    pub rated_voltage_kv: f64,
    /// Service factor (typically 1.0–1.15).
    pub service_factor: f64,
    /// Insulation thermal class.
    pub thermal_class: ThermalClass,
    /// Protection configuration.
    pub config: MotorProtConfig,

    // -- internal state (not pub) --
    /// Thermal utilisation (0.0–1.0+).
    thermal_state: f64,
    /// Relay clock at last thermal update \[s\].
    last_update_s: f64,
    /// Recorded trip events.
    trip_history: Vec<MotorTrip>,
    /// Accumulated stall (locked-rotor) time \[s\].
    stall_time_s: f64,
    /// Under-voltage timer accumulator \[s\].
    undervoltage_timer_s: f64,
    /// Relay clock at last trip \[s\] (used for restart lockout).
    last_trip_time_s: f64,
    /// Last phase-unbalance value \[%\] (for status snapshot).
    last_unbalance_pct: f64,
    /// Trip-cause flags for status snapshot.
    tripped_thermal: bool,
    tripped_phase: bool,
    tripped_locked_rotor: bool,
    tripped_ground: bool,
    tripped_undervoltage: bool,
}

impl MotorProtectionRelay {
    /// Construct a new relay.
    ///
    /// # Arguments
    /// * `motor_id` — tag / name string
    /// * `rated_current_a` — full-load current \[A\]
    /// * `rated_voltage_kv` — rated terminal voltage \[kV\]
    /// * `service_factor` — service factor (e.g. 1.15)
    /// * `thermal_class` — IEC insulation class
    /// * `config` — protection settings
    pub fn new(
        motor_id: String,
        rated_current_a: f64,
        rated_voltage_kv: f64,
        service_factor: f64,
        thermal_class: ThermalClass,
        config: MotorProtConfig,
    ) -> Self {
        Self {
            motor_id,
            rated_current_a,
            rated_voltage_kv,
            service_factor,
            thermal_class,
            config,
            thermal_state: 0.0,
            last_update_s: 0.0,
            trip_history: Vec::new(),
            stall_time_s: 0.0,
            undervoltage_timer_s: 0.0,
            last_trip_time_s: f64::NEG_INFINITY,
            last_unbalance_pct: 0.0,
            tripped_thermal: false,
            tripped_phase: false,
            tripped_locked_rotor: false,
            tripped_ground: false,
            tripped_undervoltage: false,
        }
    }

    // -----------------------------------------------------------------------
    // Thermal model
    // -----------------------------------------------------------------------

    /// Update the thermal state using a first-order exponential model.
    ///
    /// # Arguments
    /// * `current_a` — instantaneous phase current magnitude \[A\]
    /// * `dt_s` — elapsed time step \[s\]
    /// * `current_time_s` — current relay clock \[s\]
    ///
    /// # Errors
    /// Returns [`MotorProtError::ThermalTrip`] when θ reaches or exceeds 1.0.
    pub fn update_thermal_state(
        &mut self,
        current_a: f64,
        dt_s: f64,
        current_time_s: f64,
    ) -> Result<(), MotorProtError> {
        let m = current_a / self.rated_current_a;
        let m_sq = m * m;
        let theta = self.thermal_state;

        let new_theta = if m > 1.0 {
            // Heating branch — compute τ_heat from NEMA trip class formula
            let tau_heat = Self::heating_tau(self.config.overload_trip_class, m);
            let d_theta = (m_sq - theta) / tau_heat;
            theta + d_theta * dt_s
        } else {
            // Cooling branch — τ_cool = 2 × base τ (at full load)
            let tau_cool = 2.0 * f64::from(self.config.overload_trip_class) * 60.0;
            let d_theta = -theta / tau_cool;
            theta + d_theta * dt_s
        };

        // Clamp to reasonable range
        self.thermal_state = new_theta.clamp(0.0, 1.5);
        self.last_update_s = current_time_s;

        if self.thermal_state >= 1.0 {
            self.tripped_thermal = true;
            self.record_trip(
                current_time_s,
                TripCause::ThermalOverload,
                self.thermal_state,
                current_a,
            );
            return Err(MotorProtError::ThermalTrip);
        }
        Ok(())
    }

    /// Compute heating time-constant \[s\] for the NEMA trip-class formula.
    ///
    /// `τ = trip_class × 60 / ln(M² / (M² − 1))` where M = I / I_rated > 1.
    fn heating_tau(trip_class: u8, m: f64) -> f64 {
        let m_sq = m * m;
        let denom = m_sq - 1.0;
        if denom < 1e-9 {
            // M ≈ 1 — effectively very long time-constant
            return f64::from(trip_class) * 60.0 * 1e6;
        }
        let ratio = m_sq / denom;
        if ratio <= 0.0 {
            return f64::from(trip_class) * 60.0;
        }
        let ln_ratio = ratio.ln();
        if ln_ratio < 1e-12 {
            return f64::from(trip_class) * 60.0 * 1e6;
        }
        f64::from(trip_class) * 60.0 / ln_ratio
    }

    // -----------------------------------------------------------------------
    // Phase-voltage unbalance
    // -----------------------------------------------------------------------

    /// Check phase-voltage unbalance using the NEMA MG-1 definition.
    ///
    /// `unbalance = 100 × max|Vx − V_avg| / V_avg`
    ///
    /// # Arguments
    /// * `va`, `vb`, `vc` — phase voltage magnitudes \[kV or pu — consistent units\]
    ///
    /// # Returns
    /// `Ok(unbalance_pct)` or `Err(PhaseUnbalanceTrip(pct))` if above threshold.
    pub fn check_phase_unbalance(
        &mut self,
        va: f64,
        vb: f64,
        vc: f64,
        current_time_s: f64,
    ) -> Result<f64, MotorProtError> {
        let avg = (va + vb + vc) / 3.0;
        if avg < 1e-12 {
            // Zero-voltage condition — treat as severe unbalance
            self.last_unbalance_pct = 100.0;
            self.tripped_phase = true;
            self.record_trip(
                current_time_s,
                TripCause::PhaseUnbalance,
                self.thermal_state,
                0.0,
            );
            return Err(MotorProtError::PhaseUnbalanceTrip(100.0));
        }
        let max_dev = (va - avg).abs().max((vb - avg).abs()).max((vc - avg).abs());
        let pct = 100.0 * max_dev / avg;
        self.last_unbalance_pct = pct;

        if pct > self.config.phase_unbalance_pct_trip {
            self.tripped_phase = true;
            self.record_trip(
                current_time_s,
                TripCause::PhaseUnbalance,
                self.thermal_state,
                0.0,
            );
            return Err(MotorProtError::PhaseUnbalanceTrip(pct));
        }
        Ok(pct)
    }

    // -----------------------------------------------------------------------
    // Locked-rotor / stall
    // -----------------------------------------------------------------------

    /// Check locked-rotor / stall condition and accumulate stall time.
    ///
    /// Stall is detected when `speed_rpm < 5 %` of rated AND
    /// `current_a > 3 × rated_current_a`.  Accumulated stall time is compared
    /// against [`MotorProtConfig::locked_rotor_time_s`].
    ///
    /// # Arguments
    /// * `current_a` — phase current magnitude \[A\]
    /// * `speed_rpm` — measured rotor speed \[rpm\]
    /// * `rated_rpm` — rated synchronous speed \[rpm\]
    /// * `dt_s` — time step \[s\]
    /// * `current_time_s` — relay clock \[s\]
    ///
    /// # Errors
    /// Returns [`MotorProtError::LockedRotorTrip`] when stall time exceeds limit.
    pub fn check_locked_rotor(
        &mut self,
        current_a: f64,
        speed_rpm: f64,
        rated_rpm: f64,
        dt_s: f64,
        current_time_s: f64,
    ) -> Result<(), MotorProtError> {
        let stalled = speed_rpm < 0.05 * rated_rpm && current_a > 3.0 * self.rated_current_a;
        if stalled {
            self.stall_time_s += dt_s;
            if self.stall_time_s > self.config.locked_rotor_time_s {
                self.tripped_locked_rotor = true;
                self.record_trip(
                    current_time_s,
                    TripCause::LockedRotor,
                    self.thermal_state,
                    current_a,
                );
                return Err(MotorProtError::LockedRotorTrip);
            }
        } else {
            self.stall_time_s = 0.0;
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Ground fault
    // -----------------------------------------------------------------------

    /// Instantaneous ground-fault protection.
    ///
    /// Trips immediately when `|ig_a|` exceeds
    /// [`MotorProtConfig::ground_fault_threshold_a`].
    ///
    /// # Arguments
    /// * `ig_a` — earth-leakage (zero-sequence) current \[A\]
    /// * `current_time_s` — relay clock \[s\]
    ///
    /// # Errors
    /// Returns [`MotorProtError::GroundFaultTrip`] on excess leakage.
    pub fn check_ground_fault(
        &mut self,
        ig_a: f64,
        current_time_s: f64,
    ) -> Result<(), MotorProtError> {
        if ig_a.abs() > self.config.ground_fault_threshold_a {
            self.tripped_ground = true;
            self.record_trip(
                current_time_s,
                TripCause::GroundFault,
                self.thermal_state,
                ig_a.abs(),
            );
            return Err(MotorProtError::GroundFaultTrip);
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Under-voltage
    // -----------------------------------------------------------------------

    /// Definite-time under-voltage protection (500 ms delay).
    ///
    /// # Arguments
    /// * `v_pu` — terminal voltage in per-unit \[pu\]
    /// * `dt_s` — time step \[s\]
    /// * `current_time_s` — relay clock \[s\]
    ///
    /// # Errors
    /// Returns [`MotorProtError::UndervoltageTrip`] after 500 ms below threshold.
    pub fn check_undervoltage(
        &mut self,
        v_pu: f64,
        dt_s: f64,
        current_time_s: f64,
    ) -> Result<(), MotorProtError> {
        if v_pu < self.config.undervoltage_pu_trip {
            self.undervoltage_timer_s += dt_s;
        } else {
            self.undervoltage_timer_s = 0.0;
        }
        if self.undervoltage_timer_s > 0.5 {
            self.tripped_undervoltage = true;
            self.record_trip(
                current_time_s,
                TripCause::Undervoltage,
                self.thermal_state,
                0.0,
            );
            return Err(MotorProtError::UndervoltageTrip);
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // NEMA trip curve
    // -----------------------------------------------------------------------

    /// NEMA trip-curve time-to-trip for a given overload factor.
    ///
    /// Formula: `t = class × SF² / (M² − SF²)` where SF = 1.15.
    ///
    /// # Arguments
    /// * `trip_class` — NEMA class (5, 10, 20, 30)
    /// * `overload_factor` — `I / I_rated` (must be > SF to obtain finite time)
    ///
    /// # Returns
    /// Time-to-trip \[s\], or `f64::INFINITY` if below the SF threshold.
    pub fn nema_trip_curve(trip_class: u8, overload_factor: f64) -> f64 {
        let sf = 1.15_f64;
        let sf_sq = sf * sf;
        let m_sq = overload_factor * overload_factor;
        let denom = m_sq - sf_sq;
        if denom <= 0.0 {
            return f64::INFINITY;
        }
        let class_f = match trip_class {
            5 => 5.0_f64,
            10 => 10.0,
            20 => 20.0,
            30 => 30.0,
            _ => return f64::INFINITY,
        };
        class_f * sf_sq / denom
    }

    // -----------------------------------------------------------------------
    // Reset / manual
    // -----------------------------------------------------------------------

    /// Manually reset thermal model (e.g. after confirmed cool-down).
    ///
    /// Clears thermal state, stall timer, and trip flags so the motor can
    /// restart.  The trip history is preserved for logging.
    pub fn reset_thermal(&mut self) {
        self.thermal_state = 0.0;
        self.stall_time_s = 0.0;
        self.undervoltage_timer_s = 0.0;
        self.tripped_thermal = false;
        self.tripped_phase = false;
        self.tripped_locked_rotor = false;
        self.tripped_ground = false;
        self.tripped_undervoltage = false;
    }

    /// Issue a manual trip (e.g. operator command or emergency stop).
    ///
    /// Records a [`TripCause::ManualTrip`] event in the trip history.
    pub fn manual_trip(&mut self, current_time_s: f64, current_a: f64) {
        self.record_trip(
            current_time_s,
            TripCause::ManualTrip,
            self.thermal_state,
            current_a,
        );
    }

    // -----------------------------------------------------------------------
    // Status
    // -----------------------------------------------------------------------

    /// Return a snapshot of all protection alarm and trip flags.
    ///
    /// # Arguments
    /// * `current_time_s` — relay clock \[s\] used to evaluate restart lockout.
    pub fn protection_status(&self, current_time_s: f64) -> MotorProtStatus {
        let elapsed_since_trip = current_time_s - self.last_trip_time_s;
        let restart_locked =
            !self.trip_history.is_empty() && elapsed_since_trip < self.config.restart_lockout_s;

        MotorProtStatus {
            thermal_state: self.thermal_state,
            thermal_alarm: self.thermal_state > 0.9,
            thermal_trip: self.tripped_thermal,
            phase_unbalance_pct: self.last_unbalance_pct,
            phase_unbalance_trip: self.tripped_phase,
            locked_rotor_trip: self.tripped_locked_rotor,
            ground_fault_trip: self.tripped_ground,
            undervoltage_trip: self.tripped_undervoltage,
            restart_locked_out: restart_locked,
            trip_count: self.trip_history.len(),
        }
    }

    /// Read-only access to the trip history log.
    pub fn trip_history(&self) -> &[MotorTrip] {
        &self.trip_history
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Record a trip event and update the last-trip timestamp.
    fn record_trip(
        &mut self,
        time_s: f64,
        cause: TripCause,
        thermal_state_at_trip: f64,
        current_at_trip_a: f64,
    ) {
        self.last_trip_time_s = time_s;
        self.trip_history.push(MotorTrip {
            time_s,
            cause,
            thermal_state_at_trip,
            current_at_trip_a,
        });
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_relay(trip_class: u8) -> MotorProtectionRelay {
        let config = MotorProtConfig {
            overload_trip_class: trip_class,
            ground_fault_threshold_a: 1.0,
            locked_rotor_time_s: 10.0,
            phase_unbalance_pct_trip: 5.0,
            undervoltage_pu_trip: 0.85,
            restart_lockout_s: 300.0,
            ..MotorProtConfig::default()
        };
        MotorProtectionRelay::new(
            "M_test".into(),
            100.0, // rated_current_a = 100 A
            0.4,   // rated_voltage_kv
            1.15,  // service_factor
            ThermalClass::ClassF,
            config,
        )
    }

    /// Class 10 NEMA curve at 6× FLA should give a finite, positive trip time.
    #[test]
    fn test_class10_overload_6x_fla() {
        let t = MotorProtectionRelay::nema_trip_curve(10, 6.0);
        // At 6× FLA the formula gives 10 × 1.15² / (36 − 1.3225) ≈ 0.38 s
        assert!(t.is_finite(), "Expected finite trip time, got {}", t);
        assert!(t > 0.0, "Trip time must be positive");
        // Sanity: faster than at 2× FLA
        let t_slow = MotorProtectionRelay::nema_trip_curve(10, 2.0);
        assert!(
            t < t_slow,
            "6x should trip faster than 2x: {} vs {}",
            t,
            t_slow
        );
    }

    /// 6 % phase unbalance must trip; 3 % must not trip.
    #[test]
    fn test_phase_unbalance_trip_and_no_trip() {
        let mut relay = default_relay(10);

        // 3 % — should pass
        let v_avg = 230.0_f64;
        let va = v_avg * 1.015;
        let vb = v_avg * 0.985;
        let vc = v_avg;
        let result_ok = relay.check_phase_unbalance(va, vb, vc, 0.0);
        assert!(result_ok.is_ok(), "3% unbalance should not trip");

        // 6 % — should trip (max deviation = 6% of avg → unbalance = 6%)
        let mut relay2 = default_relay(10);
        let va6 = v_avg * 1.06;
        let vb6 = v_avg * 0.94;
        let vc6 = v_avg;
        let result_trip = relay2.check_phase_unbalance(va6, vb6, vc6, 1.0);
        assert!(
            matches!(result_trip, Err(MotorProtError::PhaseUnbalanceTrip(_))),
            "6% unbalance should trip"
        );
    }

    /// Accumulate 11 s of locked-rotor time (limit = 10 s) → trip.
    #[test]
    fn test_locked_rotor_trip_after_11s() {
        let mut relay = default_relay(10);
        // rated_rpm = 1500, speed = 0 (stalled), current = 400 A (4× FLA)
        let dt = 1.0_f64;
        let mut tripped = false;
        for i in 0..15 {
            let t = i as f64;
            let res = relay.check_locked_rotor(400.0, 0.0, 1500.0, dt, t);
            if res.is_err() {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "Locked-rotor should trip after > 10 s stall");
        assert_eq!(
            relay.trip_history().last().map(|tr| tr.cause),
            Some(TripCause::LockedRotor)
        );
    }

    /// Ground fault above threshold triggers an immediate trip.
    #[test]
    fn test_ground_fault_instant_trip() {
        let mut relay = default_relay(10);
        // threshold = 1.0 A, inject 2.0 A
        let result = relay.check_ground_fault(2.0, 0.5);
        assert!(
            matches!(result, Err(MotorProtError::GroundFaultTrip)),
            "Ground fault should trip immediately"
        );
        assert_eq!(relay.trip_history().len(), 1);
        assert_eq!(relay.trip_history()[0].cause, TripCause::GroundFault);
    }

    /// Thermal state rises under overload, then decays after load removal.
    #[test]
    fn test_thermal_accumulation_and_cooling() {
        let mut relay = default_relay(10);
        let i_overload = 150.0; // 1.5× FLA
        let dt = 1.0_f64;

        // Run 30 heating steps
        for step in 0..30 {
            let _ = relay.update_thermal_state(i_overload, dt, step as f64);
        }
        let hot = relay.thermal_state;
        assert!(hot > 0.0, "Thermal state should have risen");

        // Cool for 120 s at rated current (no further heating)
        let cool_start = relay.thermal_state;
        for step in 30..150 {
            let _ = relay.update_thermal_state(100.0, dt, step as f64); // rated → cooling
        }
        let cooled = relay.thermal_state;
        assert!(
            cooled < cool_start,
            "Thermal state should decrease during cooling: {} < {}",
            cooled,
            cool_start
        );
    }

    /// Motor running at 115 % FLA (= 1 × SF) must not trip immediately.
    #[test]
    fn test_service_factor_no_immediate_trip() {
        let mut relay = default_relay(10);
        // 115 A = 1.15 × 100 A FLA (exactly at SF)
        let result = relay.update_thermal_state(115.0, 0.1, 0.1);
        // Should not trip in the first tiny time step
        assert!(
            result.is_ok(),
            "SF-rated current must not cause immediate trip"
        );
        assert!(relay.thermal_state < 1.0);
    }

    /// After a trip the relay must enforce the restart lockout period.
    #[test]
    fn test_restart_lockout_enforcement() {
        let mut relay = default_relay(10);
        // Force a ground-fault trip at t = 0
        let _ = relay.check_ground_fault(5.0, 0.0);

        // Immediately after trip — should be locked out
        let status_early = relay.protection_status(10.0); // 10 s elapsed
        assert!(
            status_early.restart_locked_out,
            "Should be locked out at 10 s"
        );

        // After lockout period (300 s) — should be unlocked
        let status_late = relay.protection_status(350.0); // 350 s elapsed
        assert!(
            !status_late.restart_locked_out,
            "Should be unlocked at 350 s"
        );
    }

    /// Two sequential trips must both appear in the trip history.
    #[test]
    fn test_multiple_trips_stored_in_history() {
        let mut relay = default_relay(10);

        // Trip 1: ground fault
        let _ = relay.check_ground_fault(3.0, 1.0);

        // Trip 2: phase unbalance (6 % — max deviation = 6% of avg)
        let v_avg = 230.0_f64;
        let _ = relay.check_phase_unbalance(v_avg * 1.06, v_avg * 0.94, v_avg, 2.0);

        assert_eq!(relay.trip_history().len(), 2, "Both trips must be recorded");
        assert_eq!(relay.trip_history()[0].cause, TripCause::GroundFault);
        assert_eq!(relay.trip_history()[1].cause, TripCause::PhaseUnbalance);
    }

    /// Under-voltage below 0.85 pu held for > 500 ms must trip.
    #[test]
    fn test_undervoltage_trip_after_500ms() {
        let mut relay = default_relay(10);
        let dt = 0.1_f64; // 100 ms steps
        let mut tripped = false;
        for i in 0..10 {
            let t = i as f64 * dt;
            if relay.check_undervoltage(0.80, dt, t).is_err() {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "Under-voltage should trip after > 500 ms");
    }

    /// NEMA curve: Class 20 must trip slower than Class 10 at same overload.
    #[test]
    fn test_nema_class_ordering() {
        let t10 = MotorProtectionRelay::nema_trip_curve(10, 3.0);
        let t20 = MotorProtectionRelay::nema_trip_curve(20, 3.0);
        let t30 = MotorProtectionRelay::nema_trip_curve(30, 3.0);
        assert!(t10 < t20, "Class 10 faster than 20: {} vs {}", t10, t20);
        assert!(t20 < t30, "Class 20 faster than 30: {} vs {}", t20, t30);
        assert!(t10.is_finite() && t20.is_finite() && t30.is_finite());
    }
}
