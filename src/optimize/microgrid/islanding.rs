/// Islanding detection for distributed generation.
///
/// Implements passive detection methods used to identify when a portion of
/// the grid has become electrically isolated (islanded) from the main grid.
///
/// # Methods
/// - **ROCOF** (Rate of Change of Frequency): |df/dt| > threshold
/// - **Vector Surge**: sudden phase angle jump
/// - **Under/Over Frequency** (ANSI 81): frequency outside [47.5, 51.5] Hz (IEC)
/// - **Under/Over Voltage** (ANSI 27/59): voltage outside [0.88, 1.10] p.u.
use serde::{Deserialize, Serialize};

/// Settings for the islanding detection relay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandingDetector {
    /// ROCOF threshold [Hz/s] — typical: 0.5–2.0 Hz/s
    pub rocof_threshold_hz_s: f64,
    /// Vector surge threshold `degrees` — typical: 2–12°
    pub vector_surge_deg: f64,
    /// Under-frequency trip level `Hz`
    pub freq_lower_hz: f64,
    /// Over-frequency trip level `Hz`
    pub freq_upper_hz: f64,
    /// Under-voltage trip level [p.u.]
    pub voltage_lower_pu: f64,
    /// Over-voltage trip level [p.u.]
    pub voltage_upper_pu: f64,
    /// Minimum time delay before tripping `s` (prevents nuisance trips)
    pub trip_delay_s: f64,

    // Internal state
    prev_freq_hz: f64,
    prev_phase_deg: f64,
    alarm_time: f64,
    current_alarm: Option<IslandingCause>,
}

/// Reason an islanding alarm was raised.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum IslandingCause {
    Rocof,
    VectorSurge,
    UnderFrequency,
    OverFrequency,
    UnderVoltage,
    OverVoltage,
}

/// Islanding trip event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandingTrip {
    pub cause: IslandingCause,
    pub time_s: f64,
    pub freq_hz: f64,
    pub voltage_pu: f64,
}

/// Current measurement from the grid connection point.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GridMeasurement {
    /// Fundamental frequency `Hz`
    pub freq_hz: f64,
    /// Voltage magnitude [p.u.]
    pub voltage_pu: f64,
    /// Voltage phase angle `degrees`
    pub phase_deg: f64,
    /// Elapsed time since last measurement `s`
    pub dt_s: f64,
}

impl IslandingDetector {
    /// Create a detector with IEC 62116 / IEEE 1547 default settings.
    pub fn ieee_1547() -> Self {
        Self {
            rocof_threshold_hz_s: 1.0,
            vector_surge_deg: 8.0,
            freq_lower_hz: 59.3,
            freq_upper_hz: 60.5,
            voltage_lower_pu: 0.88,
            voltage_upper_pu: 1.10,
            trip_delay_s: 0.16, // one cycle at 60 Hz
            prev_freq_hz: 60.0,
            prev_phase_deg: 0.0,
            alarm_time: 0.0,
            current_alarm: None,
        }
    }

    /// Create a detector with EN 50438 (European) settings for 50 Hz systems.
    pub fn en_50438() -> Self {
        Self {
            rocof_threshold_hz_s: 0.5,
            vector_surge_deg: 6.0,
            freq_lower_hz: 47.5,
            freq_upper_hz: 51.5,
            voltage_lower_pu: 0.85,
            voltage_upper_pu: 1.15,
            trip_delay_s: 0.10,
            prev_freq_hz: 50.0,
            prev_phase_deg: 0.0,
            alarm_time: 0.0,
            current_alarm: None,
        }
    }

    /// Process a new measurement. Returns a trip event if islanding is detected.
    pub fn update(&mut self, m: GridMeasurement, elapsed_s: f64) -> Option<IslandingTrip> {
        let rocof = (m.freq_hz - self.prev_freq_hz) / m.dt_s;
        let phase_jump = (m.phase_deg - self.prev_phase_deg).abs();
        // Normalise phase jump to [-180, 180]
        let phase_jump = if phase_jump > 180.0 {
            360.0 - phase_jump
        } else {
            phase_jump
        };

        self.prev_freq_hz = m.freq_hz;
        self.prev_phase_deg = m.phase_deg;

        // Detect alarm condition
        let alarm_cause = if rocof.abs() > self.rocof_threshold_hz_s {
            Some(IslandingCause::Rocof)
        } else if phase_jump > self.vector_surge_deg {
            Some(IslandingCause::VectorSurge)
        } else if m.freq_hz < self.freq_lower_hz {
            Some(IslandingCause::UnderFrequency)
        } else if m.freq_hz > self.freq_upper_hz {
            Some(IslandingCause::OverFrequency)
        } else if m.voltage_pu < self.voltage_lower_pu {
            Some(IslandingCause::UnderVoltage)
        } else if m.voltage_pu > self.voltage_upper_pu {
            Some(IslandingCause::OverVoltage)
        } else {
            None
        };

        match (alarm_cause, self.current_alarm) {
            (Some(cause), _) => {
                if self.current_alarm.is_none() {
                    self.alarm_time = elapsed_s;
                    self.current_alarm = Some(cause);
                }
                // Trip if alarm has persisted for trip_delay_s
                if elapsed_s - self.alarm_time >= self.trip_delay_s {
                    self.current_alarm = None;
                    return Some(IslandingTrip {
                        cause,
                        time_s: elapsed_s,
                        freq_hz: m.freq_hz,
                        voltage_pu: m.voltage_pu,
                    });
                }
            }
            (None, _) => {
                // Clear alarm if condition no longer present
                self.current_alarm = None;
            }
        }

        None
    }

    /// Reset detector state (e.g., after reconnection to grid).
    pub fn reset(&mut self, freq_hz: f64) {
        self.prev_freq_hz = freq_hz;
        self.prev_phase_deg = 0.0;
        self.alarm_time = 0.0;
        self.current_alarm = None;
    }
}

/// Simulate islanding detection over a time series of measurements.
///
/// Returns the trip event if detected, otherwise None.
pub fn simulate_islanding(
    detector: &mut IslandingDetector,
    measurements: &[GridMeasurement],
) -> Option<IslandingTrip> {
    let mut t = 0.0_f64;
    for m in measurements {
        t += m.dt_s;
        if let Some(trip) = detector.update(*m, t) {
            return Some(trip);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_operation_no_trip() {
        let mut det = IslandingDetector::ieee_1547();
        let measurements: Vec<GridMeasurement> = (0..100)
            .map(|_| GridMeasurement {
                freq_hz: 60.0,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            })
            .collect();
        let trip = simulate_islanding(&mut det, &measurements);
        assert!(trip.is_none(), "Should not trip under normal conditions");
    }

    #[test]
    fn test_rocof_triggers_trip() {
        let mut det = IslandingDetector::ieee_1547();
        // trip_delay_s = 0.16; dt = 0.02 → need ≥ 9 samples at ROCOF > threshold
        // Start with normal, then sustained ROCOF = 2 Hz/s > 1 Hz/s threshold
        let mut measurements = vec![GridMeasurement {
            freq_hz: 60.0,
            voltage_pu: 1.0,
            phase_deg: 0.0,
            dt_s: 0.02,
        }];
        // 20 samples of 2 Hz/s frequency drop (total duration 0.4s >> trip_delay 0.16s)
        let mut freq = 60.0_f64;
        for _ in 0..20 {
            freq -= 2.0 * 0.02; // 2 Hz/s × dt
            measurements.push(GridMeasurement {
                freq_hz: freq,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            });
        }
        let trip = simulate_islanding(&mut det, &measurements);
        assert!(trip.is_some(), "ROCOF should trigger islanding trip");
        assert_eq!(trip.unwrap().cause, IslandingCause::Rocof);
    }

    #[test]
    fn test_under_frequency_trip() {
        let mut det = IslandingDetector::ieee_1547();
        // Frequency drops below 59.3 Hz
        let measurements: Vec<GridMeasurement> = (0..20)
            .map(|_| GridMeasurement {
                freq_hz: 59.0,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            })
            .collect();
        let trip = simulate_islanding(&mut det, &measurements);
        assert!(trip.is_some(), "Under-frequency should trigger trip");
        assert_eq!(trip.unwrap().cause, IslandingCause::UnderFrequency);
    }

    #[test]
    fn test_over_voltage_trip() {
        let mut det = IslandingDetector::ieee_1547();
        let measurements: Vec<GridMeasurement> = (0..20)
            .map(|_| GridMeasurement {
                freq_hz: 60.0,
                voltage_pu: 1.15,
                phase_deg: 0.0,
                dt_s: 0.02,
            })
            .collect();
        let trip = simulate_islanding(&mut det, &measurements);
        assert!(trip.is_some(), "Over-voltage should trigger trip");
        assert_eq!(trip.unwrap().cause, IslandingCause::OverVoltage);
    }

    #[test]
    fn test_trip_delay_prevents_nuisance() {
        let mut det = IslandingDetector::ieee_1547();
        det.trip_delay_s = 0.10;
        // Only one sample with anomaly → alarm set but not yet tripped
        let measurements = vec![
            GridMeasurement {
                freq_hz: 60.0,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            },
            GridMeasurement {
                freq_hz: 58.0,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            }, // anomaly
            GridMeasurement {
                freq_hz: 60.0,
                voltage_pu: 1.0,
                phase_deg: 0.0,
                dt_s: 0.02,
            }, // back to normal
        ];
        let trip = simulate_islanding(&mut det, &measurements);
        assert!(
            trip.is_none(),
            "Single-cycle anomaly should not trip before delay"
        );
    }

    #[test]
    fn test_en_50438_settings() {
        let det = IslandingDetector::en_50438();
        assert!(det.freq_lower_hz < 50.0);
        assert!(det.rocof_threshold_hz_s < 1.0);
    }
}
