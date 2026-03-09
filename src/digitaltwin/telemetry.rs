/// SCADA/PMU telemetry ingestion for the OxiGrid digital twin.
///
/// Provides `TelemetryBatch` — a unified container for a single scan cycle's
/// worth of SCADA measurement points and PMU synchrophasor frames.  Includes
/// quality filtering, normalisation to per-unit, and conversion to the
/// `Measurement` format consumed by the WLS state estimator.
use crate::io::pmu::PmuFrame;
use crate::powerflow::state_estimation::{Measurement, MeasurementType};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// SCADA measurement point
// ─────────────────────────────────────────────────────────────────────────────

/// Measurement type for a SCADA data point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScadaMeasType {
    /// RMS voltage magnitude at a bus \[kV\]
    VoltageMagnitude,
    /// Active power injection / flow \[MW\]
    ActivePower,
    /// Reactive power injection / flow \[MVAr\]
    ReactivePower,
    /// RMS current magnitude \[kA\]
    Current,
    /// System / bus frequency \[Hz\]
    Frequency,
    /// Transformer tap position (integer steps, dimensionless)
    TapPosition,
    /// Breaker / switch status (0 = open, 1 = closed)
    BreakerStatus,
}

/// A single SCADA measurement sample from one RTU scan cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScadaPoint {
    /// RTU/datapoint ID assigned by the SCADA historian.
    pub point_id: u32,
    /// UTC timestamp of the measurement [µs since Unix epoch].
    pub timestamp_us: i64,
    /// Measurement type (determines engineering units).
    pub measurement_type: ScadaMeasType,
    /// 0-based index of the bus or branch this point refers to.
    pub bus_or_branch: usize,
    /// Raw measured value in engineering units.
    pub value: f64,
    /// Quality code: 0 = good, non-zero = questionable / bad.
    pub quality: u8,
}

impl ScadaPoint {
    /// Construct a new SCADA point.
    pub fn new(
        point_id: u32,
        timestamp_us: i64,
        measurement_type: ScadaMeasType,
        bus_or_branch: usize,
        value: f64,
        quality: u8,
    ) -> Self {
        Self {
            point_id,
            timestamp_us,
            measurement_type,
            bus_or_branch,
            value,
            quality,
        }
    }

    /// Returns `true` if the quality code indicates a good measurement.
    pub fn is_good(&self) -> bool {
        self.quality == 0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry source
// ─────────────────────────────────────────────────────────────────────────────

/// Origin of a `TelemetryBatch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TelemetrySource {
    /// Traditional SCADA (slower, poll-based, seconds to minutes).
    Scada,
    /// PMU synchrophasor stream from a specific unit.
    Pmu {
        /// IEEE C37.118 PMU station identifier.
        pmu_id: u16,
    },
    /// Mixture of SCADA and PMU data in the same batch.
    Mixed,
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry batch
// ─────────────────────────────────────────────────────────────────────────────

/// One scan-cycle's worth of telemetry from SCADA and/or PMU sources.
///
/// Can contain any combination of `ScadaPoint`s and `PmuFrame`s.
/// `to_se_measurements()` converts the entire batch to the `Measurement` format
/// expected by `StateEstimator`.
#[derive(Debug, Clone)]
pub struct TelemetryBatch {
    /// Primary data source for this batch.
    pub source: TelemetrySource,
    /// UTC timestamp at which the scan cycle started [µs since epoch].
    pub scan_time_us: i64,
    /// SCADA measurement points collected in this cycle.
    pub scada_points: Vec<ScadaPoint>,
    /// PMU synchrophasor frames (one per PMU per reporting interval).
    pub pmu_frames: Vec<PmuFrame>,
}

impl TelemetryBatch {
    /// Create an empty batch for the given source and scan time.
    pub fn new(source: TelemetrySource, scan_time_us: i64) -> Self {
        Self {
            source,
            scan_time_us,
            scada_points: Vec::new(),
            pmu_frames: Vec::new(),
        }
    }

    /// Append a SCADA measurement point to this batch.
    pub fn add_scada(&mut self, point: ScadaPoint) {
        self.scada_points.push(point);
    }

    /// Append a PMU frame to this batch.
    pub fn add_pmu_frame(&mut self, frame: PmuFrame) {
        self.pmu_frames.push(frame);
    }

    /// Convert the SCADA points in this batch to `Measurement` objects suitable
    /// for the WLS state estimator.
    ///
    /// `base_mva` is used to normalise MW/MVAr → pu.
    /// Voltage magnitudes are expected in kV and normalised with the bus nominal
    /// voltage stored in the SCADA metadata (simplified to 1 pu base here — the
    /// caller should pre-normalise kV to pu before ingestion for highest accuracy).
    ///
    /// Points with non-zero quality codes are silently skipped.
    pub fn to_se_measurements(&self, base_mva: f64) -> Vec<Measurement> {
        let mut measurements = Vec::with_capacity(self.scada_points.len());

        for point in &self.scada_points {
            if !point.is_good() {
                continue;
            }
            let bus = point.bus_or_branch;

            match point.measurement_type {
                ScadaMeasType::VoltageMagnitude => {
                    // Assume value already in pu (or caller pre-normalised).
                    measurements.push(Measurement {
                        mtype: MeasurementType::VoltageMagnitude,
                        bus,
                        to_bus: None,
                        value: point.value,
                        sigma: 0.01, // 1% typical SCADA voltage accuracy
                    });
                }
                ScadaMeasType::ActivePower => {
                    let value_pu = point.value / base_mva;
                    measurements.push(Measurement {
                        mtype: MeasurementType::PowerInjection,
                        bus,
                        to_bus: None,
                        value: value_pu,
                        sigma: 0.02,
                    });
                }
                ScadaMeasType::ReactivePower => {
                    let value_pu = point.value / base_mva;
                    measurements.push(Measurement {
                        mtype: MeasurementType::ReactiveInjection,
                        bus,
                        to_bus: None,
                        value: value_pu,
                        sigma: 0.03, // Q measurements noisier
                    });
                }
                ScadaMeasType::Current => {
                    // Current magnitude mapped to active power injection as a proxy
                    // (SE model does not have a dedicated current-magnitude type).
                    measurements.push(Measurement {
                        mtype: MeasurementType::PowerInjection,
                        bus,
                        to_bus: None,
                        value: point.value,
                        sigma: 0.02,
                    });
                }
                ScadaMeasType::Frequency
                | ScadaMeasType::TapPosition
                | ScadaMeasType::BreakerStatus => {
                    // These types are handled separately by the twin (not SE inputs).
                }
            }
        }

        // Extract voltage phasors from PMU frames if present.
        for frame in &self.pmu_frames {
            // Each PMU frame carries phasors; the phasor index maps to a bus index
            // in the twin's network model.  We treat phasors with magnitude ≥ 0.5 pu
            // as voltage phasors (current phasors are typically much smaller).
            for (ph_idx, phasor) in frame.phasors.iter().enumerate() {
                if phasor.magnitude >= 0.5 {
                    measurements.push(Measurement {
                        mtype: MeasurementType::VoltageMagnitude,
                        bus: ph_idx,
                        to_bus: None,
                        value: phasor.magnitude,
                        sigma: 0.001, // PMU much more accurate than SCADA
                    });
                    // Voltage angle is not a standard WLS measurement type in the DC SE;
                    // encode as active power injection with the angle value (proxy for
                    // synchrophasor angle measurements until a dedicated type is added).
                    measurements.push(Measurement {
                        mtype: MeasurementType::PowerInjection,
                        bus: ph_idx,
                        to_bus: None,
                        value: phasor.angle_rad,
                        sigma: 0.001,
                    });
                }
            }
        }

        measurements
    }

    /// Return a new batch containing only points that are within `max_age_us`
    /// of `scan_time_us` and have quality code 0.
    pub fn filter_quality(&self, max_age_us: i64) -> TelemetryBatch {
        let cutoff = self.scan_time_us - max_age_us;
        let scada_points = self
            .scada_points
            .iter()
            .filter(|p| p.is_good() && p.timestamp_us >= cutoff)
            .cloned()
            .collect();

        TelemetryBatch {
            source: self.source,
            scan_time_us: self.scan_time_us,
            scada_points,
            pmu_frames: self.pmu_frames.clone(),
        }
    }

    /// Return `true` if the batch contains at least one usable measurement.
    pub fn is_empty(&self) -> bool {
        self.scada_points.is_empty() && self.pmu_frames.is_empty()
    }

    /// Total number of data points (SCADA + PMU frames).
    pub fn len(&self) -> usize {
        self.scada_points.len() + self.pmu_frames.len()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry statistics
// ─────────────────────────────────────────────────────────────────────────────

/// Quality and coverage statistics for a single telemetry batch.
#[derive(Debug, Clone)]
pub struct TelemetryStats {
    /// Total number of SCADA points received (all qualities).
    pub n_points_received: usize,
    /// Number of SCADA points with quality = 0.
    pub n_points_good: usize,
    /// Number of SCADA points with quality ≠ 0.
    pub n_points_bad: usize,
    /// Number of PMU frames in the batch.
    pub n_pmu_frames: usize,
    /// Wall-clock latency from measurement timestamp to scan time \[ms\].
    pub scan_latency_ms: f64,
    /// Percentage of buses in the network that have at least one recent
    /// good measurement in this batch [0..100].
    pub coverage_pct: f64,
}

/// Compute `TelemetryStats` for a batch given the total number of network buses
/// and the maximum acceptable age in microseconds.
pub fn compute_telemetry_stats(
    batch: &TelemetryBatch,
    n_buses: usize,
    max_age_us: i64,
) -> TelemetryStats {
    let n_received = batch.scada_points.len();
    let n_good = batch.scada_points.iter().filter(|p| p.is_good()).count();
    let n_bad = n_received - n_good;

    let cutoff = batch.scan_time_us - max_age_us;

    // Count distinct buses with at least one fresh good measurement.
    let mut covered_buses: Vec<usize> = batch
        .scada_points
        .iter()
        .filter(|p| p.is_good() && p.timestamp_us >= cutoff)
        .map(|p| p.bus_or_branch)
        .collect();
    // PMU frames also contribute (phasor index approximates bus index).
    for frame in &batch.pmu_frames {
        for (ph_idx, _) in frame.phasors.iter().enumerate() {
            covered_buses.push(ph_idx);
        }
    }
    covered_buses.sort_unstable();
    covered_buses.dedup();
    let n_covered = covered_buses.len().min(n_buses);

    // Average latency from individual point timestamps to scan time.
    let scan_latency_ms = if n_received == 0 {
        0.0
    } else {
        let total_lag_us: i64 = batch
            .scada_points
            .iter()
            .map(|p| (batch.scan_time_us - p.timestamp_us).max(0))
            .sum();
        (total_lag_us as f64 / n_received as f64) / 1_000.0
    };

    let coverage_pct = if n_buses == 0 {
        100.0
    } else {
        100.0 * n_covered as f64 / n_buses as f64
    };

    TelemetryStats {
        n_points_received: n_received,
        n_points_good: n_good,
        n_points_bad: n_bad,
        n_pmu_frames: batch.pmu_frames.len(),
        scan_latency_ms,
        coverage_pct,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_voltage_point(bus: usize, v_pu: f64, ts: i64, quality: u8) -> ScadaPoint {
        ScadaPoint::new(
            bus as u32,
            ts,
            ScadaMeasType::VoltageMagnitude,
            bus,
            v_pu,
            quality,
        )
    }

    #[test]
    fn test_telemetry_to_se_measurements() {
        let mut batch = TelemetryBatch::new(TelemetrySource::Scada, 1_000_000);
        batch.add_scada(make_voltage_point(0, 1.02, 1_000_000, 0));
        batch.add_scada(ScadaPoint::new(
            1,
            1_000_000,
            ScadaMeasType::ActivePower,
            0,
            50.0,
            0,
        ));
        let meas = batch.to_se_measurements(100.0);
        assert_eq!(meas.len(), 2);
        // Voltage should be VoltageMagnitude at value 1.02
        assert!(meas
            .iter()
            .any(|m| m.mtype == MeasurementType::VoltageMagnitude));
        // Active power should be normalised to 0.5 pu
        let p = meas
            .iter()
            .find(|m| m.mtype == MeasurementType::PowerInjection)
            .expect("P meas");
        assert!((p.value - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_telemetry_quality_filter() {
        let mut batch = TelemetryBatch::new(TelemetrySource::Scada, 2_000_000);
        // Good, fresh
        batch.add_scada(make_voltage_point(0, 1.0, 1_900_000, 0));
        // Bad quality
        batch.add_scada(make_voltage_point(1, 1.0, 1_900_000, 1));
        // Too old (age > 500_000 µs)
        batch.add_scada(make_voltage_point(2, 1.0, 1_000_000, 0));

        let filtered = batch.filter_quality(500_000);
        assert_eq!(filtered.scada_points.len(), 1);
        assert_eq!(filtered.scada_points[0].bus_or_branch, 0);
    }

    #[test]
    fn test_compute_telemetry_stats() {
        let mut batch = TelemetryBatch::new(TelemetrySource::Scada, 2_000_000);
        batch.add_scada(make_voltage_point(0, 1.0, 1_990_000, 0));
        batch.add_scada(make_voltage_point(1, 1.0, 1_990_000, 0));
        batch.add_scada(make_voltage_point(2, 1.0, 1_990_000, 1)); // bad
        let stats = compute_telemetry_stats(&batch, 4, 100_000);
        assert_eq!(stats.n_points_received, 3);
        assert_eq!(stats.n_points_good, 2);
        assert_eq!(stats.n_points_bad, 1);
        assert!((stats.coverage_pct - 50.0).abs() < 1e-9); // 2/4
    }
}
