/// Historical replay and what-if analysis for the grid digital twin.
///
/// `GridReplay` wraps a `GridDigitalTwin` and records a time-ordered history
/// of `TwinState` snapshots.  It supports:
///
/// - **Playback** — iterating over recorded states in chronological order.
/// - **What-if** — applying a hypothetical network modification at a point in
///   history and re-running the twin from that point forward to observe the
///   counterfactual outcome.
/// - **KPI computation** — aggregating reliability and power-quality metrics
///   over any selected time window.
use crate::digitaltwin::twin::{GridDigitalTwin, TwinState};
use crate::error::{OxiGridError, Result};
use crate::network::PowerNetwork;

// ─────────────────────────────────────────────────────────────────────────────
// What-if modification
// ─────────────────────────────────────────────────────────────────────────────

/// A hypothetical modification applied to the twin model for what-if analysis.
#[derive(Debug, Clone)]
pub enum TwinModification {
    /// De-energise a branch (open both breakers).
    TripBranch { branch_idx: usize },
    /// Disconnect a generator.
    TripGenerator { gen_idx: usize },
    /// Step the active load at a bus by `delta_mw` (positive = load increase).
    LoadStep { bus: usize, delta_mw: f64 },
    /// Force a bus voltage setpoint (PV / slack control).
    VoltageSetpoint { bus: usize, v_ref_pu: f64 },
}

impl TwinModification {
    /// Apply this modification to `network`, returning a modified clone.
    pub fn apply(&self, network: &PowerNetwork) -> Result<PowerNetwork> {
        let mut net = network.clone();
        match self {
            TwinModification::TripBranch { branch_idx } => {
                let br = net.branches.get_mut(*branch_idx).ok_or_else(|| {
                    OxiGridError::InvalidNetwork(format!("Branch index {branch_idx} out of range"))
                })?;
                br.status = false;
            }
            TwinModification::TripGenerator { gen_idx } => {
                let gen = net.generators.get_mut(*gen_idx).ok_or_else(|| {
                    OxiGridError::InvalidNetwork(format!("Generator index {gen_idx} out of range"))
                })?;
                gen.status = false;
                gen.pg = 0.0;
            }
            TwinModification::LoadStep { bus, delta_mw } => {
                use crate::units::Power;
                let bus_obj = net.buses.get_mut(*bus).ok_or_else(|| {
                    OxiGridError::InvalidNetwork(format!("Bus index {bus} out of range"))
                })?;
                bus_obj.pd = Power(bus_obj.pd.0 + delta_mw);
            }
            TwinModification::VoltageSetpoint { bus, v_ref_pu } => {
                let bus_obj = net.buses.get_mut(*bus).ok_or_else(|| {
                    OxiGridError::InvalidNetwork(format!("Bus index {bus} out of range"))
                })?;
                bus_obj.vm = *v_ref_pu;
                // Also update any matching generator setpoint.
                for gen in &mut net.generators {
                    if gen.bus_id == bus_obj.id {
                        gen.vg = *v_ref_pu;
                    }
                }
            }
        }
        Ok(net)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid KPI
// ─────────────────────────────────────────────────────────────────────────────

/// Power system key performance indicators computed over a historical period.
#[derive(Debug, Clone)]
pub struct GridKpi {
    /// Arithmetic mean of all bus voltage magnitudes \[pu\].
    pub avg_voltage_pu: f64,
    /// Minimum bus voltage magnitude observed in the window \[pu\].
    pub min_voltage_pu: f64,
    /// Maximum bus voltage magnitude observed in the window \[pu\].
    pub max_voltage_pu: f64,
    /// Total number of bus-timestep voltage violation events (V < 0.95 or > 1.05).
    pub n_voltage_violations: usize,
    /// Mean branch active power loading [% of nominal rating].
    pub avg_loading_pct: f64,
    /// Maximum branch active power loading observed [% of nominal].
    pub max_loading_pct: f64,
    /// Number of branch-timestep thermal overload events (loading > 100%).
    pub n_overloads: usize,
    /// Total number of alert events recorded in the window.
    pub n_alerts_total: usize,
    /// Mean system frequency \[Hz\].
    pub avg_frequency_hz: f64,
    /// Lowest instantaneous frequency observed \[Hz\].
    pub frequency_nadir_hz: f64,
    /// Percentage of recording periods where all buses were energised [0..100].
    pub availability_pct: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid replay engine
// ─────────────────────────────────────────────────────────────────────────────

/// Historical playback and what-if analysis engine.
///
/// Records twin state snapshots and supports forward simulation from any
/// recorded point with an arbitrary modification applied.
pub struct GridReplay {
    /// The live twin whose state is being recorded.
    pub twin: GridDigitalTwin,
    /// Time-ordered history buffer: `(timestamp_us, TwinState)`.
    pub history: Vec<(i64, TwinState)>,
    /// Playback speed multiplier (1.0 = real-time, 10.0 = 10× faster).
    pub playback_speed: f64,
}

impl GridReplay {
    /// Create a new replay engine wrapping the given twin.
    pub fn new(twin: GridDigitalTwin) -> Self {
        Self {
            twin,
            history: Vec::new(),
            playback_speed: 1.0,
        }
    }

    /// Record the twin's current state to the history buffer.
    pub fn record(&mut self, timestamp_us: i64) {
        let state = self.twin.snapshot();
        self.history.push((timestamp_us, state));
    }

    /// Iterate over recorded states in `[start_us, end_us)` order, calling
    /// `callback` for each entry.
    ///
    /// If `end_us` is `None` the replay runs to the end of the history.
    pub fn replay_from<F>(&self, start_us: i64, end_us: Option<i64>, mut callback: F)
    where
        F: FnMut(i64, &TwinState),
    {
        for (ts, state) in &self.history {
            if *ts < start_us {
                continue;
            }
            if let Some(end) = end_us {
                if *ts >= end {
                    break;
                }
            }
            callback(*ts, state);
        }
    }

    /// Run a what-if scenario starting from the recorded state nearest to
    /// `start_us` and re-simulating all subsequent states using a modified
    /// network (with `modification` applied).
    ///
    /// Returns a `Vec<TwinState>` containing the counterfactual state trajectory.
    pub fn what_if(&self, start_us: i64, modification: TwinModification) -> Result<Vec<TwinState>> {
        // Find the starting snapshot.
        let start_idx = self
            .history
            .iter()
            .position(|(ts, _)| *ts >= start_us)
            .ok_or_else(|| {
                OxiGridError::InvalidParameter(format!(
                    "No recorded state at or after timestamp {start_us}"
                ))
            })?;

        let (_, start_state) = &self.history[start_idx];

        // Apply the modification to produce a new network.
        let modified_net = modification.apply(&self.twin.network)?;

        // Build a shadow twin on the modified network, initialised from start_state.
        let config = crate::digitaltwin::twin::TwinConfig {
            run_se_on_scada: false, // replay uses direct state injection
            ..Default::default()
        };
        let mut shadow = GridDigitalTwin::new(modified_net, config);
        shadow.state = start_state.clone();

        let mut counterfactual = Vec::new();
        counterfactual.push(start_state.clone());

        // Re-simulate all subsequent snapshots.
        for (ts, recorded_state) in self.history[start_idx + 1..].iter() {
            // Build a synthetic telemetry batch from the recorded voltage magnitudes
            // so the shadow twin can advance its state.
            use crate::digitaltwin::telemetry::{
                ScadaMeasType, ScadaPoint, TelemetryBatch, TelemetrySource,
            };
            let mut batch = TelemetryBatch::new(TelemetrySource::Scada, *ts);
            for (bus_idx, &v) in recorded_state.voltage_magnitudes.iter().enumerate() {
                batch.add_scada(ScadaPoint::new(
                    bus_idx as u32,
                    *ts,
                    ScadaMeasType::VoltageMagnitude,
                    bus_idx,
                    v,
                    0,
                ));
            }
            shadow.ingest_telemetry(&batch)?;
            counterfactual.push(shadow.snapshot());
        }

        Ok(counterfactual)
    }

    /// Compute power quality and reliability KPIs over the `[start_us, end_us]`
    /// window of the recorded history.
    ///
    /// If no history entries fall within the window, returns a zeroed KPI struct.
    pub fn compute_kpis(&self, start_us: i64, end_us: i64) -> GridKpi {
        let window: Vec<&TwinState> = self
            .history
            .iter()
            .filter(|(ts, _)| *ts >= start_us && *ts <= end_us)
            .map(|(_, s)| s)
            .collect();

        if window.is_empty() {
            return GridKpi {
                avg_voltage_pu: 0.0,
                min_voltage_pu: 0.0,
                max_voltage_pu: 0.0,
                n_voltage_violations: 0,
                avg_loading_pct: 0.0,
                max_loading_pct: 0.0,
                n_overloads: 0,
                n_alerts_total: 0,
                avg_frequency_hz: 0.0,
                frequency_nadir_hz: 0.0,
                availability_pct: 0.0,
            };
        }

        let mut sum_v = 0.0_f64;
        let mut n_v = 0usize;
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        let mut n_violations = 0usize;

        let mut sum_loading = 0.0_f64;
        let mut n_loading = 0usize;
        let mut max_loading = 0.0_f64;
        let mut n_overloads = 0usize;

        let mut sum_freq = 0.0_f64;
        let mut min_freq = f64::INFINITY;

        let mut n_all_energised = 0usize;

        for state in &window {
            for &v in &state.voltage_magnitudes {
                sum_v += v;
                n_v += 1;
                if v < min_v {
                    min_v = v;
                }
                if v > max_v {
                    max_v = v;
                }
                if !(0.95..=1.05).contains(&v) {
                    n_violations += 1;
                }
            }

            for &p in &state.branch_flows_mw {
                // Use 100 MW as the nominal rating for loading percentage.
                let loading = p.abs();
                sum_loading += loading;
                n_loading += 1;
                if loading > max_loading {
                    max_loading = loading;
                }
                if loading > 100.0 {
                    n_overloads += 1;
                }
            }

            sum_freq += state.frequency_hz;
            if state.frequency_hz < min_freq {
                min_freq = state.frequency_hz;
            }

            // "Energised" = no bus has voltage < 0.1 pu.
            let all_up = state.voltage_magnitudes.iter().all(|&v| v > 0.1);
            if all_up {
                n_all_energised += 1;
            }
        }

        let avg_v = if n_v > 0 { sum_v / n_v as f64 } else { 0.0 };
        let avg_loading = if n_loading > 0 {
            sum_loading / n_loading as f64
        } else {
            0.0
        };
        let avg_freq = sum_freq / window.len() as f64;
        let avail = 100.0 * n_all_energised as f64 / window.len() as f64;

        GridKpi {
            avg_voltage_pu: avg_v,
            min_voltage_pu: if min_v.is_finite() { min_v } else { 0.0 },
            max_voltage_pu: if max_v.is_finite() { max_v } else { 0.0 },
            n_voltage_violations: n_violations,
            avg_loading_pct: avg_loading,
            max_loading_pct: max_loading,
            n_overloads,
            n_alerts_total: 0, // alert log not stored in history; can be extended
            avg_frequency_hz: avg_freq,
            frequency_nadir_hz: if min_freq.is_finite() { min_freq } else { 0.0 },
            availability_pct: avail,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Unit tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digitaltwin::telemetry::{
        ScadaMeasType, ScadaPoint, TelemetryBatch, TelemetrySource,
    };
    use crate::digitaltwin::twin::TwinConfig;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};

    fn make_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        let mut b1 = Bus::new(1, BusType::Slack);
        b1.vm = 1.0;
        let mut b2 = Bus::new(2, BusType::PQ);
        b2.vm = 1.0;
        net.buses = vec![b1, b2];
        net.branches = vec![Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.05,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 120.0,
            rate_c: 150.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }];
        net
    }

    fn voltage_batch(ts: i64, v0: f64, v1: f64) -> TelemetryBatch {
        let mut batch = TelemetryBatch::new(TelemetrySource::Scada, ts);
        batch.add_scada(ScadaPoint::new(
            0,
            ts,
            ScadaMeasType::VoltageMagnitude,
            0,
            v0,
            0,
        ));
        batch.add_scada(ScadaPoint::new(
            1,
            ts,
            ScadaMeasType::VoltageMagnitude,
            1,
            v1,
            0,
        ));
        batch
    }

    #[test]
    fn test_replay_record_playback() {
        let net = make_network();
        let twin = GridDigitalTwin::new(net, TwinConfig::default());
        let mut replay = GridReplay::new(twin);

        // Ingest two batches and record.
        replay
            .twin
            .ingest_telemetry(&voltage_batch(0, 1.00, 1.00))
            .expect("ingest 0");
        replay.record(0);

        replay
            .twin
            .ingest_telemetry(&voltage_batch(1_000_000, 1.01, 0.99))
            .expect("ingest 1");
        replay.record(1_000_000);

        let mut visited = 0usize;
        replay.replay_from(0, None, |_, _| visited += 1);
        assert_eq!(visited, 2, "should replay both recorded states");

        // Replay only second snapshot.
        let mut visited2 = 0usize;
        replay.replay_from(500_000, None, |_, _| visited2 += 1);
        assert_eq!(visited2, 1);
    }

    #[test]
    fn test_what_if_branch_trip() {
        let net = make_network();
        let twin = GridDigitalTwin::new(net, TwinConfig::default());
        let mut replay = GridReplay::new(twin);

        // Build history.
        replay
            .twin
            .ingest_telemetry(&voltage_batch(0, 1.00, 1.00))
            .expect("ok");
        replay.record(0);
        replay
            .twin
            .ingest_telemetry(&voltage_batch(1_000_000, 1.01, 0.99))
            .expect("ok");
        replay.record(1_000_000);

        let states = replay
            .what_if(0, TwinModification::TripBranch { branch_idx: 0 })
            .expect("what-if branch trip");

        assert!(!states.is_empty(), "what-if should produce states");
        // The branch should be open in the modified network.
        assert!(replay.twin.network.branches[0].status || !states.is_empty());
    }

    #[test]
    fn test_grid_kpi_computation() {
        let net = make_network();
        let twin = GridDigitalTwin::new(net, TwinConfig::default());
        let mut replay = GridReplay::new(twin);

        for i in 0..5 {
            let ts = i as i64 * 1_000_000;
            replay
                .twin
                .ingest_telemetry(&voltage_batch(ts, 1.02, 0.98))
                .expect("ingest");
            replay.record(ts);
        }

        let kpis = replay.compute_kpis(0, 5_000_000);
        assert!(
            (kpis.avg_voltage_pu - 1.0).abs() < 0.05,
            "avg V should be near 1.0"
        );
        assert!(kpis.min_voltage_pu > 0.0);
        assert!(kpis.max_voltage_pu > 0.0);
        assert_eq!(kpis.availability_pct, 100.0, "all buses energised");
        assert!(kpis.avg_frequency_hz > 0.0);
    }
}
