//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

use super::types::{
    GridEvent, GridOpsStatistics, QdGridOpsConfig, QdGridOpsResult, ScheduledEvent, SimBranch,
    SimGenerator, SimLoad, SystemSnapshot,
};

/// Event-driven quasi-dynamic grid operations simulator.
pub struct GridOperationsSimulator {
    pub config: QdGridOpsConfig,
    pub generators: Vec<SimGenerator>,
    pub loads: Vec<SimLoad>,
    pub branches: Vec<SimBranch>,
    pub storages: Vec<SimStorage>,
    pub scheduled_events: Vec<ScheduledEvent>,
    pub clock: SimClock,
    pub branch_susceptances: Vec<f64>,
}
impl GridOperationsSimulator {
    pub fn new(
        config: QdGridOpsConfig,
        generators: Vec<SimGenerator>,
        loads: Vec<SimLoad>,
        branches: Vec<SimBranch>,
        storages: Vec<SimStorage>,
        duration_s: f64,
        dt_s: f64,
    ) -> Self {
        let branch_susceptances: Vec<f64> = branches
            .iter()
            .map(|b| {
                if b.rating_mva > 0.0 {
                    1.0 / b.rating_mva
                } else {
                    1.0
                }
            })
            .collect();
        Self {
            config,
            generators,
            loads,
            branches,
            storages,
            scheduled_events: Vec::new(),
            clock: SimClock::new(0.0, duration_s, dt_s),
            branch_susceptances,
        }
    }
    /// Schedule a grid event to occur at `time_s`.
    pub fn schedule_event(&mut self, time_s: f64, event: GridEvent, description: String) {
        self.scheduled_events.push(ScheduledEvent {
            time_s,
            event,
            description,
        });
        self.scheduled_events.sort_by(|a, b| {
            a.time_s
                .partial_cmp(&b.time_s)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    /// Run the quasi-dynamic simulation loop.
    pub fn run(&mut self) -> Result<QdGridOpsResult, OxiGridError> {
        if self.clock.end_time_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "end_time_s must be positive".to_string(),
            ));
        }
        if self.clock.dt_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_s must be positive".to_string(),
            ));
        }
        let mut snapshots: Vec<SystemSnapshot> = Vec::new();
        let mut events_log: Vec<(f64, String)> = Vec::new();
        let mut frequency_history: Vec<(f64, f64)> = Vec::new();
        let nominal = self.config.nominal_frequency_hz;
        let mut freq_hz = nominal;
        let dt_s = self.clock.dt_s;
        loop {
            let t = self.clock.current_time_s;
            let mut step_events: Vec<String> = Vec::new();
            let mut i = 0;
            while i < self.scheduled_events.len() {
                if self.scheduled_events[i].time_s <= t {
                    let se = self.scheduled_events.remove(i);
                    let desc = self.process_event(&se.event);
                    step_events.push(desc.clone());
                    events_log.push((t, desc));
                } else {
                    i += 1;
                }
            }
            let delta_p = self.compute_power_balance();
            freq_hz = self.update_frequency(delta_p, freq_hz, dt_s);
            let delta_f = freq_hz - nominal;
            if delta_f.abs() > self.config.frequency_deadband_hz {
                self.apply_agc(delta_f, dt_s);
                let delta_p2 = self.compute_power_balance();
                freq_hz = self.update_frequency(delta_p2, freq_hz, dt_s);
            }
            if freq_hz < self.config.ufls_threshold_hz {
                let shed = self.apply_ufls(freq_hz);
                if shed > 0.0 {
                    let msg = format!("UFLS shed {shed:.2} MW at f={freq_hz:.3} Hz");
                    step_events.push(msg.clone());
                    events_log.push((t, msg));
                }
            }
            self.compute_branch_flows();
            let voltages: Vec<f64> = vec![1.0; self.config.n_buses];
            let violations = self.check_violations(freq_hz, &voltages);
            for v in &violations {
                events_log.push((t, v.clone()));
            }
            self.update_storage_soc(dt_s);
            let snap = self.take_snapshot(freq_hz, step_events, violations);
            frequency_history.push((t, freq_hz));
            snapshots.push(snap);
            if !self.clock.advance() {
                break;
            }
        }
        let statistics = Self::compute_statistics(&snapshots, dt_s);
        Ok(QdGridOpsResult {
            snapshots,
            events_log,
            frequency_history,
            statistics,
        })
    }
    /// Process a single grid event, mutating system state. Returns a human-readable description.
    pub(super) fn process_event(&mut self, event: &GridEvent) -> String {
        match event {
            GridEvent::GeneratorTrip {
                bus,
                capacity_mw,
                reason,
            } => {
                for gen in &mut self.generators {
                    if gen.bus == *bus {
                        gen.is_online = false;
                        gen.p_mw = 0.0;
                    }
                }
                format!("GeneratorTrip: bus={bus} cap={capacity_mw:.1} MW reason={reason}")
            }
            GridEvent::LineTrip { branch_id, reason } => {
                for br in &mut self.branches {
                    if br.id == *branch_id {
                        br.is_online = false;
                        br.current_flow_mw = 0.0;
                        br.loading_pct = 0.0;
                    }
                }
                format!("LineTrip: branch={branch_id} reason={reason}")
            }
            GridEvent::LoadIncrease { bus, delta_mw } => {
                for load in &mut self.loads {
                    if load.bus == *bus {
                        load.p_mw += delta_mw;
                    }
                }
                format!("LoadIncrease: bus={bus} delta={delta_mw:.2} MW")
            }
            GridEvent::LoadDecrease { bus, delta_mw } => {
                for load in &mut self.loads {
                    if load.bus == *bus {
                        load.p_mw = (load.p_mw - delta_mw).max(0.0);
                    }
                }
                format!("LoadDecrease: bus={bus} delta={delta_mw:.2} MW")
            }
            GridEvent::GeneratorReconnect { bus, capacity_mw } => {
                for gen in &mut self.generators {
                    if gen.bus == *bus {
                        gen.is_online = true;
                        gen.p_max_mw = *capacity_mw;
                        gen.p_mw = gen.p_min_mw;
                    }
                }
                format!("GeneratorReconnect: bus={bus} cap={capacity_mw:.1} MW")
            }
            GridEvent::LineReconnect { branch_id } => {
                for br in &mut self.branches {
                    if br.id == *branch_id {
                        br.is_online = true;
                    }
                }
                format!("LineReconnect: branch={branch_id}")
            }
            GridEvent::StorageCharge { bus, rate_mw } => {
                for st in &mut self.storages {
                    if st.bus == *bus {
                        st.power_mw = -rate_mw;
                    }
                }
                format!("StorageCharge: bus={bus} rate={rate_mw:.2} MW")
            }
            GridEvent::StorageDischarge { bus, rate_mw } => {
                for st in &mut self.storages {
                    if st.bus == *bus {
                        st.power_mw = *rate_mw;
                    }
                }
                format!("StorageDischarge: bus={bus} rate={rate_mw:.2} MW")
            }
            GridEvent::AutomaticGenControl { area_mw } => {
                format!("AGC signal: area_mw={area_mw:.2} MW")
            }
            GridEvent::UnderFrequencyLoadShedding { buses, shed_mw } => {
                let per_bus = if buses.is_empty() {
                    0.0
                } else {
                    shed_mw / buses.len() as f64
                };
                for b in buses {
                    for load in &mut self.loads {
                        if load.bus == *b {
                            load.p_mw = (load.p_mw - per_bus).max(0.0);
                        }
                    }
                }
                format!("UFLS: buses={buses:?} shed={shed_mw:.2} MW")
            }
            GridEvent::VoltageLimitViolation { bus, voltage_pu } => {
                format!("VoltageLimitViolation: bus={bus} V={voltage_pu:.4} pu")
            }
            GridEvent::OverloadAlarm {
                branch_id,
                loading_pct,
            } => {
                format!("OverloadAlarm: branch={branch_id} loading={loading_pct:.1}%")
            }
        }
    }
    /// Compute net power balance: total online generation minus total load minus storage injection.
    pub(super) fn compute_power_balance(&self) -> f64 {
        let gen: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let load: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        let storage_net: f64 = self.storages.iter().map(|s| s.power_mw).sum();
        gen + storage_net - load
    }
    /// Update system frequency using the swing equation.
    ///
    /// df/dt = ΔP / (2 * H * S_base)
    pub(super) fn update_frequency(&self, delta_p_mw: f64, freq_hz: f64, dt_s: f64) -> f64 {
        let h_inertia = 5.0_f64;
        let s_base = self.config.base_mva;
        let df_dt = delta_p_mw / (2.0 * h_inertia * s_base);
        let new_freq = freq_hz + df_dt * dt_s;
        let nom = self.config.nominal_frequency_hz;
        new_freq.clamp(nom - 5.0, nom + 5.0)
    }
    /// Apply Automatic Generation Control (AGC) to restore frequency.
    pub(super) fn apply_agc(&mut self, delta_f_hz: f64, dt_s: f64) {
        let bias = 10.0 * self.config.base_mva;
        let ace = delta_f_hz * bias;
        let total_participation: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.agc_participation)
            .sum();
        if total_participation <= 0.0 {
            return;
        }
        for gen in &mut self.generators {
            if !gen.is_online || gen.agc_participation <= 0.0 {
                continue;
            }
            let fraction = gen.agc_participation / total_participation;
            let delta_p_gen = -ace * fraction;
            let max_ramp = gen.ramp_rate_mw_per_min * dt_s / 60.0;
            let delta_p_clamped = delta_p_gen.clamp(-max_ramp, max_ramp);
            gen.p_mw = (gen.p_mw + delta_p_clamped).clamp(gen.p_min_mw, gen.p_max_mw);
        }
    }
    /// Apply Under-Frequency Load Shedding in steps.
    ///
    /// Each 0.2 Hz below `ufls_threshold_hz` triggers one shedding step.
    pub(super) fn apply_ufls(&mut self, frequency_hz: f64) -> f64 {
        let threshold = self.config.ufls_threshold_hz;
        if frequency_hz >= threshold {
            return 0.0;
        }
        let steps_below = ((threshold - frequency_hz) / 0.2).floor() as usize;
        let n_steps = steps_below.min(self.config.ufls_shed_pct.len());
        if n_steps == 0 {
            return 0.0;
        }
        let total_shedable: f64 = self
            .loads
            .iter()
            .filter(|l| l.is_shedable)
            .map(|l| l.p_mw)
            .sum();
        let mut total_shed = 0.0_f64;
        for step_idx in 0..n_steps {
            let shed_fraction = self.config.ufls_shed_pct[step_idx];
            let target_shed = total_shedable * shed_fraction;
            let mut remaining = target_shed;
            let mut indices: Vec<usize> = self
                .loads
                .iter()
                .enumerate()
                .filter(|(_, l)| l.is_shedable && l.p_mw > 0.0)
                .map(|(i, _)| i)
                .collect();
            indices.sort_by(|&a, &b| self.loads[b].priority.cmp(&self.loads[a].priority));
            for idx in indices {
                if remaining <= 0.0 {
                    break;
                }
                let shed = self.loads[idx].p_mw.min(remaining);
                self.loads[idx].p_mw -= shed;
                remaining -= shed;
                total_shed += shed;
            }
        }
        total_shed
    }
    /// DC power flow approximation: distribute flows proportional to branch susceptances.
    pub(super) fn compute_branch_flows(&mut self) {
        let total_gen: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let total_load: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        let net_power = total_gen - total_load;
        let total_susceptance: f64 = self
            .branches
            .iter()
            .enumerate()
            .filter(|(_, b)| b.is_online)
            .map(|(i, _)| self.branch_susceptances.get(i).copied().unwrap_or(1.0))
            .sum();
        for (i, branch) in self.branches.iter_mut().enumerate() {
            if !branch.is_online {
                branch.current_flow_mw = 0.0;
                branch.current_flow_mvar = 0.0;
                branch.loading_pct = 0.0;
                continue;
            }
            let b_i = self.branch_susceptances.get(i).copied().unwrap_or(1.0);
            branch.current_flow_mw = if total_susceptance > 0.0 {
                (b_i / total_susceptance) * net_power * 0.5
            } else {
                0.0
            };
            branch.loading_pct =
                (branch.current_flow_mw.abs() / branch.rating_mva.max(1.0)) * 100.0;
        }
    }
    /// Check and return violation strings for frequency, voltage, and branch overloads.
    pub(super) fn check_violations(&self, freq_hz: f64, voltages: &[f64]) -> Vec<String> {
        let mut violations = Vec::new();
        let nom = self.config.nominal_frequency_hz;
        if freq_hz < nom - 0.5 {
            violations.push(format!(
                "UnderFrequency: f={freq_hz:.3} Hz (nominal {nom} Hz)"
            ));
        } else if freq_hz > nom + 0.5 {
            violations.push(format!(
                "OverFrequency: f={freq_hz:.3} Hz (nominal {nom} Hz)"
            ));
        }
        for (bus, &v) in voltages.iter().enumerate() {
            if v < self.config.voltage_min_pu {
                violations.push(format!("UnderVoltage: bus={bus} V={v:.4} pu"));
            } else if v > self.config.voltage_max_pu {
                violations.push(format!("OverVoltage: bus={bus} V={v:.4} pu"));
            }
        }
        for branch in &self.branches {
            if branch.is_online && branch.loading_pct > self.config.max_branch_loading_pct {
                violations.push(format!(
                    "BranchOverload: branch={} loading={:.1}%",
                    branch.id, branch.loading_pct
                ));
            }
        }
        violations
    }
    /// Update storage state-of-charge based on current power dispatch.
    pub(super) fn update_storage_soc(&mut self, dt_s: f64) {
        for st in &mut self.storages {
            let soc_delta = if st.power_mw > 0.0 {
                let energy_out = st.power_mw * dt_s / 3600.0;
                -energy_out / (st.capacity_mwh.max(1e-9) * st.efficiency.max(1e-9))
            } else if st.power_mw < 0.0 {
                let energy_in = st.power_mw.abs() * dt_s / 3600.0 * st.efficiency;
                energy_in / st.capacity_mwh.max(1e-9)
            } else {
                0.0
            };
            st.soc = (st.soc + soc_delta).clamp(0.0, 1.0);
        }
    }
    /// Compute summary statistics from the recorded snapshots.
    pub(super) fn compute_statistics(snapshots: &[SystemSnapshot], dt_s: f64) -> GridOpsStatistics {
        if snapshots.is_empty() {
            return GridOpsStatistics {
                duration_s: 0.0,
                total_energy_mwh: 0.0,
                renewable_energy_mwh: 0.0,
                load_served_pct: 100.0,
                shed_energy_mwh: 0.0,
                total_co2_ton: 0.0,
                min_frequency_hz: 50.0,
                max_frequency_hz: 50.0,
                n_frequency_violations: 0,
                n_voltage_violations: 0,
                n_line_trips: 0,
                n_generator_trips: 0,
                n_load_shed_events: 0,
                system_resilience_index: 1.0,
            };
        }
        let duration_s = snapshots.last().map(|s| s.time_s).unwrap_or(0.0)
            - snapshots.first().map(|s| s.time_s).unwrap_or(0.0);
        let dt_h = dt_s / 3600.0;
        let mut total_energy_mwh = 0.0_f64;
        let mut renewable_energy_mwh = 0.0_f64;
        let mut total_co2_ton = 0.0_f64;
        let mut total_load_mwh = 0.0_f64;
        let mut shed_energy_mwh = 0.0_f64;
        let mut min_freq = f64::INFINITY;
        let mut max_freq = f64::NEG_INFINITY;
        let mut n_freq_violations = 0usize;
        let mut n_volt_violations = 0usize;
        let mut n_line_trips = 0usize;
        let mut n_gen_trips = 0usize;
        let mut n_load_shed_events = 0usize;
        let nominal = if !snapshots.is_empty() && snapshots[0].frequency_hz > 55.0 {
            60.0
        } else {
            50.0
        };
        for snap in snapshots {
            let gen_mwh = snap.total_generation_mw * dt_h;
            total_energy_mwh += gen_mwh;
            total_load_mwh += snap.total_load_mw * dt_h;
            for gen in &snap.generators {
                if gen.is_online {
                    let e = gen.p_mw * dt_h;
                    let fuel = gen.fuel_type.to_lowercase();
                    if fuel.contains("wind") || fuel.contains("solar") || fuel.contains("pv") {
                        renewable_energy_mwh += e;
                    }
                    total_co2_ton += gen.p_mw * gen.co2_kg_per_mwh * dt_h / 1000.0;
                }
            }
            if (snap.frequency_hz - nominal).abs() > 0.5 {
                n_freq_violations += 1;
            }
            min_freq = min_freq.min(snap.frequency_hz);
            max_freq = max_freq.max(snap.frequency_hz);
            for v in &snap.violations {
                let vl = v.to_lowercase();
                if vl.contains("voltage") {
                    n_volt_violations += 1;
                }
            }
            for ev in &snap.events_this_step {
                let el = ev.to_lowercase();
                if el.contains("linetrip") || el.contains("line trip") {
                    n_line_trips += 1;
                }
                if el.contains("generatortrip") || el.contains("generator trip") {
                    n_gen_trips += 1;
                }
                if el.contains("ufls") || el.contains("shed") {
                    n_load_shed_events += 1;
                }
            }
            if snap.power_imbalance_mw < 0.0 {
                shed_energy_mwh += snap.power_imbalance_mw.abs() * dt_h;
            }
        }
        let load_served_pct = if total_load_mwh > 0.0 {
            ((total_load_mwh - shed_energy_mwh) / total_load_mwh * 100.0).clamp(0.0, 100.0)
        } else {
            100.0
        };
        let system_resilience_index =
            1.0 - shed_energy_mwh / (total_energy_mwh + shed_energy_mwh + 1e-9);
        if min_freq.is_infinite() {
            min_freq = nominal;
        }
        if max_freq.is_infinite() {
            max_freq = nominal;
        }
        GridOpsStatistics {
            duration_s,
            total_energy_mwh,
            renewable_energy_mwh,
            load_served_pct,
            shed_energy_mwh,
            total_co2_ton,
            min_frequency_hz: min_freq,
            max_frequency_hz: max_freq,
            n_frequency_violations: n_freq_violations,
            n_voltage_violations: n_volt_violations,
            n_line_trips,
            n_generator_trips: n_gen_trips,
            n_load_shed_events,
            system_resilience_index: system_resilience_index.clamp(0.0, 1.0),
        }
    }
    /// Take a snapshot of the current system state.
    pub(super) fn take_snapshot(
        &self,
        freq_hz: f64,
        events: Vec<String>,
        violations: Vec<String>,
    ) -> SystemSnapshot {
        let total_generation_mw: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let total_load_mw: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        let total_losses_mw = total_generation_mw * 0.02;
        let power_imbalance_mw = total_generation_mw - total_load_mw - total_losses_mw;
        let n_generators_online = self.generators.iter().filter(|g| g.is_online).count();
        SystemSnapshot {
            time_s: self.clock.current_time_s,
            generators: self.generators.clone(),
            loads: self.loads.clone(),
            branches: self.branches.clone(),
            storages: self.storages.clone(),
            frequency_hz: freq_hz,
            total_generation_mw,
            total_load_mw,
            total_losses_mw,
            power_imbalance_mw,
            events_this_step: events,
            violations,
            n_generators_online,
        }
    }
}
/// Battery storage unit state.
#[derive(Debug, Clone)]
pub struct SimStorage {
    pub bus: usize,
    pub soc: f64,
    pub capacity_mwh: f64,
    pub power_mw: f64,
    pub max_charge_mw: f64,
    pub max_discharge_mw: f64,
    pub efficiency: f64,
}
/// Discrete-event simulation clock.
#[derive(Debug, Clone)]
pub struct SimClock {
    pub current_time_s: f64,
    pub dt_s: f64,
    pub end_time_s: f64,
}
impl SimClock {
    pub fn new(start_s: f64, end_s: f64, dt_s: f64) -> Self {
        Self {
            current_time_s: start_s,
            dt_s,
            end_time_s: end_s,
        }
    }
    /// Advance the clock by one timestep. Returns `false` when simulation is complete.
    pub fn advance(&mut self) -> bool {
        self.current_time_s += self.dt_s;
        self.current_time_s <= self.end_time_s
    }
    /// Hour of day (0–24) corresponding to `current_time_s`.
    pub fn time_of_day_h(&self) -> f64 {
        (self.current_time_s / 3600.0) % 24.0
    }
}
/// Configuration for the legacy grid operations simulator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridOpsConfig {
    pub simulation_hours: usize,
    pub dt_minutes: f64,
    pub operator_skill: f64,
    pub automation_level: f64,
    pub contingency_probability: f64,
    pub weather_events: bool,
}
