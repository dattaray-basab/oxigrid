use crate::battery::ecm::TwoRcModel;
/// Battery pack configuration: series/parallel cell arrangements.
///
/// # Topology
/// A pack of Ns cells in series × Np cells in parallel.
///
/// - Pack voltage    = Ns × Cell voltage
/// - Pack capacity   = Np × Cell capacity (Ah)
/// - Pack resistance = Ns/Np × Cell resistance
///
/// Passive cell balancing bleeds excess SoC from high-SoC cells.
use crate::battery::BatteryModel;
use crate::units::{Current, Energy, StateOfCharge, Temperature, Voltage};
use serde::{Deserialize, Serialize};

/// BMS interface: reports pack-level state and supports balancing commands.
pub trait Bms {
    fn pack_state(&self) -> PackState;
    fn balance_step(&mut self, dt: f64);
}

/// Aggregated pack-level state.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PackState {
    pub voltage: Voltage,
    pub current: Current,
    pub soc_min: StateOfCharge,
    pub soc_max: StateOfCharge,
    pub soc_mean: StateOfCharge,
    pub soc_imbalance: f64, // max - min
    pub temperature_max: Temperature,
    pub capacity_wh: Energy,
}

/// Series-parallel pack using 2RC Thevenin cells.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeriesParallelPack {
    pub ns: usize,                   // cells in series
    pub np: usize,                   // cells in parallel
    pub cells: Vec<Vec<TwoRcModel>>, // [series_idx][parallel_idx]
    pub balancing_current_a: f64,    // passive balancing bleed current [A]
}

impl SeriesParallelPack {
    /// Create pack with identical cells.
    pub fn uniform(ns: usize, np: usize, cell_template: TwoRcModel) -> Self {
        let cells = (0..ns)
            .map(|_| (0..np).map(|_| cell_template.clone()).collect())
            .collect();
        Self {
            ns,
            np,
            cells,
            balancing_current_a: 0.05,
        }
    }

    /// Pack nominal voltage (V).
    pub fn nominal_voltage(&self) -> f64 {
        let v_cell = self.cells[0][0].ocv_curve.ocv(self.cells[0][0].soc);
        v_cell * self.ns as f64
    }

    /// Pack nominal capacity (Ah).
    pub fn nominal_capacity_ah(&self) -> f64 {
        self.cells[0][0].capacity_ah * self.np as f64
    }

    /// Step all cells with distributed current.
    /// Pack current is shared equally among parallel cells.
    pub fn step(&mut self, pack_current: Current, dt: f64, temp: Temperature) -> PackState {
        let cell_current = Current(pack_current.0 / self.np as f64);

        let mut v_pack = 0.0_f64;
        let mut soc_vals = Vec::with_capacity(self.ns * self.np);
        let mut temp_max = temp.0;

        for series_row in &mut self.cells {
            // Series voltage = average of parallel cells in that group
            let mut v_row = 0.0_f64;
            for cell in series_row.iter_mut() {
                let state = cell.step(cell_current, dt, temp);
                v_row += state.voltage.0;
                soc_vals.push(state.soc.0);
                temp_max = temp_max.max(temp.0);
            }
            v_pack += v_row / self.np as f64;
        }

        let soc_min = soc_vals.iter().cloned().fold(f64::INFINITY, f64::min);
        let soc_max = soc_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let soc_mean = soc_vals.iter().sum::<f64>() / soc_vals.len() as f64;

        PackState {
            voltage: Voltage(v_pack),
            current: pack_current,
            soc_min: StateOfCharge::new(soc_min),
            soc_max: StateOfCharge::new(soc_max),
            soc_mean: StateOfCharge::new(soc_mean),
            soc_imbalance: soc_max - soc_min,
            temperature_max: Temperature(temp_max),
            capacity_wh: Energy(soc_mean * self.nominal_capacity_ah() * v_pack / self.ns as f64),
        }
    }

    pub fn pack_state_snapshot(&self) -> PackState {
        let soc_vals: Vec<f64> = self
            .cells
            .iter()
            .flat_map(|row| row.iter().map(|c| c.soc))
            .collect();

        let mut v_pack = 0.0_f64;
        for row in &self.cells {
            v_pack += row.iter().map(|c| c.ocv_curve.ocv(c.soc)).sum::<f64>() / self.np as f64;
        }

        let soc_min = soc_vals.iter().cloned().fold(f64::INFINITY, f64::min);
        let soc_max = soc_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let soc_mean = soc_vals.iter().sum::<f64>() / soc_vals.len() as f64;

        PackState {
            voltage: Voltage(v_pack),
            current: Current(0.0),
            soc_min: StateOfCharge::new(soc_min),
            soc_max: StateOfCharge::new(soc_max),
            soc_mean: StateOfCharge::new(soc_mean),
            soc_imbalance: soc_max - soc_min,
            temperature_max: Temperature(298.15),
            capacity_wh: Energy(soc_mean * self.nominal_capacity_ah() * v_pack / self.ns as f64),
        }
    }
}

impl Bms for SeriesParallelPack {
    fn pack_state(&self) -> PackState {
        self.pack_state_snapshot()
    }

    /// Passive balancing: bleed current from cells above mean SoC.
    fn balance_step(&mut self, dt: f64) {
        let soc_mean: f64 = self
            .cells
            .iter()
            .flat_map(|row| row.iter().map(|c| c.soc))
            .sum::<f64>()
            / (self.ns * self.np) as f64;

        for row in &mut self.cells {
            for cell in row.iter_mut() {
                if cell.soc > soc_mean + 0.005 {
                    // Bleed discharge
                    let dsoc = self.balancing_current_a * dt / (3600.0 * cell.capacity_ah);
                    cell.soc = (cell.soc - dsoc).clamp(0.0, 1.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::battery::OcvSocCurve;

    fn make_cell() -> TwoRcModel {
        TwoRcModel::new(
            OcvSocCurve::nmc_default(),
            0.02,
            0.015,
            3000.0,
            0.01,
            500.0,
            3.0,
        )
    }

    #[test]
    fn test_pack_voltage_scales_with_ns() {
        let cell = make_cell();
        let v_cell = cell.ocv_curve.ocv(cell.soc);

        let pack_4s1p = SeriesParallelPack::uniform(4, 1, cell.clone());
        let pack_8s1p = SeriesParallelPack::uniform(8, 1, cell.clone());

        let v4 = pack_4s1p.nominal_voltage();
        let v8 = pack_8s1p.nominal_voltage();

        assert!((v4 - 4.0 * v_cell).abs() < 1e-9);
        assert!((v8 - 8.0 * v_cell).abs() < 1e-9);
    }

    #[test]
    fn test_pack_capacity_scales_with_np() {
        let cell = make_cell();
        let cap_cell = cell.capacity_ah;

        let pack_1s2p = SeriesParallelPack::uniform(1, 2, cell.clone());
        let pack_1s4p = SeriesParallelPack::uniform(1, 4, cell.clone());

        assert!((pack_1s2p.nominal_capacity_ah() - 2.0 * cap_cell).abs() < 1e-9);
        assert!((pack_1s4p.nominal_capacity_ah() - 4.0 * cap_cell).abs() < 1e-9);
    }

    #[test]
    fn test_pack_discharge_soc_decreases() {
        let mut pack = SeriesParallelPack::uniform(4, 2, make_cell());
        let initial_soc = pack.pack_state().soc_mean.0;
        // Discharge at 1C of total capacity for 360s (10% discharge)
        let cap = pack.nominal_capacity_ah();
        for _ in 0..360 {
            pack.step(Current(cap), 1.0, Temperature(298.15));
        }
        let final_soc = pack.pack_state().soc_mean.0;
        assert!(final_soc < initial_soc);
    }

    #[test]
    fn test_balancing_reduces_imbalance() {
        let cell1 = make_cell();
        let mut cell2 = make_cell();
        cell2.soc = 0.7; // Intentional imbalance

        let mut pack = SeriesParallelPack::uniform(1, 2, cell1);
        pack.cells[0][1].soc = 0.7;

        let imbalance_before = pack.pack_state().soc_imbalance;
        for _ in 0..1000 {
            pack.balance_step(1.0);
        }
        let imbalance_after = pack.pack_state().soc_imbalance;
        assert!(imbalance_after < imbalance_before);
    }

    #[test]
    fn test_pack_total_energy_equals_voltage_times_capacity() {
        let pack = SeriesParallelPack::uniform(4, 2, make_cell());
        let v = pack.nominal_voltage();
        let cap = pack.nominal_capacity_ah();
        let energy = v * cap; // Wh
        assert!(energy > 0.0, "energy must be positive, got {}", energy);
        assert!(energy.is_finite(), "energy must be finite");
        // 4s2p: ~4*4.2 V * 2*3.0 Ah = ~100.8 Wh
        assert!(
            energy > 50.0 && energy < 200.0,
            "energy {} out of expected range",
            energy
        );
    }

    #[test]
    fn test_pack_construction_from_uniform() {
        let pack = SeriesParallelPack::uniform(3, 4, make_cell());
        assert_eq!(pack.ns, 3);
        assert_eq!(pack.np, 4);
        assert_eq!(pack.cells.len(), 3);
        assert_eq!(pack.cells[0].len(), 4);
    }

    #[test]
    fn test_pack_step_returns_finite_voltage() {
        let mut pack = SeriesParallelPack::uniform(4, 2, make_cell());
        let state = pack.step(Current(6.0), 1.0, Temperature(298.15));
        assert!(state.voltage.0.is_finite(), "pack voltage must be finite");
        assert!(state.voltage.0 > 0.0, "pack voltage must be positive");
    }

    #[test]
    fn test_pack_snapshot_soc_mean_equals_cell_soc() {
        let pack = SeriesParallelPack::uniform(2, 2, make_cell());
        let state = pack.pack_state_snapshot();
        assert!(
            (state.soc_mean.0 - 1.0).abs() < 1e-9,
            "fresh pack soc_mean={} expected 1.0",
            state.soc_mean.0
        );
        assert!(
            (state.soc_imbalance).abs() < 1e-9,
            "fresh pack imbalance={} expected 0.0",
            state.soc_imbalance
        );
    }

    #[test]
    fn test_pack_charge_increases_soc() {
        let cell = make_cell();
        let mut pack = SeriesParallelPack::uniform(2, 1, cell);
        // Discharge first
        for _ in 0..1000 {
            pack.step(Current(3.0), 1.0, Temperature(298.15));
        }
        let soc_after_discharge = pack.pack_state_snapshot().soc_mean.0;
        // Now charge
        for _ in 0..1000 {
            pack.step(Current(-3.0), 1.0, Temperature(298.15));
        }
        let soc_after_charge = pack.pack_state_snapshot().soc_mean.0;
        assert!(
            soc_after_charge > soc_after_discharge,
            "soc after charge {} should exceed soc after discharge {}",
            soc_after_charge,
            soc_after_discharge
        );
    }

    #[test]
    fn test_pack_bms_trait_pack_state() {
        use crate::battery::pack::Bms;
        let pack = SeriesParallelPack::uniform(2, 2, make_cell());
        let bms_state = pack.pack_state();
        let snap_state = pack.pack_state_snapshot();
        assert!(
            (bms_state.voltage.0 - snap_state.voltage.0).abs() < 1e-9,
            "bms voltage {} != snapshot voltage {}",
            bms_state.voltage.0,
            snap_state.voltage.0
        );
        assert!(
            (bms_state.soc_mean.0 - snap_state.soc_mean.0).abs() < 1e-9,
            "bms soc_mean {} != snapshot soc_mean {}",
            bms_state.soc_mean.0,
            snap_state.soc_mean.0
        );
    }
}
