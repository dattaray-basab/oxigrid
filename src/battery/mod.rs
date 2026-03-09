//! Battery modelling: equivalent circuit models, SoC estimation, thermal, aging, and P2D.
//!
//! # Sub-modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`ecm`]     | Equivalent Circuit Models (Rint, 1RC, 2RC Thevenin) with OCV-SoC curve |
//! | [`soc`]     | SoC estimation: Coulomb counting, Extended Kalman Filter (EKF), UKF |
//! | [`thermal`] | Lumped thermal model: Joule heating, convective cooling, temperature |
//! | [`pack`]    | Series/parallel cell packs, passive balancing, BMS interface |
//! | [`aging`]   | SEI growth (calendar + cycling), lithium plating, capacity fade |
//! | [`p2d`]     | Pseudo-2D / DFN electrochemical model (Single Particle Model) |
//!
//! # Key Traits
//!
//! - [`BatteryModel`]: `terminal_voltage()`, `step()`, `state()` — implemented by all ECMs.
pub mod fast_charging;
pub use fast_charging::{
    CcStage, ChargingProtocol, ChargingState, FastChargingConfig, FastChargingResult,
    FastChargingSimulator, FcBatteryParams, ProtocolComparison,
};

pub mod advanced_bms;
pub mod degradation_model;
pub use degradation_model::{
    AgingConditions, AgingMode, CapacityFade, CycleCounterDoD, DegradationChemistry,
    DegradationMechanism, DegradationModel, ResistanceGrowth,
};
pub mod aging;
pub mod aging_map;
pub mod bms;
pub mod charging;
pub mod ecm;
pub mod electrothermal;
pub mod fault_detection;
pub mod grid_services;
pub mod marketplace;
pub mod p2d;
pub mod pack;
pub mod safety;
pub mod scheduler;
pub mod second_life;
pub mod soc;
pub mod soh_prediction;
pub mod sop;
pub mod state_estimation;
pub mod thermal;
pub use electrothermal::{
    CurrentStep, DriveCycle, ElectrothermalCell, ElectrothermalPack, ElectrothermalSimulator,
    ElectrothermalState, EtSimConfig, EtSimResult, PackEtResult, ThermalManagement,
};
pub mod thermal_management;
pub mod v2g_second_life;

use crate::units::{Current, Energy, StateOfCharge, Temperature, Voltage};
use serde::{Deserialize, Serialize};

/// Instantaneous state of a battery cell or pack.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BatteryState {
    pub voltage: Voltage,
    pub soc: StateOfCharge,
    pub temperature: Temperature,
    pub internal_resistance: f64, // Ohms
    pub capacity_remaining: Energy,
    pub current: Current,
}

/// Core trait for battery equivalent circuit models.
pub trait BatteryModel {
    /// Terminal voltage given current operating conditions.
    fn terminal_voltage(&self, soc: StateOfCharge, current: Current, temp: Temperature) -> Voltage;

    /// Advance the model by one time step.
    ///
    /// `current` is positive for discharge, negative for charge.
    /// `dt` is the time step in seconds.
    fn step(&mut self, current: Current, dt: f64, temp: Temperature) -> BatteryState;

    /// Current state without advancing time.
    fn state(&self) -> BatteryState;
}

/// OCV-SoC lookup table with linear interpolation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OcvSocCurve {
    /// Sorted (soc, ocv_volts) pairs.  SoC in [0, 1].
    points: Vec<(f64, f64)>,
}

impl OcvSocCurve {
    /// Create from a list of (soc, ocv_v) pairs (need not be sorted).
    pub fn new(mut points: Vec<(f64, f64)>) -> Self {
        points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        Self { points }
    }

    /// Typical LiFePO4 OCV curve (approximate).
    pub fn lfp_default() -> Self {
        Self::new(vec![
            (0.00, 3.000),
            (0.05, 3.100),
            (0.10, 3.180),
            (0.20, 3.250),
            (0.30, 3.280),
            (0.40, 3.300),
            (0.50, 3.320),
            (0.60, 3.340),
            (0.70, 3.350),
            (0.80, 3.360),
            (0.90, 3.380),
            (0.95, 3.400),
            (1.00, 3.650),
        ])
    }

    /// Typical NMC/NCR OCV curve (approximate).
    pub fn nmc_default() -> Self {
        Self::new(vec![
            (0.00, 3.000),
            (0.05, 3.400),
            (0.10, 3.520),
            (0.20, 3.620),
            (0.30, 3.680),
            (0.40, 3.720),
            (0.50, 3.760),
            (0.60, 3.810),
            (0.70, 3.860),
            (0.80, 3.920),
            (0.90, 3.980),
            (0.95, 4.060),
            (1.00, 4.200),
        ])
    }

    /// Linearly interpolate OCV at given SoC.
    pub fn ocv(&self, soc: f64) -> f64 {
        let soc = soc.clamp(0.0, 1.0);
        let pts = &self.points;

        if pts.is_empty() {
            return 0.0;
        }
        if soc <= pts[0].0 {
            return pts[0].1;
        }
        if soc >= pts[pts.len() - 1].0 {
            return pts[pts.len() - 1].1;
        }

        // Binary search for bracket
        let pos = pts.partition_point(|&(s, _)| s <= soc);
        let (s0, v0) = pts[pos - 1];
        let (s1, v1) = pts[pos];
        let alpha = (soc - s0) / (s1 - s0);
        v0 + alpha * (v1 - v0)
    }

    /// Numerical derivative dOCV/dSoC at given SoC (used by EKF).
    pub fn docv_dsoc(&self, soc: f64) -> f64 {
        let eps = 1e-4;
        let soc = soc.clamp(eps, 1.0 - eps);
        (self.ocv(soc + eps) - self.ocv(soc - eps)) / (2.0 * eps)
    }
}

pub use advanced_bms::{
    AdvancedBms, AdvancedBmsConfig, BalancingStrategy, BmsChemistry, BmsError, BmsFault,
    BmsFaultType, BmsResult, CellState as AdvancedCellState, FaultSeverity, PackState,
};
pub use second_life::{
    BatteryChemistry, PackAssemblyResult, PortfolioAllocation, SecondLifeApplication,
    SecondLifeEconomics, SecondLifeEconomicsResult, SecondLifePackAssembler, SecondLifePortfolio,
    SohAssessment, SohGrade, SohResult,
};
pub use soh_prediction::{
    ChargeStrategy, SohError, SohForecast, SohPredictionConfig, SohPredictor, UsagePattern,
};
pub use state_estimation::{
    BatteryChemistryType, BatteryPackEstimator, BatteryStateEstimatorConfig, EcmParameters,
    PackStateEstimate,
};
pub use v2g_second_life::{
    EvBatteryProfile, SecondLifeAssessment2, V2gComparison, V2gDegradationConfig,
    V2gSecondLifeAnalyzer, V2gSecondLifeEconomics,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ocv_interpolation() {
        let curve = OcvSocCurve::lfp_default();
        // At SoC = 0 should be 3.0 V
        assert!((curve.ocv(0.0) - 3.0).abs() < 1e-6);
        // At SoC = 1 should be 3.65 V
        assert!((curve.ocv(1.0) - 3.65).abs() < 1e-6);
        // Monotone increasing
        let v50 = curve.ocv(0.50);
        let v90 = curve.ocv(0.90);
        assert!(v90 > v50);
    }

    #[test]
    fn test_ocv_clamp() {
        let curve = OcvSocCurve::nmc_default();
        let v_low = curve.ocv(-0.1);
        let v_high = curve.ocv(1.1);
        assert_eq!(v_low, curve.ocv(0.0));
        assert_eq!(v_high, curve.ocv(1.0));
    }
}
