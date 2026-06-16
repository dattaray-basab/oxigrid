//! Power network data model and Y-bus construction.
//!
//! Provides the core data structures for representing an AC power system:
//!
//! - [`bus`]        — `Bus` and `BusType` (Slack / PV / PQ)
//! - [`branch`]     — `Branch` (π-model: r, x, b, tap, shift, ratings)
//! - [`topology`]   — `PowerNetwork` (buses + branches + generators) with
//!   `from_matpower()`, `validate()`, `admittance_matrix()`, `solve_powerflow()`
//! - [`admittance`] — `build_y_bus()` — sparse `CsMat<Complex64>` Y-bus via π-model
//! - [`formats`]    — MATPOWER `.m`, IEEE-CDF, and pandapower JSON parsers
//! - [`reduction`]  — PTDF/LODF matrices, Ward equivalents, Kron reduction, REI, coherency analysis
pub mod admittance;
pub mod advanced_partition;
pub mod asset_lifecycle;
pub mod asset_management;
pub mod branch;
pub mod bus;
pub mod congestion;
pub mod contingency;
pub mod dlr;
pub mod energy_flow;
pub mod extreme_weather;
pub mod facts;
pub mod feeder_automation;
pub mod flisr;
pub mod formats;
pub mod hvdc;
pub mod hvdc_control;
pub mod impedance_spectroscopy;
pub mod metrics;
pub mod partition;
pub mod reconfiguration;
pub mod reduction;
pub mod reliability_assessment;
pub mod self_healing;
pub mod smart_transformer;
pub mod thevenin;
pub mod topology;
pub mod topology_optimization;
pub mod transformer;
pub mod upfc;

pub use branch::Branch;
pub use bus::{Bus, BusType};
pub use topology::{Generator, PowerNetwork};

pub use reduction::{
    BusRole, CoherencyAnalyzer, CoherencyGroup, ExtendedWardEquivalentResult, KronReducer,
    ReiEquivalent, ReiResult, WardEquivalent, WardEquivalentConfig, WardEquivalentResult,
};

pub mod geospatial;
pub use geospatial::{
    ClusterResult, GeoBoundingBox, GeoLine, GeoNode, GeoNodeType, GeoPoint, GeographicClustering,
    LineType, RoutingResult, SpatialAnalysis, TransmissionRouter,
};

pub mod network_design;
pub use network_design::{
    ExpansionCandidate, ExpansionPlan, ExpansionPlanner, NetworkEdge, SitingResult,
    SteinerTreeResult, SteinerTreeSolver, SubstationSiting, TopologyNode,
};

pub mod reliability_indices;
pub use reliability_indices::{
    BulkSystemReliability, CapacityOutageTable, CotReliabilityCalculator, CotReliabilityIndices,
    CustomerData, DeratedState, FeederReliability, GeneratingUnit, GenerationUnit,
    InterruptionCause, InterruptionEvent, LoadData, MonteCarloReliabilityResult,
    ReliabilityCalculator, ReliabilityConfig, ReliabilityIndices,
};

pub mod offshore_substation;
pub use offshore_substation::{
    CableType, CollectorArray, ElectricalLossBreakdown, ExportCable, OffshoreElectricalSystem,
    OffshoreSubstation, OffshoreSubstationType, OffshoreSystemDesigner,
};

pub mod resilience;

pub mod resilience_planning;
pub use resilience_planning::{
    extreme_event_key, ComponentVulnerability, ExtremHardeningMeasure, ExtremeEventType,
    ExtremeHardeningOption, ExtremeResilienceMetrics, ExtremeResiliencePlan, FireSeverity,
    ResiliencePlanner,
};

pub mod voltage_regulation;
pub use voltage_regulation::*;

pub mod cable_sizing;
pub use cable_sizing::{
    CableDatabase, CableInsulation, CableSizingEngine, CableSizingResult, CableSpec,
    ConductorMaterial, InstallationConditions, InstallationMethod, ThermalRating, VoltageClass,
};

pub mod dc_switching;
pub use dc_switching::{
    ConverterOperatingPoint, ConverterStation, ConverterTopology, DcBreaker, DcBreakerType,
    DcCable, DcFaultEvent, DcFaultType, DcGridSolution, DcSwitchingSimulator, DcTransientResult,
    DcTransientState,
};

#[cfg(test)]
mod tests {
    use super::*;

    // 1. Bus::new sets id, vm, va, gs to expected defaults.
    #[test]
    fn test_bus_new_defaults() {
        let b = Bus::new(1, BusType::Slack);
        assert_eq!(b.id, 1);
        assert!((b.vm - 1.0).abs() < 1e-12, "vm should default to 1.0");
        assert!((b.va - 0.0).abs() < 1e-12, "va should default to 0.0");
        assert!((b.gs - 0.0).abs() < 1e-12, "gs should default to 0.0");
    }

    // 2. BusType is preserved correctly for PV buses.
    #[test]
    fn test_bus_type_pv() {
        let b = Bus::new(2, BusType::PV);
        assert_eq!(b.bus_type, BusType::PV);
    }

    // 3. Branch::effective_tap() returns 1.0 when tap == 0.0.
    #[test]
    fn test_branch_effective_tap_zero() {
        let br = Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        };
        assert_eq!(br.effective_tap(), 1.0);
    }

    // 4. Branch::effective_tap() returns the stored tap when tap != 0.0.
    #[test]
    fn test_branch_effective_tap_nonzero() {
        let br = Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.05,
            shift: 0.0,
            status: true,
        };
        assert!((br.effective_tap() - 1.05).abs() < 1e-9);
    }

    // 5. Branch::tap_complex() with tap=1.0 and shift=0.0 is purely real (≈ 1+0j).
    #[test]
    fn test_branch_tap_complex_no_shift() {
        let br = Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        };
        let tc = br.tap_complex();
        assert!(
            (tc.re - 1.0).abs() < 1e-6,
            "real part should be ≈ 1.0, got {}",
            tc.re
        );
        assert!(
            tc.im.abs() < 1e-6,
            "imaginary part should be ≈ 0.0, got {}",
            tc.im
        );
    }

    // 6. PowerNetwork::new sets base_mva and starts with empty buses.
    #[test]
    fn test_power_network_new() {
        let net = PowerNetwork::new(100.0);
        assert!((net.base_mva - 100.0).abs() < 1e-12);
        assert!(net.buses.is_empty());
    }

    // 7. validate() on an empty network (no slack bus) returns Err.
    #[test]
    fn test_power_network_validate_empty_no_slack() {
        let net = PowerNetwork::new(100.0);
        let result = net.validate();
        assert!(
            result.is_err(),
            "empty network with no slack bus should fail validation"
        );
    }
}
