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
pub mod branch;
pub mod bus;
pub mod contingency;
pub mod formats;
pub mod metrics;
pub mod partition;
pub mod reduction;
pub mod topology;

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
