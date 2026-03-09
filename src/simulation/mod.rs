//! Cyber-physical co-simulation for power systems.
//!
//! This module provides a framework for co-simulating physical power system
//! dynamics together with a communication and control layer, enabling analysis
//! of cyber-physical interactions including:
//!
//! - Communication latency and packet loss
//! - Cyber attack injection (FDI, replay, DoS, man-in-the-middle)
//! - CUSUM-based anomaly detection
//!
//! # Quick Start
//!
//! ```rust,ignore
//! use oxigrid::simulation::cosim::{CosimConfig, CosimEngine, CosimState};
//!
//! let config = CosimConfig {
//!     total_time_s: 10.0,
//!     ..CosimConfig::default()
//! };
//! let engine = CosimEngine::new(config);
//! let initial = CosimState {
//!     time_s: 0.0,
//!     voltage_pu: vec![1.0; 5],
//!     power_mw: vec![50.0; 5],
//!     control_signals: vec![1.0; 5],
//!     stale_measurements: vec![false; 5],
//!     attack_active: false,
//! };
//! let result = engine.run(initial).expect("simulation failed");
//! assert!(!result.attack_detected);
//! ```

pub mod cosim;

pub use cosim::{CosimConfig, CosimEngine, CosimError, CosimResult, CosimState, CyberAttack};

pub mod grid_ops;
pub use grid_ops::{
    ActionOutcome,
    EventType,
    // Quasi-dynamic event-driven simulator
    GridEvent,
    GridOperationsSimulator,
    GridOpsConfig,
    GridOpsError,
    GridOpsResult,
    GridOpsSimulator,
    GridOpsStatistics,
    OperationalEvent,
    OperatorAction,
    OperatorActionType,
    QdGridOpsConfig,
    QdGridOpsResult,
    ScenarioBuilder,
    ScheduledEvent,
    SimBranch,
    SimClock,
    SimGenerator,
    SimLoad,
    SimStorage,
    SystemSnapshot,
};

pub mod protocol;
pub use protocol::{
    Message, MessageType, NetworkLink, ProtocolSimResult, ProtocolSimulator, ProtocolType, SimError,
};

pub mod comm_network;
pub use comm_network::{
    CommLink, CommMessage, CommMessageType, CommNetworkSim, CommProtocol, DeliveryResult,
    LatencyStats,
};

pub mod operator_training;
pub use operator_training::{
    OtsConfig, OtsOperatorAction, ScenarioType, SessionReport, TraineeAction, TrainingSession,
};

pub mod operator_training_v2;
pub use operator_training_v2::{
    Alarm, AlarmSeverity, DebriefReport, KpiTarget, ScenarioLibrary, SystemState,
    TraineeActionType, TraineeResponse, TrainingEvent, TrainingEventType, TrainingScenario,
    TrainingSimSession,
};

pub mod cyber_physical;
// Note: CommLink and CommProtocol are intentionally not re-exported here to
// avoid name collision with the identically named types in comm_network.
// Access them as `cyber_physical::CommLink` / `cyber_physical::CommProtocol`.
pub use cyber_physical::{
    CommNetwork, CommNode, CommNodeType, CyberPhysicalConfig, CyberPhysicalResult,
    CyberPhysicalSimulator, CyberPhysicalState, DosAttack, FdiAttack, FdiAttackType,
};
